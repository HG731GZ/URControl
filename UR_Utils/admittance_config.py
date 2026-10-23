"""导纳默认参数、寄存器约定及脚本渲染；每轮使用外部传入的独立配置副本。"""

from copy import deepcopy
from math import isfinite, pi
from pathlib import Path
from string import Template


# 以下值仅为低速联调起点，尚未针对具体工具和接触环境整定。
# 界面仅编辑柔顺轴、M/D/K、目标载荷、连续死区和末端柔顺方向，共 13 项配置。
# 其余滤波、限幅、伺服、通信和启动停止参数在本文件修改；修改后重启程序生效。
# 界面恢复默认仅重置上述 13 项，不会覆盖本文件；每轮仍传递并记录完整配置。
# 通信频率、心跳、超时和启动条件仅在本文件调整。
TELEOP_REFERENCE_HZ = 100.0
ADMITTANCE_STATE_HZ = 100.0
ADMITTANCE_HEARTBEAT_HZ = 50.0
ADMITTANCE_WATCHDOG_MIN_HZ = 10.0
ADMITTANCE_HEARTBEAT_TIMEOUT_S = 0.08
MASTER_FRAME_TIMEOUT_S = 0.5
ADMITTANCE_BIAS_DURATION_S = 0.5
ADMITTANCE_START_TIMEOUT_S = 5.0
ADMITTANCE_STOP_TIMEOUT_S = 2.0
ADMITTANCE_STATE_TIMEOUT_S = 1.0
STATIONARY_MAX_QD_RAD_S = 0.01
TELEOP_SYNC_TOLERANCE_RAD = 0.05

# 伺服及最终命令限幅仅在本文件调整。
SERVO_LOOKAHEAD_S = 0.08
SERVO_GAIN = 300.0
NOMINAL_MAX_QD_RAD_S = [0.7] * 6
COMMAND_MAX_QD_RAD_S = [1.0] * 6
COMMAND_MAX_QDD_RAD_S2 = [2.0] * 6
COMMAND_MAX_TRACKING_ERROR_RAD = [0.2] * 6
# 最终命令持续落后于候选目标超过此时间时停止，不把滞后写入导纳偏移。
COMMAND_LAG_TIMEOUT_S = 0.5
# 对称绝对关节范围；后端可用原控制器的范围覆盖本轮参数。
JOINT_POSITION_LIMIT_RAD = [2.0 * pi] * 6
# 末端逆解相对上一周期关节命令的最大变化，不替代最终速度限制。
CART_MAX_IK_STEP_RAD = [0.05] * 6

# 关节惯量：千克平方米；阻尼：牛米秒每弧度；载荷：牛米。
JOINT_AXES = [1, 1, 1, 1, 1, 1]
JOINT_MASS = [6.0, 6.0, 4.0, 1.0, 1.0, 0.5]
JOINT_DAMPING = [30.0, 30.0, 20.0, 5.0, 5.0, 3.0]
JOINT_TARGET_TORQUE_NM = [0.0] * 6
JOINT_DEADBAND_NM = [1.5, 1.5, 1.0, 0.3, 0.3, 0.2]
# 滤波和偏移、速度上限留在本文件；恢复刚度 K 仅用于遥操作。
JOINT_FILTER_CUTOFF_HZ = 10.0
TELEOP_JOINT_STIFFNESS = [30.0, 30.0, 20.0, 5.0, 5.0, 3.0]
TELEOP_JOINT_MAX_OFFSET_RAD = [0.10] * 6
TELEOP_JOINT_MAX_OFFSET_SPEED_RAD_S = [0.10] * 6
FREE_JOINT_MAX_SPEED_RAD_S = [0.10] * 6

# 柔顺方向在本轮启动时固定为基座方向或启动时工具方向；中心始终在当前 TCP。
CART_FRAME = "base"
CART_AXES = [1, 1, 1, 1, 1, 1]
# 前三轴质量和阻尼为千克、牛秒每米；后三轴为千克平方米、牛米秒每弧度。
CART_MASS = [10.0, 10.0, 10.0, 0.5, 0.5, 0.5]
CART_DAMPING = [100.0, 100.0, 100.0, 5.0, 5.0, 5.0]
CART_TARGET_WRENCH = [0.0] * 6
CART_DEADBAND = [3.0, 3.0, 3.0, 0.2, 0.2, 0.2]
# 滤波和偏移、速度上限留在本文件；恢复刚度 K 仅用于遥操作。
CART_FILTER_CUTOFF_HZ = 10.0
TELEOP_CART_STIFFNESS = [200.0, 200.0, 200.0, 10.0, 10.0, 10.0]
TELEOP_CART_MAX_POSITION_OFFSET_M = 0.03
TELEOP_CART_MAX_ROTATION_OFFSET_RAD = 0.15
TELEOP_CART_MAX_OFFSET_LINEAR_SPEED_M_S = 0.03
TELEOP_CART_MAX_OFFSET_ANGULAR_SPEED_RAD_S = 0.15
TELEOP_CART_MAX_JOINT_CORRECTION_RAD = [0.15] * 6
FREE_CART_MAX_LINEAR_SPEED_M_S = 0.03
FREE_CART_MAX_ANGULAR_SPEED_RAD_S = 0.15
STOP_JOINT_ACCEL_RAD_S2 = 1.0

# 输入和输出具有独立地址空间；整段上半区仅由专用导纳连接写入。
ADMITTANCE_INPUT_INT_BASE = 24
ADMITTANCE_INPUT_FLOAT_BASE = 24
ADMITTANCE_OUTPUT_INT_BASE = 24
ADMITTANCE_OUTPUT_FLOAT_BASE = 24


def snapshot():
    """返回本轮可独立保存和覆盖的参数副本，所有值均可写入采集元数据。"""
    return {
        name: deepcopy(value)
        for name, value in globals().items()
        if name.isupper() and not name.startswith("_")
    }


def units(reference_source, space, *, config=None):
    """说明遥操作偏移和主动柔顺速度的不同含义，以及载荷测量坐标。"""
    if reference_source not in ("teleop", "free") or space not in ("joint", "cartesian"):
        raise ValueError("导纳场景或空间无效")
    joint = space == "joint"
    teleop = reference_source == "teleop"
    return {
        "reference_source": reference_source,
        "space": space,
        "q_actual": "rad",
        "q_cmd": "rad",
        "q_master": "rad" if teleop else None,
        "q_nom": "rad" if teleop else None,
        "q_start": None if teleop else "rad",
        "source_seq": "接收帧计数" if teleop else None,
        "robot_timestamp": "s",
        "drive": ["N*m"] * 6 if joint else ["N"] * 3 + ["N*m"] * 3,
        "state_name": ("delta_q" if joint else "delta_x") if teleop else "v_free",
        "state": (["rad"] * 6 if joint else ["m"] * 3 + ["rad"] * 3)
        if teleop else (["rad/s"] * 6 if joint else ["m/s"] * 3 + ["rad/s"] * 3),
        "cartesian_frame": CART_FRAME if config is None else config["CART_FRAME"],
        "cartesian_frame_definition": "基座方向或启动时工具方向固定，柔顺中心为当前TCP",
        "force_point": "先将法兰测量力矩平移到当前实际TCP，再旋转到柔顺方向",
        "joint_torque_source": "机器人端get_joint_torques补偿力矩",
        "rotation_composition": "平移单独相加；基座方向旋转增量左乘参考旋转",
        "target_load_sign": "测量外载荷减参考载荷",
    }


def validate_config(config=None):
    """在上传运动脚本之前检查数值、六轴维度和通信及限幅约束。"""
    values = snapshot() if config is None else deepcopy(config)
    expected = snapshot()
    if values.keys() != expected.keys():
        raise ValueError("导纳参数必须包含完整的已声明配置，且不能包含未知字段")
    for name, default in expected.items():
        value = values[name]
        if isinstance(default, str):
            if value not in ("base", "tool_start"):
                raise ValueError("CART_FRAME 必须为 base 或 tool_start")
        elif isinstance(default, list):
            if not isinstance(value, (list, tuple)) or len(value) != 6:
                raise ValueError(f"{name} 必须包含六个数值")
            if any(isinstance(x, bool) or not isinstance(x, (int, float)) or not isfinite(x) for x in value):
                raise ValueError(f"{name} 必须为有限数值")
            if name.endswith("_AXES"):
                if any(x not in (0, 1) for x in value):
                    raise ValueError(f"{name} 只能包含 0 或 1")
            elif "TARGET" not in name:
                allow_zero = "STIFFNESS" in name or "DEADBAND" in name
                if any(x < 0 if allow_zero else x <= 0 for x in value):
                    raise ValueError(f"{name} 数值范围无效")
            values[name] = list(value)
        elif isinstance(value, bool) or not isinstance(value, (int, float)) or not isfinite(value) or value <= 0:
            raise ValueError(f"{name} 必须为正有限数值")

    for name, width in (("ADMITTANCE_INPUT_INT_BASE", 5), ("ADMITTANCE_INPUT_FLOAT_BASE", 6),
                        ("ADMITTANCE_OUTPUT_INT_BASE", 5), ("ADMITTANCE_OUTPUT_FLOAT_BASE", 24)):
        base = values[name]
        if not isinstance(base, int) or base < 24 or base + width > 48:
            raise ValueError(f"{name} 的配方必须完全位于 24 至 47 上半区寄存器")
    if not 0.03 <= values["SERVO_LOOKAHEAD_S"] <= 0.2 or not 100 <= values["SERVO_GAIN"] <= 2000:
        raise ValueError("servoj 的前瞻时间或增益超出官方允许范围")
    if any(a >= b for a, b in zip(values["NOMINAL_MAX_QD_RAD_S"], values["COMMAND_MAX_QD_RAD_S"])):
        raise ValueError("名义跟踪速度必须小于最终速度上限，为导纳运动预留余量")
    if any(a + b > c for a, b, c in zip(values["NOMINAL_MAX_QD_RAD_S"],
                                       values["TELEOP_JOINT_MAX_OFFSET_SPEED_RAD_S"],
                                       values["COMMAND_MAX_QD_RAD_S"])):
        raise ValueError("名义跟踪和关节导纳速度之和不能超过最终速度上限")
    if any(a > b for a, b in zip(values["FREE_JOINT_MAX_SPEED_RAD_S"], values["COMMAND_MAX_QD_RAD_S"])):
        raise ValueError("主动柔顺关节速度不能超过最终速度上限")
    if not 2 / values["ADMITTANCE_HEARTBEAT_HZ"] < values["ADMITTANCE_HEARTBEAT_TIMEOUT_S"] < 1 / values["ADMITTANCE_WATCHDOG_MIN_HZ"]:
        raise ValueError("软件心跳超时必须覆盖两个心跳周期，并早于机器人 RTDE 看门狗")
    if values["ADMITTANCE_START_TIMEOUT_S"] <= values["ADMITTANCE_BIAS_DURATION_S"]:
        raise ValueError("启动超时必须大于偏置采样时长")
    if values["ADMITTANCE_STOP_TIMEOUT_S"] <= max(values["COMMAND_MAX_QD_RAD_S"]) / values["STOP_JOINT_ACCEL_RAD_S2"]:
        raise ValueError("停止超时必须覆盖最大命令速度的减速时间")
    if max(values["ADMITTANCE_STATE_HZ"], values["TELEOP_REFERENCE_HZ"], values["ADMITTANCE_HEARTBEAT_HZ"]) > 500:
        raise ValueError("通信频率不能超过 500 Hz")
    return values


def render_script(reference_source, space, *, config=None):
    """将已验证的固定类型参数写入完整主程序，不执行或连接机器人。"""
    if reference_source not in ("teleop", "free") or space not in ("joint", "cartesian"):
        raise ValueError("导纳场景或空间无效")
    values = validate_config(config)
    rendered = {name: repr(value) for name, value in values.items()}
    rendered["REFERENCE_SOURCE"] = "1" if reference_source == "teleop" else "2"
    rendered["ADMITTANCE_SPACE"] = "1" if space == "joint" else "2"
    rendered["CART_FRAME_TOOL_START"] = "True" if values["CART_FRAME"] == "tool_start" else "False"
    rendered["WATCHDOG_VARIABLE"] = f'"input_int_register_{values["ADMITTANCE_INPUT_INT_BASE"] + 1}"'
    template = Path(__file__).with_name("scripts").joinpath("admittance_control.script").read_text(encoding="utf-8")
    script = Template(template).substitute(rendered)
    # 源文件保留中文说明；上传只使用 ASCII，并保留空行以对应控制器错误行号。
    script = "\n".join("" if line.lstrip().startswith("#") else line
                       for line in script.splitlines()) + "\n"
    script.encode("ascii")
    return script
