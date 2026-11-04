"""六关节 K/D 阻抗的 SI 参数契约；实机调参基础上的当前默认值。"""

from copy import deepcopy
import math
import re
from pathlib import Path

DEFAULTS = {
    "JOINT_K": [2400.0, 3200, 2600, 1000, 800, 1000],
    "JOINT_D": [150.0, 240, 180, 60, 40, 40],
    "TAU_MAX_NM": [24.0, 24, 18, 6, 6, 6],
    "TAU_RATE_MAX_NM_S": [300.0, 300, 200, 200, 200, 200],
    "REF_MAX_QD_RAD_S": [0.8235294117647058, 0.7058823529411765, 0.7058823529411765, 1.0, 1.0, 1.0],
    "REF_MAX_QDD_RAD_S2": [30.0, 30.0, 30.0, 60.0, 60.0, 60.0],
    "JOINT_POSITION_LIMIT_RAD": [6.283185307179586, 6.283185307179586, 6.283185307179586, 6.283185307179586, 6.283185307179586, 6.283185307179586],
    "ENVELOPE_RAD": [0.22, 0.22, 0.22, 0.22, 0.22, 0.22],
    "ACTUAL_MAX_QD_RAD_S": [1.2, 1.2, 1.2, 1.2, 1.2, 1.2],
    "VISCOUS_SCALE": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "COULOMB_SCALE": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "SOURCE_TIMEOUT_S": 0.25,
    "HEARTBEAT_TIMEOUT_S": 0.2,
    "FEEDBACK_TIMEOUT_S": 0.25,
    "SEND_TIMEOUT_S": 0.02,
    "HARD_WATCHDOG_HZ": 1.0,
    "STOP_ACCEL_RAD_S2": 8.0,
    "STOP_TIMEOUT_S": 4.0,
    "STATIONARY_QD_RAD_S": 0.01,
    "STOP_CONFIRM_S": 0.1,
    "START_TOLERANCE_RAD": 0.003,
    "RAMP_S": 0.5,
    "START_TIMEOUT_S": 8.0,
    "END_HOLD_S": 1.0,
    "END_SETTLE_TIMEOUT_S": 5.0,
    "PLAYBACK_MAX_LAG_S": 0.1,
    "MAX_SOURCE_STEP_RAD": 0.1,
    "CYCLE_MAX_S": 0.010,
    "EXPECTED_TCP_OFFSET": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
}


def snapshot():
    return deepcopy(DEFAULTS)


def validate(config=None):
    cfg = snapshot() if config is None else deepcopy(config)
    if set(cfg) != set(DEFAULTS):
        raise ValueError(f"参数键不匹配：{set(cfg)^set(DEFAULTS)}")
    for key, default in DEFAULTS.items():
        value = cfg[key]
        if isinstance(default, list):
            if len(value) != 6 or any(not math.isfinite(float(x)) for x in value):
                raise ValueError(f"{key} 必须是六个有限数")
            cfg[key] = [float(x) for x in value]
            if key in ("VISCOUS_SCALE", "COULOMB_SCALE"):
                if any(value):
                    raise ValueError("当前阻抗控制要求全部摩擦补偿为 0")
            elif key != "EXPECTED_TCP_OFFSET" and any(x <= 0 for x in value):
                raise ValueError(f"{key} 必须为正")
        else:
            cfg[key] = float(value)
            if not math.isfinite(cfg[key]) or cfg[key] <= 0:
                raise ValueError(f"{key} 不合法")
    if not 0.002 < cfg["CYCLE_MAX_S"] <= 0.010:
        raise ValueError("循环间隔门槛须大于 2 ms 且不超过 10 ms")
    braking = max(cfg["ACTUAL_MAX_QD_RAD_S"]) / cfg["STOP_ACCEL_RAD_S2"]
    if cfg["HEARTBEAT_TIMEOUT_S"] + braking + 0.1 >= 1 / cfg["HARD_WATCHDOG_HZ"]:
        raise ValueError("软件失联、制动和余量超过硬看门狗预算")
    return cfg


def render_script(config, run_id, reference_source="trajectory"):
    if reference_source not in ("teleop", "trajectory", "step"):
        raise ValueError("未知阻抗目标源")
    cfg = validate(config)
    script = (
        Path(__file__)
        .with_name("scripts")
        .joinpath("impedance_control.script")
        .read_text()
    )
    for key, value in cfg.items():
        script = script.replace("{{" + key + "}}", str(value))
    script = script.replace("{{RUN_ID}}", str(int(run_id)))
    script = script.replace("{{DIRECT_STEP}}", str(reference_source == "step"))
    script = script.replace("{{TELEOP}}", str(reference_source == "teleop"))
    if "{{" in script:
        raise ValueError("脚本包含未替换的参数")
    # 展开固定六轴循环，使 URScript 指令计算保持在 2 ms 周期内。
    lines = script.splitlines()
    while any("while j < 6:" in line for line in lines):
        begin = next(i for i, line in enumerate(lines) if "while j < 6:" in line)
        depth, end = 1, begin + 1
        while depth:
            stripped = lines[end].strip()
            if stripped.startswith(("if ", "while ", "def ", "thread ")):
                depth += 1
            if stripped == "end":
                depth -= 1
            end += 1
        body = [
            line for line in lines[begin + 1 : end - 1] if line.strip() != "j = j+1"
        ]
        expanded = []
        for axis in range(6):
            expanded.extend(re.sub(r"\bj\b", str(axis), line) for line in body)
        if lines[begin - 1].strip() == "local j = 0":
            begin -= 1
        lines[begin:end] = expanded
    script = "\n".join(lines) + "\n"
    return script
