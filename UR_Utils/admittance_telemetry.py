"""导纳 RTDE 状态配方、帧长度及日志字段说明。"""


# 字段顺序同时用于机器人配方与日志；向量在采集时复制为独立列表。
ROBOT_FIELDS = (
    ("timestamp", "DOUBLE", "s", "机器人控制器运行时间"),
    ("runtime_state", "UINT32", None, "机器人程序运行状态"),
    ("actual_q", "VECTOR6D", "rad", "实际关节角"),
    ("actual_qd", "VECTOR6D", "rad/s", "实际关节角速度"),
    ("actual_TCP_pose", "VECTOR6D", ["m"] * 3 + ["rad"] * 3, "基座坐标下的实际 TCP 位姿"),
    ("safety_status_bits", "UINT32", None, "安全状态位"),
    ("robot_mode", "INT32", None, "机器人模式"),
    ("safety_mode", "INT32", None, "安全模式"),
    ("robot_status_bits", "UINT32", None, "机器人状态位"),
    ("speed_scaling", "DOUBLE", "比例", "轨迹限速比例"),
    ("target_speed_fraction", "DOUBLE", "比例", "目标速度比例"),
    ("actual_TCP_speed", "VECTOR6D", ["m/s"] * 3 + ["rad/s"] * 3, "实际 TCP 速度"),
    ("actual_TCP_force", "VECTOR6D", ["N"] * 3 + ["N*m"] * 3, "RTDE 实际 TCP 力和力矩；不是补偿关节力矩，也不是导纳处理后的 drive"),
    ("actual_current", "VECTOR6D", "A", "实际关节电流；不等同于补偿关节力矩"),
    ("joint_temperatures", "VECTOR6D", "摄氏度", "关节温度"),
    ("target_q", "VECTOR6D", "rad", "控制器轨迹目标关节角；不等同于脚本 q_cmd"),
    ("target_qd", "VECTOR6D", "rad/s", "控制器轨迹目标关节角速度"),
    ("actual_execution_time", "DOUBLE", "ms", "控制器实时线程执行耗时"),
    ("tcp_offset", "VECTOR6D", ["m"] * 3 + ["rad"] * 3, "TCP 相对法兰的配置偏移"),
    ("payload", "DOUBLE", "kg", "当前负载质量"),
    ("payload_cog", "VECTOR3D", "m", "当前负载质心"),
    ("payload_inertia", "VECTOR6D", "kg*m^2", "当前负载惯性张量"),
)

_TYPE_BYTES = {"DOUBLE": 8, "UINT32": 4, "INT32": 4, "VECTOR6D": 48, "VECTOR3D": 24}
TELEMETRY_SCHEMA_VERSION = 3
PAGE_NAMES = {1: "anchor", 2: "load", 3: "execution", 4: "timing"}
_COMMON_INT_FIELDS = (
    "phase", "exit_reason", "adopted_source_seq", "model_sample_seq", "reference_source",
    "heartbeat_echo", "heartbeat_age_us", "heartbeat_max_interval_us", "page_seq_begin",
    "page_seq_end", "page_type", "run_id",
)
_PAGE_INT_FIELDS = {
    1: ("snapshot_time_us", "dt_base_us", "dt_model_design_us", "bias_sample_count",
        "observation_axis", "motion_axis", "diagnostic_mode", "reserved", "reserved",
        "reserved", "reserved", "snapshot_time_us"),
    2: ("sample_time_us", "dt_model_actual_us", "dt_model_max_us", "compute_duration_us",
        "bias_sample_count", "generated_seq", "submitted_seq", "limit_reason",
        "load_duration_us", "publish_duration_us", "bias_ready", "snapshot_time_us"),
    3: ("generated_seq", "submitted_seq", "adopted_seq", "frame", "based_on_seq", "start_frame",
        "end_frame", "gap_count", "limit_reason", "servoj_start_interval_us",
        "servoj_max_interval_us", "snapshot_time_us"),
    4: ("generated_seq", "submitted_seq", "adopted_seq", "frame", "based_on_seq", "start_frame",
        "end_frame", "gap_count", "limit_reason", "bias_sample_count", "bias_ready", "snapshot_time_us"),
}
TIMING_FLOAT_FIELDS = (
    "sample_time_s", "generated_time_s", "submitted_time_s", "adopted_time_s",
    "target_age_s", "target_max_age_s", "dt_model_actual_s", "dt_model_max_s",
    "servoj_start_interval_s", "servoj_max_interval_s", "dt_base_s", "dt_model_design_s",
    *(f"bias_std_nm[{axis}]" for axis in range(6)),
    "compute_duration_s", "compute_max_duration_s", "servoj_duration_s",
    "servoj_duration_max_s", "publish_duration_s", "publish_max_duration_s",
)


def output_int_count(config: dict) -> int:
    """全部模式使用 24 个整数，完整页身份不随诊断轴改变。"""
    return 24


def state_frequency(config: dict) -> float:
    """诊断使用专属频率，普通导纳使用明确配置的状态频率。"""
    return (config["ADMITTANCE_STATE_HZ"] if config["DIAGNOSTIC_MODE"] == "admittance"
            else config["DIAGNOSTIC_STATE_HZ"])


def output_recipe(config: dict) -> tuple[list[str], list[str]]:
    """返回标准机器人字段及本项目占用的完整输出寄存器配方。"""
    names = [field[0] for field in ROBOT_FIELDS]
    types = [field[1] for field in ROBOT_FIELDS]
    names += [f"output_int_register_{config['ADMITTANCE_OUTPUT_INT_BASE'] + index}" for index in range(24)]
    types += ["INT32"] * 24
    names += [f"output_double_register_{config['ADMITTANCE_OUTPUT_FLOAT_BASE'] + index}" for index in range(24)]
    types += ["DOUBLE"] * 24
    return names, types


def output_packet_size(config: dict) -> int:
    """计算完整 RTDE 数据包字节数，包含三字节头和一字节配方编号。"""
    _, types = output_recipe(config)
    return 4 + sum(_TYPE_BYTES[field_type] for field_type in types)


def robot_sample(state) -> dict:
    """提取独立机器人状态，不保留 RTDE 接收对象及其中的向量引用。"""
    result = {}
    for name, field_type, _, _ in ROBOT_FIELDS:
        value = getattr(state, name)
        if field_type.startswith("VECTOR"):
            result[name] = list(value)
        elif field_type in ("INT32", "UINT32"):
            result[name] = int(value)
        else:
            result[name] = float(value)
    return result


def log_schema(config: dict, space: str) -> dict:
    """记录第三版完整冻结页布局，禁止把不同页拼成同一模型或执行样本。"""
    int_base = config["ADMITTANCE_OUTPUT_INT_BASE"]
    float_base = config["ADMITTANCE_OUTPUT_FLOAT_BASE"]
    registers = {f"output_int_register_{int_base + index}": name
                 for index, name in enumerate(_COMMON_INT_FIELDS)}
    for index in range(12):
        registers[f"output_int_register_{int_base + 12 + index}"] = {
            PAGE_NAMES[layout]: fields[index] for layout, fields in _PAGE_INT_FIELDS.items()
        }
    for index in range(24):
        block, axis = divmod(index, 6)
        registers[f"output_double_register_{float_base + index}"] = {
            "anchor": f"q_anchor[{axis}]" if block == 0 else "未使用",
            "load": f"{('tau_raw_nm', 'bias_nm', 'filtered_nm', 'drive')[block]}[{axis}]",
            "execution": f"{('q_cmd', 'qd_cmd', 'q_nom', 'delta_q')[block]}[{axis}]",
            "timing": TIMING_FLOAT_FIELDS[index],
        }
    return {
        "telemetry_schema_version": TELEMETRY_SCHEMA_VERSION,
        "layout": "continuous_servo_frozen_pages",
        "robot_fields": {name: {"rtde_type": field_type, "unit": unit, "description": description}
                         for name, field_type, unit, description in ROBOT_FIELDS},
        "output_registers": registers,
        "output_packet_bytes": output_packet_size(config),
        "input_int_registers": {
            f"input_int_register_{config['ADMITTANCE_INPUT_INT_BASE'] + index}": name
            for index, name in enumerate(("enable", "heartbeat", "source_seq", "space",
                                          "reference_source", "anchor_ack", "run_id", "scaling_ok"))
        },
        "program_seen": "已收到运行身份与本轮一致、起止序号相同且大于零的完整页",
        "lifecycle_valid": "本包属于本轮完整页；停止状态不依赖锚点确认",
        "derived_valid": "本包属于本轮完整页且锚点已确认；原始 RTDE 机器人状态仍有独立采样时刻",
        "page_identity": "每页有 run_id、page_seq、model_sample_seq、snapshot_time_s 和到达关联时间；缓存保留各自身份，不跨页拼成同步样本",
        "continuous.execution": "最近完整执行页；q_cmd/qd_cmd 为执行器已采用命令，生成和提交身份不代表已经执行",
        "continuous.timing": "最近完整计时页；最大间隔、缺口计数由执行器累计，发布丢页不会清零",
        "load": "最近完整六轴载荷页；raw/bias/filtered/drive 同页冻结，偏置未完成时 bias/filtered/drive 为 None",
        "q_start": "本轮固定锚点，只在完整锚点页确认一次，不因页面切换改变",
        "q_cmd": "仅当前执行页有效；其他类型页清空顶层命令，展示历史采用值需读取带身份的 continuous.execution",
        "drive": "仅当前载荷页且偏置已完成时有效；保持及预设微动仍计算载荷，但不以外力驱动轨迹",
        "time_encoding": "整数 _us 为相对脚本起点或间隔的微秒，浮点 _s 为秒；未获得的负计时解码为 None；bias_std_nm 是六轴总体标准差",
        "host_monotonic_s": "主机单调时间，用于对齐接收、发送和控制诊断",
        "host_time_unix_s": "主机 Unix 时间，用于与外部日志对齐",
        "host_processing": "packet_prepare/send/receive/decode/log_submit 分别保存最近与最大耗时，绝对发送期限和迟到单独记录；未验收完整业务负载预算",
        "scaling_contract": "空闲零缩放不禁止上传；上传后无运动等待 runtime_state=2 且双比例首次为 1，再发送 scaling_ok=1；确认后运行中任一比例变化锁定为 0，由执行器按原因 17 停止；若伺服间隔先超预算可先报告 16，保留实际缩放及最先退出原因",
        "unreported_values": "未使用寄存器不解码为测量值；不同页间不插值补齐、不以模型生成值代替执行采用值",
    }
