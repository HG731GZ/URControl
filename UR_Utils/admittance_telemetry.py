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
_INT_FIELDS = ("phase", "exit_reason", "adopted_source_seq", "cycle_count", "reference_source",
               "heartbeat_echo", "heartbeat_age_us", "heartbeat_max_interval_us")


def output_recipe(config: dict) -> tuple[list[str], list[str]]:
    """返回标准机器人字段及本项目占用的完整输出寄存器配方。"""
    names = [field[0] for field in ROBOT_FIELDS]
    types = [field[1] for field in ROBOT_FIELDS]
    names += [f"output_int_register_{config['ADMITTANCE_OUTPUT_INT_BASE'] + index}" for index in range(8)]
    types += ["INT32"] * 8
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
    """给每轮日志写入配方和实际可观测量的含义。"""
    int_base = config["ADMITTANCE_OUTPUT_INT_BASE"]
    float_base = config["ADMITTANCE_OUTPUT_FLOAT_BASE"]
    registers = {f"output_int_register_{int_base + index}": name
                 for index, name in enumerate(_INT_FIELDS)}
    for offset, name in ((0, "drive"), (6, "delta_q" if space == "joint" else "delta_x"),
                         (12, "q_cmd"), (18, "q_nom")):
        for axis in range(6):
            registers[f"output_double_register_{float_base + offset + axis}"] = f"{name}[{axis}]"
    return {
        "robot_fields": {name: {"rtde_type": field_type, "unit": unit, "description": description}
                         for name, field_type, unit, description in ROBOT_FIELDS},
        "output_registers": registers,
        "output_packet_bytes": output_packet_size(config),
        "host_monotonic_s": "主机单调时间，用于对齐接收、发送和控制诊断",
        "host_time_unix_s": "主机 Unix 时间，用于与外部日志对齐",
        "drive": "脚本经过静止偏置扣除、滤波、目标载荷扣除和连续死区后的有效驱动载荷",
        "unreported_values": "脚本未上报的补偿关节力矩原始样本、偏置、滤波中间量和导纳内部速度未记录为测量值",
        "program_seen": "已看到属于本轮脚本的初始化寄存器；为假时输出寄存器可能来自此前程序",
    }
