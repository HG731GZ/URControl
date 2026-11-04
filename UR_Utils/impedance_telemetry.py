"""外部寄存器 24～47，独立协议 v1；浮点页始终为同周期快照。"""

PHASES = {1: "ARMED", 2: "RAMPING", 3: "RUNNING", 4: "ENDING", 5: "STOPPING"}
FAULTS = {
    0: None,
    1: "运行/协议身份错误",
    2: "心跳超时",
    3: "目标无效或断流",
    4: "目标越界/跳变/起点错误",
    5: "实际位置或速度守卫",
    6: "启动超时",
    7: "机器人循环间隔超限",
}
INPUT_NAMES = [f"input_int_register_{i}" for i in range(24, 34)] + [
    f"input_double_register_{i}" for i in range(24, 32)
]
INPUT_TYPES = ["INT32"] * 10 + ["DOUBLE"] * 8
STANDARD = {
    "timestamp": "DOUBLE",
    "actual_q": "VECTOR6D",
    "actual_qd": "VECTOR6D",
    "runtime_state": "UINT32",
    "robot_mode": "INT32",
    "safety_mode": "INT32",
    "speed_scaling": "DOUBLE",
    "target_speed_fraction": "DOUBLE",
    "payload": "DOUBLE",
    "tcp_offset": "VECTOR6D",
    "actual_TCP_pose": "VECTOR6D",
    "joint_position_deviation_ratios": "VECTOR6D",
    "collision_detection_ratios": "VECTOR6D",
    "actual_current_as_torque": "VECTOR6D",
    "actual_execution_time": "DOUBLE",
}
OUTPUT_NAMES = (
    list(STANDARD)
    + [f"output_int_register_{i}" for i in range(24, 47)]
    + [f"output_double_register_{i}" for i in range(24, 48)]
)
OUTPUT_TYPES = list(STANDARD.values()) + ["INT32"] * 23 + ["DOUBLE"] * 24


def decode(state, run_id):
    i = lambda n: getattr(state, f"output_int_register_{n}")
    if i(24) != run_id or i(45) != run_id or i(29) != i(30) or i(29) <= 0:
        return None
    values = [getattr(state, f"output_double_register_{n}") for n in range(24, 48)]
    return dict(
        run_id=run_id,
        cycle_seq=i(28),
        adopted_reference_seq=i(27),
        source_seq=i(37),
        phase=PHASES.get(i(25), "UNKNOWN"),
        fault=FAULTS.get(i(26), "未知故障"),
        fault_code=i(26),
        q_ref=values[:6],
        q_actual=values[6:12],
        qd_actual=values[12:18],
        tau_cmd=values[18:],
        final=bool(i(46)),
        missed_cycles=i(34),
        limit_mask=i(31),
        max_interval_s=i(32) / 1e6,
        max_compute_s=i(33) / 1e6,
        saturated_cycles=i(35),
        shaped_cycles=i(36),
        peak_error_rad=[i(n) / 1e6 for n in range(38, 44)],
        sample_time_s=i(44) / 1000,
        robot_timestamp_s=state.timestamp,
    )
