"""阻抗寄存器协议 v3，诊断页与六轴补偿实测力矩页独立提交。"""

VERSION = 3

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
    f"input_double_register_{i}" for i in range(24, 38)
]
INPUT_TYPES = ["INT32"] * 10 + ["DOUBLE"] * 14
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
TELEOP_STANDARD = {**STANDARD, "actual_TCP_force": "VECTOR6D"}
OUTPUT_REGISTERS = {
    **{f"output_int_register_{i}": "INT32" for i in range(24, 48)},
    **{f"output_double_register_{i}": "DOUBLE" for i in range(24, 48)},
}


def decode(state, run_id):
    i = lambda n: getattr(state, f"output_int_register_{n}")
    if i(24) != run_id or i(45) <= 0:
        return None
    values = [getattr(state, f"output_double_register_{n}") for n in range(24, 48)]
    flags = i(25)
    phase, fault, limit_mask = flags & 7, (flags >> 3) & 7, flags >> 6
    final = phase == 5 or fault != 0
    axis_page = [i(n) / 1e6 for n in range(38, 44)]
    return dict(
        run_id=run_id,
        cycle_seq=i(45) - 1,
        adopted_reference_seq=i(34),
        source_seq=i(37),
        phase=PHASES.get(phase, "UNKNOWN"),
        fault=FAULTS.get(fault, "未知故障"),
        fault_code=fault,
        q_ref=values[:6],
        q_actual=values[6:12],
        qd_actual=values[12:18],
        tau_cmd=values[18:],
        final=final,
        missed_cycles=int(fault == 7),
        limit_mask=limit_mask,
        max_interval_s=i(32) / 1e6,
        max_compute_s=i(33) / 1e6,
        saturated_cycles=i(35),
        shaped_cycles=i(36),
        peak_error_rad=axis_page if final else None,
        qd_ref=None if final else axis_page,
        sample_time_s=i(44) / 1000,
        robot_timestamp_s=state.timestamp,
    )


def decode_compensated_torque(state, run_id, last_sequence, received_monotonic):
    """从一份 RTDE 帧解出已提交的六轴实测值和来源身份。"""
    sequence = state.output_int_register_46
    if state.output_int_register_24 != run_id or state.runtime_state != 2 or sequence <= last_sequence:
        return None
    return {
        "source": "impedance_get_joint_torques", "run_id": run_id,
        "sample_seq": sequence,
        "robot_timestamp_s": state.timestamp, "received_monotonic_s": received_monotonic,
        "values_nm": [getattr(state, f"output_int_register_{n}") / 1e6 for n in range(26, 32)],
    }
