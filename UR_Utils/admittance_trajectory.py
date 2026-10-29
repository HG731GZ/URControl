"""无接触预设轨迹的离散时标；只计算命令，不连接机器人。

URScript 使用同一组渲染后的整数帧数和标量，每个模型片段仍分别计算两帧。
这里给出限幅前的期望位置，最终命令仍经过脚本的逐帧可行区间约束。
"""

from math import ceil, cos, pi, sqrt


def profile_parameters(config):
    """将静止起止梯形的各阶段向上量化到 2 ms 基础帧。"""
    h = config["SERVO_DT_S"]
    amplitude = config["DIAGNOSTIC_MOTION_AMPLITUDE_RAD"]
    speed = config["DIAGNOSTIC_MOTION_MAX_QD_RAD_S"]
    acceleration = config["DIAGNOSTIC_MOTION_MAX_QDD_RAD_S2"]
    peak = min(speed, sqrt(amplitude * acceleration))
    ramp_frames = max(1, ceil(peak / acceleration / h - 1e-12))
    ramp_s = ramp_frames * h
    cruise_frames = max(0, ceil((amplitude / peak - ramp_s) / h - 1e-12))
    cruise_s = cruise_frames * h
    peak = amplitude / (ramp_s + cruise_s)
    # 三角形轨迹的量化也不能反向把峰速/加速度推高。
    while peak > speed + 1e-12 or peak / ramp_s > acceleration + 1e-12:
        cruise_frames += 1
        cruise_s = cruise_frames * h
        peak = amplitude / (ramp_s + cruise_s)
    trapezoid_frames = 2 * ramp_frames + cruise_frames
    # adm_step 的保守离散制动上界会在 ±A 节点留下短暂收敛尾段。
    # 先给最终限幅命令 100 ms 到达并归零，再开始完整的节点/末尾静止窗口。
    settle_frames = ceil(.1 / h - 1e-12)
    move_frames = trapezoid_frames + settle_frames
    hold_frames = ceil(config["DIAGNOSTIC_MOTION_NODE_HOLD_S"] / h - 1e-12)
    tail_frames = ceil(config["DIAGNOSTIC_MOTION_TAIL_HOLD_S"] / h - 1e-12)
    total_frames = 4 * (hold_frames + move_frames) + tail_frames
    return {
        "ramp_frames": ramp_frames,
        "cruise_frames": cruise_frames,
        "move_frames": move_frames,
        "trapezoid_frames": trapezoid_frames,
        "settle_frames": settle_frames,
        "settle_s": settle_frames * h,
        "hold_frames": hold_frames,
        "tail_frames": tail_frames,
        "total_frames": total_frames,
        "ramp_s": ramp_s,
        "cruise_s": cruise_s,
        "peak_speed_rad_s": peak,
        "acceleration_rad_s2": peak / ramp_s,
        "duration_s": total_frames * h,
    }


def desired_offset(config, frame):
    """返回本轮有效运行开始后的第 frame 帧期望偏移，锚点为零。"""
    h = config["SERVO_DT_S"]
    amplitude = config["DIAGNOSTIC_MOTION_AMPLITUDE_RAD"]
    if config["DIAGNOSTIC_MOTION_PROFILE"] == "cosine":
        t = frame * h
        period = config["DIAGNOSTIC_MOTION_PERIOD_S"]
        return .5 * amplitude * (1 - cos(2 * pi * t / period)) if 0 <= t < period else 0.
    plan = profile_parameters(config)
    if frame <= 0 or frame >= plan["total_frames"]:
        return 0.
    block = plan["hold_frames"] + plan["move_frames"]
    leg = min(4, frame // block)
    if leg >= 4:
        return 0.
    starts = (0., amplitude, 0., -amplitude)
    directions = (1., -1., -1., 1.)
    move_frame = frame - leg * block - plan["hold_frames"]
    if move_frame <= 0:
        return starts[leg]
    if move_frame >= plan["trapezoid_frames"]:
        return starts[leg] + directions[leg] * amplitude
    t = move_frame * h
    ramp = plan["ramp_s"]
    cruise = plan["cruise_s"]
    speed = plan["peak_speed_rad_s"]
    accel = plan["acceleration_rad_s2"]
    if t < ramp:
        distance = .5 * accel * t * t
    elif t < ramp + cruise:
        distance = .5 * speed * ramp + speed * (t - ramp)
    else:
        remaining = (plan["trapezoid_frames"] - move_frame) * h
        distance = amplitude - .5 * accel * remaining * remaining
    return starts[leg] + directions[leg] * distance
