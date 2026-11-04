"""已安排现场监护的 J1～J6 单轮联调，沿用 GUI 后端并固定整次调参的工作区中心。"""

import argparse
import json
import math
from pathlib import Path
import signal
import time

from admittance_manual import read_robot_state, require_stopped
from UR_Utils.impedance_config import snapshot, validate
from UR_Utils.impedance_process import ImpedanceProcess
from UR_Utils.impedance_step import JointStep
from UR_Utils.impedance_trajectory import JointTrajectory, generate_axis_tuning


def generate_move(anchor, axis, target, speed, config):
    """以五次平滑曲线移动到固定工作区内的一个位置，供人工回位使用。"""
    distance = target - anchor[axis]
    speed = min(speed, config["REF_MAX_QD_RAD_S"][axis] * 0.85)
    acceleration = config["REF_MAX_QDD_RAD_S2"][axis] * 0.85
    duration = max(1.875 * abs(distance) / speed,
                   math.sqrt((10 / math.sqrt(3)) * abs(distance) / acceleration), 0.1)
    count = math.ceil(duration * 100)
    rows = []
    for index in range(count + 1):
        u = index / count
        q = list(anchor)
        q[axis] += distance * (10 * u**3 - 15 * u**4 + 6 * u**5)
        rows.append(tuple(q))
    return JointTrajectory(tuple(rows), 100.0, "absolute")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--workspace-state", type=Path, required=True,
                        help="本次现场确认的初始只读状态文件，后续各轮复用同一文件")
    parser.add_argument("--mode", choices=("step", "trajectory", "move"), required=True)
    parser.add_argument("--target-offset", type=float, default=0.04,
                        help="阶跃或单段平滑回位终点相对固定工作区中心的角度，单位 rad")
    parser.add_argument("--amplitude", type=float, default=0.05)
    parser.add_argument("--speed", type=float, default=0.05)
    parser.add_argument("--reference-speed-limit", type=float,
                        help="指定参考速度上限；未指定时由往返速度除以 0.85 得到，单位 rad/s")
    parser.add_argument("--acceleration", type=float, default=0.4)
    parser.add_argument("--actual-speed-limit", type=float, default=0.30,
                        help="目标轴实际速度停止阈值，单位 rad/s")
    parser.add_argument("--stop-acceleration", type=float, default=2.0,
                        help="本轮 stopj 减速度，单位 rad/s²")
    parser.add_argument("--stop-at-speed", type=float,
                        help="到达指定实际速度后主动停止，用于现场观察制动距离，单位 rad/s")
    parser.add_argument("--hold", type=float, default=5.0)
    parser.add_argument("--k", type=float, required=True)
    parser.add_argument("--d", type=float, required=True)
    parser.add_argument("--torque", type=float, default=18.0)
    parser.add_argument("--slew", type=float, default=150.0)
    parser.add_argument("--log-directory", type=Path, required=True)
    parser.add_argument("--axis", type=int, choices=(1, 2, 3, 4, 5, 6), default=1)
    parser.add_argument("--workspace-radius", type=float, default=0.2)
    args = parser.parse_args()
    axis = args.axis - 1
    if not math.isfinite(args.speed) or args.speed <= 0:
        raise ValueError("联调速度必须是正有限数")
    if not 0.08 <= args.workspace_radius <= 0.3:
        raise ValueError("工作区半径须在现场确认的 0.08～0.3 rad 内")

    workspace = json.loads(args.workspace_state.read_text())
    if workspace["robot_address"] != args.robot_ip:
        raise ValueError("工作区状态与本轮机器人地址不一致")
    center = workspace["samples"][-1]["actual_q"]
    before = read_robot_state(args.robot_ip)
    require_stopped(before, 0.01)
    q = before["samples"][-1]["actual_q"]
    if any(abs(a - b) > 0.02 for j, (a, b) in enumerate(zip(q, center)) if j != axis):
        raise ValueError("非调参轴已离开原始姿态，需重新现场确认工作区")
    if args.actual_speed_limit <= 0 or args.stop_acceleration <= 0:
        raise ValueError("实际速度阈值和停止减速度必须为正")
    if args.stop_at_speed is not None and not 0 < args.stop_at_speed < args.actual_speed_limit:
        raise ValueError("主动停止速度必须在零与实际速度停止阈值之间")
    cfg = snapshot()
    # 按当前循环间隔门槛预留三周期反应距离，再扣除理论制动距离及锚点余量。
    braking = (args.actual_speed_limit * 3 * cfg["CYCLE_MAX_S"]
               + args.actual_speed_limit ** 2 / (2 * args.stop_acceleration))
    remaining = args.workspace_radius - braking - abs(q[axis] - center[axis]) - 0.005
    if remaining <= 0:
        raise ValueError("当前位置没有足够的目标轴工作区余量")
    cfg["JOINT_K"][axis], cfg["JOINT_D"][axis] = args.k, args.d
    cfg["TAU_MAX_NM"][axis], cfg["TAU_RATE_MAX_NM_S"][axis] = args.torque, args.slew
    cfg["ENVELOPE_RAD"] = [0.02] * 6
    cfg["ENVELOPE_RAD"][axis] = remaining
    cfg["ACTUAL_MAX_QD_RAD_S"] = [0.08] * 6
    cfg["ACTUAL_MAX_QD_RAD_S"][axis] = args.actual_speed_limit
    cfg["STOP_ACCEL_RAD_S2"] = args.stop_acceleration
    cfg["START_TOLERANCE_RAD"] = 0.003
    cfg["END_HOLD_S"] = 1.0
    cfg["REF_MAX_QD_RAD_S"][axis] = args.speed / 0.85
    if args.reference_speed_limit is not None:
        cfg["REF_MAX_QD_RAD_S"][axis] = args.reference_speed_limit
    cfg["REF_MAX_QDD_RAD_S2"][axis] = args.acceleration
    cfg = validate(cfg)
    if args.mode == "step":
        reference = JointStep(tuple(q), axis, center[axis] + args.target_offset - q[axis], hold_s=args.hold)
    elif args.mode == "move":
        reference = generate_move(q, axis, center[axis] + args.target_offset, args.speed, cfg)
    else:
        reference = generate_axis_tuning(q, axis, args.amplitude, args.speed, cfg)
    reference.bind(q, cfg, reference.sample_hz)
    if any(abs(row[axis] - center[axis]) > args.workspace_radius - braking - 0.01 for row in reference.rows):
        raise ValueError("本轮目标超出原始工作区扣除制动距离后的范围")
    if args.log_directory.exists():
        raise FileExistsError("本轮日志目录已存在，请使用新的轮次名称")
    args.log_directory.parent.mkdir(parents=True, exist_ok=True)
    plan_path = args.workspace_state.parent / (args.log_directory.name + "_plan.json")
    plan_path.write_text(json.dumps({
        "axis": args.axis, "mode": args.mode, "workspace_state": str(args.workspace_state.resolve()),
        "workspace_center": center, "workspace_radius_rad": args.workspace_radius, "preflight": before,
        "target": list(reference.rows[-1]), "parameters": cfg,
        "braking_reserve_rad": braking, "stop_at_speed_rad_s": args.stop_at_speed,
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    backend = ImpedanceProcess(
        args.robot_ip, reference_source="step" if args.mode == "step" else "trajectory", config=cfg, trajectory=reference,
        log_directory=args.log_directory,
    )
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda signum, frame: backend.request_stop("人工中止"))
    print(json.dumps({"开始单轮": args.mode, "目标轴": args.axis, "起点": q[axis],
                      "终点": reference.rows[-1][axis], "K": args.k, "D": args.d,
                      "实际速度守卫": cfg["ACTUAL_MAX_QD_RAD_S"][axis],
                      "本轮包络": remaining}, ensure_ascii=False), flush=True)
    backend.start()
    previous = None
    planned_stop = False
    deadline = time.monotonic() + reference.duration() + cfg["START_TIMEOUT_S"] + cfg["END_SETTLE_TIMEOUT_S"] + 3
    try:
        while backend.active:
            status = backend.get_status()
            if status["phase"] != previous:
                previous = status["phase"]
                print(json.dumps({"阶段": previous, "错误": status["last_error"]}, ensure_ascii=False), flush=True)
            if previous == "STOP_UNCONFIRMED":
                break
            sample = status.get("sample")
            if (args.stop_at_speed is not None and not planned_stop and sample is not None
                    and abs(sample["qd_actual"][axis]) >= args.stop_at_speed):
                planned_stop = True
                backend.request_stop()
                (args.log_directory / "manual_stop_trigger.json").write_text(
                    json.dumps(sample, ensure_ascii=False, indent=2), encoding="utf-8")
                print("已达到计划停止速度，正在确认停止。", flush=True)
            if time.monotonic() > deadline:
                backend.request_stop("单轮联调超过计划时限")
            time.sleep(0.02)
    finally:
        backend.stop()
    status = backend.get_status()
    print(json.dumps({k: status.get(k) for k in (
        "phase", "stopped_confirmed", "completed_trajectory", "last_error", "stop_error", "log_directory"
    )}, ensure_ascii=False, indent=2), flush=True)
    after = read_robot_state(args.robot_ip)
    require_stopped(after, cfg["STATIONARY_QD_RAD_S"])
    (args.log_directory / "manual_postcheck.json").write_text(
        json.dumps(after, ensure_ascii=False, indent=2), encoding="utf-8")
    finished_as_planned = planned_stop if args.stop_at_speed is not None else status.get("completed_trajectory")
    return int(bool(status["last_error"] or not finished_as_planned))


if __name__ == "__main__":
    raise SystemExit(main())
