"""已安排现场监护的 J1～J6 单轮联调，沿用 GUI 后端并固定整次调参的工作区中心。"""

import argparse
import json
from pathlib import Path
import signal
import time

from admittance_manual import read_robot_state, require_stopped
from UR_Utils.impedance_config import snapshot, validate
from UR_Utils.impedance_process import ImpedanceProcess
from UR_Utils.impedance_step import JointStep
from UR_Utils.impedance_trajectory import generate_axis_tuning


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--workspace-state", type=Path, required=True,
                        help="本次现场确认的初始只读状态文件，后续各轮复用同一文件")
    parser.add_argument("--mode", choices=("step", "trajectory"), required=True)
    parser.add_argument("--target-offset", type=float, default=0.04,
                        help="阶跃终点相对固定工作区中心的角度，单位 rad")
    parser.add_argument("--amplitude", type=float, default=0.05)
    parser.add_argument("--speed", type=float, default=0.05)
    parser.add_argument("--acceleration", type=float, default=0.4)
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
    if not 0.08 <= args.workspace_radius <= 0.2:
        raise ValueError("工作区半径须在 0.08～0.2 rad 内")

    workspace = json.loads(args.workspace_state.read_text())
    if workspace["robot_address"] != args.robot_ip:
        raise ValueError("工作区状态与本轮机器人地址不一致")
    center = workspace["samples"][-1]["actual_q"]
    before = read_robot_state(args.robot_ip)
    require_stopped(before, 0.01)
    q = before["samples"][-1]["actual_q"]
    if any(abs(a - b) > 0.02 for j, (a, b) in enumerate(zip(q, center)) if j != axis):
        raise ValueError("非调参轴已离开原始姿态，需重新现场确认工作区")
    # 从现场确认的工作区扣除制动距离和锚点余量。
    remaining = args.workspace_radius - 0.03 - abs(q[axis] - center[axis]) - 0.005
    if remaining <= 0:
        raise ValueError("当前位置没有足够的目标轴工作区余量")
    cfg = snapshot()
    cfg["JOINT_K"][axis], cfg["JOINT_D"][axis] = args.k, args.d
    cfg["TAU_MAX_NM"][axis], cfg["TAU_RATE_MAX_NM_S"][axis] = args.torque, args.slew
    cfg["ENVELOPE_RAD"] = [0.02] * 6
    cfg["ENVELOPE_RAD"][axis] = remaining
    cfg["ACTUAL_MAX_QD_RAD_S"] = [0.08] * 6
    cfg["ACTUAL_MAX_QD_RAD_S"][axis] = 0.30
    cfg["MAX_ERROR_RAD"] = [0.02] * 6
    cfg["MAX_ERROR_RAD"][axis] = 0.10
    cfg["START_TOLERANCE_RAD"] = 0.003
    cfg["END_HOLD_S"] = 1.0
    cfg["REF_MAX_QD_RAD_S"][axis] = args.speed / 0.85
    cfg["REF_MAX_QDD_RAD_S2"][axis] = args.acceleration
    cfg = validate(cfg)
    if args.mode == "step":
        reference = JointStep(tuple(q), axis, center[axis] + args.target_offset - q[axis], hold_s=args.hold)
    else:
        reference = generate_axis_tuning(q, axis, args.amplitude, args.speed, cfg)
    reference.bind(q, cfg, reference.sample_hz)
    if any(abs(row[axis] - center[axis]) > args.workspace_radius - 0.05 for row in reference.rows):
        raise ValueError("本轮目标须位于原始工作区扣除 0.05 rad 余量的范围内")
    if args.log_directory.exists():
        raise FileExistsError("本轮日志目录已存在，请使用新的轮次名称")
    args.log_directory.parent.mkdir(parents=True, exist_ok=True)
    plan_path = args.workspace_state.parent / (args.log_directory.name + "_plan.json")
    plan_path.write_text(json.dumps({
        "axis": args.axis, "mode": args.mode, "workspace_state": str(args.workspace_state.resolve()),
        "workspace_center": center, "preflight": before,
        "target": list(reference.rows[-1]), "parameters": cfg,
    }, ensure_ascii=False, indent=2), encoding="utf-8")
    backend = ImpedanceProcess(
        args.robot_ip, reference_source=args.mode, config=cfg, trajectory=reference,
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
    deadline = time.monotonic() + reference.duration() + cfg["START_TIMEOUT_S"] + cfg["END_SETTLE_TIMEOUT_S"] + 3
    try:
        while backend.active:
            status = backend.get_status()
            if status["phase"] != previous:
                previous = status["phase"]
                print(json.dumps({"阶段": previous, "错误": status["last_error"]}, ensure_ascii=False), flush=True)
            if previous == "STOP_UNCONFIRMED":
                break
            if time.monotonic() > deadline:
                backend.request_stop("单轮联调超过计划时限")
            time.sleep(0.02)
    finally:
        backend.stop()
    status = backend.get_status()
    print(json.dumps({k: status.get(k) for k in (
        "phase", "stopped_confirmed", "completed_trajectory", "last_error", "stop_error", "log_directory"
    )}, ensure_ascii=False, indent=2), flush=True)
    return int(bool(status["last_error"] or not status.get("completed_trajectory")))


if __name__ == "__main__":
    raise SystemExit(main())
