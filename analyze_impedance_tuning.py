"""从实机完整日志统计单轴阶跃或往返效果，保存带单位的结果与曲线。"""

import argparse
import json
from pathlib import Path

import numpy as np

POSITION_TOLERANCE_RAD = float(np.deg2rad(0.2))
TRACKING_RMS_TOLERANCE_DEG = (0.2, 0.2, 0.2, 0.5, 0.5, 0.5)


def analyze(directory, axis=0, plot=True):
    directory = Path(directory)
    metadata = json.loads((directory / "metadata.json").read_text())
    summary = json.loads((directory / "summary.json").read_text())
    samples, standard, references, seen = [], [], {}, set()
    for line in (directory / "records.jsonl").open():
        row = json.loads(line)
        p = row["payload"]
        if row["kind"] == "reference":
            references[p["reference_seq"]] = p
        elif row["kind"] == "standard":
            standard.append(p)
        elif row["kind"] == "sample":
            identity = p["run_id"], p["cycle_seq"]
            if identity[0] == metadata["run_id"] and identity not in seen and p["phase"] in ("RUNNING", "ENDING"):
                seen.add(identity)
                samples.append(p)
    if not samples:
        raise ValueError("没有有效运行样本")
    t = np.array([s["sample_time_s"] for s in samples])
    t -= t[0]
    q = np.array([s["q_actual"][axis] for s in samples])
    r = np.array([s["q_ref"][axis] for s in samples])
    v = np.array([s["qd_actual"][axis] for s in samples])
    tau = np.array([s["tau_cmd"][axis] for s in samples])
    source = np.array([references[s["adopted_reference_seq"]]["q_source"][axis] for s in samples])
    positions = np.array([references[s["adopted_reference_seq"]]["row_position"] for s in samples])
    error = r - q
    tail = t >= t[-1] - 1.0
    anchor = np.array(metadata["anchor"])
    actual = np.array([s["actual_q"] for s in standard])
    speeds = np.array([s["actual_qd"] for s in standard])
    report = {
        "record_type": "single_axis_impedance_response", "axis": axis + 1,
        "source": metadata["source"], "run_id": metadata["run_id"],
        "parameters": metadata["parameters"], "sample_count": len(samples),
        "duration_s": float(t[-1]), "stopped_confirmed": summary["stopped_confirmed"],
        "completed_schedule": bool(summary.get("completed_trajectory")),
        "last_error": summary.get("last_error"), "stop_error": summary.get("stop_error"),
        "log_complete": summary["log_complete"], "log_dropped": summary["log_dropped"],
        "reference_rmse_rad": float(np.sqrt(np.mean(error ** 2))),
        "source_rmse_rad": float(np.sqrt(np.mean((source - q) ** 2))),
        "source_peak_error_rad": float(np.max(np.abs(source - q))),
        "reference_peak_error_rad": float(np.max(np.abs(error))),
        "tail_mean_error_rad": float(np.mean(error[tail])),
        "final_error_rad": float(error[-1]),
        "tail_peak_to_peak_rad": float(np.ptp(q[tail])),
        "tail_peak_speed_rad_s": float(np.max(np.abs(v[tail]))),
        "peak_speed_rad_s": np.max(np.abs(speeds), axis=0).tolist(),
        "actual_min_rad": np.min(actual, axis=0).tolist(),
        "actual_max_rad": np.max(actual, axis=0).tolist(),
        "max_excursion_from_anchor_rad": np.max(np.abs(actual - anchor), axis=0).tolist(),
        "peak_command_torque_nm": float(np.max(np.abs(tau))),
        "saturated_cycles": max(s["saturated_cycles"] for s in samples),
        "max_loop_interval_s": max(s["max_interval_s"] for s in samples),
        "snapshot_gaps": int(np.sum(np.maximum(0, np.diff([s["cycle_seq"] for s in samples]) // 5 - 1))),
        "standard_max_interval_s": float(np.max(np.diff([s["timestamp"] for s in standard]))),
        "position_deviation_ratios": np.max([s["joint_position_deviation_ratios"] for s in standard], axis=0).tolist(),
        "collision_detection_ratios": np.max([s["collision_detection_ratios"] for s in standard], axis=0).tolist(),
        "position_tolerance_deg": 0.2,
    }
    evidence_complete = bool(
        report["completed_schedule"] and report["stopped_confirmed"]
        and not report["last_error"] and not report["stop_error"]
        and report["log_complete"] and report["log_dropped"] == 0
        and report["snapshot_gaps"] == 0
    )
    if metadata["source"] == "trajectory":
        # 去掉起点和末点保持，避免用静止样本稀释运动阶段的跟踪误差。
        moving = (positions > 0) & (positions < np.max(positions))
        source_rms = float(np.sqrt(np.mean((source[moving] - q[moving]) ** 2)))
        reference_rms = float(np.sqrt(np.mean(error[moving] ** 2)))
        rms_tolerance = float(np.deg2rad(TRACKING_RMS_TOLERANCE_DEG[axis]))
        report["tracking"] = {
            "rms_tolerance_deg": TRACKING_RMS_TOLERANCE_DEG[axis],
            "motion_sample_count": int(np.sum(moving)),
            "motion_source_rmse_rad": source_rms,
            "motion_reference_rmse_rad": reference_rms,
            "motion_source_peak_error_rad": float(np.max(np.abs(source[moving] - q[moving]))),
            "motion_reference_peak_error_rad": float(np.max(np.abs(error[moving]))),
            "source_rms_within_tolerance": source_rms <= rms_tolerance if evidence_complete else None,
            "reference_rms_within_tolerance": reference_rms <= rms_tolerance if evidence_complete else None,
            "evaluation_basis": "以运动阶段的生成目标误差评价跟踪；生效参考误差单列。",
        }
    if metadata["source"] == "step":
        changes = np.flatnonzero(np.abs(np.diff(r)) > 1e-5) + 1
        if len(changes) != 1:
            raise ValueError("阶跃报告要求日志中恰好存在一次位置参考跳变")
        start = int(changes[0])
        pre = float(np.mean(q[max(0, start - 50):start]))
        amplitude = r[start] - pre
        progress = (q[start:] - pre) / amplitude
        relative = t[start:] - t[start]
        crossings = [np.flatnonzero(progress >= fraction) for fraction in (0.1, 0.9)]
        tolerance = POSITION_TOLERANCE_RAD
        outside = np.flatnonzero(np.abs(error[start:]) > tolerance)
        settled_index = int(outside[-1] + 1) if len(outside) else 0
        settling = None
        if settled_index < len(relative) and relative[-1] - relative[settled_index] >= 1.0:
            settling = float(relative[settled_index])
        report["step"] = {
            "start_time_s": float(t[start]), "command_amplitude_rad": float(r[start] - r[start - 1]),
            "rise_10_90_s": float(relative[crossings[1][0]] - relative[crossings[0][0]]) if all(len(c) for c in crossings) else None,
            "overshoot_percent": float(max(0, np.max(progress) - 1) * 100),
            "achieved_fraction_tail": float(np.mean(progress[-min(100, len(progress)):])),
            "settling_tolerance_rad": tolerance, "settling_time_s": settling,
            "timing_resolution_s": 0.01,
            "final_error_within_tolerance": bool(abs(error[-1]) <= tolerance) if evidence_complete else None,
            "settled_within_tolerance": settling is not None if evidence_complete else None,
        }
    path = directory / "response_report.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    if plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True, constrained_layout=True)
        axs[0].plot(t, source - anchor[axis], color="#8c98a4", ls=":", label="Source")
        axs[0].plot(t, r - anchor[axis], color="#2076b4", label="Reference")
        axs[0].plot(t, q - anchor[axis], color="#e27c30", label="Measured")
        axs[0].set_ylabel("Position offset (rad)")
        axs[0].legend(ncol=3)
        axs[1].plot(t, source - q, color="#8c98a4", ls=":", label="Source error")
        axs[1].plot(t, error, color="#2076b4", label="Reference error")
        band = rms_tolerance if metadata["source"] == "trajectory" else POSITION_TOLERANCE_RAD
        axs[1].axhline(band, color="#8c98a4", ls="--", lw=1)
        axs[1].axhline(-band, color="#8c98a4", ls="--", lw=1)
        axs[1].set_ylabel("Position error (rad)")
        axs[1].legend(ncol=2)
        axs[2].plot(t, tau, color="#e27c30")
        axs[2].set_ylabel("Command torque (Nm)")
        axs[2].set_xlabel("Time since RUNNING (s)")
        for ax in axs:
            ax.grid(alpha=0.2)
        cfg = metadata["parameters"]
        fig.suptitle(f'J{axis + 1} {metadata["source"]} | K={cfg["JOINT_K"][axis]:g}, D={cfg["JOINT_D"][axis]:g}\n'
                     f'Tail error={report["tail_mean_error_rad"]:.5f} rad | Peak speed={report["peak_speed_rad_s"][axis]:.3f} rad/s')
        fig.savefig(directory / "response.png", dpi=160)
        plt.close(fig)
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory")
    parser.add_argument("--axis", type=int, choices=range(1, 7), default=1)
    args = parser.parse_args()
    report = analyze(args.directory, args.axis - 1)
    print(json.dumps({k: v for k, v in report.items() if k != "parameters"}, ensure_ascii=False, indent=2))
