"""按采纳身份对齐源目标，同周期 q_ref/q_actual 计算误差；不插造 URSim 运动。"""

import argparse
import csv
import json
from pathlib import Path
import numpy as np


def analyze(directory, output=None):
    directory = Path(directory)
    output = Path(output or directory)
    output.mkdir(parents=True, exist_ok=True)
    metadata = json.loads((directory / "metadata.json").read_text())
    version = metadata["schema_version"]
    if version not in (1, 2):
        raise ValueError(f"未知阻抗日志版本 {version}")
    final_peak = None
    refs = {}
    samples = []
    seen = set()
    standard = []
    for line in (directory / "records.jsonl").open():
        row = json.loads(line)
        p = row["payload"]
        if row["kind"] == "reference":
            refs[p["reference_seq"]] = p
        elif row["kind"] == "standard":
            standard.append(p)
        elif row["kind"] == "sample":
            if version == 2 and p["final"]:
                final_peak = p["peak_error_rad"]
                continue
            key = (p["run_id"], p["cycle_seq"])
            if key not in seen and p["phase"] in ("RUNNING", "ENDING"):
                samples.append(p)
                seen.add(key)
    if not samples:
        raise ValueError("没有 RUNNING/ENDING 冻结样本，不能绘制跟踪效果")
    t = np.array([s["sample_time_s"] for s in samples])
    t -= t[0]
    q = np.array([s["q_actual"] for s in samples])
    r = np.array([s["q_ref"] for s in samples])
    tau = np.array([s["tau_cmd"] for s in samples])
    source = np.array(
        [
            refs.get(s["adopted_reference_seq"], {}).get("q_source", [float("nan")] * 6)
            for s in samples
        ]
    )
    error = r - q
    shape = source - r
    summary = json.loads((directory / "summary.json").read_text())
    peak = final_peak if version == 2 else np.max([s["peak_error_rad"] for s in samples], axis=0).tolist()
    sampled = {
        "sample_count": len(samples),
        "duration_s": float(t[-1]),
        "sampled_rmse_rad": np.sqrt(np.mean(error**2, axis=0)).tolist(),
        "sampled_max_error_rad": np.max(np.abs(error), axis=0).tolist(),
        "sampled_final_mean_error_rad": np.mean(
            error[t >= t[-1] - 0.5], axis=0
        ).tolist(),
        "robot_accumulated_peak_error_rad": peak,
        "sampled_source_shaping_rmse_rad": np.sqrt(
            np.nanmean(shape**2, axis=0)
        ).tolist(),
        "sampled_peak_tau_nm": np.max(np.abs(tau), axis=0).tolist(),
        "sampled_actual_excursion_rad": np.ptp(q, axis=0).tolist(),
        "robot_max_interval_s": max(s["max_interval_s"] for s in samples),
        "robot_max_compute_s_quantized": max(s["max_compute_s"] for s in samples),
        "robot_saturated_cycles": max(s["saturated_cycles"] for s in samples),
        "robot_shaped_cycles": max(s["shaped_cycles"] for s in samples),
        "aligned_source_samples": int(np.sum(np.isfinite(source[:, 0]))),
        "frozen_sample_gaps": int(
            np.sum(np.diff([s["cycle_seq"] for s in samples]) > 5)
        ),
        "note": "URScript time(0) has controller-step resolution here; compute=0 does not mean zero CPU work.",
    }
    summary["analysis"] = sampled
    summary["source_directory"] = str(directory.resolve())
    (output / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2)
    )
    (output / "metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2)
    )
    speeds = {}
    if version == 2:
        speeds["reference_rad_s"] = [s["qd_ref"] for s in samples]
        speeds["actual_rad_s"] = [s["qd_actual"] for s in samples]
        if metadata["source"] == "teleop":
            for name in ("qd_source_raw", "qd_source_used"):
                speeds[name] = [
                    refs[s["adopted_reference_seq"]][name]
                    if s["adopted_reference_seq"] in refs else [float("nan")] * 6
                    for s in samples
                ]
    with (output / "tracking.csv").open("w") as f:
        writer = csv.writer(f)
        writer.writerow(
            ["time_s", "cycle_seq", "adopted_reference_seq"]
            + [
                f"{kind}_j{j}"
                for kind in (
                    "source_rad",
                    "reference_rad",
                    "actual_rad",
                    "error_rad",
                    "tau_nm",
                    *speeds,
                )
                for j in range(1, 7)
            ]
        )
        for k, s in enumerate(samples):
            writer.writerow(
                [
                    t[k],
                    s["cycle_seq"],
                    s["adopted_reference_seq"],
                    *source[k],
                    *r[k],
                    *q[k],
                    *error[k],
                    *tau[k],
                    *(v for series in speeds.values() for v in series[k]),
                ]
            )
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update(
        {
            "font.size": 11,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "savefig.dpi": 160,
        }
    )
    anchor = np.array(metadata["anchor"])
    fig, axs = plt.subplots(3, 2, figsize=(13, 9), sharex=True)
    for j, ax in enumerate(axs.flat):
        ax.plot(
            t,
            np.rad2deg(source[:, j] - anchor[j]),
            color="#888888",
            ls=":",
            label="Source (identity aligned)",
        )
        ax.plot(
            t,
            np.rad2deg(r[:, j] - anchor[j]),
            color="#2474b5",
            label="Impedance reference",
        )
        ax.plot(
            t,
            np.rad2deg(q[:, j] - anchor[j]),
            color="#df7c25",
            ls="--",
            label="Measured",
        )
        ax.set_title(
            f'J{j+1} | RMSE {np.rad2deg(sampled["sampled_rmse_rad"][j]):.3f} deg'
        )
        ax.set_ylabel("Offset (deg)")
        ax.grid(alpha=0.2)
        if j >= 4:
            ax.set_xlabel("Time since RUNNING (s)")
    axs[0, 0].legend(fontsize=8)
    stationary = np.max(np.ptp(q, axis=0)) < 1e-8 and np.max(np.ptp(r, axis=0)) > 0.01
    title = (
        "URSim 5.26.3: torque commands do not move simulated joints"
        if stationary
        else "Six-axis joint impedance tracking"
    )
    fig.suptitle(title)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(output / "tracking_six_axes.png", bbox_inches="tight")
    plt.close(fig)
    j = int(np.argmax(np.ptp(r, axis=0)))
    fig, axs = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    axs[0].plot(
        t, np.rad2deg(source[:, j] - anchor[j]), ":", color="#888888", label="Source"
    )
    axs[0].plot(t, np.rad2deg(r[:, j] - anchor[j]), color="#2474b5", label="Reference")
    axs[0].plot(
        t, np.rad2deg(q[:, j] - anchor[j]), "--", color="#df7c25", label="Measured"
    )
    axs[0].set_ylabel("Offset (deg)")
    axs[0].legend(ncol=3)
    axs[1].plot(t, np.rad2deg(error[:, j]), color="#2474b5")
    axs[1].set_ylabel("Ref - actual (deg)")
    axs[2].plot(t, tau[:, j], color="#df7c25")
    axs[2].set_ylabel("Command torque (N m)")
    axs[2].set_xlabel("Time since RUNNING (s)")
    for ax in axs:
        ax.grid(alpha=0.2)
    fig.suptitle(
        f"J{j+1} | "
        + title
        + "\n"
        + f'Friction scales = 0 | Max loop interval {sampled["robot_max_interval_s"]*1000:.3f} ms'
    )
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(output / "tracking.png", bbox_inches="tight")
    fig.savefig(output / "tracking.pdf", bbox_inches="tight")
    plt.close(fig)
    return summary


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("directory")
    p.add_argument("--output")
    args = p.parse_args()
    print(json.dumps(analyze(args.directory, args.output)["analysis"], indent=2))
