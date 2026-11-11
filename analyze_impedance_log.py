"""按采纳身份对齐源目标，同周期 q_ref/q_actual 计算误差；不插造 URSim 运动。"""

import argparse
import csv
import json
from pathlib import Path
import numpy as np


def timing_statistics(rows):
    """按相邻新源的位置有无变化区分运动和保持，同源重发沿用最近分类。"""
    result = {}
    for state in ("position_changed", "position_held", "initial"):
        result[state] = {"new_intervals": [], "ages": [], "repeat_count": 0, "max_step_rad": 0.0,
                         "longest_interval": None}
    previous = None
    state = "initial"
    for seq, q, stamp, age in rows:
        if previous is not None:
            old_seq, old_q, old_stamp = previous
            if seq == old_seq:
                result[state]["repeat_count"] += 1
            else:
                step = max(abs(x-y) for x, y in zip(q, old_q))
                state = "position_changed" if step > 0 else "position_held"
                interval_ms = 1000*(stamp-old_stamp)
                result[state]["new_intervals"].append(interval_ms)
                longest = result[state]["longest_interval"]
                if longest is None or interval_ms > longest["interval_ms"]:
                    result[state]["longest_interval"] = dict(
                        previous_source_seq=old_seq, source_seq=seq, interval_ms=interval_ms)
                result[state]["max_step_rad"] = max(step, result[state]["max_step_rad"])
        if previous is None or seq != previous[0]:
            previous = (seq, q, stamp)
        if age is not None:
            result[state]["ages"].append(1000*age)
    for group in result.values():
        for key, label in (("new_intervals", "new_source_interval_ms"), ("ages", "source_age_ms")):
            values = group.pop(key)
            group[label] = (dict(count=len(values), p95=float(np.percentile(values, 95)),
                                 p99=float(np.percentile(values, 99)), maximum=max(values))
                            if values else None)
    return result


def udp_handoff_statistics(stages, published, kernel_read_delays, trace_summary):
    """在首末 RTDE 源帧之间对账，缺少发布记录的旧日志只报告两端差集。"""
    sent = stages["rtde_send"]
    if not sent or not stages["udp_receive"]:
        return None
    first, last = sent[0][0], sent[-1][0]
    received = {seq for seq, *_ in stages["udp_receive"] if first <= seq <= last}
    submitted = {seq for seq, *_ in stages["adapter_submit"] if first <= seq <= last}
    skipped = received - submitted
    result = dict(first_source_seq=first, last_source_seq=last,
                  received=len(received), adapter_submitted=len(submitted), received_not_submitted=len(skipped),
                  received_not_published=None, published_not_submitted=None, kernel_to_read_ms=None)
    if published and trace_summary and trace_summary["log_complete"]:
        result["received_not_published"] = len(skipped - published)
        result["published_not_submitted"] = len(skipped & published)
    delays = [delay for seq, delay in kernel_read_delays if first <= seq <= last]
    if delays:
        result["kernel_to_read_ms"] = dict(count=len(delays), median=float(np.median(delays)),
                                          p95=float(np.percentile(delays, 95)), maximum=max(delays))
    return result


def analyze_timing(directory, refs, timing):
    """各阶段分别用本时钟统计间隔，控制器采纳身份可关联主机发送记录。"""
    stages = {key: [] for key in ("udp_receive", "adapter_submit", "ipc_read", "rtde_send", "adoption")}
    published = set()
    kernel_read_delays = []
    trace = directory / "source_timing.jsonl"
    if trace.exists():
        with trace.open() as stream:
            for line in stream:
                row = json.loads(line)
                kind, p = row["kind"], row["payload"]
                if kind == "udp_publish":
                    published.add(p["source_seq"])
                if kind == "udp_receive" and p.get("kernel_rx_unix_ns") is not None:
                    delay_ms = (p["read_unix_ns"] - p["kernel_rx_unix_ns"]) / 1e6
                    kernel_read_delays.append((p["source_seq"], delay_ms))
                if kind in ("udp_receive", "adapter_submit"):
                    received = p["source_monotonic_s"]
                    stamp = received if kind == "udp_receive" else p["submit_done_monotonic_s"]
                    stages[kind].append((p["source_seq"], p["q_source"], stamp, stamp-received))
    for p in timing["ipc_read"]:
        stamp = p["ipc_read_monotonic_s"]
        stages["ipc_read"].append((p["source_seq"], p["q_source"], stamp, stamp-p["source_monotonic_s"]))
    unmatched = {}
    for kind in ("rtde_send", "adoption"):
        unmatched[kind] = 0
        for p in timing[kind]:
            ref = refs.get(p["reference_seq"])
            if ref is None:
                unmatched[kind] += 1
                continue
            if kind == "rtde_send":
                stamp = p["send_done_monotonic_s"]
                age = stamp-ref["source_monotonic_s"]
            else:
                stamp, age = p["robot_timestamp_s"], None
            stages[kind].append((ref["source_seq"], ref["q_source"], stamp, age))
    trace_summary = directory / "source_timing_summary.json"
    source_trace = json.loads(trace_summary.read_text()) if trace_summary.exists() else None
    return {"stages": {k: {"observations": len(rows), **timing_statistics(rows)} for k, rows in stages.items()},
            "unmatched_reference": unmatched,
            "source_trace": source_trace,
            "udp_handoff": udp_handoff_statistics(stages, published, kernel_read_delays, source_trace)}


def analyze(directory, output=None):
    directory = Path(directory)
    output = Path(output or directory)
    output.mkdir(parents=True, exist_ok=True)
    metadata = json.loads((directory / "metadata.json").read_text())
    version = metadata["schema_version"]
    if version not in (1, 2, 3, 4, 5):
        raise ValueError(f"未知阻抗日志版本 {version}")
    has_tcp_wrench = version >= 4
    has_measured_torque = version >= 5
    wrench_by_stamp = {}
    torque_by_stamp = {}
    final_peak = None
    refs = {}
    samples = []
    seen = set()
    timing = {"ipc_read": [], "rtde_send": [], "adoption": []}
    for line in (directory / "records.jsonl").open():
        row = json.loads(line)
        p = row["payload"]
        if row["kind"] == "reference":
            refs[p["reference_seq"]] = p
        elif has_tcp_wrench and row["kind"] == "standard":
            wrench_by_stamp[p["timestamp"]] = p["actual_TCP_force"]
            if has_measured_torque and p["compensated_joint_torque"] is not None:
                measured = p["compensated_joint_torque"]
                torque_by_stamp[measured["robot_timestamp_s"]] = measured
        elif row["kind"] in timing:
            timing[row["kind"]].append(p)
        elif row["kind"] == "sample":
            if version >= 2 and p["final"]:
                final_peak = p["peak_error_rad"]
                continue
            key = (p["run_id"], p["cycle_seq"])
            if key not in seen and p["phase"] in ("RUNNING", "ENDING"):
                samples.append(p)
                seen.add(key)
    if not samples:
        raise ValueError("没有 RUNNING/ENDING 冻结样本，不能绘制跟踪效果")
    t = np.array([s["sample_time_s"] for s in samples])
    if has_tcp_wrench:
        # 用首次读到快照的机器人反馈时刻对齐标准帧；冻结时刻另列保留。
        t = np.array([s["robot_timestamp_s"] for s in samples])
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
    peak = final_peak if version >= 2 else np.max([s["peak_error_rad"] for s in samples], axis=0).tolist()
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
    if version >= 3 and metadata["source"] == "teleop":
        sampled["teleop_timing"] = analyze_timing(directory, refs, timing)
    if has_tcp_wrench:
        sampled["tcp_wrench_matched_snapshots"] = sum(
            s["robot_timestamp_s"] in wrench_by_stamp for s in samples
        )
    summary["analysis"] = sampled
    summary["source_directory"] = str(directory.resolve())
    (output / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2)
    )
    (output / "metadata.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2)
    )
    speeds = {}
    if version >= 2:
        speeds["reference_rad_s"] = [s["qd_ref"] for s in samples]
        speeds["actual_rad_s"] = [s["qd_actual"] for s in samples]
        if metadata["source"] == "teleop":
            for name in ("qd_source_raw", "qd_source_used"):
                speeds[name] = [
                    refs[s["adopted_reference_seq"]][name]
                    if s["adopted_reference_seq"] in refs else [float("nan")] * 6
                    for s in samples
                ]
    wrench_columns = [
        "robot_timestamp_s", "sample_time_s", "tcp_fx_n", "tcp_fy_n", "tcp_fz_n",
        "tcp_mx_nm", "tcp_my_nm", "tcp_mz_nm",
    ] if has_tcp_wrench else []
    torque_columns = (
        ["torque_sample_seq"] + [f"compensated_tau_nm_j{j}" for j in range(1, 7)]
        if has_measured_torque else []
    )
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
            + wrench_columns + torque_columns
        )
        for k, s in enumerate(samples):
            wrench_row = []
            torque_row = []
            if has_tcp_wrench:
                stamp = s["robot_timestamp_s"]
                wrench_row = [stamp, s["sample_time_s"], *wrench_by_stamp.get(stamp, [None] * 6)]
            if has_measured_torque:
                measured = torque_by_stamp.get(s["robot_timestamp_s"])
                torque_row = ([measured["sample_seq"], *measured["values_nm"]]
                              if measured is not None else [None] * len(torque_columns))
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
                    *wrench_row,
                    *torque_row,
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
    time_label = ("Robot feedback time since first tracking frame (s)" if has_tcp_wrench
                  else "Time since RUNNING (s)")
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
            ax.set_xlabel(time_label)
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
    panels = 5 if has_tcp_wrench else 3
    fig, axs = plt.subplots(panels, 1, figsize=(12, 2.5 * panels + 0.5), sharex=True)
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
    axs[2].plot(t, tau[:, j], color="#df7c25", label="Command")
    axs[2].set_ylabel("Command torque (N m)")
    if has_tcp_wrench:
        # 保留接触瞬间的原始采样峰值；标准帧丢失时曲线断开。
        start = samples[0]["robot_timestamp_s"]
        stop = samples[-1]["robot_timestamp_s"]
        if has_measured_torque:
            # 保留两次关节快照之间的独立实测点，缺测处不连线。
            torque_times = [stamp for stamp in torque_by_stamp if start <= stamp <= stop]
            axs[2].plot(np.array(torque_times) - start,
                        [torque_by_stamp[stamp]["values_nm"][j] for stamp in torque_times],
                        ".", markersize=2, color="#2474b5", label="Measured (UR compensated)")
            axs[2].set_ylabel("Joint torque (N m)")
            axs[2].legend(ncol=2)
        stamps = np.array([stamp for stamp in wrench_by_stamp if start <= stamp <= stop])
        wrench = np.array([wrench_by_stamp[stamp] for stamp in stamps], dtype=float).reshape(-1, 6)
        gaps = np.flatnonzero(np.diff(stamps) > 0.003) + 1
        force_time = np.insert(stamps - start, gaps, np.nan)
        wrench = np.insert(wrench, gaps, np.nan, axis=0)
        for ax, values, prefix, unit in (
            (axs[3], wrench[:, :3], "F", "N"), (axs[4], wrench[:, 3:], "M", "N m")
        ):
            for axis, value in zip("xyz", values.T):
                ax.plot(force_time, value, label=prefix + axis, linewidth=1)
            ax.set_ylabel(f"TCP {prefix} ({unit})")
            ax.legend(ncol=3)
    axs[-1].set_xlabel(time_label)
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
