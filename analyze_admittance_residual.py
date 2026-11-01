"""固定参考紧凑辨识日志的只读分析与整轮留出比较；不连接设备、不部署模型。"""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
from pathlib import Path

import numpy as np

from UR_Utils.admittance_trajectory import profile_parameters


def command_phase(config, frame):
    """按已采用基础帧标注预设命令阶段，不把计划阶段冒充实际运动确认。"""
    plan = profile_parameters(config)
    block = plan["hold_frames"] + plan["move_frames"]
    leg = min(4, max(0, frame) // block)
    direction = config["DIAGNOSTIC_MOTION_DIRECTION"]
    directions = (direction, -direction, -direction, direction)
    if leg == 4:
        return "hold", directions[-1], leg
    offset = frame - leg * block - plan["hold_frames"]
    if offset <= 0:
        return ("initial_hold", 0, leg) if leg == 0 else ("hold", directions[leg - 1], leg)
    if offset < plan["ramp_frames"]:
        return "acceleration", directions[leg], leg
    if offset < plan["ramp_frames"] + plan["cruise_frames"]:
        return "cruise", directions[leg], leg
    if offset < plan["trapezoid_frames"]:
        return "deceleration", directions[leg], leg
    return "settling", directions[leg], leg


def metrics(rows, values, threshold):
    """对实际记录值统计；积分及峰值持续时间只跨连续模型序号，不补齐缺口。"""
    if not rows:
        return {"count": 0}
    array = np.asarray(values, dtype=float)
    integral = absolute_integral = above_time = longest = streak = 0.
    for index in range(1, len(rows)):
        a, b = rows[index - 1], rows[index]
        if b["model_sample_seq"] != a["model_sample_seq"] + 1:
            streak = 0.
            continue
        dt = b["sample_start_s"] - a["sample_start_s"]
        integral += .5 * (array[index - 1] + array[index]) * dt
        absolute_integral += .5 * (abs(array[index - 1]) + abs(array[index])) * dt
        if abs(array[index - 1]) > threshold and abs(array[index]) > threshold:
            above_time += dt
            streak += dt
            longest = max(longest, streak)
        else:
            streak = 0.
    return {
        "count": len(rows), "mean_nm": float(np.mean(array)),
        "rms_nm": float(np.sqrt(np.mean(array * array))),
        "quantiles_nm": dict(zip(("p01", "p50", "p95", "p99"), np.quantile(array, (.01, .5, .95, .99)).tolist())),
        "absolute_max_nm": float(np.max(np.abs(array))),
        "signed_integral_nm_s": float(integral), "absolute_integral_nm_s": float(absolute_integral),
        "peak_threshold_nm": threshold, "observed_above_threshold_s": above_time,
        "longest_observed_peak_s": longest,
    }


def onset_events(rows, axis, motion_threshold):
    """报告采用命令起动和实际速度持续越过门槛的时刻；不估算人的施力时刻。"""
    events = []
    for leg in range(4):
        window = [row for row in rows if row["leg"] == leg
                  and row["command_phase"] in ("acceleration", "cruise", "deceleration", "settling")]
        commanded = next((row for row in window if abs(row["adopted_qd_cmd_rad_s"]) > 1e-7), None)
        if commanded is None:
            continue
        streak = []
        actual = confirmed = None
        for row in window:
            speed = row["qd_actual_rad_s"][axis] * row["direction"]
            if speed >= motion_threshold:
                if streak and row["model_sample_seq"] != streak[-1]["model_sample_seq"] + 1:
                    streak = []
                streak.append(row)
                if row["sample_start_s"] - streak[0]["sample_start_s"] >= .012:
                    actual, confirmed = streak[0], row
                    break
            else:
                streak = []
        events.append({
            "leg": leg, "direction": commanded["direction"],
            "command_onset_s": commanded["sample_start_s"],
            "actual_onset_s": None if actual is None else actual["sample_start_s"],
            "actual_confirmed_s": None if confirmed is None else confirmed["sample_start_s"],
            "command_to_actual_s": None if actual is None else actual["sample_start_s"] - commanded["sample_start_s"],
        })
    return events


def analyze_run(run_dir, *, motion_threshold=.0005):
    """按本轮身份及独立模型序号汇总第八版记录，失败轮也保留明确缺口与停止结果。"""
    run_dir = Path(run_dir)
    metadata = json.loads((run_dir / "metadata.json").read_text(encoding="utf-8"))
    context = metadata["context"]
    controller = context["controller"]
    config = controller["parameters"]
    if controller["telemetry_schema"]["telemetry_schema_version"] != 8:
        raise ValueError("专用分析只接纳第八版紧凑辨识；旧轮换载荷页不足以还原同步起动波形")
    axis = config["DIAGNOSTIC_AXIS"] - 1
    pages = {}
    duplicates = backwards = 0
    previous = -1
    final_statistics = timing = None
    temperatures = []
    with (run_dir / "records.jsonl").open(encoding="utf-8") as stream:
        for line in stream:
            record = json.loads(line)
            if record["run_id"] != metadata["run_id"] or record["log_schema_version"] != metadata["log_schema_version"]:
                raise ValueError("记录身份或日志版本与元数据不符")
            if record["kind"] != "rtde_state":
                continue
            payload = record["payload"]
            control = payload["control"]
            if not payload["derived_valid"]:
                continue
            if control["page_type"] == "statistics" and control["statistics"]["final"]:
                final_statistics = control["statistics"]
            elif control["page_type"] == "timing":
                timing = control["continuous"]["timing"]
            elif control["page_type"] == "identification":
                page = control["identification"]
                if page["run_id"] != controller["run_id"] or page["telemetry_schema_version"] != 8 or page["observation_axis"] != axis + 1:
                    raise ValueError("辨识页运行身份、版本或观测轴不符")
                if page["sample_phase"] != 2 or not page["bias_ready"]:
                    continue
                seq = page["model_sample_seq"]
                backwards += int(seq < previous)
                previous = seq
                if seq in pages:
                    duplicates += 1
                    continue
                row = dict(page)
                row["residual_nm"] = row["input_raw_nm"][axis] - row["bias_nm"]
                frame = row["frame"] - row["running_frame"]
                phase, direction, leg = command_phase(config, frame)
                row.update(command_phase=phase, direction=direction, leg=leg)
                pages[seq] = row
                temperatures.append(payload["raw"]["joint_temperatures"])
    rows = [pages[key] for key in sorted(pages)]
    issues = []
    if (not context["no_contact_confirmed"] or context["reference_source"] != "fixed"
            or config["DIAGNOSTIC_MODE"] != "preset_joint_motion" or config["JOINT_LOAD_SOURCE"] != "joint_torque"):
        issues.append("本轮未声明固定参考关节力矩的无接触预设采集条件")
    if not rows:
        issues.append("没有有效运行段辨识样本")
    if backwards:
        issues.append("模型序号发生倒退")
    if any(b["frame"] <= a["frame"] or b["adopted_seq"] <= a["adopted_seq"] for a, b in zip(rows, rows[1:])):
        issues.append("采样采用帧或片段身份未持续前进")
    intervals = np.diff([row["sample_start_s"] for row in rows])
    spans = [row["sample_span_s"] for row in rows]
    if any(not np.isfinite(value) or value < 0 for value in spans) or any(intervals <= 0):
        issues.append("采样时间无效或未前进")
    numeric_rows = [[*row["input_raw_nm"], *row["q_actual_rad"], *row["qd_actual_rad_s"],
                     row["bias_nm"], row["filtered_uncorrected_nm"], row["drive_uncorrected_nm"],
                     row["adopted_q_cmd_rad"], row["adopted_qd_cmd_rad_s"]] for row in rows]
    if rows and not np.all(np.isfinite(np.asarray(numeric_rows, dtype=float))):
        raise ValueError("辨识样本含无效数值，不能用于拟合")
    missing = None
    if final_statistics is None or not final_statistics["complete"] or final_statistics["validation_errors"]:
        issues.append("缺少有效完整终态统计")
    else:
        first, last = final_statistics["start_sample_seq"], final_statistics["end_sample_seq"]
        missing = last - first + 1 - sum(first <= seq <= last for seq in pages)
        if missing or any(seq < first or seq > last for seq in pages):
            issues.append("终态统计范围存在缺失或范围外辨识样本")
    status = metadata["finish_summary"]["status"] if metadata["finished"] else None
    if status is None or not status["stopped_confirmed"] or status["exit_reason"] != 11 or status["last_error"] or status["safety_mode"] != 1:
        issues.append("本轮未正常结束并确认静止")
    bridge = context["process_bridge"]
    if not metadata["finished"] or metadata["dropped_count"] or metadata["skipped_count"] or bridge["dropped_count"] or bridge["pending_gap"]:
        issues.append("日志未收尾或有丢弃/停记")
    if timing is None:
        issues.append("缺少停止后的累计预算页")
    elif (timing["compute_max_duration_s"] >= config["MODEL_DT_S"] - config["TIMING_TOLERANCE_S"]
          or timing["servoj_max_interval_s"] > config["SERVO_START_MAX_INTERVAL_S"] + config["TIMING_TOLERANCE_S"]):
        issues.append("模型或执行器超预算")
    if timing is not None and timing["gap_count"]:
        issues.append("执行器出现间隔告警，不能将本轮视为无新增时序问题的辨识依据")
    grouped = defaultdict(list)
    for row in rows:
        grouped[f"{row['command_phase']}:{row['direction']:+d}"].append(row)
    threshold = config["JOINT_DEADBAND_NM"][axis]
    tail = [row for row in rows if row["leg"] == 4 and row["sample_start_s"] >= rows[-1]["sample_start_s"] - 5.]
    report = {
        "schema_version": 1, "run_dir": str(run_dir.resolve()), "run_id": metadata["run_id"], "axis": axis + 1,
        "eligible_for_fitting": not issues, "issues": issues,
        "sampling": {
            "unique_model_samples": len(rows), "duplicate_rtde_observations": duplicates,
            "backwards_count": backwards, "missing_against_final_statistics": missing,
            "internal_missing": 0 if not rows else rows[-1]["model_sample_seq"] - rows[0]["model_sample_seq"] + 1 - len(rows),
            "effective_hz": (len(rows) - 1) / float(np.sum(intervals)) if len(rows) > 1 else None,
            "interval_median_s": float(np.median(intervals)) if len(intervals) else None,
            "interval_max_s": float(np.max(intervals)) if len(intervals) else None,
            "sample_span_max_s": max(spans) if spans else None,
            "sample_span_note": "起止为控制器时钟读数，微秒编码不等于微秒分辨率；相同读数不代表物理同时或零耗时。",
        },
        "raw_residual_by_command_phase": {key: metrics(group, [r["residual_nm"] for r in group], threshold)
                                           for key, group in grouped.items()},
        "onset_events": onset_events(rows, axis, motion_threshold),
        "onset_definition": {"actual_threshold_rad_s": motion_threshold, "continuous_dwell_s": .012,
                             "command_threshold_rad_s": 1e-7, "note": "离线门槛，仅作起动对齐；计划阶段不等于实际运动状态，没有外力真值或人的施力时刻"},
        "final_statistics": final_statistics, "timing": timing,
        "tail_last_5s": {
            "raw_residual": metrics(tail, [row["residual_nm"] for row in tail], threshold),
            "filtered_uncorrected": metrics(tail, [row["filtered_uncorrected_nm"] for row in tail], threshold),
            "actual_speed_max_rad_s": max(abs(row["qd_actual_rad_s"][axis]) for row in tail) if tail else None,
            "position_span_rad": float(np.ptp([row["q_actual_rad"][axis] for row in tail])) if tail else None,
        },
        "conditions": {"parameters": config,
                       "startup_bias_nm": rows[0]["bias_nm"] if rows else None,
                       "q_actual_min_rad": np.min([row["q_actual_rad"] for row in rows], axis=0).tolist() if rows else None,
                       "q_actual_max_rad": np.max([row["q_actual_rad"] for row in rows], axis=0).tolist() if rows else None},
        "stop": None if status is None else {key: status[key] for key in ("stopped_confirmed", "exit_reason", "last_error")},
        "temperature_range_c": None if not temperatures else {
            "min": np.min(temperatures, axis=0).tolist(), "max": np.max(temperatures, axis=0).tolist()},
        "scope": "同模型周期的有限跨度采样；相邻RTDE标准字段不是同次模型采样。缺口不插值，原始残差已减本轮启动偏置。模型off，预测零占位不代表补偿效果。",
    }
    return report, rows


def compare_runs(train_dirs, holdout_dirs):
    """比较方向常值与增加起停/保持分段均值的离线基线；整轮留出，不导出可部署控制参数。"""
    train_paths = [Path(path).resolve() for path in train_dirs]
    holdout_paths = [Path(path).resolve() for path in holdout_dirs]
    if len(set(train_paths)) < 2 or not holdout_paths or len(set(train_paths + holdout_paths)) != len(train_paths + holdout_paths):
        raise ValueError("至少两轮独立训练与一轮整轮留出，目录不得重复或交叉")
    runs = [(path, *analyze_run(path)) for path in train_paths + holdout_paths]
    if len({report["run_id"] for _, report, _ in runs}) != len(runs):
        raise ValueError("训练与留出记录包含相同运行身份")
    if any(not report["eligible_for_fitting"] for _, report, _ in runs):
        raise ValueError("存在采样或停止质量不合格的轮次；先查看各轮报告，不拟合缺失波形")
    if len({report["axis"] for _, report, _ in runs}) != 1:
        raise ValueError("逐轴辨识不能混合不同观测轴")
    moving = defaultdict(list)
    memory = defaultdict(list)
    for _, _, rows in runs[:len(train_paths)]:
        for row in rows:
            phase, direction = row["command_phase"], row["direction"]
            if phase == "initial_hold":
                continue
            # 拟合 UR 信号的绝对基线；预测变化再减本轮偏置，不能重复叠加历史偏置。
            raw = row["input_raw_nm"][row["observation_axis"] - 1]
            memory[f"{phase}:{direction:+d}"].append(raw)
            if phase in ("acceleration", "cruise", "deceleration"):
                moving[str(direction)].append(raw)
    moving_coefficients = {key: float(np.mean(values)) for key, values in moving.items()}
    memory_coefficients = {key: float(np.mean(values)) for key, values in memory.items()}
    held_out = []
    for path, report, rows in runs[len(train_paths):]:
        groups = defaultdict(list)
        for row in rows:
            groups[f"{row['command_phase']}:{row['direction']:+d}"].append(row)
        comparisons = {}
        for key, group in groups.items():
            phase, direction = group[0]["command_phase"], group[0]["direction"]
            if phase != "initial_hold" and key not in memory_coefficients:
                raise ValueError(f"留出阶段 {key} 不在训练覆盖范围内")
            bias = group[0]["bias_nm"]
            moving_prediction = moving_coefficients[str(direction)] - bias if phase in ("acceleration", "cruise", "deceleration") else 0.
            memory_prediction = memory_coefficients[key] - bias if phase != "initial_hold" else 0.
            comparisons[key] = {
                label: metrics(group, [row["residual_nm"] - prediction for row in group], 1.)
                for label, prediction in (("uncorrected", 0.), ("moving_only", moving_prediction), ("phase_and_hold_memory", memory_prediction))
            }
        held_out.append({"run_dir": str(path), "run_id": report["run_id"], "by_phase": comparisons,
                         "conditions": report["conditions"], "temperature_range_c": report["temperature_range_c"]})
    return {
        "model_version": "offline_absolute_phase_means_v1", "deployment_allowed": False,
        "axis": runs[0][1]["axis"], "training_runs": [str(p) for p in train_paths],
        "training_run_ids": [report["run_id"] for _, report, _ in runs[:len(train_paths)]],
        "training_conditions": [{"conditions": report["conditions"], "temperature_range_c": report["temperature_range_c"]}
                                for _, report, _ in runs[:len(train_paths)]],
        "moving_only_absolute_ur_torque_nm": moving_coefficients,
        "phase_and_hold_memory_absolute_ur_torque_nm": memory_coefficients,
        "baseline_definition": "分段拟合原始 UR 补偿力矩的绝对均值；预测残差=该分段均值−本轮启动偏置。初始保持预测变化为零。修正值=(UR力矩−本轮偏置)−预测残差，不叠加历史偏置。",
        "holdout": held_out,
        "scope": "离线分段均值基线，仅用于判断停车记忆是否必要。使用已采用预设轨迹阶段，不能用于未知人推状态；不拟合窄尖峰，不外推姿态/速度/温度，不自动选择死区或部署。初始保持预测为零，不叠加历史偏置。",
    }


def main():
    """输出采样质量报告，或显式指定训练轮与留出轮进行离线基线比较。"""
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    inspect = subparsers.add_parser("inspect", help="分析单轮采样、阶段、起动与停止证据")
    inspect.add_argument("run_dir", type=Path)
    inspect.add_argument("--output", type=Path, required=True)
    compare = subparsers.add_parser("compare", help="至少两轮训练、一轮整轮留出；输出只供离线比较")
    compare.add_argument("--train", type=Path, nargs="+", required=True)
    compare.add_argument("--holdout", type=Path, nargs="+", required=True)
    compare.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = analyze_run(args.run_dir)[0] if args.command == "inspect" else compare_runs(args.train, args.holdout)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps({"报告": str(args.output.resolve())}, ensure_ascii=False))


if __name__ == "__main__":
    main()
