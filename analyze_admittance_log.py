"""只读汇总导纳日志，供人工排查；不连接设备、不修改日志、不作通过或失败判定。"""

from __future__ import annotations

import argparse
from collections import Counter, deque
from datetime import datetime, timezone
import json
import math
from pathlib import Path


def numeric_value(value):
    """识别日志声明的空值及非有限数值标记，不将缺测替换为零。"""
    if value is None:
        return None
    if isinstance(value, dict) and set(value) == {"non_finite_float"}:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"统计字段必须为数字、空值或已声明的非有限标记，实际为 {value!r}")
    return value if math.isfinite(value) else None


class NumericSummary:
    """逐条累计范围及最大值的位置，只保存常量大小的统计结果。"""

    def __init__(self):
        self.count = 0
        self.missing = 0
        self.minimum = None
        self.maximum = None
        self.maximum_location = None

    def add(self, value, location=None):
        value = numeric_value(value)
        if value is None:
            self.missing += 1
            return
        self.count += 1
        if self.minimum is None or value < self.minimum:
            self.minimum = value
        if self.maximum is None or value > self.maximum:
            self.maximum = value
            self.maximum_location = location

    def report(self, *, maximum_only=False):
        if not self.count:
            return {"状态": "未取得有限数值", "缺测或非有限数量": self.missing}
        result = {"有限数值数量": self.count, "缺测或非有限数量": self.missing,
                  "最大值": self.maximum}
        if not maximum_only:
            result["最小值"] = self.minimum
        if self.maximum_location is not None:
            result["最大值位置"] = self.maximum_location
        return result


class SequenceSummary:
    """仅保留整数标识作精确去重，不保存对应状态记录。"""

    def __init__(self):
        self.seen = set()
        self.repeats = 0
        self.previous = None
        self.backwards = 0

    def add(self, value):
        repeated = value in self.seen
        if repeated:
            self.repeats += 1
        self.seen.add(value)
        if self.previous is not None and value < self.previous:
            self.backwards += 1
        self.previous = value
        return not repeated

    def report(self):
        if not self.seen:
            return {"状态": "未取得", "唯一数量": 0, "重复次数": 0}
        first, last = min(self.seen), max(self.seen)
        return {"唯一数量": len(self.seen), "重复次数": self.repeats,
                "最小标识": first, "最大标识": last,
                "已观测首尾之间未出现的标识数量": last - first + 1 - len(self.seen),
                "相邻记录标识倒退次数": self.backwards}


def wall_time(value):
    """将已记录的 Unix 纳秒转换为明确标注 UTC 的时间。"""
    return datetime.fromtimestamp(value / 1_000_000_000, timezone.utc).isoformat()


def record_location(record, robot_timestamp=None):
    """保留日志行的稳定序号和机器人时间，不将主机时间冒充采样时间。"""
    location = {"记录序号": record["seq"], "机器人timestamp_s": robot_timestamp}
    if "process_bridge" in record["payload"]:
        location["通信进程日志桥"] = record["payload"]["process_bridge"]
    return location


def stop_summary(record, status):
    """提取停止及生命周期事件所需的小摘要，避免保存完整参数和状态。"""
    result = {"记录序号": record["seq"], "类型": record["kind"],
              "记录墙钟_UTC": wall_time(record["wall_time_ns"]),
              "阶段": status["phase"], "退出码": status["exit_reason"],
              "退出原因": status["exit_reason_text"], "错误信息": status["last_error"]}
    if "host_monotonic_s" in record["payload"]:
        result["来源主机单调时间_s"] = record["payload"]["host_monotonic_s"]
    if "process_bridge" in record["payload"]:
        result["通信进程日志桥"] = record["payload"]["process_bridge"]
    return result


class HostBudgetSummary:
    """按真实日志字段汇总主机预算；旧记录没有的 CPU 和分段数据保持未取得。"""

    def __init__(self):
        self.heartbeat_intervals = NumericSummary()
        self.previous_heartbeat = None
        self.worst_heartbeat = None
        self.send_failures = []
        self.send_contracts = []
        self.send_entry_remaining = NumericSummary()
        self.host_timing = {}
        self.process_bridge_gaps = []
        self.progress_contracts = []
        self.progress_sources = Counter()
        self.socket_drained = Counter()
        self.last_socket_drained = None
        self.progress = {name: NumericSummary() for name in (
            "state_progress_lag_s", "state_progress_max_lag_s", "state_timestamp_interval_s",
            "state_host_interval_s", "rtde_buffered_bytes", "rtde_buffered_max_bytes",
            "state_progress_rate", "state_progress_window_host_s", "state_progress_window_robot_s",
            "rtde_last_drained_monotonic_s", "rtde_drain_count",
            "rtde_backlog_duration_s", "rtde_backlog_max_duration_s",
            "wake_lateness_s",
        )}

    def add_progress(self, values, record, robot_timestamp=None):
        """汇总各事件的状态快照；排空常发生在无新状态包时，须同时读取心跳及生命周期。"""
        location = record_location(record, robot_timestamp)
        observed = False
        for name, summary in self.progress.items():
            if name in values:
                summary.add(values[name], location)
                observed = True
        if "rtde_socket_drained" in values:
            drained = values["rtde_socket_drained"]
            self.socket_drained[drained] += 1
            self.last_socket_drained = {"值": drained, "位置": location}
            observed = True
        if observed:
            self.progress_sources[record["kind"]] += 1

    def add(self, record):
        kind, payload = record["kind"], record["payload"]
        if kind == "rtde_input":
            self.send_entry_remaining.add(
                payload["send_entry_remaining_s"] if "send_entry_remaining_s" in payload else None,
                record_location(record),
            )
            if "host_timing" in payload["timing"]:
                self.host_timing.update(payload["timing"]["host_timing"])
            self.add_progress(payload["timing"], record)
            if not payload["heartbeat_advanced"]:
                return
            if self.previous_heartbeat is not None:
                previous = self.previous_heartbeat
                interval = payload["host_monotonic_s"] - previous["payload"]["host_monotonic_s"]
                old_maximum = self.heartbeat_intervals.maximum
                self.heartbeat_intervals.add(interval, record_location(record))
                if old_maximum is None or interval > old_maximum:
                    self.worst_heartbeat = {
                        "前次记录序号": previous["seq"], "本次记录序号": record["seq"],
                        "前次成功发送_monotonic_s": previous["payload"]["host_monotonic_s"],
                        "本次成功发送_monotonic_s": payload["host_monotonic_s"],
                        "已记录成功发送间隔_s": interval,
                        "本次发送证据": {key: payload[key] for key in (
                            "send_started_monotonic_s", "send_duration_s",
                            "send_entry_remaining_s",
                            "heartbeat_deadline_monotonic_s", "send_deadline_monotonic_s",
                            "transport_timing", "timing",
                        ) if key in payload},
                    }
            self.previous_heartbeat = record
        elif kind == "rtde_input_send_failed":
            self.send_failures.append({
                "记录序号": record["seq"],
                **({"来源主机单调时间_s": payload["host_monotonic_s"]}
                   if "host_monotonic_s" in payload else {}),
                **({"来源主机Unix时间_s": payload["host_time_unix_s"]}
                   if "host_time_unix_s" in payload else {}),
                "完整原始发送证据": payload,
            })
        elif kind == "rtde_state":
            control = payload["control"]
            if "host_timing" in control:
                self.host_timing.update(control["host_timing"])
            self.add_progress(control, record, payload["raw"]["timestamp"])
        elif "status" in payload:
            status = payload["status"]
            if "host_timing" in status:
                self.host_timing.update(status["host_timing"])
            self.add_progress(status, record)
        elif kind == "connection_ready":
            self.send_contracts.append({
                "记录序号": record["seq"],
                "原始发送约定": {key: payload[key] for key in (
                    "runtime_send_timing_version", "runtime_send_contract", "runtime_send_timeout_s",
                ) if key in payload},
            })
            if "state_progress_contract" in payload:
                self.progress_contracts.append({
                    "记录序号": record["seq"], "原始约定": payload["state_progress_contract"],
                })
        elif kind == "process_bridge_gap":
            self.process_bridge_gaps.append({"记录序号": record["seq"], **payload})

    def report(self, metadata):
        return {
            "已记录心跳发送间隔_s": self.heartbeat_intervals.report(),
            "最坏已记录心跳间隔的原始证据": self.worst_heartbeat,
            "单包发送失败": {
                "已记录失败次数": len(self.send_failures),
                "失败事件及原始证据": self.send_failures,
            },
            "成功发送包进入发送段的剩余期限_s": self.send_entry_remaining.report(),
            "发送计时与期限约定": {
                "连接记录原始约定": self.send_contracts,
                "说明": "仅保留该轮实际记录的版本、约定和期限；旧日志未记录的字段不套用当前实现。",
            },
            "主机分段墙钟与线程CPU": {
                "状态": "已取得" if self.host_timing else "未取得分段线程 CPU 观测",
                "各段最后累计观测": self.host_timing,
            },
            "状态接收推进与唤醒": {
                **{name: item.report() for name, item in self.progress.items()},
                "rtde_socket_drained": ({
                    "已排空快照数量": self.socket_drained[True],
                    "未排空快照数量": self.socket_drained[False],
                    "最后观测": self.last_socket_drained,
                } if self.socket_drained else {"状态": "该轮未记录 socket 排空观测"}),
            },
            "状态推进语义": {
                "连接记录原始约定": self.progress_contracts,
                "快照来源计数": dict(self.progress_sources),
                "说明": (
                    "连接原始约定按该轮日志保留；未声明约定的旧日志不能套用新版排空窗口含义。"
                    "旧版 state_progress_lag_s 是累计主机与机器人时钟推进差，不能当作真实积压或单向年龄。"
                    "新版推进差仅在未排空窗口累计，socket 不可读且用户缓冲为空时重置；"
                    "rtde_backlog_duration_s 才是连续未排空窗口的主机时长，仍不等同于某一状态包的网络延迟。"
                    "state_progress_rate 是窗口内机器人时间推进量除以主机时间推进量；"
                    "state_progress_window_host_s 与 state_progress_window_robot_s 保留实际窗口长度。"
                    "统计包含原始状态、心跳及生命周期快照，同一累计值可能重复出现；"
                    "排空快照数量不是排空调用次数，实际累计次数见 rtde_drain_count。"
                ),
            },
            "日志线程预算": (metadata["logger_timing"] if "logger_timing" in metadata
                                else {"状态": "旧日志未记录日志线程预算"}),
            "通信进程日志桥": {
                "缺口事件": self.process_bridge_gaps,
                "最终累计摘要": (metadata["context"]["process_bridge"]
                                 if "process_bridge" in metadata["context"]
                                 else {"状态": "该轮未取得通信进程桥结束摘要"}),
            },
            "解释": (
                "wall 为单调墙钟耗时，cpu 为执行该操作的单个线程 CPU；两者差值包含等待和被抢占，"
                "不能单独证明 GIL 或网络原因。各段可能嵌套，不能直接相加；max_wall_cpu_s 对应最大墙钟同次样本。"
                "主机与机器人时钟推进差不等于真实积压；具体窗口及排空含义见状态推进语义和连接记录原文。"
                "心跳间隔来自已记录成功发送，日志关闭或缺口可跨过未记录发送；须同时检查开关和丢弃。"
                "成功发送间隔不包含失败包，最大间隔正常不能证明没有发送失败；"
                "单包发送失败须独立查看原始证据，未记录失败也不等于完整记录了所有发送。"
                "send_entry_remaining_s 为进入发送段时距本包期限的余量，旧日志缺失保持缺测。"
                "日志队列年龄是提交至出队时间；元数据中的 metadata_write 不含最后一次写自身。"
            ),
        }


def analyze(run_dir, *, motion_windows=None, tail_seconds=3.0):
    """按显式版本流式读取；旧布局与连续伺服布局分别解释。"""
    run_dir = Path(run_dir).resolve()
    metadata_path = run_dir / "metadata.json"
    records_path = run_dir / "records.jsonl"
    with metadata_path.open(encoding="utf-8") as stream:
        metadata = json.load(stream)
    version = metadata["log_schema_version"]
    if version == 3:
        return analyze_continuous(run_dir, metadata, motion_windows=motion_windows,
                                  tail_seconds=tail_seconds)
    if version not in (1, 2):
        raise ValueError(f"{metadata_path} 的日志版本为 {version}；本工具只支持已声明的版本 1、2 和 3")
    context = metadata["context"]
    kinds = Counter()
    phases = Counter()
    valid_count = 0
    running_count = 0
    first_running = None
    last_running = None
    previous_running = None
    timestamp_intervals = NumericSummary()
    nonpositive_intervals = 0
    missing_running_timestamps = 0
    anchor = None
    anchor_change_count = 0
    drive_max = [NumericSummary() for _ in range(6)]
    command_offset_max = [NumericSummary() for _ in range(6)]
    actual_offset_max = [NumericSummary() for _ in range(6)]
    snapshot_sequences = SequenceSummary()
    sample_cycles = SequenceSummary()
    running_sample_cycles = SequenceSummary()
    diagnostic_axes = set()
    last_diagnostic = None
    diagnostic_ranges = {name: NumericSummary() for name in
                         ("tau_raw_nm", "bias_nm", "filtered_nm")}
    timing_fields = ("dt_model_actual_s", "servoj_start_interval_s", "servoj_duration_s",
                     "cycle_duration_s", "load_duration_s", "compute_duration_s", "publish_duration_s")
    diagnostic_timing = {name: NumericSummary() for name in timing_fields}
    operator_events = []
    logging_events = []
    stop_events = []
    gap_events = []
    gap_dropped_count = 0
    last_lifecycle = None
    host_budget = HostBudgetSummary()

    with records_path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(f"{records_path}:{line_number} 不是完整有效的 JSON 记录：{error.msg}") from error
            if record["log_schema_version"] != version or record["run_id"] != metadata["run_id"]:
                raise ValueError(f"{records_path}:{line_number} 的版本或 run_id 与 metadata.json 不一致")
            kind = record["kind"]
            host_budget.add(record)
            kinds[kind] += 1
            payload = record["payload"]

            if kind == "operator_event":
                operator_events.append({
                    "记录序号": record["seq"], "事件": payload["label"], "备注": payload["note"],
                    "事件墙钟_UTC": wall_time(payload["event_wall_time_ns"]),
                    "事件距日志开始_s": (payload["event_monotonic_ns"] - metadata["started_monotonic_ns"]) / 1_000_000_000,
                    "最近机器人状态": payload["latest_robot_state"], "关联说明": payload["association"],
                })
            elif kind == "logging_enabled":
                logging_events.append({
                    "记录序号": record["seq"], "开启": payload["enabled"],
                    "事件墙钟_UTC": wall_time(payload["wall_time_ns"]),
                    "累计停记跳过数量": payload["skipped_count"],
                })
            elif kind == "log_gap":
                gap_dropped_count += payload["dropped_count"]
                gap_events.append({"记录序号": record["seq"], "原因": payload["reason"],
                                   "丢弃数量": payload["dropped_count"],
                                   "首个丢弃序号": payload["first_seq"], "末个丢弃序号": payload["last_seq"],
                                   "记录类型": payload["kinds"]})
            elif kind in ("controller_phase", "controller_complete", "stop_requested",
                          "external_stop_confirmed", "controller_cancelled_before_start", "controller_error"):
                status = payload["status"]
                if kind != "controller_phase" or status["phase"] in (3, 4) or status["exit_reason"] != 0:
                    stop_events.append(stop_summary(record, status))
            elif kind == "controller_worker_failed":
                stop_events.append({
                    **stop_summary(record, payload["status"]),
                    "等待外部停止确认": payload["awaiting_external_stop_confirmation"],
                })

            if kind != "rtde_state":
                continue
            control = payload["control"]
            lifecycle_valid = payload["derived_valid"] if version == 1 else payload["lifecycle_valid"]
            if lifecycle_valid:
                last_lifecycle = stop_summary(record, control)
            if not payload["derived_valid"]:
                continue
            valid_count += 1
            phase = control["phase"]
            phases[phase] += 1
            current_anchor = control["q_start"]
            if current_anchor is not None:
                if anchor is None:
                    anchor = current_anchor
                elif current_anchor != anchor:
                    anchor_change_count += 1

            diagnostic = None if version == 1 else control["diagnostic"]
            if diagnostic is not None:
                diagnostic_axes.add(diagnostic["axis"])
                snapshot_sequences.add(diagnostic["snapshot_seq"])
                sample_cycles.add(diagnostic["sample_cycle"])
                # 停止入口才结算的异常周期也须保留，不能只查看 phase2 内的极值。
                last_diagnostic = {"记录序号": record["seq"], "phase": phase, **{
                    name: diagnostic[name] for name in (
                        "sample_cycle", "sample_time_s", "timing_cycle", "snapshot_seq",
                        "dt_base_s", "cycle_max_s", "cycle_overrun_count", "bias_sample_count",
                        "bias_std_nm", "cycle_duration_s",
                    )
                }}
            if phase != 2:
                continue
            running_count += 1
            robot = payload["raw"]
            timestamp = numeric_value(robot["timestamp"])
            location = record_location(record, timestamp)
            if timestamp is not None:
                current_running = (timestamp, control["cycle_count"])
                if first_running is None:
                    first_running = current_running
                last_running = current_running
                if previous_running is not None:
                    interval = timestamp - previous_running
                    timestamp_intervals.add(interval, location)
                    nonpositive_intervals += int(interval <= 0)
                previous_running = timestamp
            else:
                missing_running_timestamps += 1
                previous_running = None

            for axis in range(6):
                drive = numeric_value(control["drive"][axis])
                drive_max[axis].add(None if drive is None else abs(drive), location)
                command = numeric_value(control["q_cmd"][axis])
                actual = numeric_value(robot["actual_q"][axis])
                reference = None if anchor is None else numeric_value(anchor[axis])
                command_offset_max[axis].add(
                    None if command is None or reference is None else abs(command - reference), location)
                actual_offset_max[axis].add(
                    None if actual is None or reference is None else abs(actual - reference), location)

            if diagnostic is None or not running_sample_cycles.add(diagnostic["sample_cycle"]):
                continue
            diagnostic_location = {
                **location, "snapshot_seq": diagnostic["snapshot_seq"],
                "sample_cycle": diagnostic["sample_cycle"], "sample_time_s": diagnostic["sample_time_s"],
                "timing_cycle": diagnostic["timing_cycle"],
            }
            for name, summary in diagnostic_ranges.items():
                summary.add(diagnostic[name], diagnostic_location)
            for name, summary in diagnostic_timing.items():
                summary.add(diagnostic[name], diagnostic_location)

    span = None if first_running is None else last_running[0] - first_running[0]
    slope = None if span is None or span <= 0 else (last_running[1] - first_running[1]) / span
    diagnostic_report = {
        "状态": "未取得；版本 1 未声明内部诊断量" if version == 1 else "未取得有效诊断页",
        "说明": "不从电流、末端力或 drive 推造原始补偿力矩、偏置或内部周期。",
    }
    if snapshot_sequences.seen:
        diagnostic_report = {
            "状态": "已取得有效诊断页", "所选轴": sorted(diagnostic_axes),
            "全部有效诊断页snapshot_seq": snapshot_sequences.report(),
            "全部有效诊断页sample_cycle": sample_cycles.report(),
            "phase2诊断sample_cycle": running_sample_cycles.report(),
            "最后有效诊断快照_包含停止阶段": last_diagnostic,
            "最后快照说明": "覆盖全部有效阶段，包括 phase3 和 phase4；超周期可能到停止入口才结算，因此最后快照的 cycle_duration_s、cycle_max_s 和 cycle_overrun_count 应与 phase2 极值一起查看。",
            "序号解释": "只统计已观测最小与最大标识之间的缺口，不推断首帧前或末帧后丢失。缺口可能来自跨页写入导致无效页、控制器跳过输出、主机读取采样或日志缺口，不能直接归因为网络丢包。重复页不代表新的载荷样本。",
            "phase2唯一采样周期的载荷范围_Nm": {name: summary.report() for name, summary in diagnostic_ranges.items()},
            "phase2唯一采样页观测到的耗时最大值_s": {
                name: summary.report(maximum_only=True) for name, summary in diagnostic_timing.items()},
            "周期身份说明": "dt_model_actual_s、load_duration_s、compute_duration_s 属于 sample_cycle；servoj_start_interval_s、servoj_duration_s、cycle_duration_s、publish_duration_s 属于 timing_cycle。phase2 首页的 timing_cycle 可能仍指偏置末轮；最大值位置保留两种编号，不能视为同周期。",
        }
    return {
        "日志目录": str(run_dir), "日志版本": version, "run_id": metadata["run_id"],
        "场景": {"reference_source": context["reference_source"], "space": context["space"]},
        "记录数量": {"实际读取": sum(kinds.values()), "metadata记录数量": metadata["record_count"],
                     "按kind": dict(sorted(kinds.items()))},
        "RTDE": {"总数": kinds["rtde_state"], "derived_valid数量": valid_count,
                 "派生无效数量": kinds["rtde_state"] - valid_count,
                 "有效阶段数量": {str(key): value for key, value in sorted(phases.items())}},
        "phase2运行": {
            "有效RTDE数量": running_count,
            "首个机器人timestamp_s": None if first_running is None else first_running[0],
            "末个机器人timestamp_s": None if last_running is None else last_running[0],
            "机器人时间跨度_s": span,
            "相邻有效运行RTDE的timestamp间隔_s": timestamp_intervals.report(),
            "非正timestamp间隔数量": nonpositive_intervals,
            "未取得有限timestamp的运行记录数量": missing_running_timestamps,
            "首尾cycle_count斜率_周期每秒": slope,
            "周期斜率说明": "按首尾 cycle_count 差除以机器人时间跨度估计，不能视为每次 servoj 的实测频率；日志停记会使相邻已记录样本跨过缺口。",
        },
        "phase2载荷及锚点偏差": {
            "首个有效q_start_rad": anchor, "后续q_start与首个值不同的记录数": anchor_change_count,
            "drive单位": context["units_and_coordinates"]["drive"],
            "各轴drive最大绝对值": [item.report(maximum_only=True) for item in drive_max],
            "各关节q_cmd减锚点最大绝对值_rad": [item.report(maximum_only=True) for item in command_offset_max],
            "各关节actual_q减锚点最大绝对值_rad": [item.report(maximum_only=True) for item in actual_offset_max],
            "锚点说明": "使用有效派生状态中首次取得的 q_start；遥操作或未取得固定锚点时偏差为空，不将 q_nom 当作固定锚点。",
        },
        "诊断": diagnostic_report,
        "主机通信与日志预算": host_budget.report(metadata),
        "人工事件": operator_events,
        "人工事件说明": "空列表表示没有记录，不能推断没有施力或已松手。事件时间来自主机，最近机器人状态仅供异步关联。",
        "停止及生命周期事件": stop_events,
        "最后有效生命周期状态": last_lifecycle,
        "日志缺口及停记": {
            "log_gap事件数量": kinds["log_gap"], "log_gap声明丢弃数量之和": gap_dropped_count,
            "metadata丢弃数量": metadata["dropped_count"], "metadata停记跳过数量": metadata["skipped_count"],
            "缺口事件": gap_events, "日志开关事件": logging_events,
            "日志最初开启": context["debug_logging_enabled_at_start"],
            "metadata已结束": metadata["finished"], "session_finish数量": kinds["session_finish"],
        },
        "解释范围": "这是人工排查的描述性汇总，不作稳定性、安全性、回位或联调通过判定；缺失量明确保留为未取得。",
    }


class ResidualSummary:
    """逐轴累计独立载荷采样的均值、均方根和范围。"""

    def __init__(self):
        self.range = NumericSummary()
        self.total = 0.0
        self.squares = 0.0
        self.positive = 0
        self.negative = 0

    def add(self, value, location=None):
        self.range.add(value, location)
        value = numeric_value(value)
        if value is not None:
            self.total += value
            self.squares += value * value
            self.positive += int(value > 0)
            self.negative += int(value < 0)

    def report(self):
        return {
            **self.range.report(),
            "均值": self.total / self.range.count if self.range.count else None,
            "均方根": math.sqrt(self.squares / self.range.count) if self.range.count else None,
            "正值数量": self.positive, "负值数量": self.negative,
        }


def load_vectors(page):
    """只用同一载荷冻结页中的六轴量计算残差，不与其他页拼接。"""
    raw = [numeric_value(value) for value in page["tau_raw_nm"]]
    bias = [numeric_value(value) for value in page["bias_nm"]]
    residual = [None if value is None or offset is None else value - offset
                for value, offset in zip(raw, bias)]
    return {"raw": raw, "bias": bias, "raw_minus_bias": residual,
            "filtered": page["filtered_nm"], "drive": page["drive"]}


def residual_accumulators():
    """为六轴载荷的五种实际观测量创建独立统计器。"""
    return {name: [ResidualSummary() for _ in range(6)]
            for name in ("raw", "bias", "raw_minus_bias", "filtered", "drive")}


def add_residuals(accumulators, vectors, location):
    """只累计指定独立采样，不补齐未观测轴或未观测时刻。"""
    for name, summaries in accumulators.items():
        for axis, summary in enumerate(summaries):
            summary.add(vectors[name][axis], location)


def analyze_continuous(run_dir, metadata, *, motion_windows, tail_seconds):
    """读取第 3 版冻结页、原始 RTDE 和双端时序，输出描述性证据。"""
    if tail_seconds <= 0:
        raise ValueError("残差尾窗长度必须大于零")
    context = metadata["context"]
    # 启动取消可发生在控制器构造前，此时只存在界面保存的准备参数。
    config = (context["controller"]["parameters"] if "controller" in context
              else context["prepared_parameters"])
    mode = config["DIAGNOSTIC_MODE"]
    motion_axis = config["DIAGNOSTIC_MOTION_AXIS"] - 1
    if motion_windows is None:
        motion_windows = [(1.0, 3.0), (5.0, 7.0)] if mode == "preset_joint_motion" else []
    for start, end in motion_windows:
        if start < 0 or end <= start:
            raise ValueError("运动窗口必须满足 0 ≤ 起点 < 终点")
    windows = [{"start": start, "end": end, "count": 0, "zero": 0,
                "positive": 0, "negative": 0, "missing": 0,
                "target": NumericSummary(), "actual": NumericSummary(),
                "current": NumericSummary(), "speed_scaling": NumericSummary(),
                "target_speed_fraction": NumericSummary()}
               for start, end in motion_windows]
    kinds, phases, page_types, limit_reasons = Counter(), Counter(), Counter(), Counter()
    page_sequences = {name: SequenceSummary() for name in ("anchor", "load", "execution", "timing")}
    identities = {name: SequenceSummary() for name in
                  ("model_sample_seq", "generated_seq", "submitted_seq", "adopted_seq", "frame")}
    running_load_sequences = SequenceSummary()
    first_running = last_running = previous_running = None
    timestamp_intervals = NumericSummary()
    raw_timestamp_gaps = 0
    raw_missing_frames = 0
    nonpositive_intervals = 0
    lifecycle_count = derived_count = running_count = 0
    anchor = None
    anchor_changes = 0
    command_offsets = [NumericSummary() for _ in range(6)]
    command_speeds = [NumericSummary() for _ in range(6)]
    actual_offsets = [NumericSummary() for _ in range(6)]
    robot_speeds = [NumericSummary() for _ in range(6)]
    robot_currents = [NumericSummary() for _ in range(6)]
    residuals = residual_accumulators()
    tail = deque()
    servo_timing_names = (
        "target_age_s", "target_max_age_s", "dt_model_actual_s", "dt_model_max_s",
        "servoj_start_interval_s", "servoj_max_interval_s", "compute_duration_s",
        "compute_max_duration_s", "servoj_duration_s", "servoj_duration_max_s",
        "publish_duration_s", "publish_max_duration_s",
    )
    servo_timings = {name: NumericSummary() for name in servo_timing_names}
    heartbeat_names = ("heartbeat_send_interval_s", "heartbeat_send_max_interval_s",
                       "heartbeat_send_lateness_s", "heartbeat_send_max_lateness_s",
                       "heartbeat_age_us", "heartbeat_max_interval_us")
    host_timing_names = tuple(f"{name}_{suffix}" for name in
                              ("packet_prepare", "send", "receive", "decode", "log_submit")
                              for suffix in ("duration_s", "max_duration_s"))
    host_timings = {name: NumericSummary() for name in heartbeat_names + host_timing_names}
    observed_servo_gaps = NumericSummary()
    last_execution = last_timing = last_lifecycle = None
    operator_events, stop_events, gap_events, logging_events = [], [], [], []
    scaling_events = []
    observed_heartbeat_sends = NumericSummary()
    previous_heartbeat_send = None
    host_budget = HostBudgetSummary()
    expected_rtde_interval = 1.0 / config[
        "ADMITTANCE_STATE_HZ" if mode == "admittance" else "DIAGNOSTIC_STATE_HZ"]
    with (run_dir / "records.jsonl").open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            try:
                record = json.loads(line)
            except json.JSONDecodeError as error:
                raise ValueError(f"{run_dir / 'records.jsonl'}:{line_number} 不是完整有效的 JSON 记录：{error.msg}") from error
            if record["log_schema_version"] != 3 or record["run_id"] != metadata["run_id"]:
                raise ValueError(f"{run_dir / 'records.jsonl'}:{line_number} 的版本或 run_id 与元数据不一致")
            kind, payload = record["kind"], record["payload"]
            host_budget.add(record)
            kinds[kind] += 1
            if kind == "rtde_input" and payload["heartbeat_advanced"]:
                sent = payload["host_monotonic_s"]
                if previous_heartbeat_send is not None:
                    observed_heartbeat_sends.add(sent - previous_heartbeat_send, record_location(record))
                previous_heartbeat_send = sent
            elif kind == "operator_event":
                operator_events.append({"记录序号": record["seq"], **payload})
            elif kind == "log_gap":
                gap_events.append({"记录序号": record["seq"], **payload})
            elif kind == "logging_enabled":
                logging_events.append({"记录序号": record["seq"], **payload})
            elif kind in ("speed_scaling_confirmed", "speed_scaling_changed"):
                scaling_events.append({
                    **stop_summary(record, payload["status"]),
                    "robot_timestamp_s": payload["robot_timestamp_s"],
                    "host_monotonic_s": payload["host_monotonic_s"],
                    "speed_scaling": payload["speed_scaling"],
                    "target_speed_fraction": payload["target_speed_fraction"],
                })
            elif kind in ("controller_phase", "controller_complete", "stop_requested",
                          "external_stop_confirmed", "controller_cancelled_before_start",
                          "controller_error", "controller_worker_failed"):
                status = payload["status"]
                if kind != "controller_phase" or status["phase"] in (3, 4) or status["exit_reason"]:
                    stop_events.append({**stop_summary(record, status),
                                        "静止退出已确认": status["stopped_confirmed"]})
            if kind != "rtde_state":
                continue
            control, robot = payload["control"], payload["raw"]
            timestamp = numeric_value(robot["timestamp"])
            location = record_location(record, timestamp)
            for name, summary in host_timings.items():
                summary.add(control[name], location)
            if not payload["lifecycle_valid"]:
                continue
            lifecycle_count += 1
            phase = control["phase"]
            phases[phase] += 1
            last_lifecycle = {**stop_summary(record, control),
                              "静止退出已确认": control["stopped_confirmed"],
                              "runtime_state": robot["runtime_state"], "actual_qd": robot["actual_qd"]}
            if control["q_start"] is not None:
                if anchor is None:
                    anchor = control["q_start"]
                elif control["q_start"] != anchor:
                    anchor_changes += 1
            if phase == 2:
                running_count += 1
                if timestamp is not None:
                    if first_running is None:
                        first_running = timestamp
                    last_running = timestamp
                    if previous_running is not None:
                        interval = timestamp - previous_running
                        timestamp_intervals.add(interval, location)
                        nonpositive_intervals += int(interval <= 0)
                        # 半帧分界避开浮点表示误差，只描述已记录 RTDE 的缺帧。
                        if interval > 1.5 * expected_rtde_interval:
                            raw_timestamp_gaps += 1
                            raw_missing_frames += max(0, round(interval / expected_rtde_interval) - 1)
                    previous_running = timestamp
                    relative_time = timestamp - first_running
                    for window in windows:
                        if window["start"] <= relative_time < window["end"]:
                            window["count"] += 1
                            target = numeric_value(robot["target_qd"][motion_axis])
                            window["missing"] += int(target is None)
                            if target is not None:
                                window["zero"] += int(target == 0.0)
                                window["positive"] += int(target > 0.0)
                                window["negative"] += int(target < 0.0)
                            window["target"].add(target, location)
                            window["actual"].add(robot["actual_qd"][motion_axis], location)
                            window["current"].add(robot["actual_current"][motion_axis], location)
                            for name in ("speed_scaling", "target_speed_fraction"):
                                window[name].add(robot[name], location)
                else:
                    previous_running = None
                for axis in range(6):
                    actual = numeric_value(robot["actual_q"][axis])
                    reference = None if anchor is None else numeric_value(anchor[axis])
                    actual_offsets[axis].add(None if actual is None or reference is None
                                             else abs(actual - reference), location)
                    speed = numeric_value(robot["actual_qd"][axis])
                    current = numeric_value(robot["actual_current"][axis])
                    robot_speeds[axis].add(None if speed is None else abs(speed), location)
                    robot_currents[axis].add(None if current is None else abs(current), location)
            if not payload["derived_valid"]:
                continue
            derived_count += 1
            if control["telemetry_layout"] != 3:
                raise ValueError(f"记录 {record['seq']} 的连续遥测布局不是 3")
            page_type = control["page_type"]
            page_types[page_type] += 1
            if not page_sequences[page_type].add(control["page_seq"]):
                continue
            if page_type == "anchor":
                continue
            page = control["load"] if page_type == "load" else control["continuous"][page_type]
            page_location = {**location, "page_type": page_type, "page_seq": page["page_seq"],
                             "model_sample_seq": page["model_sample_seq"],
                             "snapshot_time_s": page["snapshot_time_s"]}
            identities["model_sample_seq"].add(page["model_sample_seq"])
            identities["generated_seq"].add(page["generated_seq"])
            identities["submitted_seq"].add(page["submitted_seq"])
            limit_reasons[page["limit_reason"]] += 1
            if page_type in ("execution", "timing"):
                for name in ("adopted_seq", "frame"):
                    identities[name].add(page[name])
                observed_servo_gaps.add(page["gap_count"], page_location)
            if page_type == "execution":
                last_execution = {**page_location, **page}
                if page["phase"] == 2:
                    for axis in range(6):
                        reference = None if anchor is None else numeric_value(anchor[axis])
                        command = numeric_value(page["q_cmd"][axis])
                        speed = numeric_value(page["qd_cmd"][axis])
                        command_offsets[axis].add(None if reference is None or command is None
                                                  else abs(command - reference), page_location)
                        command_speeds[axis].add(None if speed is None else abs(speed), page_location)
            elif page_type == "timing":
                last_timing = {**page_location, **page}
                for name, summary in servo_timings.items():
                    summary.add(page[name], page_location)
            elif page_type == "load" and page["phase"] == 2:
                if not running_load_sequences.add(page["model_sample_seq"]):
                    continue
                vectors = load_vectors(page)
                add_residuals(residuals, vectors, page_location)
                sample_time = numeric_value(page["sample_time_s"])
                if sample_time is not None:
                    tail.append((sample_time, vectors, page_location))
                    while tail and tail[0][0] < sample_time - tail_seconds:
                        tail.popleft()
    tail_residuals = residual_accumulators()
    for _, vectors, location in tail:
        add_residuals(tail_residuals, vectors, location)
    return {
        "日志目录": str(run_dir), "日志版本": 3, "run_id": metadata["run_id"],
        "实际主机环境": context["host_environment"],
        "机器人连接环境": context["rtde"] if "rtde" in context else None,
        "场景": {"reference_source": context["reference_source"], "space": context["space"],
                 "diagnostic_mode": mode, "motion_axis": motion_axis + 1},
        "记录数量": {"实际读取": sum(kinds.values()), "metadata记录数量": metadata["record_count"],
                     "按kind": dict(kinds)},
        "RTDE": {"总数": kinds["rtde_state"], "lifecycle_valid数量": lifecycle_count,
                 "derived_valid数量": derived_count, "有效阶段数量": dict(phases),
                 "有效冻结页类型数量": dict(page_types)},
        "phase2运行": {
            "原始RTDE包数量": running_count, "首个机器人timestamp_s": first_running,
            "末个机器人timestamp_s": last_running,
            "机器人时间跨度_s": None if first_running is None else last_running - first_running,
            "相邻已记录RTDE间隔_s": timestamp_intervals.report(),
            "非正timestamp间隔数量": nonpositive_intervals,
            "已记录RTDE缺口次数": raw_timestamp_gaps, "按配置帧长估算缺少帧数": raw_missing_frames,
            "缺口说明": "RTDE/主机/日志观测缺口与机器人执行器 gap_count 分开；不能由前者断言伺服断帧或网络丢包。",
        },
        "连续伺服身份": {
            "各页发布序号": {name: item.report() for name, item in page_sequences.items()},
            "观测生成提交采用身份": {name: item.report() for name, item in identities.items()},
            "执行器累计帧缺口": observed_servo_gaps.report(maximum_only=True),
            "最后执行页": last_execution, "最后计时页": last_timing,
            "限幅原因页计数": dict(limit_reasons),
            "解释": "不同页独立冻结；这里只统计收到的身份。页序号或模型序号缺失不代表生产者漏算；不同冻结页的身份倒退也不等于执行器倒退。q_cmd/qd_cmd仅取当前执行页。",
        },
        "全阶段机器人周期及目标年龄_s": {name: item.report(maximum_only=True)
                                              for name, item in servo_timings.items()},
        "微动窗口": [{
            "相对首个phase2包的窗口_s": [window["start"], window["end"]],
            "轴": motion_axis + 1, "原始RTDE包数量": window["count"],
            "target_qd精确为零数量": window["zero"],
            "target_qd正值数量": window["positive"], "target_qd负值数量": window["negative"],
            "target_qd缺测数量": window["missing"],
            "target_qd_rad_s": window["target"].report(), "actual_qd_rad_s": window["actual"].report(),
            "actual_current_A": window["current"].report(),
            "speed_scaling": window["speed_scaling"].report(),
            "target_speed_fraction": window["target_speed_fraction"].report(),
        } for window in windows],
        "微动窗口说明": "窗口按 [起点,终点) 取原始 RTDE，包含有效生命周期下尚未取得完整冻结页的包。零速度计数只在所列运动窗口解释，不用于固定保持或转折处。",
        "phase2锚点和运动": {
            "q_start_rad": anchor, "锚点变化记录数": anchor_changes,
            "各关节已采用q_cmd相对锚点最大绝对值_rad": [item.report(maximum_only=True) for item in command_offsets],
            "各关节已采用qd_cmd最大绝对值_rad_s": [item.report(maximum_only=True) for item in command_speeds],
            "各关节actual_q相对锚点最大绝对值_rad": [item.report(maximum_only=True) for item in actual_offsets],
            "各关节actual_qd最大绝对值_rad_s": [item.report(maximum_only=True) for item in robot_speeds],
            "各关节actual_current最大绝对值_A": [item.report(maximum_only=True) for item in robot_currents],
            "观测覆盖说明": "q_cmd/qd_cmd极值来自已收到的独立执行页，不声称覆盖每个伺服帧；actual_q/actual_qd/actual_current来自有效生命周期的原始RTDE包。",
        },
        "六轴载荷_Nm": {
            "独立运行载荷model_sample_seq": running_load_sequences.report(),
            "完整运行段": {name: [item.report() for item in values] for name, values in residuals.items()},
            "尾窗长度_s": tail_seconds,
            "尾窗实际首末sample_time_s": None if not tail else [tail[0][0], tail[-1][0]],
            "尾窗独立样本数": len(tail),
            "尾窗": {name: [item.report() for item in values] for name, values in tail_residuals.items()},
            "解释": "只按当前载荷页且 phase=2 的 model_sample_seq 去重；raw-bias来自同页。尾窗以最后已收到运行载荷的采样时刻结束，不外推停止后残差。",
        },
        "主机与心跳": {
            "已记录心跳发送间隔_s": observed_heartbeat_sends.report(),
            "状态累计时序": {name: item.report(maximum_only=True) for name, item in host_timings.items()},
            "解释": "heartbeat_age_us与heartbeat_max_interval_us为机器人微秒值；其余带_s字段为主机秒值，使用各自时钟，不相减当作链路延迟。",
        },
        "主机通信与日志预算": host_budget.report(metadata),
        "人工事件": operator_events, "停止及生命周期事件": stop_events,
        "速度缩放握手和变化": scaling_events,
        "最后有效生命周期状态": last_lifecycle,
        "日志缺口及停记": {"缺口事件": gap_events, "开关事件": logging_events,
                             "metadata丢弃数量": metadata["dropped_count"],
                             "metadata停记跳过数量": metadata["skipped_count"],
                             "metadata已结束": metadata["finished"]},
        "解释范围": "只读描述性汇总，不作联调通过、安全性或真机动力学判定；不同冻结页与原始 RTDE 不冒充同步采样。",
    }


def main():
    """从命令行读取一轮日志目录并输出中文 JSON 报告。"""
    parser = argparse.ArgumentParser(description="只读分析导纳日志；不连接设备、不修改日志、不自动判定通过或失败。")
    parser.add_argument("run_dir", type=Path, help="包含 metadata.json 和 records.jsonl 的单轮日志目录")
    parser.add_argument("--motion-window", type=float, nargs=2, action="append", metavar=("START", "END"),
                        help="第 3 版日志的原始 RTDE 运动窗口，单位秒，可重复；默认微动取 1～3 与 5～7 秒")
    parser.add_argument("--tail-seconds", type=float, default=3.0, help="第 3 版六轴残差尾窗秒数，默认 3")
    args = parser.parse_args()
    print(json.dumps(analyze(args.run_dir, motion_windows=args.motion_window, tail_seconds=args.tail_seconds),
                     ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
