"""导纳调试日志：控制线程只入队，文件线程负责序列化、落盘和刷新。"""

from __future__ import annotations

from contextlib import ExitStack
from datetime import datetime
import json
import math
import os
from pathlib import Path
import platform
from queue import Empty, Full, Queue
import re
import sys
import threading
import time


LOG_SCHEMA_VERSION = 3


def _timing_update(previous, started_wall, started_cpu, *, seq=None, kind=None):
    """生成独立计时快照；最大墙钟样本保留同次 CPU 耗时及记录身份。"""
    finished_cpu = time.thread_time()
    finished_wall = time.monotonic()
    wall = finished_wall - started_wall
    cpu = finished_cpu - started_cpu
    result = {
        "count": 1 if previous is None else previous["count"] + 1,
        "last_wall_s": wall, "last_cpu_s": cpu,
        "last_started_monotonic_s": started_wall,
        "last_finished_monotonic_s": finished_wall,
        "max_cpu_s": cpu if previous is None else max(cpu, previous["max_cpu_s"]),
    }
    if previous is None or wall > previous["max_wall_s"]:
        result.update(
            max_wall_s=wall, max_wall_cpu_s=cpu,
            max_wall_started_monotonic_s=started_wall,
            max_wall_finished_monotonic_s=finished_wall,
            max_wall_seq=seq, max_wall_kind=kind,
        )
    else:
        result.update({key: value for key, value in previous.items() if key.startswith("max_wall")})
    return result


def _json_value(value):
    """在写线程内保留异常数值的身份，以明确标记对象生成标准 JSON。"""
    if isinstance(value, float) and not math.isfinite(value):
        label = "NaN" if math.isnan(value) else ("Infinity" if value > 0 else "-Infinity")
        return {"non_finite_float": label}
    if isinstance(value, dict):
        return {key: _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    return value


class AdmittanceLogger:
    """每轮导纳使用一个实例；传入的字典及其子对象提交后不得再修改。"""

    def __init__(self, base_dir=None, *, enabled=True, queue_capacity=8192):
        if queue_capacity < 1:
            raise ValueError("导纳日志队列容量必须大于零")
        self.base_dir = (Path(__file__).resolve().parent / "logs" / "admittance"
                         if base_dir is None else Path(base_dir))
        self._enabled = bool(enabled)
        self._queue = Queue(maxsize=queue_capacity)
        self._lock = threading.Lock()
        self._stop_requested = threading.Event()
        self._thread = None
        self._active = False
        self._accepting = False
        self._ever_enabled = False
        self._path = None
        self._run_id = None
        self._metadata = None
        self._finish_summary = None
        self._finished_wall_time_ns = None
        self._finished_monotonic_ns = None
        self._sequence = 0
        self._record_count = 0
        self._dropped_count = 0
        self._skipped_count = 0
        self._pending_gap = None
        self._error = None
        self._enable_history = []
        self._producer_timing = {}
        self._writer_timing = {}
        self._queue_high_watermark = 0
        self._queue_age = None

    def start(self, metadata):
        """开始本轮；调用方提供 run_id、参数修订、模式、配置、TCP、单位及版本。"""
        with self._lock:
            if self._thread is not None:
                raise RuntimeError("一个导纳日志实例只能启动一轮")
            self._run_id = metadata["run_id"]
            self._metadata = {
                **metadata,
                "host_environment": {
                    "python_version": sys.version,
                    "python_executable": sys.executable,
                    "python_prefix": sys.prefix,
                    "python_no_user_site": bool(sys.flags.no_user_site),
                    "python_dont_write_bytecode": sys.dont_write_bytecode,
                    "operating_system": platform.system(),
                    "operating_system_release": platform.release(),
                    "machine": platform.machine(),
                    "process_id": os.getpid(),
                },
            }
            self._started_wall_time_ns = time.time_ns()
            self._started_monotonic_ns = time.monotonic_ns()
            self._started_local = datetime.now().astimezone()
            self._active = True
            self._accepting = True
            if self._enabled:
                self._begin_logging_locked()
            self._thread = threading.Thread(
                target=self._write_loop, name=f"AdmittanceLog-{self._run_id}", daemon=True,
            )
            self._thread.start()
        return self.get_status()

    def record(self, kind, payload, *, force=False):
        """提交已独立拥有且可序列化的字典；此方法不复制数据、不访问文件。"""
        started_wall, started_cpu = time.monotonic(), time.thread_time()
        with self._lock:
            self._producer_timing["record_lock_wait"] = _timing_update(
                self._producer_timing.get("record_lock_wait"), started_wall, started_cpu,
                kind=kind,
            )
            if not self._accepting or self._error is not None:
                return False
            if not self._enabled and not (force and self._ever_enabled):
                self._skipped_count += 1
                return False
            started_wall, started_cpu = time.monotonic(), time.thread_time()
            accepted = self._enqueue_locked(kind, payload)
            self._producer_timing["record_enqueue"] = _timing_update(
                self._producer_timing.get("record_enqueue"), started_wall, started_cpu,
                seq=self._sequence, kind=kind,
            )
            return accepted

    def update_context(self, context):
        """合并本轮固定上下文；关闭时只保存在内存，提交后不得修改其中对象。"""
        with self._lock:
            if not self._accepting or self._error is not None:
                return
            self._metadata = {**self._metadata, **context}
            if self._enabled:
                self._enqueue_locked("session_context", {"context": self._metadata})

    def set_enabled(self, enabled, *, context=None):
        """开关边界独立于普通数据；重新开启时由调用方提交当前参数及状态快照。"""
        enabled = bool(enabled)
        with self._lock:
            if enabled != self._enabled:
                self._enabled = enabled
                if self._accepting and self._error is None:
                    if enabled and not self._ever_enabled:
                        self._begin_logging_locked()
                    boundary = {
                        "enabled": enabled,
                        "context": context,
                        "skipped_count": self._skipped_count,
                        "wall_time_ns": time.time_ns(),
                        "monotonic_ns": time.monotonic_ns(),
                    }
                    if enabled:
                        boundary["session_context"] = self._metadata
                    self._enable_history.append(boundary)
                    if self._ever_enabled:
                        self._enqueue_locked("logging_enabled", boundary)
        return self.get_status()

    def finish(self, summary=None):
        """停止接收并等待队列排空；仅由生命周期或日志收尾线程调用，不得在通信线程调用。"""
        with self._lock:
            if self._thread is None:
                raise RuntimeError("导纳日志尚未启动")
            if threading.current_thread() is self._thread:
                raise RuntimeError("日志写入线程不能等待自身结束")
            if self._accepting:
                self._accepting = False
                self._finish_summary = summary
                self._finished_wall_time_ns = time.time_ns()
                self._finished_monotonic_ns = time.monotonic_ns()
                if self._ever_enabled and self._error is None:
                    self._enqueue_locked("session_finish", {"summary": summary})
                self._stop_requested.set()
            thread = self._thread
        thread.join()
        return self.get_status()

    def close(self):
        """等同于不补充摘要地结束本轮日志。"""
        return self.finish()

    def get_status(self):
        """返回轻量状态；pending_count 表示尚未交给文件线程的队列长度。"""
        with self._lock:
            return {
                "enabled": self._enabled,
                "active": self._active,
                "path": None if self._path is None else str(self._path),
                "run_id": self._run_id,
                "record_count": self._record_count,
                "dropped_count": self._dropped_count,
                "skipped_count": self._skipped_count,
                "pending_count": self._queue.qsize(),
                "queue_capacity": self._queue.maxsize,
                "error": self._error,
                "logger_timing": self._timing_snapshot_locked(),
            }

    def _timing_snapshot_locked(self):
        """只复制聚合指标外层；各次计时和队列年龄快照发布后不再修改。"""
        return {
            "schema_version": 1,
            "producer": dict(self._producer_timing),
            "writer": dict(self._writer_timing),
            "queue_observed_high_watermark": self._queue_high_watermark,
            "queue_age": self._queue_age,
        }

    def _observe_writer(self, name, started_wall, started_cpu, *, record=None):
        """文件线程独占指标写入，不为序列化或磁盘操作持有生产者锁。"""
        self._writer_timing[name] = _timing_update(
            self._writer_timing.get(name), started_wall, started_cpu,
            seq=None if record is None else record["seq"],
            kind=None if record is None else record["kind"],
        )

    def _observe_queue_age(self, record):
        """出队时记录从提交到取出的年龄；高水位只表示实际观测到的队列长度。"""
        dequeued = time.monotonic()
        age = dequeued - record["monotonic_ns"] / 1_000_000_000
        previous = self._queue_age
        sample = {
            "count": 1 if previous is None else previous["count"] + 1,
            "last_s": age, "last_dequeued_monotonic_s": dequeued,
        }
        if previous is None or age > previous["max_s"]:
            sample.update(max_s=age, max_seq=record["seq"], max_kind=record["kind"],
                          max_dequeued_monotonic_s=dequeued)
        else:
            sample.update({key: value for key, value in previous.items() if key.startswith("max_")})
        self._queue_age = sample

    def _begin_logging_locked(self):
        """只确定目录名和开始事件；实际创建目录由文件线程执行。"""
        self._ever_enabled = True
        run_name = re.sub(r"[^A-Za-z0-9_.-]", "_", str(self._run_id))
        self._path = self.base_dir / (
            self._started_local.strftime("%Y%m%d_%H%M%S_%f") + "_" + run_name
        )
        self._enqueue_locked("session_start", {
            "metadata": self._metadata,
            "started_wall_time_ns": self._started_wall_time_ns,
            "started_monotonic_ns": self._started_monotonic_ns,
            "skipped_count": self._skipped_count,
        })

    def _make_record_locked(self, kind, payload):
        self._sequence += 1
        return {
            "log_schema_version": LOG_SCHEMA_VERSION,
            "run_id": self._run_id,
            "seq": self._sequence,
            "kind": kind,
            "wall_time_ns": time.time_ns(),
            "monotonic_ns": time.monotonic_ns(),
            "logging_enabled": self._enabled,
            "payload": payload,
        }

    def _enqueue_locked(self, kind, payload):
        """先标记已发生的数据缺口，队列已满时立即放弃当前记录。"""
        if self._pending_gap is not None and not self._queue.full():
            gap = self._make_record_locked("log_gap", self._pending_gap)
            self._queue.put_nowait(gap)
            self._pending_gap = None
        record = self._make_record_locked(kind, payload)
        try:
            self._queue.put_nowait(record)
            self._queue_high_watermark = max(self._queue_high_watermark, self._queue.qsize())
            return True
        except Full:
            self._note_drop_locked(record)
            return False

    def _note_drop_locked(self, record):
        self._dropped_count += 1
        if self._pending_gap is None:
            self._pending_gap = {
                "reason": "日志队列已满，控制线程未等待磁盘",
                "dropped_count": 0,
                "first_seq": record["seq"],
                "last_seq": record["seq"],
                "first_monotonic_ns": record["monotonic_ns"],
                "last_monotonic_ns": record["monotonic_ns"],
                "kinds": {},
            }
        gap = self._pending_gap
        gap["dropped_count"] += 1
        gap["last_seq"] = record["seq"]
        gap["last_monotonic_ns"] = record["monotonic_ns"]
        gap["kinds"][record["kind"]] = gap["kinds"].get(record["kind"], 0) + 1

    def _metadata_document(self, *, finished):
        """元数据保留首轮输入、开关历史和结束摘要，便于恢复日志中的缺口。"""
        with self._lock:
            return {
                "log_schema_version": LOG_SCHEMA_VERSION,
                "run_id": self._run_id,
                "format": "records.jsonl：每行一个 JSON 对象，kind 标识数据来源",
                "started_at": self._started_local.isoformat(),
                "started_wall_time_ns": self._started_wall_time_ns,
                "started_monotonic_ns": self._started_monotonic_ns,
                "record_timestamps": {
                    "wall_time_ns": "Unix 墙钟纳秒，用于跨文件关联",
                    "monotonic_ns": "主机单调时钟纳秒，用于本轮间隔计算",
                    "payload_timestamps": "原始来源时间，不等同于主机提交时间",
                    "process_bridge": "跨进程记录的 payload.host_monotonic_s 保留通信进程发生时刻；顶层时间为父日志入队时刻，process_bridge.sequence/source_pid 标识来源，received_monotonic_s 为父桥接收时刻",
                    "continuous_pages": "连续伺服各页有独立冻结身份；生成、提交和实际采用不同，禁止将不同页拼成同步样本",
                    "operator_event": "人工事件使用主机点击处理时刻；latest_robot_state 仅关联最近收到的状态，不代表机器人同周期采样",
                },
                "non_finite_float_encoding": {
                    "description": "原始非有限浮点数编码为单键对象，有限数值保持 JSON 数字，不以零替代",
                    "key": "non_finite_float",
                    "values": ["NaN", "Infinity", "-Infinity"],
                },
                "context": self._metadata,
                "enable_history": list(self._enable_history),
                "queue_capacity": self._queue.maxsize,
                "finished": finished,
                "finished_wall_time_ns": self._finished_wall_time_ns if finished else None,
                "finished_monotonic_ns": self._finished_monotonic_ns if finished else None,
                "finish_summary": self._finish_summary if finished else None,
                "record_count": self._record_count,
                "dropped_count": self._dropped_count,
                "skipped_count": self._skipped_count,
                "logger_timing": self._timing_snapshot_locked(),
                "logger_timing_contract": {
                    "wall_clock": "单调墙钟耗时，包含等待、被抢占和执行，不是 Unix 时间",
                    "cpu_clock": "仅当前调用线程的 CPU 耗时；与墙钟之差不能单独证明 GIL 竞争",
                    "max_sample": "max_wall_cpu_s 与 max_wall_s 来自同一次操作，含单调起止时刻及可用的 seq/kind",
                    "queue": "有界队列仍使用 put_nowait；年龄为提交至出队的观测时间，高水位为入队后观测值",
                    "scope": "指标汇总自身开销不计入该分段；metadata_write 不包含正在写入自身的这一次元数据操作",
                },
            }

    def _write_metadata(self, *, finished):
        started_wall, started_cpu = time.monotonic(), time.thread_time()
        temporary = self._path / "metadata.json.tmp"
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump(_json_value(self._metadata_document(finished=finished)), stream,
                      ensure_ascii=False, allow_nan=False, indent=2)
            stream.write("\n")
        temporary.replace(self._path / "metadata.json")
        self._observe_writer("metadata_write", started_wall, started_cpu)

    def _write_record(self, stream, record):
        started_wall, started_cpu = time.monotonic(), time.thread_time()
        encoded = json.dumps(_json_value(record), ensure_ascii=False, allow_nan=False,
                             separators=(",", ":")) + "\n"
        self._observe_writer("serialize", started_wall, started_cpu, record=record)
        started_wall, started_cpu = time.monotonic(), time.thread_time()
        stream.write(encoded)
        self._observe_writer("write", started_wall, started_cpu, record=record)
        started_wall, started_cpu = time.monotonic(), time.thread_time()
        with self._lock:
            self._record_count += 1
        self._observe_writer("count_update", started_wall, started_cpu, record=record)

    def _flush(self, stream):
        """单独观测文件刷新；仍由原有开关、周期和结束边界调用。"""
        started_wall, started_cpu = time.monotonic(), time.thread_time()
        stream.flush()
        self._observe_writer("flush", started_wall, started_cpu)

    def _write_loop(self):
        """文件错误在此边界明确报告并停止日志，不重试、不改变机器人控制。"""
        operation = "等待首条日志"
        try:
            with ExitStack() as files:
                stream = None
                next_flush = time.monotonic() + 1.0
                while True:
                    try:
                        record = self._queue.get(timeout=max(0.0, next_flush - time.monotonic()))
                    except Empty:
                        record = None
                    if record is not None:
                        self._observe_queue_age(record)
                        if stream is None:
                            operation = "创建日志目录"
                            self._path.mkdir(parents=True, exist_ok=False)
                            operation = "写入启动元数据"
                            self._write_metadata(finished=False)
                            operation = "打开日志数据文件"
                            stream = files.enter_context(
                                (self._path / "records.jsonl").open("w", encoding="utf-8")
                            )
                        operation = f"写入 {record['kind']} 记录（seq={record['seq']}）"
                        self._write_record(stream, record)
                        if record["kind"] in ("session_context", "logging_enabled"):
                            operation = "刷新本轮上下文元数据"
                            self._write_metadata(finished=False)
                        if (record["kind"] == "logging_enabled"
                                and not record["payload"]["enabled"]):
                            operation = "刷新关闭开关之前的日志"
                            self._flush(stream)
                    if time.monotonic() >= next_flush:
                        if stream is not None:
                            operation = "刷新日志文件"
                            self._flush(stream)
                        next_flush = time.monotonic() + 1.0
                    if self._stop_requested.is_set() and self._queue.empty():
                        with self._lock:
                            final_gap = None
                            if self._pending_gap is not None:
                                final_gap = self._make_record_locked("log_gap", self._pending_gap)
                                self._pending_gap = None
                        if final_gap is not None:
                            operation = "写入最后的数据缺口"
                            self._write_record(stream, final_gap)
                        break
                if stream is not None:
                    operation = "刷新结束日志"
                    self._flush(stream)
                    operation = "写入结束元数据"
                    self._write_metadata(finished=True)
                    operation = "关闭日志数据文件"
        except Exception as exc:
            with self._lock:
                self._error = f"{operation}失败：{type(exc).__name__}: {exc}"
                self._accepting = False
                while True:
                    try:
                        self._queue.get_nowait()
                    except Empty:
                        break
        finally:
            with self._lock:
                self._active = False
