"""导纳进程边界：控制、状态有效性及心跳共同隔离，父进程负责界面和日志。"""

from __future__ import annotations

from copy import deepcopy
import gc
import multiprocessing as mp
import os
import pickle
from queue import Empty, Full, Queue
import sys
import threading
import time

from UR_Utils import admittance_config
from UR_Utils.URAdmittanceController import URAdmittanceController, idle_admittance_status


STATUS_PERIOD_S = 0.05
BRIDGE_LEASE_S = 1.0
BRIDGE_QUEUE_CAPACITY = 1024
STATUS_CAPACITY_BYTES = 131072


class _LatestSnapshot:
    """单格冻结快照，双方锁内只复制 bytes；生产者不等待显示消费者。"""

    def __init__(self, context, capacity: int):
        self.data = context.RawArray("B", capacity)
        self.size = context.RawValue("I", 0)
        self.sequence = context.RawValue("Q", 0)
        self.lock = context.Lock()

    def publish(self, value: dict) -> bool:
        """锁忙时保留上一快照，下一次发布直接覆盖为最新状态。"""
        encoded = pickle.dumps(value, protocol=5)
        if len(encoded) > len(self.data):
            raise ValueError("导纳 IPC 快照超过固定槽位容量")
        if not self.lock.acquire(False):
            return False
        try:
            memoryview(self.data).cast("B")[:len(encoded)] = encoded
            self.size.value = len(encoded)
            self.sequence.value += 1
        finally:
            self.lock.release()
        return True

    def read(self, last_sequence: int):
        """只返回最新完整快照，解码在共享锁之外进行。"""
        if not self.lock.acquire(False):
            return last_sequence, None
        try:
            sequence = self.sequence.value
            encoded = bytes(memoryview(self.data).cast("B")[:self.size.value]) if sequence != last_sequence else None
        finally:
            self.lock.release()
        return sequence, None if encoded is None else pickle.loads(encoded)


class _ChildLogSink:
    """子进程只提交有界冻结消息，不执行磁盘写入或父进程日志编码。"""

    def __init__(self, queue):
        self.queue = queue
        self.sequence = 0
        self.dropped_count = 0
        self.pending_gap = None
        self.source_pid = os.getpid()

    def _send(self, operation: str, payload: dict) -> bool:
        self.sequence += 1
        envelope = {"operation": operation, "payload": payload, "bridge_seq": self.sequence,
                    "source_pid": self.source_pid}
        encoded = pickle.dumps(envelope, protocol=5)
        try:
            self.queue.put_nowait(encoded)
            return True
        except Full:
            self.dropped_count += 1
            kind = payload.get("kind", operation)
            if self.pending_gap is None:
                self.pending_gap = {"first_seq": self.sequence, "last_seq": self.sequence,
                                    "drop_count": 0, "kinds": {}, "source_pid": self.source_pid}
            self.pending_gap["last_seq"] = self.sequence
            self.pending_gap["drop_count"] += 1
            self.pending_gap["kinds"][kind] = self.pending_gap["kinds"].get(kind, 0) + 1
            return False

    def record(self, kind: str, payload: dict) -> bool:
        """保留控制器原始采样时钟，队列满时明确记缺口，不阻塞控制线程。"""
        if self.pending_gap is not None:
            gap = self.pending_gap
            self.pending_gap = None
            if not self._send("record", {"kind": "process_bridge_gap", "payload": gap}):
                new_gap = self.pending_gap
                kinds = dict(gap["kinds"])
                for name, count in new_gap["kinds"].items():
                    kinds[name] = kinds.get(name, 0) + count
                self.pending_gap = {**gap, "last_seq": new_gap["last_seq"],
                                    "drop_count": gap["drop_count"] + new_gap["drop_count"],
                                    "kinds": kinds}
        return self._send("record", {"kind": kind, "payload": payload})

    def update_context(self, context: dict) -> None:
        """固定上下文与事件使用相同 FIFO，不改变原始运行身份。"""
        self._send("context", context)

    def finish(self, status: dict, *, timeout: float) -> None:
        """控制停止后提交最终计数，父进程消费此标记后才可结束日志。"""
        self.sequence += 1
        summary = {"source_pid": self.source_pid, "last_bridge_seq": self.sequence,
                   "dropped_count": self.dropped_count, "pending_gap": self.pending_gap,
                   "queue_capacity": BRIDGE_QUEUE_CAPACITY}
        envelope = {"operation": "terminal", "payload": {"status": status, "summary": summary},
                    "bridge_seq": self.sequence, "source_pid": self.source_pid}
        self.queue.put(pickle.dumps(envelope, protocol=5), timeout=timeout)


def _process_main(arguments, event_writer, status_slot, reference_slot, stop_event,
                  lease_clock, worker_failed, bridge_failed, started_event, drained_ack, confirmation_queue):
    """子进程主线程拥有完整控制链；父失联时在相同链路请求停止。"""
    parent = mp.parent_process()
    event_queue = Queue(maxsize=BRIDGE_QUEUE_CAPACITY)
    sender_error = []

    def send_events():
        """发送线程仅写冻结 bytes；写端只有本进程持有，退出后父读端可见 EOF。"""
        try:
            while True:
                event_writer.send_bytes(event_queue.get())
        except (BrokenPipeError, EOFError, OSError) as exc:
            sender_error.append(f"导纳日志管道发送失败：{exc}")
            bridge_failed.set()
            stop_event.set()
        finally:
            event_writer.close()

    threading.Thread(target=send_events, name="AdmittanceEventWriter", daemon=True).start()
    sink = _ChildLogSink(event_queue)
    sink.update_context({"communication_process": {
        "pid": os.getpid(), "parent_pid": os.getppid(), "start_method": "spawn",
        "python_executable": sys.executable, "status_period_s": STATUS_PERIOD_S,
        "automatic_gc_enabled": gc.isenabled(),
        "bridge_lease_s": BRIDGE_LEASE_S, "bridge_queue_capacity": BRIDGE_QUEUE_CAPACITY,
    }})
    reference_sequence = 0
    last_publish = 0.0
    last_lifecycle = None
    owner_error = None

    def service(backend):
        """每批控制工作前检查拥有者，并以有限工作处理最新参考和显示快照。"""
        nonlocal reference_sequence, last_publish, last_lifecycle, owner_error
        now = time.monotonic()
        if stop_event.is_set() and not backend._stop_logged:
            backend.request_stop()
        if owner_error is None:
            if not parent.is_alive():
                owner_error = "父进程已退出，导纳请求停止"
            elif bridge_failed.is_set():
                owner_error = sender_error[0] if sender_error else "父进程日志/状态桥接已断开，导纳请求停止"
            elif now - lease_clock.value > BRIDGE_LEASE_S:
                owner_error = "父进程日志/状态桥接超过租约期限，导纳请求停止"
            if owner_error is not None:
                backend._set_error(owner_error)
                backend.request_stop()
        reference_sequence, reference = reference_slot.read(reference_sequence)
        if reference is not None and not stop_event.is_set() and not backend._stop_requested.is_set():
            backend.submit_reference(reference["q"], source_seq=reference["source_seq"])
        with backend._state_lock():
            lifecycle = tuple(backend._status[name] for name in
                              ("active", "phase", "exit_reason", "last_error", "stopping")) + (worker_failed.is_set(),)
        if lifecycle != last_lifecycle or now - last_publish >= STATUS_PERIOD_S:
            snapshot = backend.get_status()
            snapshot.update(communication_process_pid=os.getpid(), process_worker_failed=worker_failed.is_set(),
                            process_bridge_dropped_count=sink.dropped_count,
                            process_status_published_monotonic_s=now)
            if status_slot.publish(snapshot):
                last_publish, last_lifecycle = now, lifecycle

    backend = URAdmittanceController(**arguments, stop_event=stop_event,
                                     log_sink=sink, owner_service=service)
    started_event.set()
    backend.run_in_current_thread()
    if backend.needs_external_stop_confirmation():
        worker_failed.set()
        service(backend)
        while parent.is_alive():
            try:
                sample = confirmation_queue.get(timeout=0.05)
            except Empty:
                continue
            backend.confirm_external_stop(sample)
            break
        if backend.active:
            # 拥有者死亡且通信已失败时不再发送心跳；机器人按原失联契约停止。
            return
    if bridge_failed.is_set():
        return
    final_status = backend.get_status()
    final_status.update(communication_process_pid=os.getpid(), process_worker_failed=worker_failed.is_set(),
                        process_bridge_dropped_count=sink.dropped_count,
                        process_status_published_monotonic_s=time.monotonic())
    sink.finish(final_status, timeout=arguments["config"]["ADMITTANCE_STOP_TIMEOUT_S"])
    while parent.is_alive() and not drained_ack.wait(0.05):
        if bridge_failed.is_set():
            return


class AdmittanceProcess:
    """父进程代理保持原控制 API，任何未确认故障仍占有执行权。"""

    def __init__(self, robot_ip, reference_source, space, *, q_master=None, source_seq=None,
                 config=None, stop_event=None, log_sink=None):
        self.robot_ip = robot_ip
        self.reference_source = reference_source
        self.space = space
        self.parameters = deepcopy(admittance_config.snapshot() if config is None else config)
        admittance_config.validate_run_config(reference_source, space, config=self.parameters)
        self._context = mp.get_context("spawn")
        self._stop_event = self._context.Event()
        self._source_stop_event = stop_event
        self._log_sink = log_sink
        self._event_reader, self._event_writer = self._context.Pipe(duplex=False)
        self._status_slot = _LatestSnapshot(self._context, STATUS_CAPACITY_BYTES)
        self._reference_slot = _LatestSnapshot(self._context, 1024)
        self._lease_clock = self._context.RawValue("d", time.monotonic())
        self._worker_failed = self._context.Event()
        self._bridge_failed = self._context.Event()
        self._started_event = self._context.Event()
        self._drained_ack = self._context.Event()
        self._confirmation_queue = self._context.Queue(maxsize=1)
        self._finished = threading.Event()
        self._bridge_finished = threading.Event()
        self._lock = threading.Lock()
        self._process = None
        self._bridge = None
        self._closed = False
        self._status_sequence = 0
        self._submitted_seq = source_seq
        self._submitted_q = None if q_master is None else list(q_master)
        self._arguments = {"robot_ip": robot_ip, "reference_source": reference_source, "space": space,
                           "q_master": self._submitted_q, "source_seq": source_seq, "config": self.parameters}
        self._status = idle_admittance_status()
        self._status.update(active=True, starting=True, stopped_confirmed=False, phase=0,
                            phase_name="启动中", parameters=self.parameters, reference_source=reference_source,
                            space=space, communication_process_pid=None, process_worker_failed=False,
                            process_bridge_dropped_count=0, process_bridge=None,
                            process_status_published_monotonic_s=None,
                            process_status_received_monotonic_s=None, process_status_age_s=None)

    @property
    def active(self) -> bool:
        with self._lock:
            return self._status["active"]

    def start(self) -> None:
        """普通运动交接完成后启动进程，Qt/GIL 不再拥有心跳执行路径。"""
        if self._source_stop_event is not None and self._source_stop_event.is_set():
            self.cancel_before_start()
            return
        self._lease_clock.value = time.monotonic()
        self._process = self._context.Process(
            target=_process_main, name="URAdmittanceProcess", daemon=True,
            args=(self._arguments, self._event_writer, self._status_slot, self._reference_slot, self._stop_event,
                  self._lease_clock, self._worker_failed, self._bridge_failed, self._started_event, self._drained_ack,
                  self._confirmation_queue),
        )
        self._process.start()
        self._event_writer.close()
        self._bridge = threading.Thread(target=self._bridge_loop, name="AdmittanceProcessBridge", daemon=True)
        self._bridge.start()

    def wait_started(self, timeout=None) -> bool:
        """等待子进程建立控制对象，不把它解释为机器人已进入运行阶段。"""
        return self._started_event.wait(timeout)

    def request_stop(self) -> None:
        """共享停止事件直接到控制拥有者，不经过可能积压的日志队列。"""
        self._stop_event.set()
        with self._lock:
            if self._status["active"]:
                self._status = {**self._status, "stopping": True, "starting": False, "running": False}

    def stop(self) -> None:
        """等待本轮退出静止及日志桥接收尾，故障时交由父只读连接确认。"""
        self.request_stop()
        deadline = time.monotonic() + self.parameters["ADMITTANCE_STOP_TIMEOUT_S"]
        while not self._finished.wait(0.01):
            if self.needs_external_stop_confirmation():
                raise RuntimeError("导纳独立进程停止尚需原只读连接确认")
            if time.monotonic() >= deadline:
                raise RuntimeError("导纳停止尚未确认，保留执行权，禁止恢复其他运动")

    def close(self) -> None:
        """先按停止契约完成，再等待已无控制资源的进程退出。"""
        if self._closed:
            return
        if self.active:
            self.stop()
        if self._process is not None and self._process.pid is not None:
            self._process.join(timeout=self.parameters["ADMITTANCE_STOP_TIMEOUT_S"])
            if self._process.is_alive():
                raise RuntimeError("导纳进程资源尚未退出")
        self._event_reader.close()
        self._event_writer.close()
        self._confirmation_queue.close()
        self._confirmation_queue.join_thread()
        self._closed = True

    def cancel_before_start(self, error=None) -> None:
        """只允许进程启动前取消；已启动则保留执行权并走停止流程。"""
        if self._process is not None and self._process.pid is not None:
            self.request_stop()
            if error is not None:
                with self._lock:
                    self._status = {**self._status, "last_error": error}
            return
        with self._lock:
            self._status = {**self._status, "active": False, "starting": False, "running": False,
                            "stopping": False, "stopped_confirmed": True, "phase": 4,
                            "phase_name": "已停止", "last_error": error}
            status = self._status
        if self._log_sink is not None:
            self._log_sink.record("controller_cancelled_before_start", {"status": status})
        self._finished.set()

    def get_status(self) -> dict:
        """返回父进程已接收的最新冻结状态，不向机器人发起查询。"""
        with self._lock:
            status = self._status
        result = deepcopy(status)
        published = result["process_status_published_monotonic_s"]
        result["process_status_age_s"] = None if published is None else max(0.0, time.monotonic() - published)
        return result

    def submit_reference(self, q, *, source_seq: int) -> bool:
        """覆盖最新待采用目标；异步停止开始后返回 False，不向已停止的本轮继续提交。"""
        if self.reference_source != "teleop":
            raise RuntimeError("定点柔顺期间不能提交主端关节目标")
        values, sequence = URAdmittanceController._validated_reference(q, source_seq)
        if not self.active or self._stop_event.is_set():
            return False
        if self._submitted_seq is not None and sequence < self._submitted_seq:
            raise ValueError("主端源帧序号不能倒退")
        if sequence == self._submitted_seq and values.tolist() != self._submitted_q:
            raise ValueError("同一主端帧序号不能对应不同关节目标")
        if self._reference_slot.publish({"q": values.tolist(), "source_seq": sequence}):
            self._submitted_seq, self._submitted_q = sequence, values.tolist()
            return True
        return False

    def update_reference(self, q, *, source_seq: int) -> None:
        """沿用主端帧身份语义的显式更新入口。"""
        self.submit_reference(q, source_seq=source_seq)

    def needs_external_stop_confirmation(self) -> bool:
        """通信拥有者已失败时，父进程仍须核实新鲜停止反馈。"""
        return self.active and (self._worker_failed.is_set()
                               or self._process is not None and self._process.exitcode is not None)

    def confirm_external_stop(self, sample: dict) -> None:
        """只接受原只读连接已经核实的程序退出与连续静止证据。"""
        if not self.needs_external_stop_confirmation():
            raise RuntimeError("导纳进程尚未失败，不能外部释放执行权")
        if self._process.is_alive() and not self._bridge_failed.is_set():
            self._confirmation_queue.put(sample, timeout=self.parameters["ADMITTANCE_STOP_TIMEOUT_S"])
            if not self._finished.wait(self.parameters["ADMITTANCE_STOP_TIMEOUT_S"]):
                raise RuntimeError("停止已由父进程核实，但子进程日志收尾未完成")
            return
        if self._process.is_alive():
            self._confirmation_queue.put(sample, timeout=self.parameters["ADMITTANCE_STOP_TIMEOUT_S"])
            self._process.join(timeout=self.parameters["ADMITTANCE_STOP_TIMEOUT_S"])
            if self._process.is_alive():
                raise RuntimeError("停止已核实，但失效桥接对应的子进程尚未退出")
        if not self._bridge_finished.wait(self.parameters["ADMITTANCE_STOP_TIMEOUT_S"]):
            raise RuntimeError("停止已核实，日志桥仍未退出")
        status = {**self.get_status(), **sample, "active": False, "running": False, "starting": False,
                  "stopping": False, "stopped_confirmed": True, "phase": 4, "phase_name": "已停止"}
        if self._log_sink is not None:
            self._log_sink.record("external_stop_confirmed", {"sample": sample, "status": status})
            self._log_sink.record("controller_complete", {"status": status})
        with self._lock:
            self._status = status
        self._finished.set()

    def _accept_status(self) -> None:
        """普通状态不能提前释放执行权，终止标记负责日志先后顺序。"""
        self._status_sequence, status = self._status_slot.read(self._status_sequence)
        if status is None:
            return
        if not status["active"]:
            status = {**status, "active": True, "stopped_confirmed": False, "stopping": True}
        if self._stop_event.is_set():
            status = {**status, "stopping": True, "starting": False, "running": False}
        status["process_status_received_monotonic_s"] = time.monotonic()
        with self._lock:
            self._status = status

    def _bridge_loop(self) -> None:
        """父进程唯一桥按 FIFO 交付日志，消费终止标记后才公开 inactive。"""
        try:
            while True:
                self._lease_clock.value = time.monotonic()
                if self._source_stop_event is not None and self._source_stop_event.is_set():
                    self._stop_event.set()
                self._accept_status()
                if not self._event_reader.poll(0.02):
                    if self._process.exitcode is not None:
                        raise RuntimeError(f"导纳进程退出但未交付终止标记，exitcode={self._process.exitcode}")
                    continue
                encoded = self._event_reader.recv_bytes()
                envelope = pickle.loads(encoded)
                operation, payload = envelope["operation"], envelope["payload"]
                bridge = {"sequence": envelope["bridge_seq"], "source_pid": envelope["source_pid"],
                          "received_monotonic_s": time.monotonic()}
                if operation == "record":
                    if self._log_sink is not None:
                        self._log_sink.record(payload["kind"], {**payload["payload"], "process_bridge": bridge})
                elif operation == "context":
                    if self._log_sink is not None:
                        self._log_sink.update_context(payload)
                elif operation == "terminal":
                    summary = payload["summary"]
                    status = {**payload["status"], "process_bridge": summary,
                              "process_bridge_dropped_count": summary["dropped_count"],
                              "process_status_received_monotonic_s": time.monotonic()}
                    if self._log_sink is not None:
                        if summary["pending_gap"] is not None:
                            self._log_sink.record("process_bridge_gap", summary["pending_gap"])
                        self._log_sink.update_context({"process_bridge": summary})
                        self._log_sink.record("process_bridge_complete", {"summary": summary})
                    with self._lock:
                        self._status = status
                    self._drained_ack.set()
                    self._finished.set()
                    return
        except Exception as exc:
            self._stop_event.set()
            self._worker_failed.set()
            self._bridge_failed.set()
            with self._lock:
                self._status = {**self._status, "active": True, "running": False, "starting": False,
                                "stopping": True, "stopped_confirmed": False,
                                "last_error": f"导纳进程桥接错误（{type(exc).__name__}）：{exc}",
                                "process_worker_failed": True}
        finally:
            self._event_reader.close()
            self._bridge_finished.set()
