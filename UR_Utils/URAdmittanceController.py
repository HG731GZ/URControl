from __future__ import annotations

from collections import OrderedDict
from copy import deepcopy
import threading
import time
from typing import Optional, Sequence

import numpy as np
from rtde.rtde import RTDE

from UR_Utils import admittance_config
from UR_Utils.URScriptClient import URScriptClient


PHASE_NAMES = {0: "启动中", 1: "静止偏置采样", 2: "运行中", 3: "减速停止中", 4: "已停止"}
EXIT_REASONS = {
    0: "正常", 1: "请求停止", 2: "主机心跳超时", 3: "主端数据超时",
    4: "运行模式变化", 5: "逆解不可达", 6: "末端修正过大",
    7: "关节跟踪误差过大", 8: "超出运动范围", 9: "关节目标不连续",
    10: "偏置采样期间机器人移动",
}


class URAdmittanceController:
    """机器人端导纳的专用连接；一名线程拥有全部 RTDE 收发及脚本上传。"""

    def __init__(
        self, robot_ip: str, reference_source: str, space: str, *,
        q_master: Optional[Sequence[float]] = None, source_seq: Optional[int] = None,
        config: Optional[dict] = None,
    ):
        if reference_source not in ("teleop", "free"):
            raise ValueError("参考来源必须为 teleop 或 free")
        if space not in ("joint", "cartesian"):
            raise ValueError("导纳空间必须为 joint 或 cartesian")
        self.robot_ip = robot_ip
        self.reference_source = reference_source
        self.space = space
        self.parameters = deepcopy(admittance_config.snapshot() if config is None else config)
        admittance_config.validate_config(self.parameters)
        self._script = admittance_config.render_script(reference_source, space, config=self.parameters)
        self._lock = threading.Lock()
        self._stop_requested = threading.Event()
        self._finished = threading.Event()
        self._wake = threading.Event()
        self._thread = None
        self._connection = None
        self._uploaded = False
        self._worker_failed = False
        self._seen_program = False
        self._baseline_registers = None
        self._active = True
        self._q_master = None
        self._source_seq = None
        self._sent_references = OrderedDict()
        self._heartbeat = 0
        self._last_sample_monotonic = None
        self._status = {
            "active": True, "running": False, "starting": True, "stopping": False,
            "stopped_confirmed": False, "phase": 0, "phase_name": PHASE_NAMES[0],
            "exit_reason": 0, "exit_reason_text": EXIT_REASONS[0], "last_error": None,
            "reference_source": reference_source, "space": space,
            "timestamp": None, "runtime_state": None, "actual_q": None,
            "actual_qd": None, "actual_TCP_pose": None, "safety_status_bits": None,
            "drive": None, "delta_q": None, "delta_x": None, "v_free": None,
            "q_cmd": None, "q_nom": None, "q_start": None, "q_master": None,
            "source_seq": None, "submitted_source_seq": None, "adopted_source_seq": None,
            "cycle_count": 0, "parameters": deepcopy(self.parameters),
            "robot_configuration": None,
        }
        if reference_source == "teleop":
            self.submit_reference(q_master, source_seq=source_seq)
        elif q_master is not None or source_seq is not None:
            raise ValueError("主动柔顺不接收主端参考或源序号")

    @property
    def active(self) -> bool:
        """停止确认和连接释放完成之前，仍保留本轮执行权。"""
        with self._lock:
            return self._active

    def start(self) -> None:
        """只启动后台工作线程；不会清除已到达的停止请求。"""
        self._thread = threading.Thread(target=self._run, name="URAdmittanceRTDE", daemon=True)
        self._thread.start()

    def cancel_before_start(self, error: str) -> None:
        """原控制通道尚未完成交接时终止本次启动，不上传脚本。"""
        self._set_error(error)
        self._complete()

    def submit_reference(self, q: Sequence[float], *, source_seq: int) -> None:
        """保存名义主端参考；相同帧身份不产生新的主端采样。"""
        if self.reference_source != "teleop":
            raise RuntimeError("主动柔顺期间不能提交主端关节目标")
        q_arr = np.asarray(q, dtype=float)
        if q_arr.shape != (6,) or not np.all(np.isfinite(q_arr)):
            raise ValueError("主端参考必须为六个有限关节角")
        if not isinstance(source_seq, (int, np.integer)) or isinstance(source_seq, bool) or source_seq < 0:
            raise ValueError("遥操作导纳必须提供非负整数源帧序号")
        seq = int(source_seq)
        with self._lock:
            if not self._active or self._stop_requested.is_set():
                raise RuntimeError("本轮导纳正在停止或已经结束")
            if self._source_seq is not None and seq < self._source_seq:
                raise ValueError("主端源帧序号不能倒退")
            if seq == self._source_seq and not np.array_equal(q_arr, self._q_master):
                raise ValueError("同一主端帧序号不能对应不同关节目标")
            self._q_master = q_arr.copy()
            self._source_seq = seq
            self._status["submitted_source_seq"] = seq

    def request_stop(self) -> None:
        """停止标志独立于启动锁，偏置采样和上传期间也不会丢失。"""
        self._stop_requested.set()
        self._wake.set()
        with self._lock:
            if self._active:
                self._status.update(stopping=True, starting=False, running=False)

    def stop(self) -> None:
        """等待程序退出和静止反馈；超时保留连接与执行权。"""
        self.request_stop()
        if not self._finished.wait(self.parameters["ADMITTANCE_STOP_TIMEOUT_S"]):
            message = "导纳停止尚未确认，保留专用连接，禁止恢复其他运动"
            if self.get_status()["last_error"] is None:
                self._set_error(message)
            raise RuntimeError(message)
        if self._thread is not None:
            self._thread.join()

    def get_status(self) -> dict:
        """返回一致的遥测快照；不在调用线程读写专用连接。"""
        with self._lock:
            return deepcopy(self._status)

    def needs_external_stop_confirmation(self) -> bool:
        """专用通信失败且工作线程退出后，允许原只读连接提供停止证据。"""
        return self._worker_failed and self._thread is not None and not self._thread.is_alive()

    def confirm_external_stop(self, sample: dict) -> None:
        """统一控制器已核实新的停止与静止反馈后，释放故障专用连接。"""
        if not self.needs_external_stop_confirmation():
            raise RuntimeError("工作线程尚未退出，不能由其他线程释放专用连接")
        with self._lock:
            self._status.update(sample)
        self._complete()

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._status["last_error"] = message
            self._status["running"] = False

    def _complete(self) -> None:
        """仅在上传前取消，或确认机器人程序退出后释放本轮资源。"""
        if self._connection is not None:
            self._connection.disconnect()
            self._connection = None
        with self._lock:
            self._active = False
            self._status.update(active=False, running=False, starting=False, stopping=False,
                                stopped_confirmed=True, phase=4, phase_name=PHASE_NAMES[4])
        self._finished.set()

    def _setup_connection(self):
        cfg = self.parameters
        conn = RTDE(self.robot_ip)
        self._connection = conn
        conn.connect()
        input_names = [f"input_int_register_{cfg['ADMITTANCE_INPUT_INT_BASE'] + i}" for i in range(5)]
        input_names += [f"input_double_register_{cfg['ADMITTANCE_INPUT_FLOAT_BASE'] + i}" for i in range(6)]
        packet = conn.send_input_setup(input_names, ["INT32"] * 5 + ["DOUBLE"] * 6)
        if packet is None:
            raise RuntimeError("导纳输入寄存器配方建立失败，请检查其他客户端或 URCap 占用")
        output_names = ["timestamp", "runtime_state", "actual_q", "actual_qd", "actual_TCP_pose",
                        "safety_status_bits", "tcp_offset", "payload", "payload_cog", "payload_inertia"]
        output_types = ["DOUBLE", "UINT32", "VECTOR6D", "VECTOR6D", "VECTOR6D", "UINT32",
                        "VECTOR6D", "DOUBLE", "VECTOR3D", "VECTOR6D"]
        output_names += [f"output_int_register_{cfg['ADMITTANCE_OUTPUT_INT_BASE'] + i}" for i in range(5)]
        output_types += ["INT32"] * 5
        output_names += [f"output_double_register_{cfg['ADMITTANCE_OUTPUT_FLOAT_BASE'] + i}" for i in range(24)]
        output_types += ["DOUBLE"] * 24
        if not conn.send_output_setup(output_names, output_types, frequency=cfg["ADMITTANCE_STATE_HZ"]):
            raise RuntimeError("导纳状态配方建立失败")
        if not conn.send_start():
            raise RuntimeError("导纳 RTDE 同步启动失败")
        return packet

    def _send_inputs(self, packet, *, heartbeat: bool) -> bool:
        cfg = self.parameters
        if heartbeat:
            self._heartbeat = (self._heartbeat + 1) % (2 ** 31)
        with self._lock:
            seq = self._source_seq
            q = None if self._q_master is None else self._q_master.copy()
        wire_seq = 0 if seq is None else seq % (2 ** 31)
        values = [int(not self._stop_requested.is_set()), self._heartbeat, wire_seq,
                  1 if self.space == "joint" else 2, 1 if self.reference_source == "teleop" else 2]
        for i, value in enumerate(values):
            setattr(packet, f"input_int_register_{cfg['ADMITTANCE_INPUT_INT_BASE'] + i}", value)
        for i in range(6):
            setattr(packet, f"input_double_register_{cfg['ADMITTANCE_INPUT_FLOAT_BASE'] + i}",
                    0.0 if q is None else float(q[i]))
        if not self._connection.send(packet):
            raise RuntimeError("导纳参考与心跳发送失败")
        if q is not None:
            self._sent_references[wire_seq] = (seq, q)
            self._sent_references.move_to_end(wire_seq)
            while len(self._sent_references) > 512:
                self._sent_references.popitem(last=False)
        return values[0] == 0

    def _receive_latest(self):
        """使用官方非阻塞读取，避免状态断流阻塞参考、心跳和停止发送。"""
        latest = None
        for _ in range(1024):
            state = self._connection.receive_buffered()
            if state is None:
                break
            latest = state
            self._consume_state(state)
        return latest

    def _consume_state(self, state) -> None:
        cfg = self.parameters
        ints = [int(getattr(state, f"output_int_register_{cfg['ADMITTANCE_OUTPUT_INT_BASE'] + i}")) for i in range(5)]
        values = [float(getattr(state, f"output_double_register_{cfg['ADMITTANCE_OUTPUT_FLOAT_BASE'] + i}")) for i in range(24)]
        phase, reason, adopted, cycles, source = ints
        runtime = int(state.runtime_state)
        self._last_sample_monotonic = time.monotonic()
        if not self._uploaded:
            self._baseline_registers = tuple(ints)
        if self._uploaded and runtime == 2 and source == (1 if self.reference_source == "teleop" else 2):
            # 只接受本轮初始化的低周期计数，避免上轮残留运行寄存器提前解锁界面。
            initial_cycles = 0 <= cycles <= int((cfg["ADMITTANCE_BIAS_DURATION_S"] + 0.1) * 500) + 1
            initializing = phase in (0, 1) and initial_cycles
            cancelled_at_start = (phase == 3 and initial_cycles
                                  and tuple(ints) != self._baseline_registers)
            if initializing or cancelled_at_start:
                self._seen_program = True
        entry = self._sent_references.get(adopted) if self.reference_source == "teleop" else None
        with self._lock:
            self._status.update(
                timestamp=float(state.timestamp), runtime_state=runtime,
                actual_q=list(state.actual_q), actual_qd=list(state.actual_qd),
                actual_TCP_pose=list(state.actual_TCP_pose), safety_status_bits=int(state.safety_status_bits),
            )
            if self._status["robot_configuration"] is None:
                self._status["robot_configuration"] = {
                    "tcp_offset": list(state.tcp_offset), "payload": float(state.payload),
                    "payload_cog": list(state.payload_cog), "payload_inertia": list(state.payload_inertia),
                }
            if not self._seen_program:
                return
            self._status.update(
                phase=phase, phase_name=PHASE_NAMES.get(phase, "未知阶段"), exit_reason=reason,
                exit_reason_text=EXIT_REASONS.get(reason, f"退出码 {reason}"),
                running=phase == 2 and runtime == 2 and not self._stop_requested.is_set(),
                starting=phase < 2 and not self._stop_requested.is_set(),
                stopping=phase == 3 or self._stop_requested.is_set(),
                drive=values[:6], q_cmd=values[12:18], cycle_count=cycles,
                adopted_source_seq=None if self.reference_source == "free" else adopted,
            )
            if self.reference_source == "teleop":
                self._status.update(q_nom=values[18:24], q_master=None if entry is None else entry[1].tolist(),
                                    source_seq=None if entry is None else entry[0])
                self._status["delta_q" if self.space == "joint" else "delta_x"] = values[6:12]
            else:
                self._status.update(q_start=values[18:24], v_free=values[6:12])
            if reason > 1:
                self._status["last_error"] = EXIT_REASONS.get(reason, f"导纳异常退出：{reason}")

    def _run(self) -> None:
        cfg = self.parameters
        try:
            if self._stop_requested.is_set():
                self._complete()
                return
            packet = self._setup_connection()
            if self._stop_requested.is_set():
                self._complete()
                return
            # 上传前读取实际状态，确认静止、同步以及本轮 TCP 和负载配置。
            baseline = self._connection.receive()
            if baseline is None:
                raise RuntimeError("导纳启动前未收到实际状态")
            self._consume_state(baseline)
            if baseline.runtime_state != 1:
                raise RuntimeError("原运动程序尚未退出，不能上传导纳程序")
            if max(abs(v) for v in baseline.actual_qd) > cfg["STATIONARY_MAX_QD_RAD_S"]:
                raise RuntimeError("机器人尚未静止，不能开始导纳偏置采样")
            if self.reference_source == "teleop":
                with self._lock:
                    sync_error = np.max(np.abs(self._q_master - np.asarray(baseline.actual_q)))
                if sync_error > cfg["TELEOP_SYNC_TOLERANCE_RAD"]:
                    raise RuntimeError("主从关节尚未同步，不能开启遥操作导纳")
            self._send_inputs(packet, heartbeat=True)
            if self._stop_requested.is_set():
                self._complete()
                return
            with URScriptClient(self.robot_ip) as script_client:
                # 从开始上传起，任何发送错误都按程序可能已经执行处理。
                self._uploaded = True
                script_client.send_script(self._script)
            upload_time = time.monotonic()
            next_reference = upload_time
            next_heartbeat = upload_time
            stop_sent = False
            stopped_after_timestamp = None
            while True:
                now = time.monotonic()
                heartbeat_due = now >= next_heartbeat
                reference_due = self.reference_source == "teleop" and now >= next_reference
                stop_due = self._stop_requested.is_set() and not stop_sent
                if heartbeat_due or reference_due or stop_due:
                    before_send_timestamp = self.get_status()["timestamp"]
                    stop_sent = self._send_inputs(packet, heartbeat=heartbeat_due)
                    if stop_sent and stopped_after_timestamp is None:
                        stopped_after_timestamp = before_send_timestamp
                    if heartbeat_due:
                        next_heartbeat = now + 1.0 / cfg["ADMITTANCE_HEARTBEAT_HZ"]
                    if reference_due:
                        next_reference = now + 1.0 / cfg["TELEOP_REFERENCE_HZ"]
                state = self._receive_latest()
                startup_cancelled = (
                    stop_sent and stopped_after_timestamp is not None
                    and now - upload_time >= cfg["ADMITTANCE_START_TIMEOUT_S"]
                    and state is not None and state.timestamp > stopped_after_timestamp
                )
                if state is not None and (self._seen_program or startup_cancelled) and state.runtime_state == 1:
                    if max(abs(v) for v in state.actual_qd) <= cfg["STATIONARY_MAX_QD_RAD_S"]:
                        if self.get_status()["phase"] != 4 and self.get_status()["last_error"] is None:
                            self._set_error("导纳程序已被控制器终止，程序退出和机器人静止已确认")
                        self._complete()
                        return
                if not self._stop_requested.is_set():
                    if not self._seen_program and now - upload_time > cfg["ADMITTANCE_START_TIMEOUT_S"]:
                        self._set_error("导纳启动未得到机器人确认，正在请求停止")
                        self.request_stop()
                    elif now - self._last_sample_monotonic > cfg["ADMITTANCE_STATE_TIMEOUT_S"]:
                        self._set_error("导纳状态接收超时，正在请求停止")
                        self.request_stop()
                self._wake.wait(min(0.005, 1.0 / cfg["ADMITTANCE_STATE_HZ"]))
                self._wake.clear()
        except Exception as exc:
            # 通信异常只记录一次；上传后失去停止确认时保留执行权，不重连或恢复旧程序。
            self._set_error(f"导纳后台错误：{exc}")
            self.request_stop()
            if not self._uploaded:
                self._complete()
            else:
                self._worker_failed = True
