from __future__ import annotations

from collections import OrderedDict
from copy import deepcopy
import threading
import time
from typing import Optional, Sequence

import numpy as np
from rtde.rtde import RTDE

from UR_Utils import admittance_config, admittance_telemetry
from UR_Utils.URScriptClient import URScriptClient


PHASE_NAMES = {0: "启动中", 1: "静止偏置采样", 2: "运行中", 3: "减速停止中", 4: "已停止"}
EXIT_REASONS = {
    0: "正常", 1: "请求停止", 2: "主机心跳超时", 3: "主端数据超时",
    4: "运行模式变化", 5: "逆解不可达", 6: "末端修正过大",
    7: "关节跟踪误差过大", 8: "超出运动范围", 9: "关节目标不连续",
    10: "偏置采样期间机器人移动",
}


def idle_admittance_status() -> dict:
    """返回完整的未开启快照；未产生的运行样本明确保留为空值。"""
    return {
        "active": False, "running": False, "starting": False, "stopping": False,
        "stopped_confirmed": True, "phase": 4, "phase_name": "未开启",
        "exit_reason": 0, "exit_reason_text": EXIT_REASONS[0], "last_error": None,
        "reference_source": None, "space": None,
        "timestamp": None, "runtime_state": None, "actual_q": None,
        "actual_qd": None, "actual_TCP_pose": None, "safety_status_bits": None,
        "robot_mode": None, "safety_mode": None, "robot_status_bits": None,
        "speed_scaling": None, "target_speed_fraction": None,
        "actual_TCP_speed": None, "actual_TCP_force": None, "actual_current": None,
        "joint_temperatures": None, "target_q": None, "target_qd": None,
        "actual_execution_time": None, "tcp_offset": None, "payload": None,
        "payload_cog": None, "payload_inertia": None, "program_seen": False,
        "drive": None, "delta_q": None, "delta_x": None,
        "q_cmd": None, "q_nom": None, "q_start": None, "q_master": None,
        "q_master_submitted": None, "source_seq": None,
        "submitted_source_seq": None, "adopted_source_seq": None,
        "cycle_count": None, "parameters": None, "robot_configuration": None,
        "state_received_monotonic": None, "state_receive_count": 0,
        "last_reference_sent_monotonic": None, "reference_send_count": 0,
        "last_heartbeat_sent_monotonic": None, "heartbeat_send_count": 0,
        "heartbeat_echo": None, "heartbeat_age_us": None, "heartbeat_max_interval_us": None,
        "heartbeat_send_interval_s": None, "heartbeat_send_max_interval_s": None,
        "heartbeat_send_lateness_s": None, "heartbeat_send_max_lateness_s": None,
        "heartbeat_deadline_miss_count": 0,
        "processing_duration_s": None, "processing_max_duration_s": None,
        "rtde_processing_duration_s": None, "primary_processing_duration_s": None,
    }


class URAdmittanceController:
    """机器人端导纳的专用连接；一名线程拥有全部 RTDE 收发及脚本上传。"""

    def __init__(
        self, robot_ip: str, reference_source: str, space: str, *,
        q_master: Optional[Sequence[float]] = None, source_seq: Optional[int] = None,
        config: Optional[dict] = None, stop_event: Optional[threading.Event] = None,
        log_sink=None,
    ):
        if reference_source not in ("teleop", "fixed"):
            raise ValueError("参考来源必须为 teleop 或 fixed")
        if space not in ("joint", "cartesian"):
            raise ValueError("导纳空间必须为 joint 或 cartesian")
        self.robot_ip = robot_ip
        self.reference_source = reference_source
        self.space = space
        self._log_sink = log_sink
        self.parameters = deepcopy(admittance_config.snapshot() if config is None else config)
        admittance_config.validate_run_config(reference_source, space, config=self.parameters)
        self._script = admittance_config.render_script(reference_source, space, config=self.parameters)
        self._lock = threading.Lock()
        self._stop_requested = threading.Event() if stop_event is None else stop_event
        self._stop_logged = False
        self._finished = threading.Event()
        self._wake = threading.Event()
        self._thread = None
        self._connection = None
        self._script_client = None
        self._script_error_received = False
        self._uploaded = False
        self._worker_failed = False
        self._seen_program = False
        self._baseline_registers = None
        self._last_registers = None
        self._active = True
        self._q_master = None
        self._source_seq = None
        self._sent_references = OrderedDict()
        self._heartbeat = 0
        self._last_sample_monotonic = None
        self._status = idle_admittance_status()
        self._status.update(
            active=True, starting=not self._stop_requested.is_set(),
            stopping=self._stop_requested.is_set(), stopped_confirmed=False,
            phase=0, phase_name=PHASE_NAMES[0], reference_source=reference_source,
            space=space, parameters=deepcopy(self.parameters),
        )
        if reference_source == "teleop":
            # 构造时保存启动参考；此时到达的停止标志仍由启动线程直接执行取消。
            self._q_master, self._source_seq = self._validated_reference(q_master, source_seq)
            self._status["q_master_submitted"] = self._q_master.tolist()
            self._status["submitted_source_seq"] = self._source_seq
        elif q_master is not None or source_seq is not None:
            raise ValueError("定点柔顺不接收主端参考或源序号")
        controller_context = {
            "robot_ip": robot_ip, "reference_source": reference_source, "space": space,
            "parameters": deepcopy(self.parameters),
            "units": admittance_config.units(reference_source, space, config=self.parameters),
            "telemetry_schema": admittance_telemetry.log_schema(self.parameters, space),
        }
        if self._log_sink is not None:
            # 固定上下文在关闭日志时也保留，供中途首次开启准确恢复本轮配置及脚本。
            self._log_sink.update_context({
                "controller": {**deepcopy(controller_context), "script_source": self._script},
            })
        self._record("controller_created", {
            **controller_context,
            "status": self.get_status(),
        })
        if self.reference_source == "teleop":
            self._record("reference_submitted", {
                "source_seq": self._source_seq, "q_master": self._q_master.tolist(), "initial": True,
            })

    def _record(self, kind: str, payload: dict, *, host_monotonic_s=None, host_time_unix_s=None) -> None:
        """仅向日志线程交付独立数据；调用方必须先释放状态锁。"""
        if self._log_sink is not None:
            self._log_sink.record(kind, {
                "host_monotonic_s": time.monotonic() if host_monotonic_s is None else host_monotonic_s,
                "host_time_unix_s": time.time() if host_time_unix_s is None else host_time_unix_s,
                **payload,
            })

    def _control_snapshot_locked(self) -> dict:
        """在状态锁内复制控制状态，避免日志保留后续仍会变化的对象。"""
        raw_fields = {field[0] for field in admittance_telemetry.ROBOT_FIELDS}
        return deepcopy({key: value for key, value in self._status.items()
                         if key not in raw_fields and key not in ("parameters", "robot_configuration")})

    @property
    def active(self) -> bool:
        """停止确认和连接释放完成之前，仍保留本轮执行权。"""
        with self._lock:
            return self._active

    def start(self) -> None:
        """只启动后台工作线程；不会清除已到达的停止请求。"""
        self._record("controller_start", {"status": self.get_status()})
        self._thread = threading.Thread(target=self._run, name="URAdmittanceRTDE", daemon=True)
        self._thread.start()

    def cancel_before_start(self, error: Optional[str] = None) -> None:
        """原控制通道尚未完成交接时终止本次启动，不上传脚本。"""
        if error is not None:
            self._set_error(error)
        elif self._stop_requested.is_set():
            with self._lock:
                self._status.update(exit_reason=1, exit_reason_text=EXIT_REASONS[1])
        self._record("controller_cancelled_before_start", {"status": self.get_status()})
        self._complete()

    @staticmethod
    def _validated_reference(q: Sequence[float], source_seq: int):
        """校验主端参考与本机有效接收序号，并返回独立目标副本。"""
        q_arr = np.asarray(q, dtype=float)
        if q_arr.shape != (6,) or not np.all(np.isfinite(q_arr)):
            raise ValueError("主端参考必须为六个有限关节角")
        if not isinstance(source_seq, (int, np.integer)) or isinstance(source_seq, bool) or source_seq < 0:
            raise ValueError("遥操作导纳必须提供非负整数源帧序号")
        return q_arr.copy(), int(source_seq)

    def submit_reference(self, q: Sequence[float], *, source_seq: int) -> None:
        """保存名义主端参考；相同帧身份不产生新的主端采样。"""
        if self.reference_source != "teleop":
            raise RuntimeError("定点柔顺期间不能提交主端关节目标")
        q_arr, seq = self._validated_reference(q, source_seq)
        with self._lock:
            if not self._active or self._stop_requested.is_set():
                raise RuntimeError("本轮导纳正在停止或已经结束")
            if self._source_seq is not None and seq < self._source_seq:
                raise ValueError("主端源帧序号不能倒退")
            if seq == self._source_seq and not np.array_equal(q_arr, self._q_master):
                raise ValueError("同一主端帧序号不能对应不同关节目标")
            new_sample = seq != self._source_seq
            self._q_master = q_arr.copy()
            self._source_seq = seq
            self._status["submitted_source_seq"] = seq
            self._status["q_master_submitted"] = q_arr.tolist()
            submitted_monotonic = time.monotonic()
            submitted_wall_time = time.time()
        if new_sample:
            self._record("reference_submitted", {
                "source_seq": seq, "q_master": q_arr.tolist(), "initial": False,
            }, host_monotonic_s=submitted_monotonic, host_time_unix_s=submitted_wall_time)

    def request_stop(self) -> None:
        """停止标志独立于启动锁，偏置采样和上传期间也不会丢失。"""
        requested_monotonic = time.monotonic()
        requested_wall_time = time.time()
        self._stop_requested.set()
        self._wake.set()
        with self._lock:
            if self._active:
                self._status.update(stopping=True, starting=False, running=False)
            log_request = not self._stop_logged
            self._stop_logged = True
            status = self._control_snapshot_locked() if log_request else None
        if log_request:
            self._record("stop_requested", {"status": status},
                         host_monotonic_s=requested_monotonic, host_time_unix_s=requested_wall_time)

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
        self._record("external_stop_confirmed", {"sample": deepcopy(sample), "status": self.get_status()})
        self._complete()

    def _set_error(self, message: str) -> None:
        with self._lock:
            self._status["last_error"] = message
            self._status["running"] = False
            status = self._control_snapshot_locked()
        self._record("controller_error", {"message": message, "status": status})

    def _complete(self) -> None:
        """仅在上传前取消，或确认机器人程序退出后释放本轮资源。"""
        if self._script_client is not None:
            self._script_client.close()
            self._script_client = None
        if self._connection is not None:
            self._connection.disconnect()
            self._connection = None
        complete_flags = dict(active=False, running=False, starting=False, stopping=False,
                              stopped_confirmed=True, phase=4, phase_name=PHASE_NAMES[4])
        status = self.get_status()
        status.update(complete_flags)
        # 完成事件先入队，再让界面看到 inactive，确保界面结束日志时事件已提交。
        self._record("controller_complete", {"status": status})
        with self._lock:
            self._active = False
            self._status.update(complete_flags)
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
        output_names, output_types = admittance_telemetry.output_recipe(cfg)
        if not conn.send_output_setup(output_names, output_types, frequency=cfg["ADMITTANCE_STATE_HZ"]):
            raise RuntimeError("导纳状态配方建立失败")
        if not conn.send_start():
            raise RuntimeError("导纳 RTDE 同步启动失败")
        connection_context = {
            "state_frequency_hz": cfg["ADMITTANCE_STATE_HZ"],
            "input_recipe": dict(zip(input_names, ["INT32"] * 5 + ["DOUBLE"] * 6)),
            "output_recipe": dict(zip(output_names, output_types)),
        }
        if self._log_sink is not None:
            self._log_sink.update_context({"rtde": deepcopy(connection_context)})
        self._record("connection_ready", connection_context)
        return packet

    def _send_inputs(self, packet, *, heartbeat: bool, heartbeat_deadline: Optional[float] = None) -> bool:
        cfg = self.parameters
        if heartbeat:
            self._heartbeat = (self._heartbeat + 1) % (2 ** 31)
        with self._lock:
            seq = self._source_seq
            q = None if self._q_master is None else self._q_master.copy()
        wire_seq = -1 if seq is None else seq % (2 ** 31)
        values = [int(not self._stop_requested.is_set()), self._heartbeat, wire_seq,
                  1 if self.space == "joint" else 2, 1 if self.reference_source == "teleop" else 3]
        wire_q = [0.0] * 6 if q is None else q.tolist()
        for i, value in enumerate(values):
            setattr(packet, f"input_int_register_{cfg['ADMITTANCE_INPUT_INT_BASE'] + i}", value)
        for i in range(6):
            setattr(packet, f"input_double_register_{cfg['ADMITTANCE_INPUT_FLOAT_BASE'] + i}",
                    wire_q[i])
        send_started = time.monotonic()
        if not self._connection.send(packet):
            raise RuntimeError("导纳参考与心跳发送失败")
        sent_monotonic = time.monotonic()
        sent_wall_time = time.time()
        with self._lock:
            # 参考计数统计成功携带关节参考的发送包；心跳计数统计心跳递增包。
            if q is not None:
                self._status["last_reference_sent_monotonic"] = sent_monotonic
                self._status["reference_send_count"] += 1
            if heartbeat:
                previous_sent = self._status["last_heartbeat_sent_monotonic"]
                if previous_sent is not None:
                    interval = sent_monotonic - previous_sent
                    self._status["heartbeat_send_interval_s"] = interval
                    previous_max = self._status["heartbeat_send_max_interval_s"]
                    self._status["heartbeat_send_max_interval_s"] = interval if previous_max is None else max(previous_max, interval)
                if heartbeat_deadline is not None:
                    lateness = max(0.0, sent_monotonic - heartbeat_deadline)
                    self._status["heartbeat_send_lateness_s"] = lateness
                    previous_max = self._status["heartbeat_send_max_lateness_s"]
                    self._status["heartbeat_send_max_lateness_s"] = lateness if previous_max is None else max(previous_max, lateness)
                    # 仅把跨过完整发送周期的迟到计为超期；微小调度抖动由迟到时长记录。
                    self._status["heartbeat_deadline_miss_count"] += int(lateness * cfg["ADMITTANCE_HEARTBEAT_HZ"])
                self._status["last_heartbeat_sent_monotonic"] = sent_monotonic
                self._status["heartbeat_send_count"] += 1
            timing = {key: self._status[key] for key in (
                "last_reference_sent_monotonic", "reference_send_count",
                "last_heartbeat_sent_monotonic", "heartbeat_send_count",
                "heartbeat_send_interval_s", "heartbeat_send_max_interval_s",
                "heartbeat_send_lateness_s", "heartbeat_send_max_lateness_s",
                "heartbeat_deadline_miss_count", "processing_duration_s", "processing_max_duration_s",
                "rtde_processing_duration_s", "primary_processing_duration_s",
            )} if self._log_sink is not None else None
        if q is not None:
            self._sent_references[wire_seq] = (seq, q)
            self._sent_references.move_to_end(wire_seq)
            while len(self._sent_references) > 512:
                self._sent_references.popitem(last=False)
        if self._log_sink is not None:
            registers = {f"input_int_register_{cfg['ADMITTANCE_INPUT_INT_BASE'] + i}": value
                         for i, value in enumerate(values)}
            registers.update({f"input_double_register_{cfg['ADMITTANCE_INPUT_FLOAT_BASE'] + i}": value
                              for i, value in enumerate(wire_q)})
            self._record("rtde_input", {
                "raw": registers,
                "enable": values[0], "heartbeat": self._heartbeat, "heartbeat_advanced": heartbeat,
                "source_seq": seq, "wire_source_seq": wire_seq,
                "space": self.space, "reference_source": self.reference_source,
                "q_master": None if q is None else q.tolist(), "wire_q": wire_q,
                "send_started_monotonic_s": send_started,
                "send_duration_s": sent_monotonic - send_started,
                "heartbeat_deadline_monotonic_s": heartbeat_deadline, "timing": timing,
            }, host_monotonic_s=sent_monotonic, host_time_unix_s=sent_wall_time)
        return values[0] == 0

    def _receive_latest(self, deadline: float, *, stop_sent: bool):
        """按时间和内部缓存预算读取，不在积压数据中长期占用发送线程。"""
        latest = None
        while time.monotonic() < deadline:
            if self._stop_requested.is_set() and not stop_sent:
                break
            # 官方实现会先排空 socket，再返回一帧。buffer_limit 阻止继续读取，
            # 不丢弃已有帧；内部单次 recv 为 4096 字节，阈值最多越过 4095 字节。
            state = self._connection.receive_buffered(
                buffer_limit=self.parameters["ADMITTANCE_RTDE_BUFFER_LIMIT_BYTES"])
            if state is None:
                break
            latest = state
            self._consume_state(state)
        return latest

    def _consume_state(self, state) -> None:
        received_monotonic = time.monotonic()
        received_wall_time = time.time()
        cfg = self.parameters
        robot = admittance_telemetry.robot_sample(state)
        ints = [int(getattr(state, f"output_int_register_{cfg['ADMITTANCE_OUTPUT_INT_BASE'] + i}")) for i in range(8)]
        values = [float(getattr(state, f"output_double_register_{cfg['ADMITTANCE_OUTPUT_FLOAT_BASE'] + i}")) for i in range(24)]
        phase, reason, adopted, cycles, source, heartbeat_echo, heartbeat_age, heartbeat_max_interval = ints
        self._last_registers = tuple(ints)
        runtime = int(state.runtime_state)
        self._last_sample_monotonic = received_monotonic
        was_seen_program = self._seen_program
        if not self._uploaded:
            self._baseline_registers = tuple(ints[:5])
        source_matches = source == (1 if self.reference_source == "teleop" else 3)
        if self._uploaded and runtime == 2 and source_matches:
            # 只接受本轮初始化的低周期计数，避免上轮残留运行寄存器提前解锁界面。
            initial_cycles = 0 <= cycles <= int((cfg["ADMITTANCE_BIAS_DURATION_S"] + 0.1) * 500) + 1
            fresh_registers = tuple(ints[:5]) != self._baseline_registers
            initializing = phase in (0, 1) and initial_cycles and fresh_registers
            cancelled_at_start = (phase == 3 and initial_cycles
                                  and fresh_registers)
            if initializing or cancelled_at_start:
                self._seen_program = True
        entry = self._sent_references.get(adopted) if self.reference_source == "teleop" else None
        with self._lock:
            old_phase = self._status["phase"]
            old_reason = self._status["exit_reason"]
            self._status.update(deepcopy(robot))
            self._status.update(
                state_received_monotonic=self._last_sample_monotonic,
                program_seen=self._seen_program,
            )
            self._status["state_receive_count"] += 1
            configuration = None
            if self._status["robot_configuration"] is None:
                self._status["robot_configuration"] = {
                    "tcp_offset": list(state.tcp_offset), "payload": float(state.payload),
                    "payload_cog": list(state.payload_cog), "payload_inertia": list(state.payload_inertia),
                    "timestamp": float(state.timestamp), "received_monotonic": self._last_sample_monotonic,
                }
                configuration = deepcopy(self._status["robot_configuration"])
            derived_valid = self._seen_program and source_matches
            if derived_valid:
                self._status.update(
                    phase=phase, phase_name=PHASE_NAMES.get(phase, "未知阶段"), exit_reason=reason,
                    exit_reason_text=EXIT_REASONS.get(reason, f"退出码 {reason}"),
                    running=phase == 2 and runtime == 2 and not self._stop_requested.is_set(),
                    starting=phase < 2 and not self._stop_requested.is_set(),
                    stopping=phase == 3 or self._stop_requested.is_set(),
                    drive=values[:6], q_cmd=values[12:18], q_nom=values[18:24], cycle_count=cycles,
                    adopted_source_seq=None if self.reference_source == "fixed" else adopted,
                    heartbeat_echo=heartbeat_echo, heartbeat_age_us=heartbeat_age,
                    heartbeat_max_interval_us=heartbeat_max_interval,
                )
                self._status["delta_q" if self.space == "joint" else "delta_x"] = values[6:12]
                if self.reference_source == "teleop":
                    self._status.update(q_master=None if entry is None else entry[1].tolist(),
                                        source_seq=None if entry is None else entry[0])
                elif self._status["q_start"] is None and reason == 0 and phase in (0, 1, 2):
                    # 启动检查通过的本轮状态才包含已锁定锚点；停止阶段不补抓位置。
                    self._status["q_start"] = values[18:24]
                if reason > 1:
                    self._status["last_error"] = EXIT_REASONS.get(reason, f"导纳异常退出：{reason}")
            control = self._control_snapshot_locked() if self._log_sink is not None else None
        if self._log_sink is not None:
            raw = robot
            raw.update({f"output_int_register_{cfg['ADMITTANCE_OUTPUT_INT_BASE'] + i}": value
                        for i, value in enumerate(ints)})
            raw.update({f"output_double_register_{cfg['ADMITTANCE_OUTPUT_FLOAT_BASE'] + i}": value
                        for i, value in enumerate(values)})
            self._record("rtde_state", {
                "raw": raw, "program_seen": self._seen_program,
                "source_matches": source_matches, "derived_valid": derived_valid,
                "control": control,
            }, host_monotonic_s=received_monotonic, host_time_unix_s=received_wall_time)
            if configuration is not None:
                self._record("robot_configuration", {"configuration": configuration})
            if derived_valid and (phase != old_phase or reason != old_reason or not was_seen_program):
                self._record("controller_phase", {"status": control})
            if derived_valid and reason > 1 and reason != old_reason:
                self._record("controller_error", {"message": control["last_error"], "status": control})

    @staticmethod
    def _advance_deadline(deadline: float, period: float, now: float) -> float:
        """保持原来的绝对时间网格，错过周期后跳到未来，不连续补发旧心跳。"""
        return deadline + (int((now - deadline) / period) + 1) * period

    def _record_processing_duration(self, started: float) -> None:
        """只统计收发、解析和生命周期处理的耗时，不计线程等待。"""
        duration = time.monotonic() - started
        with self._lock:
            self._status["processing_duration_s"] = duration
            previous_max = self._status["processing_max_duration_s"]
            self._status["processing_max_duration_s"] = duration if previous_max is None else max(previous_max, duration)

    def _run(self) -> None:
        cfg = self.parameters
        try:
            if self._stop_requested.is_set():
                self.cancel_before_start()
                return
            packet = self._setup_connection()
            if self._stop_requested.is_set():
                self.cancel_before_start()
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
                self.cancel_before_start()
                return
            # Primary 会向上传客户端返回编译/运行错误；保留连接并持续读取，
            # 直到停止确认，避免发完即关以及漏掉脚本被控制器拒绝的具体原因。
            self._script_client = URScriptClient(self.robot_ip, port=30001)
            self._script_client.connect()
            if self._stop_requested.is_set():
                self.cancel_before_start()
                return
            # 从开始上传起，任何发送错误都按程序可能已经执行处理。
            self._record("script_upload_requested", {
                "port": 30001, "script_source": self._script,
                "robot_configuration": self.get_status()["robot_configuration"],
            })
            self._uploaded = True
            self._script_client.send_script(self._script)
            upload_time = time.monotonic()
            self._record("script_uploaded", {"port": 30001}, host_monotonic_s=upload_time)
            next_reference = upload_time
            next_heartbeat = upload_time
            heartbeat_period = 1.0 / cfg["ADMITTANCE_HEARTBEAT_HZ"]
            reference_period = 1.0 / cfg["TELEOP_REFERENCE_HZ"]
            stop_sent = False
            stopped_after_timestamp = None
            while True:
                processing_started = time.monotonic()
                try:
                    now = processing_started
                    heartbeat_due = now >= next_heartbeat
                    reference_due = self.reference_source == "teleop" and now >= next_reference
                    stop_due = self._stop_requested.is_set() and not stop_sent
                    # 每批接收结束后都先回到这里，优先发送停止和到期心跳。
                    if heartbeat_due or reference_due or stop_due:
                        with self._lock:
                            before_send_timestamp = self._status["timestamp"]
                        stop_sent = self._send_inputs(
                            packet, heartbeat=heartbeat_due,
                            heartbeat_deadline=next_heartbeat if heartbeat_due else None)
                        sent_at = time.monotonic()
                        if stop_sent and stopped_after_timestamp is None:
                            stopped_after_timestamp = before_send_timestamp
                        if heartbeat_due:
                            next_heartbeat = self._advance_deadline(next_heartbeat, heartbeat_period, sent_at)
                        if reference_due:
                            next_reference = self._advance_deadline(next_reference, reference_period, sent_at)
                    next_send = min(next_heartbeat, next_reference) if self.reference_source == "teleop" else next_heartbeat
                    rtde_started = time.monotonic()
                    state = self._receive_latest(
                        min(next_send, rtde_started + cfg["ADMITTANCE_RTDE_READ_BUDGET_S"]),
                        stop_sent=stop_sent)
                    with self._lock:
                        self._status["rtde_processing_duration_s"] = time.monotonic() - rtde_started
                    now = time.monotonic()
                    startup_cancelled = (
                        stop_sent and stopped_after_timestamp is not None
                        and (self._script_error_received
                             or now - upload_time >= cfg["ADMITTANCE_START_TIMEOUT_S"])
                        and state is not None and state.timestamp > stopped_after_timestamp
                    )
                    if state is not None and (self._seen_program or startup_cancelled) and state.runtime_state == 1:
                        if max(abs(v) for v in state.actual_qd) <= cfg["STATIONARY_MAX_QD_RAD_S"]:
                            with self._lock:
                                unexpected_exit = self._status["phase"] != 4 and self._status["last_error"] is None
                            if unexpected_exit:
                                self._set_error("导纳程序已被控制器终止，程序退出和机器人静止已确认")
                            self._complete()
                            return
                    if not self._stop_requested.is_set():
                        if not self._seen_program and now - upload_time > cfg["ADMITTANCE_START_TIMEOUT_S"]:
                            with self._lock:
                                runtime = self._status["runtime_state"]
                            self._set_error(
                                "导纳启动未得到机器人确认，正在请求停止"
                                f"（runtime_state={runtime}，输出整数={self._last_registers}）"
                            )
                            self.request_stop()
                        elif now - self._last_sample_monotonic > cfg["ADMITTANCE_STATE_TIMEOUT_S"]:
                            self._set_error("导纳状态接收超时，正在请求停止")
                            self.request_stop()
                    if (self._stop_requested.is_set() and not stop_sent) or time.monotonic() >= next_send:
                        continue
                    primary_started = time.monotonic()
                    script_error = self._script_client.poll_script_error(
                        deadline=min(next_send, primary_started + cfg["ADMITTANCE_PRIMARY_READ_BUDGET_S"]),
                        max_read_bytes=cfg["ADMITTANCE_PRIMARY_READ_MAX_BYTES"],
                        buffer_limit_bytes=cfg["ADMITTANCE_PRIMARY_BUFFER_LIMIT_BYTES"])
                    with self._lock:
                        self._status["primary_processing_duration_s"] = time.monotonic() - primary_started
                    if script_error is not None:
                        self._script_error_received = True
                        self._set_error(script_error)
                        self.request_stop()
                    if (self._stop_requested.is_set() and not stop_sent) or time.monotonic() >= next_send:
                        continue
                finally:
                    self._record_processing_duration(processing_started)
                wait_s = min(0.005, 1.0 / cfg["ADMITTANCE_STATE_HZ"], max(0.0, next_send - time.monotonic()))
                self._wake.wait(wait_s)
                self._wake.clear()
        except Exception as exc:
            # 通信异常只记录一次；上传后失去停止确认时保留执行权，不重连或恢复旧程序。
            self._set_error(f"导纳后台错误：{exc}")
            self.request_stop()
            if not self._uploaded:
                self._complete()
            else:
                self._worker_failed = True
                self._record("controller_worker_failed", {
                    "status": self.get_status(), "awaiting_external_stop_confirmation": True,
                })
