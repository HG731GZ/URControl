from __future__ import annotations

from collections import OrderedDict
from copy import deepcopy
from contextlib import contextmanager
from importlib.metadata import version
import math
import secrets
import threading
import time
from typing import Optional, Sequence

import numpy as np
import rtde.rtde as rtde_module

from UR_Utils import admittance_config, admittance_telemetry
from UR_Utils.admittance_transport import (
    AdmittanceRTDE, AdmittanceScriptClient, AdmittanceDashboardStop,
    clock_sample, clock_interval, elapsed_sample,
)


_RAW_STATUS_FIELDS = frozenset(field[0] for field in admittance_telemetry.ROBOT_FIELDS)


PHASE_NAMES = {0: "启动中", 1: "静止偏置采样", 2: "运行中", 3: "减速停止中", 4: "已停止"}
EXIT_REASONS = {
    0: "正常", 1: "请求停止", 2: "主机心跳超时", 3: "主端数据超时",
    4: "运行模式变化", 5: "逆解不可达", 6: "末端修正过大",
    7: "关节跟踪误差过大", 8: "超出运动范围", 9: "关节目标不连续",
    10: "偏置采样期间机器人移动", 11: "诊断定时结束", 12: "诊断周期超预算",
    13: "启动锚点确认超时", 14: "目标片段耗尽或模型停止推进",
    15: "模型计算超预算", 16: "连续伺服出现帧缺口",
    17: "速度缩放发生变化", 18: "目标交接身份无效",
    19: "受限运动实际速度超限", 20: "受限运动实际位置超出包络",
    21: "载荷或模型统计存在非有限或无效数值",
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
        "controller_interruption": None, "stop_deadline_monotonic_s": None,
        "dashboard_stop_requested": False,
        "speed_scaling": None, "target_speed_fraction": None,
        "actual_TCP_speed": None, "actual_TCP_force": None, "actual_current": None,
        "joint_temperatures": None, "target_q": None, "target_qd": None,
        "actual_execution_time": None, "tcp_offset": None, "payload": None,
        "payload_cog": None, "payload_inertia": None, "program_seen": False,
        "lifecycle_valid": False, "derived_valid": False,
        "drive": None, "delta_q": None, "delta_x": None,
        "q_cmd": None, "qd_cmd": None, "q_nom": None, "q_start": None, "q_master": None,
        "q_master_submitted": None, "source_seq": None,
        "submitted_source_seq": None, "adopted_source_seq": None,
        "cycle_count": None, "parameters": None, "robot_configuration": None,
        "controller_version": None, "diagnostic": None,
        "telemetry_layout": admittance_telemetry.TELEMETRY_SCHEMA_VERSION,
        "run_id": None, "config_id": None, "page_type": None, "page_seq": None, "model_sample_seq": None,
        "continuous": {"execution": None, "timing": None}, "load": None,
        "statistics": None, "statistics_final_received": False, "statistics_validation_errors": [],
        "scaling_ok": False, "scaling_confirmed": False,
        "scaling_change_received_monotonic_s": None,
        "state_received_monotonic": None, "state_receive_count": 0,
        "last_reference_sent_monotonic": None, "reference_send_count": 0,
        "last_heartbeat_sent_monotonic": None, "heartbeat_send_count": 0,
        "heartbeat_echo": None, "heartbeat_age_us": None, "heartbeat_max_interval_us": None,
        "heartbeat_send_interval_s": None, "heartbeat_send_max_interval_s": None,
        "heartbeat_send_lateness_s": None, "heartbeat_send_max_lateness_s": None,
        "heartbeat_deadline_miss_count": 0,
        "processing_duration_s": None, "processing_max_duration_s": None,
        "rtde_processing_duration_s": None, "primary_processing_duration_s": None,
        "packet_prepare_duration_s": None, "packet_prepare_max_duration_s": None,
        "send_duration_s": None, "send_max_duration_s": None,
        "send_warning_count": 0,
        "receive_duration_s": None, "receive_max_duration_s": None,
        "decode_duration_s": None, "decode_max_duration_s": None,
        "log_submit_duration_s": None, "log_submit_max_duration_s": None,
        "next_heartbeat_deadline_monotonic_s": None,
        "next_reference_deadline_monotonic_s": None,
        "host_timing": {},
        "last_wake_monotonic_s": None, "wake_deadline_monotonic_s": None,
        "wake_lateness_s": None, "wake_max_lateness_s": None,
        "rtde_buffered_bytes": 0, "rtde_buffered_max_bytes": 0,
        "state_timestamp_interval_s": None, "state_host_interval_s": None,
        "state_progress_lag_s": 0.0, "state_progress_max_lag_s": 0.0,
        "state_last_progress_monotonic_s": None,
        "state_progress_rate": None, "state_progress_window_host_s": None,
        "state_progress_window_robot_s": None,
        "rtde_socket_drained": False, "rtde_last_drained_monotonic_s": None,
        "rtde_drain_count": 0, "rtde_backlog_duration_s": 0.0, "rtde_backlog_max_duration_s": 0.0,
        "communication_process_pid": None, "process_worker_failed": False,
        "process_bridge_dropped_count": 0, "process_bridge": None,
        "process_status_published_monotonic_s": None,
        "process_status_received_monotonic_s": None, "process_status_age_s": None,
    }


class URAdmittanceController:
    """机器人端导纳的专用连接；一名线程拥有全部 RTDE 收发及脚本上传。"""

    def __init__(
        self, robot_ip: str, reference_source: str, space: str, *,
        q_master: Optional[Sequence[float]] = None, source_seq: Optional[int] = None,
        config: Optional[dict] = None, stop_event: Optional[threading.Event] = None,
        log_sink=None, owner_service=None,
    ):
        if reference_source not in ("teleop", "fixed"):
            raise ValueError("参考来源必须为 teleop 或 fixed")
        if space not in ("joint", "cartesian"):
            raise ValueError("导纳空间必须为 joint 或 cartesian")
        self.robot_ip = robot_ip
        self.reference_source = reference_source
        self.space = space
        self._log_sink = log_sink
        self._owner_service = owner_service
        self._synchronous_finished = False
        self.parameters = deepcopy(admittance_config.snapshot() if config is None else config)
        admittance_config.validate_run_config(reference_source, space, config=self.parameters)
        self._input_int_names = [f"input_int_register_{self.parameters['ADMITTANCE_INPUT_INT_BASE'] + i}"
                                 for i in range(8)]
        self._input_float_names = [f"input_double_register_{self.parameters['ADMITTANCE_INPUT_FLOAT_BASE'] + i}"
                                   for i in range(6)]
        self._config_id = secrets.randbelow(2 ** 31 - 1) + 1
        self._script = admittance_config.render_script(
            reference_source, space, config=self.parameters, config_id=self._config_id)
        self._lock = threading.Lock()
        self._stop_requested = threading.Event() if stop_event is None else stop_event
        self._stop_logged = False
        self._stop_deadline = None
        self._stop_after_timestamp = None
        self._finished = threading.Event()
        self._wake = threading.Event()
        self._thread = None
        self._connection = None
        self._script_client = None
        self._dashboard_stop = None
        self._dashboard_stop_attempted = False
        self._dashboard_stop_accepted = False
        self._controller_interruption = None
        self._runtime_running_seen = False
        self._script_error_received = False
        self._uploaded = False
        self._worker_failed = False
        self._seen_program = False
        self._baseline_registers = None
        self._baseline_timestamp = None
        self._last_registers = None
        self._diagnostic_anchor = None
        self._diagnostic_anchor_ack = 0
        self._run_id = secrets.randbelow(2 ** 31 - 1) + 1
        self._scaling_ok = 0
        self._scaling_confirmed = False
        self._scaling_changed = False
        self._active = True
        self._q_master = None
        self._source_seq = None
        self._sent_references = OrderedDict()
        self._heartbeat = 0
        self._last_sample_monotonic = None
        self._last_state_timestamp = None
        self._last_progress_monotonic = None
        self._state_progress_lag_s = 0.0
        self._progress_window = None
        self._rtde_backlog_started = None
        self._status = idle_admittance_status()
        self._status.update(
            active=True, starting=not self._stop_requested.is_set(),
            stopping=self._stop_requested.is_set(), stopped_confirmed=False,
            phase=0, phase_name=PHASE_NAMES[0], reference_source=reference_source,
            space=space, parameters=deepcopy(self.parameters),
            run_id=self._run_id, config_id=self._config_id,
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
            "run_id": self._run_id, "config_id": self._config_id,
            "units": admittance_config.units(reference_source, space, config=self.parameters),
            "telemetry_schema": admittance_telemetry.log_schema(self.parameters, space),
        }
        actual_speeds, actual_margins = admittance_config.actual_guard_limits(self.parameters)
        controller_context["actual_guard_limits"] = {
            "axis_order": ["J1", "J2", "J3", "J4", "J5", "J6"],
            "max_qd_rad_s": actual_speeds, "margin_rad": actual_margins,
            "scope": "六轴实际反馈守卫，启动、偏置及运行阶段均生效；裕量加在各轴命令范围两侧",
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

    @contextmanager
    def _state_lock(self):
        """只在通信拥有者记录状态锁等待；锁内对象发布后均不再原地修改。"""
        measured = threading.current_thread() is self._thread
        started = clock_sample() if measured else None
        self._lock.acquire()
        acquired = clock_sample() if measured else None
        try:
            yield
        finally:
            if measured:
                finished = clock_sample()
                self._merge_timing_locked("state_lock_wait", {
                    "wall_s": acquired[0] - started[0], "cpu_s": acquired[1] - started[1],
                    "started_monotonic_s": started[0], "finished_monotonic_s": acquired[0],
                })
                self._merge_timing_locked("state_lock_hold", {
                    "wall_s": finished[0] - acquired[0], "cpu_s": finished[1] - acquired[1],
                    "started_monotonic_s": acquired[0], "finished_monotonic_s": finished[0],
                })
            self._lock.release()

    def _merge_timing_locked(self, segment: str, sample: dict) -> None:
        """替换一个计时聚合，快照共享旧对象时仍保持冻结。"""
        timings = self._status["host_timing"]
        previous = timings.get(segment)
        current = {
            "last_wall_s": sample["wall_s"], "last_cpu_s": sample["cpu_s"],
            "last_started_monotonic_s": sample["started_monotonic_s"],
            "last_finished_monotonic_s": sample["finished_monotonic_s"],
            "max_wall_s": sample["wall_s"], "max_cpu_s": sample["cpu_s"], "count": 1,
            "max_wall_cpu_s": sample["cpu_s"],
            "max_wall_started_monotonic_s": sample["started_monotonic_s"],
            "max_wall_finished_monotonic_s": sample["finished_monotonic_s"],
        }
        if previous is not None:
            current["count"] = previous["count"] + 1
            current["max_cpu_s"] = max(previous["max_cpu_s"], sample["cpu_s"])
            if previous["max_wall_s"] > sample["wall_s"]:
                for key in ("max_wall_s", "max_wall_cpu_s", "max_wall_started_monotonic_s",
                            "max_wall_finished_monotonic_s"):
                    current[key] = previous[key]
        self._status["host_timing"] = {**timings, segment: current}

    def _record_timing(self, segment: str, sample: dict) -> None:
        """通信线程墙钟和 CPU 分开保存，保留旧平铺耗时字段。"""
        if threading.current_thread() is not self._thread:
            return
        with self._state_lock():
            self._merge_timing_locked(segment, sample)
            duration_name = f"{segment}_duration_s"
            if duration_name in self._status:
                self._status[duration_name] = sample["wall_s"]
            maximum_name = f"{segment}_max_duration_s"
            if maximum_name in self._status:
                previous = self._status[maximum_name]
                self._status[maximum_name] = sample["wall_s"] if previous is None else max(previous, sample["wall_s"])

    def _record(self, kind: str, payload: dict, *, host_monotonic_s=None, host_time_unix_s=None) -> None:
        """仅向日志线程交付冻结数据；调用方必须先释放状态锁。"""
        if self._log_sink is not None:
            started = clock_sample()
            self._log_sink.record(kind, {
                "host_monotonic_s": time.monotonic() if host_monotonic_s is None else host_monotonic_s,
                "host_time_unix_s": time.time() if host_time_unix_s is None else host_time_unix_s,
                **payload,
            })
            self._record_timing("log_submit", elapsed_sample(started))

    def _control_snapshot_locked(self) -> dict:
        """锁内只复制顶层引用；嵌套页按替换发布，固定参数不逐包复制。"""
        return {key: value for key, value in self._status.items()
                if key not in _RAW_STATUS_FIELDS
                and key not in ("parameters", "robot_configuration", "host_timing")}

    @property
    def active(self) -> bool:
        """停止确认和连接释放完成之前，仍保留本轮执行权。"""
        with self._state_lock():
            return self._active

    def start(self) -> None:
        """只启动后台工作线程；不会清除已到达的停止请求。"""
        self._record("controller_start", {"status": self.get_status()})
        self._thread = threading.Thread(target=self._run, name="URAdmittanceRTDE", daemon=True)
        self._thread.start()

    def run_in_current_thread(self) -> None:
        """独立进程以主线程执行完整控制链，不创建第二个通信或心跳线程。"""
        self._thread = threading.current_thread()
        self._record("controller_start", {"status": self.get_status()})
        try:
            self._run()
        finally:
            self._synchronous_finished = True

    def _service_owner(self) -> None:
        """在唯一控制路径检查进程拥有者和最新 IPC 输入。"""
        if self._owner_service is not None:
            started = clock_sample()
            self._owner_service(self)
            self._record_timing("owner_service", elapsed_sample(started))

    def cancel_before_start(self, error: Optional[str] = None) -> None:
        """原控制通道尚未完成交接时终止本次启动，不上传脚本。"""
        if error is not None:
            self._set_error(error)
        elif self._stop_requested.is_set():
            with self._state_lock():
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
        with self._state_lock():
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
        """首次请求锁定总停止期限；重复请求不延长等待，上传期间也保留停止意图。"""
        requested_monotonic = time.monotonic()
        requested_wall_time = time.time()
        self._stop_requested.set()
        self._wake.set()
        with self._state_lock():
            if self._stop_deadline is None:
                self._stop_deadline = requested_monotonic + self.parameters["ADMITTANCE_STOP_TIMEOUT_S"]
                self._stop_after_timestamp = self._status["timestamp"]
                self._status["stop_deadline_monotonic_s"] = self._stop_deadline
            if self._active:
                self._status.update(
                    stopping=True, starting=False, running=False,
                    phase_name=("控制器中断，正在停止" if self._controller_interruption is not None
                                else "正在停止，等待程序退出及静止"),
                )
            log_request = not self._stop_logged
            self._stop_logged = True
            status = self._control_snapshot_locked() if log_request else None
        if log_request:
            self._record("stop_requested", {"status": status},
                         host_monotonic_s=requested_monotonic, host_time_unix_s=requested_wall_time)

    def stop(self) -> None:
        """等待程序退出和静止反馈；超时保留连接与执行权。"""
        self.request_stop()
        if not self._finished.wait(max(0.0, self._stop_deadline - time.monotonic())):
            message = "导纳停止尚未确认，保留专用连接，禁止恢复其他运动"
            if self.get_status()["last_error"] is None:
                self._set_error(message)
            raise RuntimeError(message)
        if self._thread is not None:
            self._thread.join()

    def get_status(self) -> dict:
        """返回一致的遥测快照；不在调用线程读写专用连接。"""
        with self._state_lock():
            snapshot = dict(self._status)
        return deepcopy(snapshot)

    def needs_external_stop_confirmation(self) -> bool:
        """专用通信失败且工作线程退出后，允许原只读连接提供停止证据。"""
        return self._worker_failed and (self._synchronous_finished
                                       or self._thread is not None and not self._thread.is_alive())

    def confirm_external_stop(self, sample: dict) -> None:
        """统一控制器已核实新的停止与静止反馈后，释放故障专用连接。"""
        if not self.needs_external_stop_confirmation():
            raise RuntimeError("工作线程尚未退出，不能由其他线程释放专用连接")
        with self._state_lock():
            self._status.update(sample)
        self._record("external_stop_confirmed", {"sample": deepcopy(sample), "status": self.get_status()})
        self._complete()

    def _set_error(self, message: str) -> None:
        with self._state_lock():
            self._status["last_error"] = message
            self._status["running"] = False
            status = self._control_snapshot_locked()
        self._record("controller_error", {"message": message, "status": status})

    def _complete(self) -> None:
        """仅在上传前取消，或确认机器人程序退出后释放本轮资源。"""
        if self._dashboard_stop is not None:
            self._dashboard_stop.close()
            self._dashboard_stop = None
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
        if self._log_sink is not None:
            # 与停止确认分开保存：程序可被外部终止而没有产生最终统计。
            self._log_sink.update_context({"model_statistics_result": {
                "run_id": self._run_id, "config_id": self._config_id,
                "statistics": status["statistics"],
                "final_received": status["statistics_final_received"],
                "validation_errors": status["statistics_validation_errors"],
            }})
        # 完成事件先入队，再让界面看到 inactive，确保界面结束日志时事件已提交。
        self._record("controller_complete", {"status": status})
        with self._state_lock():
            self._active = False
            self._status.update(complete_flags)
        self._finished.set()

    def _service_stop(self) -> None:
        """异常暂停立即终止脚本；普通停止先留给唯一 stopj，全部等待共用总期限。"""
        if not self._stop_requested.is_set():
            return
        if self._stop_deadline is None:
            self.request_stop()
        now = time.monotonic()
        if now >= self._stop_deadline:
            raise TimeoutError("导纳停止超过总期限，尚未确认程序退出及六轴静止，保留控制权等待外部确认")
        # 正常寄存器停止先执行；最后半秒仅作为明确的程序终止与反馈确认预算。
        if (self._uploaded and not self._dashboard_stop_attempted
                and (self._controller_interruption is not None or now >= self._stop_deadline - 0.5)):
            self._dashboard_stop_attempted = True
            with self._state_lock():
                self._status["dashboard_stop_requested"] = True
            self._record("dashboard_stop_requested", {
                "cause": ("controller_interrupted" if self._controller_interruption is not None
                          else "register_stop_unconfirmed"),
                "deadline_monotonic_s": min(self._stop_deadline, now + 0.25),
                "status": self.get_status(),
            })
            try:
                self._dashboard_stop = AdmittanceDashboardStop(
                    self.robot_ip, deadline=min(self._stop_deadline, now + 0.25))
            except OSError as exc:
                self._record("dashboard_stop_failed", {"message": str(exc), "status": self.get_status()})
        if self._dashboard_stop is not None:
            try:
                response = self._dashboard_stop.poll()
            except (OSError, RuntimeError, UnicodeError) as exc:
                self._record("dashboard_stop_failed", {"message": str(exc), "status": self.get_status()})
                self._dashboard_stop.close()
                self._dashboard_stop = None
            else:
                if response is not None:
                    if response == "Stopped":
                        self._dashboard_stop_accepted = True
                        # 启动尚未出现运行反馈时，必须先确认终止命令生效，再等待新状态。
                        with self._state_lock():
                            self._stop_after_timestamp = self._status["timestamp"]
                    self._record("dashboard_stop_reply", {
                        "response": response, "accepted": response == "Stopped",
                        "greeting": self._dashboard_stop.greeting, "status": self.get_status(),
                    })
                    self._dashboard_stop.close()
                    self._dashboard_stop = None

    def _setup_connection(self):
        cfg = self.parameters
        conn = AdmittanceRTDE(self.robot_ip, buffer_limit_bytes=cfg["ADMITTANCE_RTDE_BUFFER_LIMIT_BYTES"])
        self._connection = conn
        conn.connect()
        controller_version = dict(zip(("major", "minor", "bugfix", "build"), conn.get_controller_version()))
        with self._state_lock():
            self._status["controller_version"] = controller_version
        input_names = self._input_int_names + self._input_float_names
        input_types = ["INT32"] * 8 + ["DOUBLE"] * 6
        packet = conn.send_input_setup(input_names, input_types)
        if packet is None:
            raise RuntimeError("导纳输入寄存器配方建立失败，请检查其他客户端或 URCap 占用")
        output_names, output_types = admittance_telemetry.output_recipe(cfg)
        state_frequency = admittance_telemetry.state_frequency(cfg)
        if not conn.send_output_setup(output_names, output_types, frequency=state_frequency):
            raise RuntimeError("导纳状态配方建立失败")
        if not conn.send_start():
            raise RuntimeError("导纳 RTDE 同步启动失败")
        connection_context = {
            "controller_version": controller_version,
            "state_frequency_hz": state_frequency,
            "input_recipe": dict(zip(input_names, input_types)),
            "output_recipe": dict(zip(output_names, output_types)),
            "output_packet_bytes": admittance_telemetry.output_packet_size(cfg),
            "rtde_library_version": version("UrRtde"),
            "rtde_library_path": rtde_module.__file__,
            "runtime_transport": "UR_Utils.admittance_transport.AdmittanceRTDE",
            "runtime_send_timeout_s": cfg["ADMITTANCE_RTDE_SEND_TIMEOUT_S"],
            "runtime_send_warning_s": cfg["ADMITTANCE_RTDE_SEND_WARNING_S"],
            "runtime_send_timing_version": 3,
            "runtime_send_contract": "从准备开始至完整写出共用单包期限；准备段仅采样时间点，"
                                     "状态锁计时及分段统计在发送成功或失败后归并；失败不重发、不续期。"
                                     "超过观测阈值累计并记录，不单独触发退出；超过退出期限仍终止本轮。"
                                     "完整本机写出不等于机器人收到，机器人端心跳与运动守卫独立生效。",
            "runtime_receive_max_syscalls": 1,
            "runtime_receive_max_packets": 1,
            "runtime_receive_max_bytes": min(4096, cfg["ADMITTANCE_RTDE_BUFFER_LIMIT_BYTES"]),
            "timing_semantics": "墙钟包含调度/GIL/系统调用等待，CPU为通信线程thread_time；预算不保证硬实时抢占",
            "state_progress_contract": "推进差仅在未排空窗口内累计；仅socket不可读且用户buffer为空时重置。"
                                       "时间戳不推进或持续未排空超过状态超时才停止；1秒窗口时钟推进比仅供观察，非单向年龄。",
        }
        if self._log_sink is not None:
            self._log_sink.update_context({"rtde": deepcopy(connection_context)})
        self._record("connection_ready", connection_context)
        return packet

    def _send_inputs(self, packet, *, heartbeat: bool, heartbeat_deadline: Optional[float] = None) -> bool:
        """发送前只准备输入并采样时间点；统计归并与日志在发送结果确定后执行。"""
        prepare_clock = clock_sample()
        prepare_started = prepare_clock[0]
        cfg = self.parameters
        send_deadline = prepare_started + cfg["ADMITTANCE_RTDE_SEND_TIMEOUT_S"]
        if heartbeat:
            self._heartbeat = (self._heartbeat + 1) % (2 ** 31)
        snapshot_wait = clock_sample()
        # 这里仅保护参考快照，避免通用状态锁的计时归并占用本包发送期限。
        with self._lock:
            snapshot_acquired = clock_sample()
            seq = self._source_seq
            q = None if self._q_master is None else self._q_master.copy()
        snapshot_finished = clock_sample()
        wire_seq = -1 if seq is None else seq % (2 ** 31)
        values = [int(not self._stop_requested.is_set()), self._heartbeat, wire_seq,
                  1 if self.space == "joint" else 2, 1 if self.reference_source == "teleop" else 3,
                  self._diagnostic_anchor_ack, self._run_id, self._scaling_ok]
        wire_q = [0.0] * 6 if q is None else q.tolist()
        for name, value in zip(self._input_int_names, values):
            setattr(packet, name, value)
        for name, value in zip(self._input_float_names, wire_q):
            setattr(packet, name, value)
        serialize_started = clock_sample()
        prepared = self._connection.prepare_inputs(packet)
        send_clock = clock_sample()
        send_started = send_clock[0]
        send_error = None
        try:
            self._connection.send_prepared(prepared, deadline=send_deadline)
        except Exception as error:
            send_error = error
            raise
        finally:
            send_finished = clock_sample()
            prepare_timing = clock_interval(prepare_clock, send_clock)
            preparation_segments = {
                "input_snapshot_wait": clock_interval(snapshot_wait, snapshot_acquired),
                "input_snapshot_copy": clock_interval(snapshot_acquired, snapshot_finished),
                "input_fields": clock_interval(snapshot_finished, serialize_started),
                "input_serialize": clock_interval(serialize_started, send_clock),
            }
            packet_total = clock_interval(prepare_clock, send_finished)
            send_warning = packet_total["wall_s"] > cfg["ADMITTANCE_RTDE_SEND_WARNING_S"]
            if send_warning:
                with self._state_lock():
                    self._status["send_warning_count"] += 1
            for name, sample in {**preparation_segments, "packet_prepare": prepare_timing,
                                 "send": clock_interval(send_clock, send_finished),
                                 "packet_total": packet_total}.items():
                self._record_timing(name, sample)
            for name in ("socket_writable_wait", "socket_send"):
                sample = self._connection.last_send_timing[name]
                if "started_monotonic_s" in sample:
                    self._record_timing(name, sample)
            if send_error is not None:
                self._record("rtde_input_send_failed", {
                    "error": str(send_error), "enable": values[0], "heartbeat": self._heartbeat,
                    "heartbeat_advanced": heartbeat, "wire_source_seq": wire_seq,
                    "send_started_monotonic_s": send_started,
                    "send_deadline_monotonic_s": send_deadline,
                    "send_entry_remaining_s": send_deadline - send_started,
                    "heartbeat_deadline_monotonic_s": heartbeat_deadline,
                    "packet_prepare": prepare_timing, "packet_total": packet_total,
                    "send_warning_threshold_s": cfg["ADMITTANCE_RTDE_SEND_WARNING_S"],
                    "send_warning_exceeded": send_warning,
                    "transport_timing": {**preparation_segments, **self._connection.last_send_timing},
                }, host_monotonic_s=send_finished[0])
        transport_timing = self._connection.last_send_timing
        sent_monotonic = transport_timing["socket_send"]["finished_monotonic_s"]
        self._scaling_changed = False
        sent_wall_time = time.time()
        with self._state_lock():
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
                "packet_prepare_duration_s", "packet_prepare_max_duration_s",
                "send_duration_s", "send_max_duration_s",
                "send_warning_count",
                "receive_duration_s", "receive_max_duration_s",
                "decode_duration_s", "decode_max_duration_s",
                "log_submit_duration_s", "log_submit_max_duration_s",
                "next_heartbeat_deadline_monotonic_s", "next_reference_deadline_monotonic_s",
                "host_timing", "last_wake_monotonic_s", "wake_deadline_monotonic_s",
                "wake_lateness_s", "wake_max_lateness_s", "rtde_buffered_bytes", "rtde_buffered_max_bytes",
                "state_timestamp_interval_s", "state_host_interval_s", "state_progress_lag_s",
                "state_progress_max_lag_s", "state_last_progress_monotonic_s",
                "state_progress_rate", "state_progress_window_host_s", "state_progress_window_robot_s",
                "rtde_socket_drained", "rtde_last_drained_monotonic_s", "rtde_drain_count",
                "rtde_backlog_duration_s", "rtde_backlog_max_duration_s",
            )} if self._log_sink is not None else None
        if q is not None:
            self._sent_references[wire_seq] = (seq, q)
            self._sent_references.move_to_end(wire_seq)
            while len(self._sent_references) > 512:
                self._sent_references.popitem(last=False)
        if self._log_sink is not None:
            registers = dict(zip(self._input_int_names, values))
            registers.update(zip(self._input_float_names, wire_q))
            self._record("rtde_input", {
                "raw": registers,
                "enable": values[0], "heartbeat": self._heartbeat, "heartbeat_advanced": heartbeat,
                "source_seq": seq, "wire_source_seq": wire_seq,
                "space": self.space, "reference_source": self.reference_source,
                "diagnostic_anchor_ack": self._diagnostic_anchor_ack,
                "run_id": self._run_id, "scaling_ok": bool(values[7]),
                "q_master": None if q is None else q.tolist(), "wire_q": wire_q,
                "send_started_monotonic_s": send_started,
                "packet_prepare_started_monotonic_s": prepare_started,
                "packet_prepare_duration_s": prepare_timing["wall_s"],
                "send_duration_s": sent_monotonic - send_started,
                "send_deadline_monotonic_s": send_deadline,
                "send_entry_remaining_s": send_deadline - send_started,
                "send_warning_threshold_s": cfg["ADMITTANCE_RTDE_SEND_WARNING_S"],
                "send_warning_exceeded": send_warning,
                "transport_timing": {**preparation_segments, "packet_prepare": prepare_timing,
                                     "packet_total": packet_total, **transport_timing},
                "heartbeat_deadline_monotonic_s": heartbeat_deadline, "timing": timing,
            }, host_monotonic_s=sent_monotonic, host_time_unix_s=sent_wall_time)
        return values[0] == 0

    def _receive_latest(self, deadline: float, *, stop_sent: bool):
        """接收、单帧解码、发布及日志共用批期限，每个单元后回看发送与停止。"""
        latest = None
        while time.monotonic() < deadline:
            if self._stop_requested.is_set() and not stop_sent:
                break
            receive_started = clock_sample()
            state = self._connection.receive_once()
            self._record_timing("receive", elapsed_sample(receive_started))
            for segment, sample in self._connection.last_receive_timing.items():
                self._record_timing(segment, sample)
            observed = time.monotonic()
            drained = self._connection.last_receive_drained
            if drained:
                # 两个缓冲均已排空才重建相对推进窗口，不假定机器人时钟与主机等速。
                self._rtde_backlog_started = None
                self._state_progress_lag_s = 0.0
            elif self._rtde_backlog_started is None:
                self._rtde_backlog_started = receive_started[0]
            backlog_duration = 0.0 if drained else observed - self._rtde_backlog_started
            with self._state_lock():
                self._status["rtde_buffered_bytes"] = self._connection.buffered_bytes
                self._status["rtde_buffered_max_bytes"] = self._connection.buffered_max_bytes
                self._status["rtde_socket_drained"] = drained
                self._status["rtde_backlog_duration_s"] = backlog_duration
                self._status["rtde_backlog_max_duration_s"] = max(
                    self._status["rtde_backlog_max_duration_s"], backlog_duration)
                if drained:
                    self._status["rtde_last_drained_monotonic_s"] = observed
                    self._status["rtde_drain_count"] += 1
                    self._status["state_progress_lag_s"] = 0.0
            if state is None:
                break
            latest = state
            self._consume_state(state)
            if self._scaling_changed:
                break
        return latest

    def _consume_state(self, state) -> None:
        decode_started = clock_sample()
        received_monotonic = decode_started[0]
        received_wall_time = time.time()
        cfg = self.parameters
        robot = admittance_telemetry.robot_sample(state)
        ints = [int(getattr(state, f"output_int_register_{cfg['ADMITTANCE_OUTPUT_INT_BASE'] + i}"))
                for i in range(24)]
        values = [float(getattr(state, f"output_double_register_{cfg['ADMITTANCE_OUTPUT_FLOAT_BASE'] + i}"))
                  for i in range(24)]
        phase, reason, adopted, sample_seq, source, heartbeat_echo, heartbeat_age, heartbeat_max_interval = ints[:8]
        self._last_registers = tuple(ints)
        runtime = int(state.runtime_state)
        timestamp = float(state.timestamp)
        interruption_message = None
        if self._uploaded:
            if runtime == 2:
                self._runtime_running_seen = True
            if (self._controller_interruption is None
                    and (robot["safety_mode"] != 1
                         or self._runtime_running_seen and runtime in (3, 4, 6))):
                self._controller_interruption = {
                    "safety_mode": robot["safety_mode"], "runtime_state": runtime,
                    "robot_timestamp_s": timestamp, "received_monotonic_s": received_monotonic,
                }
                interruption_message = (
                    "导纳控制器中断，正在终止暂停脚本并等待退出静止确认"
                    f"（safety_mode={robot['safety_mode']}，runtime_state={runtime}）"
                )
        host_interval = None
        timestamp_interval = None
        if self._last_state_timestamp is not None:
            host_interval = received_monotonic - self._last_sample_monotonic
            timestamp_interval = timestamp - self._last_state_timestamp
            if timestamp_interval < 0:
                raise RuntimeError("RTDE 控制器时间戳倒退，本轮状态身份已失效")
            self._state_progress_lag_s = max(0.0, self._state_progress_lag_s + host_interval - timestamp_interval)
        if self._last_state_timestamp is None or timestamp > self._last_state_timestamp:
            self._last_progress_monotonic = received_monotonic
        self._last_state_timestamp = timestamp
        self._last_sample_monotonic = received_monotonic
        progress_window = None
        if self._progress_window is None:
            self._progress_window = (received_monotonic, timestamp)
        elif received_monotonic - self._progress_window[0] >= 1.0:
            host_elapsed = received_monotonic - self._progress_window[0]
            robot_elapsed = timestamp - self._progress_window[1]
            progress_window = (host_elapsed, robot_elapsed, robot_elapsed / host_elapsed)
            self._progress_window = (received_monotonic, timestamp)
        was_seen_program = self._seen_program
        scaling_event = None
        full_scaling = robot["speed_scaling"] == 1.0 and robot["target_speed_fraction"] == 1.0
        if not self._uploaded:
            self._baseline_registers = tuple(ints)
            self._baseline_timestamp = float(state.timestamp)
        elif runtime == 2 and not self._scaling_confirmed and full_scaling:
            # 空闲时控制器可报告零缩放；上传后先由无运动握手等待首次双 100%。
            self._scaling_confirmed = True
            self._scaling_ok = 1
            self._scaling_changed = True
            scaling_event = "speed_scaling_confirmed"
            self._wake.set()
        elif runtime == 2 and self._scaling_confirmed and not full_scaling and self._scaling_ok:
            # 本轮一旦观察到缩放变化便锁定停止条件，恢复滑块不能重开当前轨迹。
            self._scaling_ok = 0
            self._scaling_changed = True
            scaling_event = "speed_scaling_changed"
            self._wake.set()
        source_matches = source == (1 if self.reference_source == "teleop" else 3)
        run_matches = ints[11] == self._run_id
        page_valid = ints[8] == ints[9] and ints[8] > 0 and ints[10] in admittance_telemetry.PAGE_NAMES
        lifecycle_valid = self._uploaded and source_matches and run_matches and page_valid
        if lifecycle_valid:
            # 正整数运行身份贯穿输入及全部冻结页，不再用低周期计数猜测本轮。
            self._seen_program = True
        if (lifecycle_valid and ints[10] == 1 and reason == 0
                and self._diagnostic_anchor is None):
            if (ints[20] != admittance_telemetry.TELEMETRY_SCHEMA_VERSION
                    or ints[19] != admittance_telemetry.LOAD_SOURCE_CODES[cfg["JOINT_LOAD_SOURCE"]]
                    or ints[21] != self._config_id):
                raise RuntimeError("导纳锚点页的输入来源、遥测版本或配置身份与本轮不一致")
            self._diagnostic_anchor = values[:6]
            self._diagnostic_anchor_ack = 1
        derived_valid = lifecycle_valid and self._diagnostic_anchor is not None
        page = None
        if derived_valid or lifecycle_valid and ints[10] == 5:
            page = self._decode_page(ints, values)
            page.update(received_monotonic_s=received_monotonic,
                        robot_timestamp_s=float(state.timestamp))
        entry = self._sent_references.get(adopted) if self.reference_source == "teleop" else None
        final_statistics = None
        self._record_timing("decode", elapsed_sample(decode_started))
        publish_started = clock_sample()
        with self._state_lock():
            old_phase = self._status["phase"]
            old_reason = self._status["exit_reason"]
            self._status.update(robot)
            self._status.update(
                state_received_monotonic=received_monotonic, program_seen=self._seen_program,
                lifecycle_valid=lifecycle_valid, derived_valid=derived_valid,
                scaling_ok=bool(self._scaling_ok),
                scaling_confirmed=self._scaling_confirmed,
                page_type=admittance_telemetry.PAGE_NAMES[ints[10]] if lifecycle_valid else None,
                page_seq=ints[8] if lifecycle_valid else None,
                model_sample_seq=sample_seq if lifecycle_valid else None,
                state_timestamp_interval_s=timestamp_interval, state_host_interval_s=host_interval,
                state_progress_lag_s=self._state_progress_lag_s,
                state_progress_max_lag_s=max(self._status["state_progress_max_lag_s"], self._state_progress_lag_s),
                state_last_progress_monotonic_s=self._last_progress_monotonic,
            )
            self._status["state_receive_count"] += 1
            if progress_window is not None:
                self._status.update(state_progress_window_host_s=progress_window[0],
                                    state_progress_window_robot_s=progress_window[1],
                                    state_progress_rate=progress_window[2])
            if scaling_event == "speed_scaling_changed":
                self._status["scaling_change_received_monotonic_s"] = received_monotonic
            configuration = None
            if self._status["robot_configuration"] is None:
                self._status["robot_configuration"] = {
                    "tcp_offset": list(state.tcp_offset), "payload": float(state.payload),
                    "payload_cog": list(state.payload_cog), "payload_inertia": list(state.payload_inertia),
                    "timestamp": float(state.timestamp), "received_monotonic": received_monotonic,
                }
                configuration = self._status["robot_configuration"]
            # 顶层只呈现本包对应的样本；历史独立页放在带完整身份的缓存中。
            for name in ("drive", "delta_q", "delta_x", "q_cmd", "qd_cmd", "q_nom", "q_master",
                         "source_seq", "adopted_source_seq", "cycle_count", "heartbeat_echo",
                         "heartbeat_age_us", "heartbeat_max_interval_us"):
                self._status[name] = None
            if lifecycle_valid:
                self._status.update(
                    phase=phase, phase_name=PHASE_NAMES.get(phase, "未知阶段"), exit_reason=reason,
                    exit_reason_text=EXIT_REASONS.get(reason, f"退出码 {reason}"),
                    running=phase == 2 and runtime == 2 and not self._stop_requested.is_set(),
                    starting=phase < 2 and not self._stop_requested.is_set(),
                    stopping=phase == 3 or self._stop_requested.is_set(),
                    cycle_count=sample_seq, heartbeat_echo=heartbeat_echo,
                    heartbeat_age_us=heartbeat_age, heartbeat_max_interval_us=heartbeat_max_interval,
                    adopted_source_seq=None if self.reference_source == "fixed" else adopted,
                )
                if reason > 1 and reason != 11:
                    self._status["last_error"] = EXIT_REASONS.get(reason, f"导纳异常退出：{reason}")
            if derived_valid:
                self._status["q_start"] = list(self._diagnostic_anchor)
                if ints[10] == 2:
                    old_page = self._status["load"]
                    if old_page is None or old_page["page_seq"] != page["page_seq"]:
                        self._status["load"] = page
                    self._status["drive"] = page["drive"]
                elif ints[10] in (3, 4):
                    page_name = admittance_telemetry.PAGE_NAMES[ints[10]]
                    old_page = self._status["continuous"][page_name]
                    if old_page is None or old_page["page_seq"] != page["page_seq"]:
                        self._status["continuous"] = {**self._status["continuous"], page_name: page}
                    if ints[10] == 3:
                        self._status.update(q_cmd=page["q_cmd"], qd_cmd=page["qd_cmd"],
                                            q_nom=page["q_nom"], delta_q=page["delta_q"])
                        if self.reference_source == "teleop":
                            self._status.update(q_master=None if entry is None else entry[1].tolist(),
                                                source_seq=None if entry is None else entry[0])
            if lifecycle_valid and ints[10] == 5:
                # 统计包含独立配置身份；启动或偏置中停止也保留空的最终统计，
                # 不依赖用于轨迹解码的锚点握手完成。
                previous = self._status["statistics"]
                if previous is None or previous["page_seq"] != page["page_seq"]:
                    errors = list(self._status["statistics_validation_errors"])
                    errors.extend(self._statistics_errors(page, previous))
                    errors = list(dict.fromkeys(errors))
                    page["validation_errors"] = errors
                    self._status["statistics_validation_errors"] = errors
                    self._status["statistics"] = page
                    received_final = page["final"] and page["identity_valid"] and not errors
                    self._status["statistics_final_received"] = received_final
                    if received_final:
                        final_statistics = page
            if self._controller_interruption is not None or self._stop_requested.is_set():
                self._status.update(
                    controller_interruption=self._controller_interruption,
                    running=False, starting=False, stopping=True,
                    phase_name=("控制器中断，正在停止" if self._controller_interruption is not None
                                else "正在停止，等待程序退出及静止"),
                )
        self._record_timing("state_publish", elapsed_sample(publish_started))
        if interruption_message is not None:
            self._set_error(interruption_message)
            self.request_stop()
            self._record("controller_interrupted", {"status": self.get_status()})
        elif lifecycle_valid and (phase >= 3 or reason != 0) and not self._stop_requested.is_set():
            # 机器人端定时或故障退出也必须在有限期限内取得主机停止确认。
            self.request_stop()
        if self._log_sink is not None:
            with self._state_lock():
                control = self._control_snapshot_locked()
            raw = dict(robot)
            raw.update({f"output_int_register_{cfg['ADMITTANCE_OUTPUT_INT_BASE'] + i}": value
                        for i, value in enumerate(ints)})
            raw.update({f"output_double_register_{cfg['ADMITTANCE_OUTPUT_FLOAT_BASE'] + i}": value
                        for i, value in enumerate(values)})
            self._record("rtde_state", {
                "raw": raw, "program_seen": self._seen_program,
                "source_matches": source_matches, "run_matches": run_matches, "page_valid": page_valid,
                "lifecycle_valid": lifecycle_valid, "derived_valid": derived_valid, "control": control,
            }, host_monotonic_s=received_monotonic, host_time_unix_s=received_wall_time)
            if configuration is not None:
                self._record("robot_configuration", {"configuration": configuration})
            if final_statistics is not None:
                self._record("model_statistics_final", {"statistics": final_statistics})
            if scaling_event is not None:
                self._record(scaling_event, {
                    "speed_scaling": robot["speed_scaling"],
                    "target_speed_fraction": robot["target_speed_fraction"],
                    "robot_timestamp_s": robot["timestamp"], "status": control,
                }, host_monotonic_s=received_monotonic, host_time_unix_s=received_wall_time)
            if lifecycle_valid and (phase != old_phase or reason != old_reason or not was_seen_program):
                self._record("controller_phase", {"status": control})
            if lifecycle_valid and reason > 1 and reason != 11 and reason != old_reason:
                self._record("controller_error", {"message": control["last_error"], "status": control})

    def _decode_page(self, ints: list[int], values: list[float]) -> dict:
        """按第五版布局解码冻结页，来源由已核对锚点的本轮配置确定。"""
        def seconds(value: int) -> Optional[float]:
            """整数时间以微秒传输，负值表示尚无该测量。"""
            return value / 1_000_000 if value >= 0 else None

        layout = ints[10]
        page = {
            "run_id": ints[11], "page_seq": ints[8], "page_type": admittance_telemetry.PAGE_NAMES[layout],
            "model_sample_seq": ints[3], "phase": ints[0], "snapshot_time_s": seconds(ints[23]),
        }
        if layout == 1:
            page.update(q_anchor=values[:6], dt_base_s=seconds(ints[13]),
                        dt_model_design_s=seconds(ints[14]), bias_sample_count=ints[15],
                        observation_axis=ints[16], motion_axis=ints[17], diagnostic_mode=ints[18],
                        load_source_code=ints[19], telemetry_schema_version=ints[20], config_id=ints[21])
        elif layout == 2:
            bias_ready = bool(ints[22])
            page.update(
                sample_time_s=seconds(ints[12]), dt_model_actual_s=seconds(ints[13]),
                dt_model_max_s=seconds(ints[14]), compute_duration_s=seconds(ints[15]),
                bias_sample_count=ints[16], generated_seq=ints[17], submitted_seq=ints[18],
                limit_reason=ints[19], load_duration_s=seconds(ints[20]),
                publish_duration_s=seconds(ints[21]), bias_ready=bias_ready,
                load_source=self.parameters["JOINT_LOAD_SOURCE"],
                input_raw_nm=values[:6], bias_nm=values[6:12] if bias_ready else None,
                filtered_nm=values[12:18] if bias_ready else None,
                drive=values[18:24] if bias_ready else None,
            )
        elif layout == 5:
            page["exit_reason"] = ints[1]
            page.update(zip(("statistics_schema_version", "config_id", "start_sample_seq",
                             "end_sample_seq", "valid_cycle_count", "nonzero_drive_count",
                             "first_nonzero_sample_seq", "invalid_cycle_count"), ints[12:20]))
            page.update(trajectory_complete=ints[20] == 1, final=ints[21] == 1,
                        complete=ints[22] == 1,
                        identity_valid=(ints[12] == admittance_telemetry.STATISTICS_SCHEMA_VERSION
                                        and ints[13] == self._config_id and ints[11] == self._run_id),
                        validation_errors=[])
            for name, value in zip(admittance_telemetry.STATISTICS_FLOAT_FIELDS, values[:6]):
                page[name] = value
            for name in ("start_sample_seq", "end_sample_seq", "first_nonzero_sample_seq",
                         "start_sample_time_s", "end_sample_time_s", "first_nonzero_time_s"):
                if page[name] == -1:
                    page[name] = None
            if page["valid_cycle_count"] == 0:
                for name in ("filtered_min_nm", "filtered_max_nm", "max_abs_drive_nm"):
                    page[name] = None
            if any(value not in (0, 1) for value in ints[20:23]):
                page["validation_errors"].append("statistics flags must be 0 or 1")
        else:
            page.update(dict(zip(("generated_seq", "submitted_seq", "adopted_seq", "frame", "based_on_seq",
                                  "start_frame", "end_frame", "gap_count", "limit_reason"), ints[12:21])))
            if layout == 3:
                page.update(servoj_start_interval_s=seconds(ints[21]), servoj_max_interval_s=seconds(ints[22]),
                            q_cmd=values[:6], qd_cmd=values[6:12], q_nom=values[12:18], delta_q=values[18:24])
            else:
                for index, name in enumerate(admittance_telemetry.TIMING_FLOAT_FIELDS):
                    if not 12 <= index < 18:
                        page[name] = values[index] if values[index] >= 0 else None
                page.update(bias_sample_count=ints[21], bias_ready=bool(ints[22]),
                            bias_std_nm=values[12:18] if ints[22] else None)
        return page

    @staticmethod
    def _statistics_errors(page: dict, previous: Optional[dict]) -> list[str]:
        """校验统计本身及累计单调性；异常锁存但不妨碍独立的停止确认。"""
        errors = list(page["validation_errors"])
        if not page["identity_valid"]:
            errors.append("statistics schema, run or config identity mismatch")
        count = page["valid_cycle_count"]
        invalid = page["invalid_cycle_count"]
        nonzero = page["nonzero_drive_count"]
        if min(count, invalid, nonzero) < 0 or nonzero > count:
            errors.append("statistics cycle counts are invalid")
        start, end = page["start_sample_seq"], page["end_sample_seq"]
        start_time, end_time = page["start_sample_time_s"], page["end_sample_time_s"]
        finite = lambda value: isinstance(value, (int, float)) and math.isfinite(value)
        if count + invalid > 0:
            if (start is None or end is None or start < 1 or end < start
                    or end - start + 1 != count + invalid):
                errors.append("statistics sample range does not cover every model cycle")
            if (not finite(start_time) or not finite(end_time)
                    or start_time < 0 or end_time < start_time):
                errors.append("statistics sample time range is invalid")
        elif any(value is not None for value in (start, end, start_time, end_time)):
            errors.append("empty statistics has a nonempty sample range")
        if count > 0:
            low, high, maximum = (page[name] for name in
                                  ("filtered_min_nm", "filtered_max_nm", "max_abs_drive_nm"))
            if not all(finite(value) for value in (low, high, maximum)):
                errors.append("statistics extrema contain nonfinite values")
            elif low > high or maximum < 0 or (nonzero == 0) != (maximum == 0):
                errors.append("statistics extrema disagree with cycle counts")
        first, first_time = page["first_nonzero_sample_seq"], page["first_nonzero_time_s"]
        if nonzero > 0:
            if (first is None or start is None or end is None or not start <= first <= end
                    or not finite(first_time) or not finite(start_time) or not finite(end_time)
                    or not start_time <= first_time <= end_time):
                errors.append("statistics first nonzero sample is invalid")
        elif first is not None or first_time is not None:
            errors.append("zero-drive statistics has a first nonzero sample")
        if page["final"] and page["phase"] != 4:
            errors.append("final statistics was published before stopped phase")
        if page["complete"] and (not page["final"] or not page["trajectory_complete"] or page["exit_reason"] != 11
                                 or invalid != 0 or count <= 0 or end != page["model_sample_seq"]):
            errors.append("complete statistics does not satisfy final coverage requirements")
        if previous is not None:
            if page["page_seq"] <= previous["page_seq"]:
                errors.append("statistics page sequence moved backwards")
            if previous["final"]:
                errors.append("statistics changed after final publication")
            if any(page[name] < previous[name] for name in
                   ("valid_cycle_count", "invalid_cycle_count", "nonzero_drive_count")):
                errors.append("statistics accumulated counts moved backwards")
            if previous["start_sample_seq"] is not None:
                if (start != previous["start_sample_seq"]
                        or start_time != previous["start_sample_time_s"]):
                    errors.append("statistics start sample changed")
            if previous["nonzero_drive_count"] > 0:
                if (first != previous["first_nonzero_sample_seq"]
                        or first_time != previous["first_nonzero_time_s"]):
                    errors.append("statistics first nonzero sample changed")
            if previous["valid_cycle_count"] > 0 and count > 0:
                for name, direction in (("filtered_min_nm", 1), ("filtered_max_nm", -1),
                                        ("max_abs_drive_nm", -1)):
                    if (finite(page[name]) and finite(previous[name])
                            and direction * page[name] > direction * previous[name]):
                        errors.append("statistics accumulated extrema moved backwards")
                        break
        return errors

    @staticmethod
    def _advance_deadline(deadline: float, period: float, now: float) -> float:
        """保持原来的绝对时间网格，错过周期后跳到未来，不连续补发旧心跳。"""
        return deadline + (int((now - deadline) / period) + 1) * period

    def _run(self) -> None:
        cfg = self.parameters
        try:
            self._service_owner()
            if self._stop_requested.is_set():
                self.cancel_before_start()
                return
            packet = self._setup_connection()
            self._service_owner()
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
            # 首轮仅在正常安全状态且机器人可运行时交接，保护停止不进入脚本启动等待。
            if baseline.safety_mode != 1 or baseline.robot_mode != 7:
                raise RuntimeError(
                    "机器人尚未就绪，不能上传导纳程序"
                    f"（safety_mode={baseline.safety_mode}，要求 NORMAL=1；"
                    f"robot_mode={baseline.robot_mode}，要求 RUNNING=7）"
                )
            if max(abs(v) for v in baseline.actual_qd) > admittance_config.stop_speed_limit(cfg):
                raise RuntimeError("机器人尚未静止，不能开始导纳偏置采样")
            if self.reference_source == "teleop":
                with self._state_lock():
                    sync_error = np.max(np.abs(self._q_master - np.asarray(baseline.actual_q)))
                if sync_error > cfg["TELEOP_SYNC_TOLERANCE_RAD"]:
                    raise RuntimeError("主从关节尚未同步，不能开启遥操作导纳")
            self._send_inputs(packet, heartbeat=True)
            self._service_owner()
            if self._stop_requested.is_set():
                self.cancel_before_start()
                return
            # Primary 会向上传客户端返回编译/运行错误；保留连接并持续读取，
            # 直到停止确认，避免发完即关以及漏掉脚本被控制器拒绝的具体原因。
            self._script_client = AdmittanceScriptClient(
                self.robot_ip, port=30001, timeout=min(1.0, cfg["ADMITTANCE_STOP_TIMEOUT_S"]))
            self._script_client.connect()
            self._service_owner()
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
                self._service_owner()
                processing_clock = clock_sample()
                processing_started = processing_clock[0]
                try:
                    now = processing_started
                    heartbeat_due = now >= next_heartbeat
                    reference_due = self.reference_source == "teleop" and now >= next_reference
                    stop_due = self._stop_requested.is_set() and not stop_sent
                    # 每批接收结束后都先回到这里，优先发送停止和到期心跳。
                    if heartbeat_due or reference_due or stop_due or self._scaling_changed:
                        with self._state_lock():
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
                    with self._state_lock():
                        self._status["next_heartbeat_deadline_monotonic_s"] = next_heartbeat
                        self._status["next_reference_deadline_monotonic_s"] = (
                            next_reference if self.reference_source == "teleop" else None)
                    rtde_clock = clock_sample()
                    state = self._receive_latest(
                        min(next_send, rtde_clock[0] + cfg["ADMITTANCE_RTDE_READ_BUDGET_S"]),
                        stop_sent=stop_sent)
                    self._record_timing("rtde_processing", elapsed_sample(rtde_clock))
                    now = time.monotonic()
                    startup_cancelled = (
                        stop_sent and stopped_after_timestamp is not None
                        and (self._script_error_received
                             or now - upload_time >= cfg["ADMITTANCE_START_TIMEOUT_S"])
                        and state is not None and state.timestamp > stopped_after_timestamp
                    )
                    confirmation_ready = (self._seen_program or self._runtime_running_seen
                                          or startup_cancelled or self._dashboard_stop_accepted)
                    fresh_after_stop = (state is not None and (self._stop_after_timestamp is None
                                        or state.timestamp > self._stop_after_timestamp))
                    if (state is not None and confirmation_ready and fresh_after_stop
                            and state.runtime_state == 1):
                        if max(abs(v) for v in state.actual_qd) <= admittance_config.stop_speed_limit(cfg):
                            with self._state_lock():
                                unexpected_exit = self._status["phase"] != 4 and self._status["last_error"] is None
                            if unexpected_exit:
                                self._set_error("导纳程序已被控制器终止，程序退出和机器人静止已确认")
                            self._complete()
                            return
                    self._service_stop()
                    if not self._stop_requested.is_set():
                        if not self._seen_program and now - upload_time > cfg["ADMITTANCE_START_TIMEOUT_S"]:
                            with self._state_lock():
                                runtime = self._status["runtime_state"]
                            self._set_error(
                                "导纳启动未得到机器人确认，正在请求停止"
                                f"（runtime_state={runtime}，输出整数={self._last_registers}）"
                            )
                            self.request_stop()
                        elif (now - self._last_progress_monotonic > cfg["ADMITTANCE_STATE_TIMEOUT_S"]
                              or self._rtde_backlog_started is not None
                              and now - self._rtde_backlog_started > cfg["ADMITTANCE_STATE_TIMEOUT_S"]):
                            self._set_error("导纳状态时间未推进或接收缓冲持续未排空，正在请求停止")
                            self.request_stop()
                    if (self._stop_requested.is_set() and not stop_sent) or self._scaling_changed or time.monotonic() >= next_send:
                        continue
                    primary_clock = clock_sample()
                    script_error = self._script_client.poll_script_error(
                        deadline=min(next_send, primary_clock[0] + cfg["ADMITTANCE_PRIMARY_READ_BUDGET_S"]),
                        max_read_bytes=cfg["ADMITTANCE_PRIMARY_READ_MAX_BYTES"],
                        buffer_limit_bytes=cfg["ADMITTANCE_PRIMARY_BUFFER_LIMIT_BYTES"])
                    primary_timing = elapsed_sample(primary_clock)
                    self._record_timing("primary", primary_timing)
                    with self._state_lock():
                        self._status["primary_processing_duration_s"] = primary_timing["wall_s"]
                    if script_error is not None:
                        self._script_error_received = True
                        self._set_error(script_error)
                        self.request_stop()
                    if (self._stop_requested.is_set() and not stop_sent) or self._scaling_changed or time.monotonic() >= next_send:
                        continue
                finally:
                    self._record_timing("processing", elapsed_sample(processing_clock))
                wait_clock = clock_sample()
                wait_s = min(0.005, 1.0 / admittance_telemetry.state_frequency(cfg),
                             max(0.0, next_send - wait_clock[0]))
                wake_deadline = wait_clock[0] + wait_s
                self._wake.wait(wait_s)
                wake_time = time.monotonic()
                self._wake.clear()
                self._record_timing("wake_wait", elapsed_sample(wait_clock))
                with self._state_lock():
                    lateness = max(0.0, wake_time - wake_deadline)
                    previous = self._status["wake_max_lateness_s"]
                    self._status.update(last_wake_monotonic_s=wake_time,
                                        wake_deadline_monotonic_s=wake_deadline, wake_lateness_s=lateness,
                                        wake_max_lateness_s=lateness if previous is None else max(previous, lateness))
        except Exception as exc:
            # 通信异常只记录一次；上传后失去停止确认时保留执行权，不重连或恢复旧程序。
            self._set_error(f"导纳后台错误：{exc}")
            self.request_stop()
            if self._dashboard_stop is not None:
                self._dashboard_stop.close()
                self._dashboard_stop = None
            if not self._uploaded:
                self._complete()
            else:
                self._worker_failed = True
                with self._state_lock():
                    self._status["phase_name"] = "停止未确认，等待外部确认"
                self._record("controller_worker_failed", {
                    "status": self.get_status(), "awaiting_external_stop_confirmation": True,
                })
