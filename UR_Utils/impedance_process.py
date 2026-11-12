"""spawn 隔离通信，独立停止事件和父进程租约；停止未确认保持执行权。"""

from dataclasses import replace
from copy import deepcopy
from datetime import datetime
from pathlib import Path
import multiprocessing as mp
import threading
import time
import secrets
from UR_Utils.control_process_ipc import LatestSnapshot
from UR_Utils.impedance_reference import JointTargetFrame
from UR_Utils.impedance_config import validate
from UR_Utils.URImpedanceController import URImpedanceController, idle_impedance_status


def _main(args, status_slot, torque_slot, reference_slot, input_wakeup, stop_reason_slot, stop_event, lease):
    parent = mp.parent_process()
    sequence = 0
    stop_sequence = 0
    last_publish = 0

    def wait_for_input(timeout):
        # 下一次读取总在清除通知之后，期间到达的新帧仍在单格槽中。
        input_wakeup.wait(timeout)
        input_wakeup.clear()

    def service(backend):
        nonlocal sequence, stop_sequence, last_publish
        now = time.monotonic()
        if not parent.is_alive() or now - lease.value > 1:
            backend.request_stop("父进程租约失效")
        if stop_event.is_set():
            stop_sequence, data = stop_reason_slot.read(stop_sequence)
            if data is not None:
                backend.request_stop(data["stop_reason"])
        else:
            sequence, data = reference_slot.read(sequence)
            if data is not None:
                read_at = time.monotonic()
                backend.frame = JointTargetFrame(**data)
                backend.logger.record("ipc_read", dict(
                    source_seq=backend.frame.source_seq, ipc_read_monotonic_s=read_at,
                    source_monotonic_s=backend.frame.source_monotonic_s, q_source=backend.frame.q_rad))
        if (
            now - last_publish >= 0.02
            or not backend.status["active"]
            or backend.status["phase"] == "STOP_UNCONFIRMED"
        ):
            status_slot.publish(backend.status)
            last_publish = now

    try:
        backend = URImpedanceController(**args, stop_event=stop_event, service=service,
                                        publish_torque=torque_slot.publish, wait_for_input=wait_for_input)
        backend.run()
        # 显示读取短暂占用槽位时，仍需发布最终状态。
        deadline = time.monotonic() + 0.5
        while not status_slot.publish(backend.status) and time.monotonic() < deadline:
            time.sleep(0.001)
    except Exception as exc:
        # 进程级故障必须独立确认停止后才能释放控制权。
        status_slot.publish(
            dict(
                active=True,
                phase="STOP_UNCONFIRMED",
                running=False,
                stopped_confirmed=False,
                last_error=str(exc),
                run_id=args["run_id"],
            )
        )


class ImpedanceProcess:
    def __init__(
        self,
        robot_ip,
        reference_source="trajectory",
        *,
        config=None,
        trajectory=None,
        play_hz=None,
        stop_event=None,
        log_directory=None,
        initial_reference=None,
        parameter_revision=None,
    ):
        self.robot_ip, self.reference_source = robot_ip, reference_source
        self.parameters = validate(config, reference_source=reference_source)
        if reference_source not in ("trajectory", "step", "teleop"):
            raise ValueError("未知目标源")
        if reference_source in ("trajectory", "step") and trajectory is None:
            raise ValueError("请先生成目标或加载轨迹")
        self.play_hz = (
            trajectory.sample_hz
            if play_hz is None and trajectory is not None
            else play_hz
        )
        if trajectory is not None:
            trajectory.validate_playback(self.parameters, self.play_hz)
        self.run_id = secrets.randbelow(2**30 - 1) + 1
        ctx = mp.get_context("spawn")
        self._status_slot = LatestSnapshot(ctx, 131072)
        # 力矩单独跨进程交付，避免受 50 Hz 界面状态发布节拍限制。
        self._torque_slot = LatestSnapshot(ctx, 2048)
        self._torque_sequence = 0
        self._latest_torque = None
        self._reference_slot = LatestSnapshot(ctx, 8192)
        self._input_wakeup = ctx.Event()
        self._stop_reason_slot = LatestSnapshot(ctx, 8192)
        self._submit_lock = threading.Lock()
        self._event = ctx.Event()
        self._lease = ctx.RawValue("d", time.monotonic())
        self._external_event = stop_event
        self._sequence = 0
        self._status_lock = threading.RLock()
        self._status = {
            **idle_impedance_status(),
            "active": True,
            "phase": "PREPARING",
            "stopped_confirmed": False,
            "run_id": self.run_id,
            "source": reference_source,
            "parameter_revision": parameter_revision,
        }
        directory = Path(
            log_directory
            or Path("logs/impedance")
            / (datetime.now().strftime("%Y%m%d_%H%M%S") + f"_{self.run_id}")
        ).resolve()
        directory.mkdir(parents=True, exist_ok=False)
        self.directory = directory
        self._status["log_directory"] = str(directory)
        self._args = dict(
            ip=robot_ip,
            parameters=self.parameters,
            run_id=self.run_id,
            source=reference_source,
            parameter_revision=parameter_revision,
            trajectory=trajectory,
            play_hz=self.play_hz,
            directory=str(directory),
        )
        self._process = ctx.Process(
            target=_main,
            args=(
                self._args,
                self._status_slot,
                self._torque_slot,
                self._reference_slot,
                self._input_wakeup,
                self._stop_reason_slot,
                self._event,
                self._lease,
            ),
            daemon=True,
        )
        self._bridge = None
        self._bridge_stop = threading.Event()
        self._stop_deadline = None
        self._stop_reason = None
        self._submitted_seq = 0
        if initial_reference is not None:
            self.submit_reference(
                replace(initial_reference, run_id=self.run_id)
            )

    @property
    def active(self):
        return self.get_status()["active"]

    @property
    def stop_requested(self):
        """界面或控制进程已请求停止，终态确认可能仍在进行。"""
        return self._event.is_set() or (self._external_event is not None and self._external_event.is_set())

    def get_status(self):
        with self._status_lock:
            self._sequence, value = self._status_slot.read(self._sequence)
            if value is not None:
                self._status.update(value)
            if (
                self._process.pid
                and not self._process.is_alive()
                and self._status["active"]
            ):
                self._status.update(phase="STOP_UNCONFIRMED", running=False)
            if self._stop_reason and not self._status.get("last_error"):
                self._status["last_error"] = self._stop_reason
            return deepcopy(self._status)

    def get_torque_snapshot(self, kind):
        """读取本轮命令或实测力矩，各自按首次接收时刻判断是否过期。"""
        if self._event.is_set() or not self._process.is_alive():
            return None
        with self._status_lock:
            self._torque_sequence, torques = self._torque_slot.read(self._torque_sequence)
            if torques is not None:
                self._latest_torque = torques
            sample = None if self._latest_torque is None else self._latest_torque[kind]
        if sample is None or time.monotonic() - sample["received_monotonic_s"] > self.parameters["FEEDBACK_TIMEOUT_S"]:
            return None
        return sample

    def start(self):
        if self._event.is_set():
            self.cancel_before_start()
            return
        self._lease.value = time.monotonic()
        self._process.start()
        self._bridge = threading.Thread(target=self._bridge_loop, daemon=True)
        self._bridge.start()

    def _bridge_loop(self):
        while not self._bridge_stop.wait(0.02):
            self._lease.value = time.monotonic()
            if self._external_event is not None and self._external_event.is_set():
                self.request_stop()
            self.get_status()
            if not self._process.is_alive():
                break

    def submit_reference(self, frame):
        try:
            with self._submit_lock:
                if self._event.is_set():
                    return False
                age = frame.check(self.run_id, time.monotonic(), self.parameters)
                if frame.source != self.reference_source:
                    raise ValueError("运行中不能切换目标源")
                if frame.source_seq <= self._submitted_seq:
                    return False
                remaining = self.parameters["SOURCE_TIMEOUT_S"] - age
                # 冻结帧的字段由 IPC 立即序列化，六轴元组无需事先深复制。
                if not self._reference_slot.publish(vars(frame), blocking=True, timeout=remaining):
                    raise TimeoutError("目标在等待 IPC 提交时已过期")
                self._submitted_seq = frame.source_seq
                self._input_wakeup.set()
                return True
        except (ValueError, TypeError, TimeoutError) as exc:
            self.request_stop(str(exc))
            raise

    def request_stop(self, reason=None):
        with self._status_lock:
            if self._stop_deadline is None:
                self._stop_deadline = time.monotonic() + self.parameters["STOP_TIMEOUT_S"]
            try:
                if reason and not self._stop_reason:
                    self._stop_reason = reason
                    # 首次停止先写独立原因槽，再置位停止事件；子进程只在置位后读取。
                    self._stop_reason_slot.publish({"stop_reason": reason})
            finally:
                self._event.set()
                self._input_wakeup.set()

    def stop(self):
        self.request_stop()
        while self.active and time.monotonic() < self._stop_deadline:
            if self._process.pid and not self._process.is_alive():
                break
            time.sleep(0.01)
        if self.active:
            # 子进程崩溃不能释放控制权，必须执行独立停止确认。
            from UR_Utils.control_stop import confirm_stopped

            confirm_stopped(
                self.robot_ip,
                deadline=time.monotonic() + self.parameters["STOP_TIMEOUT_S"],
                speed_limit=self.parameters["STATIONARY_QD_RAD_S"],
                window=self.parameters["STOP_CONFIRM_S"],
            )
            self._status.update(
                active=False,
                phase="STOPPED_CONFIRMED",
                stopped_confirmed=True,
                running=False,
                stop_error=None,
            )
        self.close()

    def cancel_before_start(self, error=None):
        if self._process.pid:
            self.request_stop(error)
            return
        self._event.set()
        self._status.update(
            active=False, phase="CANCELLED", stopped_confirmed=True, last_error=error
        )

    def close(self):
        if self.active:
            raise RuntimeError("停止未确认，不能释放阻抗执行权")
        self._bridge_stop.set()
        if self._bridge:
            self._bridge.join(1)
        if self._process.pid:
            self._process.join(4)
