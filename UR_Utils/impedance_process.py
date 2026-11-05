"""spawn 隔离通信，独立停止事件和父进程租约；停止未确认保持执行权。"""

from dataclasses import asdict, replace
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


def _main(args, status_slot, reference_slot, stop_event, lease):
    parent = mp.parent_process()
    sequence = 0
    last_publish = 0

    def service(backend):
        nonlocal sequence, last_publish
        now = time.monotonic()
        if not parent.is_alive() or now - lease.value > 1:
            backend.request_stop("父进程租约失效")
        sequence, data = reference_slot.read(sequence)
        if data is not None:
            if data.get("stop_reason"):
                backend.request_stop(data["stop_reason"])
            else:
                backend.frame = JointTargetFrame(**data)
        if (
            now - last_publish >= 0.02
            or not backend.status["active"]
            or backend.status["phase"] == "STOP_UNCONFIRMED"
        ):
            status_slot.publish(backend.status)
            last_publish = now

    try:
        backend = URImpedanceController(**args, stop_event=stop_event, service=service)
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
        self.parameters = validate(config)
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
        self._reference_slot = LatestSnapshot(ctx, 8192)
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
                self._reference_slot,
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
            frame.check(self.run_id, time.monotonic(), self.parameters)
            if frame.source != self.reference_source:
                raise ValueError("运行中不能切换目标源")
        except (ValueError, TypeError) as exc:
            self.request_stop(str(exc))
            raise
        if frame.source_seq <= self._submitted_seq:
            return False
        accepted = self._reference_slot.publish(asdict(frame))
        if accepted:
            self._submitted_seq = frame.source_seq
        return accepted

    def request_stop(self, reason=None):
        if self._stop_deadline is None:
            self._stop_deadline = time.monotonic() + self.parameters["STOP_TIMEOUT_S"]
        if reason:
            self._stop_reason = reason
            self._reference_slot.publish({"stop_reason": reason})
        self._event.set()

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
