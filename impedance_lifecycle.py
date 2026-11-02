"""Qt 后台交接/停止及主窗口集成；窗口隐藏不结束控制。"""

import threading
from PyQt5 import QtCore
from impedance_window import ImpedanceWindow
from UR_Utils.impedance_sources import UDPImpedanceSource
from UR_Utils.impedance_reference import JointTargetFrame
from UR_Utils.URImpedanceController import idle_impedance_status


class ImpedanceWorker(QtCore.QThread):
    def __init__(self, controller, operation, kwargs=None, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.operation = operation
        self.kwargs = kwargs or {}
        self.error = None

    def run(self):
        try:
            if self.operation == "start":
                self.controller.start_impedance(**self.kwargs)
            else:
                self.controller.stop(stop_script=True)
        except Exception as exc:
            self.error = str(exc)


class ImpedanceIntegration:
    def init_impedance(self):
        self._impedance_runs = []
        self.impedance_window = None
        self._impedance_worker = None
        self._impedance_adapter = None
        self._impedance_after_stop = None
        self._impedance_event = None
        self._impedance_owned = False
        self._impedance_timer = QtCore.QTimer(self)
        self._impedance_timer.timeout.connect(self.refresh_impedance)
        self._impedance_timer.start(100)

    def impedance_busy(self):
        return (
            self._impedance_owned
            or self._impedance_worker is not None
            or (
                self.rtde_controller is not None
                and self.rtde_controller.get_impedance_status()["active"]
            )
        )

    def on_impedance(self):
        if self.impedance_window is None:
            self.impedance_window = ImpedanceWindow(self)
            self.impedance_window.start_requested.connect(self.start_impedance_run)
            self.impedance_window.stop_requested.connect(self.request_impedance_stop)
            self.impedance_window.sync_requested.connect(self.on_sync_udp)
            self.impedance_window.move_start_requested.connect(
                self.move_impedance_start
            )
        self.refresh_impedance()
        self.impedance_window.show()
        self.impedance_window.raise_()

    def move_impedance_start(self, q):
        if (
            self.impedance_busy()
            or self.control_source != "idle"
            or self.rtde_controller is None
        ):
            return
        try:
            self.rtde_controller.moveJ(
                q, speed=0.15, acceleration=0.3, asynchronous=True
            )
            self.control_source = "moveJ"
            self._move_target = q
        except Exception as exc:
            self.append_message(str(exc))

    def start_impedance_run(self, kwargs):
        if (
            self.impedance_busy()
            or self.control_source != "idle"
            or self.admittance_mode != "off"
            or self.rtde_controller is None
        ):
            self.append_message("控制权尚未空闲，不能开始阻抗")
            return
        self._impedance_event = threading.Event()
        kwargs["stop_event"] = self._impedance_event
        if kwargs["reference_source"] == "teleop":
            s = self.udp_client.get_monitor_snapshot()
            command = s["command"]
            if (
                command is None
                or command.mode != 1
                or s["last_error"]
                or s["received_monotonic"] is None
            ):
                self.append_message("没有有效 JOINT_TRACK 首目标")
                return
            kwargs["initial_reference"] = JointTargetFrame(
                1,
                "teleop",
                s["frame_count"],
                tuple(command.q_arm),
                s["received_monotonic"],
            )
        self.udp_control_timer.stop()
        self.jog_timer.stop()
        if self.gripper is not None:
            self.gripper.pause()
        self.control_source = "impedance"
        self._impedance_owned = True
        self._launch_impedance_worker("start", kwargs)

    def _launch_impedance_worker(self, operation, kwargs=None):
        worker = ImpedanceWorker(self.rtde_controller, operation, kwargs, self)
        self._impedance_worker = worker
        worker.finished.connect(self._impedance_worker_finished)
        worker.start()

    def _impedance_worker_finished(self):
        worker = self._impedance_worker
        self._impedance_worker = None
        if worker.error:
            self.append_message("阻抗：" + worker.error)
        if worker.operation == "start" and not worker.error:
            backend = self.rtde_controller._impedance
            if backend:
                self._impedance_runs.append(
                    dict(
                        run_id=backend.run_id,
                        parameters=backend.parameters,
                        source=backend.reference_source,
                        log_directory=backend.get_status().get("log_directory"),
                    )
                )
                self.collector.update_metadata({"impedance_runs": self._impedance_runs})
            if backend and backend.reference_source == "teleop" and backend.active:
                self._impedance_adapter = UDPImpedanceSource(self.udp_client, backend)
                self._impedance_adapter.start()
        worker.deleteLater()
        self.refresh_impedance()

    def request_impedance_stop(self):
        if self._impedance_event:
            self._impedance_event.set()
        if self._impedance_adapter:
            self._impedance_adapter.close()
            self._impedance_adapter = None
        if self.gripper is not None:
            self.gripper.pause()
        if self.rtde_controller:
            self.rtde_controller.request_impedance_stop()
        if (
            self._impedance_worker is None
            and self.rtde_controller
            and self.rtde_controller.get_impedance_status()["active"]
        ):
            self._launch_impedance_worker("stop")

    def defer_until_impedance_stopped(self, action):
        if not self.impedance_busy():
            return False
        self._impedance_after_stop = action
        self.request_impedance_stop()
        return True

    def refresh_impedance(self):
        status = (
            self.rtde_controller.get_impedance_status()
            if self.rtde_controller
            else idle_impedance_status()
        )
        if (
            self._impedance_owned
            and not status["active"]
            and self._impedance_worker is None
        ):
            self._impedance_owned = False
            self.control_source = "idle"
            self._udp_synced = False
            if self._impedance_runs and self._impedance_runs[-1][
                "run_id"
            ] == status.get("run_id"):
                self._impedance_runs[-1]["final_status"] = status
                self.collector.update_metadata({"impedance_runs": self._impedance_runs})
            if self._impedance_adapter:
                self._impedance_adapter.close()
                self._impedance_adapter = None
            self.append_message(
                f'阻抗结束：{status.get("last_error") or status["phase"]}；日志 {status.get("log_directory")}'
            )
            action = self._impedance_after_stop
            self._impedance_after_stop = None
            if action and status["stopped_confirmed"]:
                QtCore.QTimer.singleShot(0, action)
        if status.get("running") and status.get("source") == "teleop":
            snapshot = self.udp_client.get_monitor_snapshot()
            command = snapshot["command"]
            import time

            if (
                command
                and command.mode == 1
                and not snapshot["last_error"]
                and snapshot["received_monotonic"]
                and time.monotonic() - snapshot["received_monotonic"] < 0.25
                and self.get_gripper_feedback() is not None
            ):
                self.gripper.set_target_position(command.q_gripper[0])
                if self.gripper.is_paused:
                    self.gripper.resume()
        elif self._impedance_owned and self.gripper is not None:
            self.gripper.pause()
        if self.impedance_window:
            self.impedance_window.update_status(
                status,
                ready=self.robot_ready
                and self.rtde_controller is not None
                and self.control_source in ("idle", "impedance")
                and self.admittance_mode == "off",
                busy=self._impedance_worker is not None,
            )
        if hasattr(self, "pushButton_Impedance"):
            hidden = self.impedance_busy() and (
                self.impedance_window is None or not self.impedance_window.isVisible()
            )
            self.pushButton_Impedance.setText(
                "阻抗运行中（窗口已隐藏）"
                if hidden
                else "关节阻抗 · " + status["phase"]
            )
            self.pushButton_Impedance.setToolTip(
                status.get("last_error") or status.get("log_directory") or ""
            )
