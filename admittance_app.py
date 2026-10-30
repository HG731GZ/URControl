"""空法兰固定参考导纳独立入口：只加载导纳窗口与正式控制进程，无需主端。"""

import argparse
from copy import deepcopy
import signal
import sys
import time

from PyQt5 import QtCore, QtWidgets

from admittance_logger import AdmittanceLogger
from admittance_manual import read_robot_state, require_empty_flange_ready, require_stopped
from admittance_window import AdmittanceWindow, OPERATOR_EVENTS
from UR_Utils import admittance_config
from UR_Utils.admittance_process import AdmittanceProcess
from window_display import configure_high_dpi


class AdmittanceTask(QtCore.QThread):
    """串行执行只读预检或停止收尾，结果回到界面线程处理。"""

    def __init__(self, action, parent):
        super().__init__(parent)
        self.action = action
        self.result = None
        self.error = None

    def run(self):
        try:
            self.result = self.action()
        except Exception as error:
            # 操作边界明确显示失败，保留未确认停止的控制权，不自动重试。
            self.error = f"{type(error).__name__}：{error}"


class StandaloneAdmittanceWindow(AdmittanceWindow):
    """独立入口关闭时必须完成停止；主程序中的监视窗口仍沿用隐藏行为。"""

    close_requested = QtCore.pyqtSignal()

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.allow_close = False
        self.ui.widget_StandaloneConnection.show()
        self.ui.pushButton_SyncMaster.hide()
        self.update_context({"standalone": True})

    def closeEvent(self, event):
        if self.allow_close:
            self._save_settings()
            event.accept()
        else:
            event.ignore()
            self.close_requested.emit()


class StandaloneAdmittanceSession(QtCore.QObject):
    """每次启动重新预检及采锚点；停止确认前禁止下一轮和退出窗口。"""

    def __init__(self, window, *, robot_ip=None, logging_enabled=False, auto_check=True):
        super().__init__(window)
        self.window = window
        self.controller = None
        self.logger = AdmittanceLogger(enabled=logging_enabled)
        self.task = None
        self.start_pending = False
        self.cancel_start = False
        self.close_pending = False
        self.stop_recheck_required = False
        self.robot_ready = False
        self.config = window.parameters.prepared_copy()
        self.run_metadata = {}
        self.last_status = None
        self.reported_error = None
        self.phase = "等待连接检查"
        saved_ip = window.settings.value("admittanceStandalone/robotAddress", "192.168.3.15")
        window.ui.lineEdit_StandaloneAddress.setText(saved_ip if robot_ip is None else robot_ip)
        window.ui.pushButton_CheckConnection.clicked.connect(self.check_connection)
        window.ui.lineEdit_StandaloneAddress.textEdited.connect(self.address_changed)
        window.start_requested.connect(self.start)
        window.stop_requested.connect(self.stop)
        window.close_requested.connect(self.close)
        window.logging_toggled.connect(self.set_logging)
        window.operator_event_requested.connect(self.record_event)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(50)
        self.sync_controls()
        if auto_check:
            QtCore.QTimer.singleShot(0, self.check_connection)

    def address_changed(self):
        self.robot_ready = False
        self.phase = "地址已更改，请检查连接"
        self.sync_controls()

    def dispatch(self, action, completed):
        """一个窗口只允许一个预检或收尾任务，引用保留到线程已经退出。"""
        task = AdmittanceTask(action, self)
        self.task = task

        def finished():
            self.task = None
            completed(task.result, task.error)
            task.deleteLater()
            self.sync_controls()
            self.finish_close()

        task.finished.connect(finished)
        task.start()
        self.sync_controls()

    def precheck(self, address, config):
        observation = read_robot_state(address)
        require_empty_flange_ready(observation, config)
        return observation

    def check_connection(self):
        if self.task is not None or self.controller is not None:
            return
        address = self.window.ui.lineEdit_StandaloneAddress.text().strip()
        config = self.window.parameters.prepared_copy()
        self.robot_ready = False
        self.phase = "正在只读检查实机状态"

        def checked(observation, error):
            self.robot_ready = error is None
            self.phase = ("空法兰实机已就绪；保持无接触，点击开始导纳" if error is None
                          else "连接检查未通过：" + error)
            self.window.append_log(self.phase)
            if error is None:
                self.window.settings.setValue("admittanceStandalone/robotAddress", address)

        self.dispatch(lambda: self.precheck(address, config), checked)

    def start(self, source, space, config, revision):
        if self.task is not None or self.controller is not None or self.close_pending:
            return
        try:
            if source != "fixed" or space != "joint":
                raise ValueError("独立入口使用固定参考关节导纳，无需主端")
            if self.window.parameters.has_unapplied_changes:
                raise ValueError("请先应用草稿参数")
            self.config = admittance_config.validate_run_config(source, space, config=config)
        except ValueError as error:
            self.window.append_log(str(error))
            return
        address = self.window.ui.lineEdit_StandaloneAddress.text().strip()
        self.start_pending = True
        self.cancel_start = False
        self.phase = "启动前检查：请保持静止、无接触"

        def checked(observation, error):
            self.start_pending = False
            if error is not None:
                self.robot_ready = False
                self.phase = "启动预检未通过：" + error
                self.window.append_log(self.phase)
                return
            if self.cancel_start:
                self.phase = "已取消启动，未上传运动程序"
                return
            self.robot_ready = True
            self.reported_error = None
            self.last_status = None
            self.stop_recheck_required = False
            self.window.clear_effective()
            self.window.set_release_alert(False)
            self.run_metadata = {
                "run_id": f"standalone_fixed_{time.time_ns()}", "robot_address": address,
                "reference_source": "fixed", "space": "joint", "parameter_revision": revision,
                "prepared_parameters": deepcopy(self.config), "manual_precheck": observation,
                "entrypoint": "admittance_app.py", "tool": "空法兰",
                "debug_logging_enabled_at_start": self.window.debug_logging_enabled,
                "reference_behavior": "fixed_anchor", "continuous_until_stop": self.config["DIAGNOSTIC_DURATION_S"] == 0,
            }
            self.logger = AdmittanceLogger(enabled=self.window.debug_logging_enabled)
            self.logger.start(self.run_metadata)
            try:
                self.controller = AdmittanceProcess(address, "fixed", "joint", config=self.config, log_sink=self.logger)
                self.controller.start()
            except Exception as error:
                # 进程创建失败时仍按拥有者状态收尾，不把可能已启动的进程当作未运行。
                self.phase = "控制进程启动失败：" + str(error)
                self.window.append_log(self.phase)
                if self.controller is not None:
                    self.controller.cancel_before_start(str(error))
                else:
                    self.dispatch(lambda message=str(error): self.logger.finish({"start_error": message}),
                                  lambda result, failure: self.window.append_log(failure or "启动失败记录已收尾"))
                return
            self.phase = "正在保持并采集偏置，请勿接触机器人"
            self.window.append_log(self.phase)

        self.dispatch(lambda: self.precheck(address, self.config), checked)

    def stop(self):
        self.cancel_start = True
        if self.controller is not None:
            self.controller.request_stop()
            self.phase = "正在停止；请保持撤力，等待程序退出及静止确认"
            if self.stop_recheck_required and self.task is None:
                self.stop_recheck_required = False
                self.finish_run()
        elif self.start_pending:
            self.phase = "正在取消启动，等待只读检查结束"
        self.sync_controls()

    def finish_run(self):
        """复用正式停止契约，再独立核对连续反馈；失败只允许显式再次复核。"""
        controller, logger = self.controller, self.logger
        address = self.run_metadata["robot_address"]
        self.phase = "正在独立确认停止并保存本轮记录"

        def finish():
            observation = read_robot_state(address)
            logger.record("standalone_stop_observations", observation)
            require_stopped(observation, admittance_config.stop_speed_limit(self.config))
            if controller.needs_external_stop_confirmation():
                controller.confirm_external_stop(observation["samples"][-1])
            controller.close()
            status = controller.get_status()
            logger.finish({"status": status, "postcheck": observation})
            return status, observation

        def finished(result, error):
            if error is not None:
                self.stop_recheck_required = True
                self.phase = "停止收尾未完成：" + error + "；保持撤力，点击停止导纳再次复核"
                self.window.append_log(self.phase)
                return
            status, observation = result
            self.last_status = status
            self.run_metadata.update(final_status=status, postcheck=observation)
            self.controller = None
            self.robot_ready = observation["samples"][-1]["safety_mode"] == 1
            reason = status["last_error"] or status["exit_reason_text"]
            self.phase = f"已确认停止：{reason}；再次启动会重新记录当前位置"
            self.window.append_log(self.phase)
            print(self.phase, flush=True)

        self.dispatch(finish, finished)

    def refresh(self):
        if self.controller is not None:
            state = self.controller.get_status()
            self.last_status = state
            self.run_metadata["status"] = state
            if state["derived_valid"] and self.window.parameters.effective_parameters is None:
                self.window.set_effective(state["parameters"], self.run_metadata["parameter_revision"])
            if self.task is None and not self.stop_recheck_required:
                self.phase = state["phase_name"]
                if state["running"]:
                    self.phase = "固定参考关节导纳运行中；松手回参考附近，结束请点击停止"
                if state["last_error"] and state["last_error"] != self.reported_error:
                    self.reported_error = state["last_error"]
                    self.window.append_log("本轮停止原因：" + self.reported_error)
                    self.window.set_release_alert(True)
                if controller_done(state) or self.controller.needs_external_stop_confirmation():
                    self.finish_run()
            if self.window.isVisible():
                snapshot = self.window.monitor.update(
                    state, None, run_id=self.run_metadata["run_id"], connected=True,
                    parameters=self.config, reference_source="fixed", space="joint")
                self.window.update_snapshot(snapshot)
        self.sync_controls()

    def sync_controls(self):
        occupied = self.controller is not None or self.start_pending
        self.window.update_context({
            "standalone": True, "robot_ready": self.robot_ready, "busy": self.task is not None,
            "occupied": occupied, "stop_pending": self.stop_recheck_required,
            "phase": self.phase, "robot_address": self.window.ui.lineEdit_StandaloneAddress.text().strip(),
            "run_id": self.run_metadata.get("run_id"), "run_metadata": self.run_metadata,
            "debug_log_status": self.logger.get_status(),
        })
        editable = not occupied and self.task is None and not self.close_pending
        self.window.ui.lineEdit_StandaloneAddress.setEnabled(editable)
        self.window.ui.pushButton_CheckConnection.setEnabled(editable)

    def set_logging(self, enabled):
        self.logger.set_enabled(enabled, context=self.run_metadata)
        self.sync_controls()

    def record_event(self, event, note):
        if self.controller is None:
            return
        state = self.controller.get_status()
        recorded = self.logger.record("operator_event", {
            "event": event, "label": OPERATOR_EVENTS[event], "note": note,
            "event_monotonic_ns": time.monotonic_ns(), "event_wall_time_ns": time.time_ns(),
            "run_id": self.run_metadata["run_id"],
            "association": "点击时最近收到的机器人状态，与物理施力时刻及机器人采样异步",
            "latest_robot_state": {
                "timestamp": state["timestamp"], "state_receive_count": state["state_receive_count"],
                "state_received_monotonic": state["state_received_monotonic"],
            },
        })
        self.window.operator_event_result(OPERATOR_EVENTS[event] + ("已提交日志" if recorded else "未记录：日志未开启"))
        if event in ("buzzing", "stop_requested"):
            self.stop()

    def close(self):
        self.close_pending = True
        self.stop()
        self.finish_close()

    def finish_close(self):
        if self.close_pending and self.controller is None and self.task is None:
            self.timer.stop()
            self.window.allow_close = True
            self.window.close()


def controller_done(state):
    """终态必须同时包含停止确认，不能只依据运行标记消失。"""
    return not state["active"] and state["stopped_confirmed"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", help="实机地址；未指定时使用上次地址，首次为 192.168.3.15")
    parser.add_argument("--log", action="store_true", help="本次开启完整导纳调试日志，运行中可在窗口切换")
    args = parser.parse_args()
    configure_high_dpi()
    app = QtWidgets.QApplication(sys.argv[:1])
    window = StandaloneAdmittanceWindow()
    session = StandaloneAdmittanceSession(window, robot_ip=args.robot_ip, logging_enabled=args.log)
    signal.signal(signal.SIGINT, lambda *_: session.close())
    signal.signal(signal.SIGTERM, lambda *_: session.close())
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
