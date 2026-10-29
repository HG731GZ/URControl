"""J1 单次接触人工联调：复用现有导纳窗口，只绑定本轮控制与人工标记。"""

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import signal
import sys
import time

from PyQt5 import QtCore, QtWidgets

from admittance_logger import AdmittanceLogger
from admittance_manual import read_robot_state, require_empty_flange_ready, require_stopped
from admittance_window import AdmittanceWindow
from UR_Utils import admittance_config
from UR_Utils.admittance_process import AdmittanceProcess
from window_display import configure_high_dpi


class ContactSession(QtCore.QObject):
    """窗口按钮启动单轮，停止确认后才能结束；不自动重新启动或复位保护停止。"""

    def __init__(self, window, args):
        super().__init__(window)
        self.window, self.args = window, args
        self.config = admittance_config.j1_contact_config()
        self.controller = self.logger = None
        self.path = None
        self.completed = False
        self.external_confirmation_attempted = False
        self.running_timestamp = None
        self.last_second = None
        self.force_samples = []
        self.force_baseline = None
        self.force_started = False
        self.release_timestamp = None
        self.alerted = False
        self.window.ui.comboBox_ReferenceSource.setCurrentIndex(1)
        self.window.ui.comboBox_AdmittanceSpace.setCurrentIndex(0)
        self.window.ui.comboBox_AngleUnit.setCurrentIndex(1)
        self.window.ui.comboBox_PlotWindow.setCurrentIndex(3)
        self.window.parameters.prepared_parameters = deepcopy(self.config)
        self.window.parameters.draft_parameters = deepcopy(self.config)
        self.window._write_editors()
        self.window.ui.lineEdit_OperatorEventNote.setText("空法兰末端，沿基座 +X，明显让位或红色提示后完全松手")
        self.window.update_context({"robot_ready": True, "robot_address": args.robot_ip,
                                    "phase": "等待现场点击开始：先无接触 3 秒，单次手推后完全松手"})
        self.window.append_log(f"本轮 60 秒。先静止 3 秒，再点“施力开始”并沿基座 +X 缓慢推动。"
                               f"明显让位或红色提示即松手，随后点“完全松手”。"
                               f"末端力增量 {args.force_cue_n:g} N 或 J1 偏移 0.02 rad 时提示撤力；提示不作为力保护。"
                               "35 秒时仍未松手也会提示，松手后至少观察 20 秒。")
        self.window.start_requested.connect(self.start)
        self.window.stop_requested.connect(self.stop)
        self.window.operator_event_requested.connect(self.mark)
        self.window.logging_toggled.connect(self.logging)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(50)
        self.notifier = QtCore.QSocketNotifier(sys.stdin.fileno(), QtCore.QSocketNotifier.Read, self)
        self.notifier.activated.connect(self.console_command)

    def console_command(self):
        """终端只提供查询、停止、重新确认静止和退出，不绕过现场开始按钮。"""
        line = sys.stdin.readline()
        if not line:
            self.notifier.setEnabled(False)
            return
        command = line.strip()
        if command == "stop":
            self.stop("terminal")
        elif command == "confirm":
            self.external_confirmation_attempted = False
        elif command == "quit" and (self.controller is None or self.completed):
            QtWidgets.QApplication.quit()
        elif command == "show":
            self.window.show()
            print(json.dumps({"已完成": self.completed, "日志": None if self.path is None else str(self.path),
                              "控制状态": None if self.controller is None else self.controller.get_status()["phase_name"]},
                             ensure_ascii=False), flush=True)

    def start(self, source, space, parameters, revision):
        """仅启动已约定且在界面显示的候选；不从本入口应用其他配置。"""
        if self.controller is not None:
            return
        if source != "fixed" or space != "joint" or parameters != self.config:
            self.window.append_log("本入口仅运行已约定的 J1 候选；参数已改变，请重新打开本轮窗口。")
            return
        try:
            before = read_robot_state(self.args.robot_ip)
            require_empty_flange_ready(before, self.config)
        except (RuntimeError, OSError, ValueError) as error:
            self.window.append_log(f"启动预检未通过：{error}")
            return
        self.logger = AdmittanceLogger(enabled=True)
        self.logger.start({
            "run_id": f"j1_contact_{time.time_ns()}", "reference_source": "fixed", "space": "joint",
            "prepared_parameters": self.config, "robot_address": self.args.robot_ip,
            "debug_logging_enabled_at_start": True, "manual_validation": True,
            "manual_precheck": before, "validation_environment": self.args.environment,
            "validation_profile": "AdmittanceProcess", "manual_label": self.args.label,
            "manual_contact_context": {"tool": "空法兰", "point": "末端", "direction": "基座 +X",
                                       "contact_planned": True, "single_contact": True,
                                       "force_increment_cue_n": self.args.force_cue_n,
                                       "offset_cue_rad": .02, "latest_release_cue_s": 35.,
                                       "cue_definition": "现场撤力提醒，不是安全额定力保护，不代替实际松手标记"},
            "manual_release_observation": {"minimum_release_s": 20., "tail_duration_s": 5.,
                                           "definition": "报告撤力趋稳和稳定偏移，由用户评价可接受范围，不预设回位容差"},
        })
        self.path = Path(self.logger.get_status()["path"])
        self.controller = AdmittanceProcess(self.args.robot_ip, "fixed", "joint", config=self.config, log_sink=self.logger)
        self.window.set_effective(self.config, revision)
        self.window.update_context({"busy": True, "occupied": True, "run_id": self.logger.get_status()["run_id"]})
        self.controller.start()
        print("接触轮次已启动：" + str(self.path), flush=True)

    def stop(self, source="window"):
        """所有停止来源都交给正式导纳停止契约。"""
        if self.controller is not None and self.controller.active:
            self.logger.record("manual_stop_requested", {"source": source, "host_monotonic_s": time.monotonic()})
            self.controller.request_stop()
            self.cue("请求停止，请保持双手离开机械臂")

    def logging(self, enabled):
        """记录开关沿用现有行为，关闭期间的数据不能补成有效撤力观察。"""
        if self.logger is not None:
            self.logger.set_enabled(enabled)

    def cue(self, reason):
        """单次锁存显著撤力提示，只记录提示时刻，不伪造完全松手事件。"""
        if self.alerted:
            return
        self.alerted = True
        self.window.set_release_alert(True)
        QtWidgets.QApplication.beep()
        self.logger.record("manual_release_cue", {"host_monotonic_s": time.monotonic(), "reason": reason})
        self.window.append_log("立即撤力：" + reason)
        print("立即撤力：" + reason, flush=True)

    def mark(self, event, note):
        """人工按钮与最近状态异步关联，只允许单次成对施力和松手标记。"""
        if self.controller is None or not self.controller.active:
            return
        state = self.controller.get_status()
        if event == "force_start":
            if self.force_baseline is None or self.force_started or not state["running"]:
                self.window.operator_event_result("需完成 3 秒无接触基线，且本轮只允许一次施力开始。")
                return
            self.force_started = True
        if event == "released":
            if not self.force_started or self.release_timestamp is not None:
                self.window.operator_event_result("需先标记施力开始；完全松手标记只能记录一次。")
                return
            self.release_timestamp = state["timestamp"]
        self.logger.record("operator_event", {
            "event": event, "label": event, "note": note,
            "event_monotonic_ns": time.monotonic_ns(), "event_wall_time_ns": time.time_ns(),
            "run_id": self.logger.get_status()["run_id"],
            "latest_robot_state": {"timestamp": state["timestamp"],
                                   "state_received_monotonic": state["state_received_monotonic"]},
            "association": "现场按钮与机器人反馈异步，不代表精确接触或撤力时刻",
        })
        self.window.operator_event_result("已记录：" + event)
        print("人工标记：" + event, flush=True)
        if event in ("buzzing", "stop_requested"):
            self.stop("operator_event:" + event)

    def refresh(self):
        """显示采样只用于人工提示，完整证据由正式 500 Hz 日志链记录。"""
        if self.controller is None or self.completed:
            return
        state = self.controller.get_status()
        if self.controller.needs_external_stop_confirmation() and not self.external_confirmation_attempted:
            self.external_confirmation_attempted = True
            self.cue("通信进程退出，等待独立停止确认")
            try:
                observation = read_robot_state(self.args.robot_ip)
                require_stopped(observation, admittance_config.stop_speed_limit(self.config))
                self.logger.record("manual_stop_observations", observation)
                self.controller.confirm_external_stop(observation["samples"][-1])
            except (RuntimeError, OSError, ValueError) as error:
                self.window.append_log(f"停止仍未确认：{error}")
                print(f"停止仍未确认：{error}", flush=True)
            state = self.controller.get_status()
        phase = state["phase_name"]
        if state["running"]:
            if self.running_timestamp is None:
                self.running_timestamp = state["timestamp"]
            elapsed = state["timestamp"] - self.running_timestamp
            if elapsed < 3:
                self.force_samples.append(state["actual_TCP_force"][:3])
                phase = f"运行 {elapsed:.1f} 秒：保持无接触，正在记录基线"
            else:
                if self.force_baseline is None:
                    self.force_baseline = [sum(row[i] for row in self.force_samples) / len(self.force_samples) for i in range(3)]
                    self.logger.record("manual_contact_baseline", {"force_base_n": self.force_baseline,
                                                                  "display_sample_count": len(self.force_samples)})
                delta = math.dist(state["actual_TCP_force"][:3], self.force_baseline)
                offset = state["actual_q"][0] - state["q_start"][0]
                if self.release_timestamp is None:
                    if delta >= self.args.force_cue_n or abs(offset) >= .02 or elapsed >= 35:
                        self.cue(f"末端力增量 {delta:.1f} N，J1 偏移 {offset:.4f} rad，运行 {elapsed:.1f} 秒")
                    phase = ("立即完全松手，并点击“完全松手”" if self.alerted else "点“施力开始”后单次缓慢推动，明显让位即松手")
                else:
                    released_s = state["timestamp"] - self.release_timestamp
                    phase = f"松手后观察 {released_s:.1f} / 至少 20 秒，请保持无接触"
                phase += f"；力增量 {delta:.1f} N；J1 偏移 {offset:.4f} rad"
            if int(elapsed) != self.last_second:
                print(phase, flush=True)
                self.last_second = int(elapsed)
        if state["stopping"] or state["last_error"]:
            self.cue(state["last_error"] or state["exit_reason_text"])
        self.window.update_context({"phase": phase, "occupied": state["active"],
                                    "stop_pending": state["stopping"], "debug_log_status": self.logger.get_status()})
        snapshot = self.window.monitor.update(state, None, run_id=self.logger.get_status()["run_id"], connected=True,
                                              parameters=self.config, reference_source="fixed", space="joint")
        self.window.update_snapshot(snapshot)
        if not state["active"] and state["stopped_confirmed"]:
            self.controller.close()
            observation = read_robot_state(self.args.robot_ip)
            require_stopped(observation, admittance_config.stop_speed_limit(self.config))
            self.logger.record("manual_postcheck", observation)
            self.logger.finish({"status": state})
            (self.path / "manual_final_status.json").write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")
            self.completed = True
            self.window.update_context({"robot_ready": False, "busy": False, "occupied": False, "stop_pending": False,
                                        "phase": "已停止并保存，请等待本轮分析", "debug_log_status": self.logger.get_status()})
            print(json.dumps({"接触轮次完成": str(self.path), "退出码": state["exit_reason"],
                              "错误": state["last_error"], "最终安全状态": observation["samples"][-1]["safety_mode"]},
                             ensure_ascii=False), flush=True)


def main():
    """创建本轮独立窗口；预览只保存画面，不连接机器人。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-ip", required=True)
    parser.add_argument("--environment", choices=("ursim", "real_robot"), required=True)
    parser.add_argument("--label", required=True)
    parser.add_argument("--empty-flange", action="store_true", required=True)
    parser.add_argument("--force-cue-n", type=float, required=True)
    parser.add_argument("--preview", type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.force_cue_n) or args.force_cue_n <= 0:
        parser.error("撤力提示值须为正有限力增量，单位 N")
    configure_high_dpi()
    app = QtWidgets.QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(False)
    window = AdmittanceWindow(settings=QtCore.QSettings("/tmp/urcontrol_j1_contact_20261029.ini", QtCore.QSettings.IniFormat))
    window.setWindowTitle("J1 单次手推联调：60 秒 · 撤力后至少观察 20 秒")
    session = ContactSession(window, args)
    def handle_signal(number, frame):
        """尚有控制权时先停止，未启动或已收尾时允许关闭窗口进程。"""
        if session.controller is None or session.completed:
            app.quit()
        else:
            session.stop("signal:" + str(number))

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    window.showMaximized()
    if args.preview:
        app.processEvents()
        window.grab().save(str(args.preview))
        print("离线预览已保存：" + str(args.preview))
        return 0
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
