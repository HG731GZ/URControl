"""关节人工接触联调：单轴、J1+J3 或六轴固定参考，记录接触与撤力观察。"""

import argparse
import json
import math
from pathlib import Path
import signal
import sys
import time

from PyQt5 import QtCore, QtWidgets

from admittance_logger import AdmittanceLogger
from admittance_manual import read_robot_state, require_empty_flange_ready, require_stopped
from admittance_window import AdmittanceWindow, OPERATOR_EVENTS
from analyze_admittance_log import analyze
from UR_Utils import admittance_config
from UR_Utils.admittance_process import AdmittanceProcess
from window_display import configure_high_dpi


class ContactSession(QtCore.QObject):
    """窗口按钮启动单轮，停止确认后才能结束；不自动重新启动或复位保护停止。"""

    def __init__(self, window, args):
        super().__init__(window)
        self.window, self.args = window, args
        # 各轴初值在配置文件独立定义；以下观测带只控制人工提示和记录。
        self.config = admittance_config.joint_contact_config(tuple(args.axes))
        self.axis = (args.observation_axis if args.observation_axis is not None
                     else self.config["DIAGNOSTIC_AXIS"]) - 1
        self.config["DIAGNOSTIC_AXIS"] = self.axis + 1
        self.axis_label = f"J{self.axis + 1}"
        self.axes_label = "六轴" if len(args.axes) == 6 else "+".join(f"J{axis}" for axis in args.axes)
        offsets, speeds, _, _ = admittance_config.joint_command_limits(self.config)
        self.command_limit = offsets[self.axis]
        self.command_speed = speeds[self.axis]
        self.motion_text = "；".join(f"J{axis} ±{offsets[axis - 1]:g} rad / {speeds[axis - 1]:g} rad/s"
                                   for axis in args.axes)
        self.sustained = args.protocol == "sustained"
        self.timed_push = args.protocol == "timed_push"
        self.boundary_hold = args.protocol == "boundary_hold"
        self.guided = self.timed_push or self.boundary_hold
        # 开始手推前的无接触基线时长，单位 s；用于显示力增量，区别于机器人内部的偏置采样时间。
        self.baseline_duration_s = 5. if self.guided else 3.
        # 单位 s：timed_push 是连续手推提示时长；boundary_hold 是最多允许的接近边界时长。
        # boundary_hold 到达后另计 hold_duration_s，未到达则在接近窗口结束时提示松手，不自动重试。
        self.push_duration_s = 20.
        # 撤力提示到计划无接触观察之间的宽限，单位 s；程序不把宽限结束自动当作真实松手证明。
        self.release_grace_s = 5.
        self.protocol_events = {}
        # 保持提示时长，单位 s；边界协议自动开始，期间离开边界也不重启计时，真实停留事后单独统计。
        self.hold_duration_s = 5.
        # 边界到达检测的命令观测带，单位 rad：abs((q_cmd - q_start) - 同侧边界偏移) <= 此值。
        # 同侧命令边界来自选定观测轴的逐轴命令参数。
        # 0.0002 只表示距边界多近时开始提示，不会加到命令行程上。
        self.boundary_command_band_rad = .0002
        # 边界到达检测的实际位置观测带，单位 rad：abs((actual_q - q_start) - 同侧边界偏移) <= 此值。
        # 允许观测实际跟踪偏差，命令仍限制在本轮所选轴的边界内。
        # 这是保持提示及事后停留统计的范围，不是 0.005 rad 跟踪停止阈值或 ±0.5 rad 实际位置守卫。
        self.boundary_actual_band_rad = .001
        # 到达确认时间，单位 s：两类观测连续满足同侧边界条件后，才开始保持提示倒计时。
        self.boundary_confirm_s = .2
        # 边界判定可采用的最大来源接收年龄，单位 s：执行页及原始反馈均须在此时间内更新。
        # 它避免用陈旧显示数据认定到达，不改变通信故障超时；两个观测页仍是异步采样。
        self.boundary_max_sample_age_s = .2
        self.boundary_candidate = None
        self.boundary_side = 0
        # 人工位移提醒线，单位 rad；single/sustained 跨过会提示撤力，timed_push/boundary_hold 仅作观测。
        # 力增量提醒值由命令行 --force-cue-n 提供，本次为 25 N；两种自动协议同样不据此提前撤力。
        # 这些提醒不构成机器人力保护。计时调整还须同步本入口的现场文案、元数据及整轮 60 秒安排，
        # 保留撤力宽限后至少 20 秒的有效观察；正在运行的一轮不会因修改源文件而更新。
        self.offset_cue_rad = .05 if self.sustained or self.guided else .02
        self.controller = self.logger = None
        self.path = None
        self.completed = False
        self.postcheck_pending = False
        self.external_confirmation_attempted = False
        self.running_timestamp = None
        self.running_monotonic = None
        self.last_second = None
        self.force_samples = []
        self.force_baseline = None
        self.force_started = False
        self.hold_started_monotonic = None
        self.release_monotonic = None
        self.alerted = False
        self.window.ui.comboBox_ReferenceSource.setCurrentIndex(1)
        self.window.ui.comboBox_AdmittanceSpace.setCurrentIndex(0)
        self.window.ui.comboBox_AngleUnit.setCurrentIndex(1)
        self.window.ui.comboBox_PlotWindow.setCurrentIndex(3)
        self.window.ui.comboBox_PlotAxis.setCurrentIndex(self.axis)
        self.window.set_candidate(self.config)
        instruction = (f"{args.contact_point}，{args.contact_direction}；轻推让位后保持力度 5 秒，不追赶位置或继续加力"
                       if self.sustained else f"{args.contact_point}，{args.contact_direction}；明显让位或红色提示后完全松手")
        if self.timed_push:
            instruction = f"启动后无接触 5 秒，按提示轻推 {self.axes_label} 20 秒，红色提示后完全松手；中途无需点击按钮"
        elif self.boundary_hold:
            instruction = "先无接触 5 秒，再缓慢推向任选一侧边界；提示保持后维持现有轻推 5 秒，红色提示后完全松手"
        self.window.ui.lineEdit_OperatorEventNote.setText(instruction)
        self.window.ui.pushButton_EventForceStart.setVisible(not self.guided)
        self.window.ui.pushButton_EventHoldStart.setVisible(self.sustained)
        self.window.ui.pushButton_EventReleased.setVisible(not self.guided)
        self.window.ui.label_ContactStage.setVisible(True)
        self.window.ui.label_ContactStage.setText(instruction)
        self.window.update_context({"robot_ready": True, "robot_address": args.robot_ip,
                                    "phase": f"等待现场点击开始：先无接触 {self.baseline_duration_s:g} 秒，再按顶部提示操作"})
        action = ("出现让位后点击左上方绿色“开始保持受载”，保持轻推力度约 5 秒，计时结束或红色提示即完全松手。"
                  if self.sustained else "明显让位或红色提示即松手。")
        if self.guided:
            self.window.ui.label_OperatorEventStatus.setText(
                "仅需启动一次；声音和顶部文字引导操作。程序记录的是提示时刻，结束后再确认实际施力与松手情况。")
        if self.boundary_hold:
            self.window.append_log(
                "本轮 60 秒：无接触 5 秒，最多用 20 秒缓慢推向同一侧边界。检测到接近边界后自动提示保持 5 秒，"
                "保持已有轻推，不继续加力、不追赶位置；无需恒力或中途点击。随后红色提示要求完全松手，"
                "留出 5 秒撤力时间，再无接触观察到结束。未到边界也按时提示松手，不自动重试。"
                f"保持 {self.axes_label} 回位模型、{self.motion_text} 和故障停止；"
                f"{args.force_cue_n:g} N 与 {self.offset_cue_rad:g} rad 仅作数值观察。")
        elif self.timed_push:
            self.window.append_log(
                "本轮 60 秒：前 5 秒无接触，随后连续轻推 20 秒。声音及红色提示后完全松手，"
                f"留出 5 秒撤力时间，再记录至少 20 秒计划无接触观察。{self.axes_label} 响应，保留回位弹簧、"
                f"{self.motion_text} 和故障停止；接近行程边界时不继续加力。"
                f"{args.force_cue_n:g} N 与 {self.offset_cue_rad:g} rad 仅作数值观察，不触发提前撤力。")
        else:
            self.window.append_log(f"本轮 60 秒。先静止 3 秒，再点“施力开始”；{args.contact_point}，{args.contact_direction}。"
                                   f"{action}随后点“完全松手”。"
                                   f"末端力增量 {args.force_cue_n:g} N 或启用轴偏移 {self.offset_cue_rad:g} rad 时提示撤力；末端测力不代表连杆接触力。"
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
            self.window.append_log(f"本入口仅运行已选择的 {self.axes_label} 候选；参数已改变，请重新打开本轮窗口。")
            return
        try:
            before = read_robot_state(self.args.robot_ip)
            require_empty_flange_ready(before, self.config)
        except (RuntimeError, OSError, ValueError) as error:
            self.window.append_log(f"启动预检未通过：{error}")
            return
        self.logger = AdmittanceLogger(enabled=True)
        self.logger.start({
            "run_id": f"joint_contact_{time.time_ns()}", "reference_source": "fixed", "space": "joint",
            "prepared_parameters": self.config, "robot_address": self.args.robot_ip,
            "debug_logging_enabled_at_start": True, "manual_validation": True,
            "manual_precheck": before, "validation_environment": self.args.environment,
            "validation_profile": "AdmittanceProcess", "manual_label": self.args.label,
            "manual_contact_context": {"tool": "空法兰", "point": self.args.contact_point,
                                       "direction": self.args.contact_direction,
                                       "enabled_axes": self.args.axes, "observation_axis": self.axis + 1,
                                       "contact_planned": True, "single_contact": not self.timed_push,
                                       "protocol": self.args.protocol,
                                       "force_increment_cue_n": self.args.force_cue_n,
                                       "offset_cue_rad": self.offset_cue_rad,
                                       "latest_release_cue_s": 30. if self.boundary_hold else 25. if self.timed_push else 35.,
                                       "cue_definition": ("力与偏移数值仅观测；定时提示要求松手，不代表实际松手，不是安全额定力保护"
                                                          if self.guided else "现场撤力提醒，不是安全额定力保护，不代替实际松手标记")},
            "manual_release_observation": {"minimum_release_s": 20., "tail_duration_s": 5.,
                                           "definition": "报告撤力趋稳和稳定偏移，由用户评价可接受范围，不预设回位容差"},
        })
        if self.sustained:
            self.logger.update_context({"manual_hold_observation": {
                "version": 1, "minimum_hold_s": self.hold_duration_s, "tail_duration_s": 2.,
                "definition": "从人工开始保持受载到完全松手，单独描述受载漂移、摆幅与速度；不预设稳定容差，不以计时证明实际受载",
            }})
        if self.guided:
            observation = {
                "version": 1, "baseline_s": self.baseline_duration_s, "push_duration_s": self.push_duration_s,
                "protocol": self.args.protocol,
                "release_grace_s": self.release_grace_s, "minimum_observation_s": 20., "tail_duration_s": 5.,
                "clock": "主机单调时间；界面倒计时与自动事件窗口共用，不以机器人模拟时间代替人工等待时间",
                "definition": "自动记录计划手推、撤力提示和计划观察起点；不是人工接触或松手事件，现场动作在运行后另行确认",
                "force_and_offset_policy": f"{self.args.force_cue_n:g} N 和 {self.offset_cue_rad:g} rad 仅作观测，不要求提前撤力；运动、通信及故障停止保持生效",
            }
            if self.boundary_hold:
                observation.update(push_duration_s=None, boundary_hold={
                    "approach_timeout_s": self.push_duration_s, "hold_duration_s": self.hold_duration_s,
                    "hold_tail_s": 2., "command_limit_rad": self.command_limit,
                    "observation_axis": self.axis + 1,
                    "command_band_rad": self.boundary_command_band_rad,
                    "actual_band_rad": self.boundary_actual_band_rad,
                    "confirmation_s": self.boundary_confirm_s,
                    "maximum_sample_age_s": self.boundary_max_sample_age_s,
                    "raw_feedback_period_s": 1. / self.config["DIAGNOSTIC_STATE_HZ"],
                    "definition": "同侧采用命令与实际反馈连续接近边界后提示保持；这些观测带只用于计时，不改变运动限幅。离开边界不重启计时，不将提示跨度当作连续受载证据。",
                    "boundary_policy": "到边界后继续原有限位保持；不调整锚点、不切换控制律、不自动卸载或延长行程",
                })
            self.logger.update_context({"manual_timed_push_observation": observation})
        self.path = Path(self.logger.get_status()["path"])
        self.controller = AdmittanceProcess(self.args.robot_ip, "fixed", "joint", config=self.config, log_sink=self.logger)
        self.window.set_effective(self.config, revision)
        self.window.update_context({"busy": True, "occupied": True, "run_id": self.logger.get_status()["run_id"]})
        self.controller.start()
        print("接触轮次已启动：" + str(self.path), flush=True)

    def stop(self, source="window"):
        """所有停止来源都交给正式导纳停止契约。"""
        if self.postcheck_pending:
            self.finish_stop(self.controller.get_status())
            return
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
        if self.guided:
            QtCore.QTimer.singleShot(250, QtWidgets.QApplication.beep)
            QtCore.QTimer.singleShot(500, QtWidgets.QApplication.beep)
        self.logger.record("manual_release_cue", {"host_monotonic_s": time.monotonic(), "reason": reason})
        self.window.append_log("立即撤力：" + reason)
        print("立即撤力：" + reason, flush=True)

    def mark(self, event, note):
        """人工按钮与最近状态异步关联，只允许单次成对施力和松手标记。"""
        if self.controller is None or not self.controller.active:
            return
        state = self.controller.get_status()
        marked_ns = time.monotonic_ns()
        if event in ("force_start", "hold_start", "released"):
            log_status = self.logger.get_status()
            if not log_status["enabled"] or not log_status["active"] or log_status["error"]:
                self.window.operator_event_result("日志未正常记录，不能提交接触阶段标记。")
                return
        if event == "force_start":
            if self.force_baseline is None or self.force_started or not state["running"] or self.alerted:
                self.window.operator_event_result("需完成 3 秒无接触基线，且本轮只允许一次施力开始。")
                return
            self.force_started = True
        if event == "hold_start":
            if (not self.sustained or not self.force_started or self.hold_started_monotonic is not None
                    or self.release_monotonic is not None or not state["running"] or self.alerted):
                self.window.operator_event_result("持续受载仅能在施力开始后、撤力提示前标记一次。")
                return
            if state["timestamp"] - self.running_timestamp > 35. - self.hold_duration_s:
                self.cue("剩余时间不足以完成保持与撤力观察，请完全松手")
                return
            self.hold_started_monotonic = marked_ns / 1_000_000_000
        if event == "released":
            if not self.force_started or self.release_monotonic is not None:
                self.window.operator_event_result("需先标记施力开始；完全松手标记只能记录一次。")
                return
            self.release_monotonic = marked_ns / 1_000_000_000
        self.logger.record("operator_event", {
            "event": event, "label": OPERATOR_EVENTS[event], "note": note,
            "event_monotonic_ns": marked_ns, "event_wall_time_ns": time.time_ns(),
            "run_id": self.logger.get_status()["run_id"],
            "latest_robot_state": {"timestamp": state["timestamp"],
                                   "state_received_monotonic": state["state_received_monotonic"]},
            "association": "现场按钮与机器人反馈异步，不代表精确接触或撤力时刻",
        })
        self.window.operator_event_result("已记录：" + OPERATOR_EVENTS[event])
        print("人工标记：" + OPERATOR_EVENTS[event], flush=True)
        if event in ("buzzing", "stop_requested"):
            self.stop("operator_event:" + event)

    def protocol_event(self, event, state, **details):
        """自动阶段事件只代表提示，不伪造人工施力或完全松手。"""
        marked_ns = time.monotonic_ns()
        self.protocol_events[event] = marked_ns
        self.logger.record("manual_protocol_event", {
            "event": event, "event_monotonic_ns": marked_ns,
            "latest_robot_timestamp_s": state["timestamp"],
            "definition": "界面自动引导时刻；不证明现场已经接触或完全松手",
            **details,
        })

    def boundary_hold_phase(self, state, elapsed, offset):
        """接近边界后提示短时维持已有轻推；计时只作操作引导，实际停留另由原始日志统计。"""
        now = time.monotonic()
        if self.alerted and "release_cue" not in self.protocol_events:
            return "请立即完全松手，等待停止确认"
        if "push_window_started" not in self.protocol_events:
            self.protocol_event("push_window_started", state)
            QtWidgets.QApplication.beep()
        execution = state["continuous"]["execution"]
        near_side = 0
        command_offset = None
        if (execution is not None and execution["phase"] == 2
                and now - execution["received_monotonic_s"] <= self.boundary_max_sample_age_s
                and now - state["state_received_monotonic"] <= self.boundary_max_sample_age_s):
            command_offset = execution["q_cmd"][self.axis] - state["q_start"][self.axis]
            side = 1 if offset >= 0 else -1
            limit = side * self.command_limit
            if (abs(command_offset - limit) <= self.boundary_command_band_rad
                    and abs(offset - limit) <= self.boundary_actual_band_rad):
                near_side = side
        approach_end = self.baseline_duration_s + self.push_duration_s
        if "boundary_hold_started" not in self.protocol_events and "release_cue" not in self.protocol_events:
            if elapsed >= approach_end:
                self.protocol_event("release_cue", state, outcome="boundary_not_reached")
                self.cue("接近边界时间结束，未取得完整到达记录；请完全松手，无需继续加力")
            else:
                if near_side == 0:
                    self.boundary_candidate = None
                elif self.boundary_candidate is None or self.boundary_candidate[0] != near_side:
                    self.boundary_candidate = (near_side, now)
                elif now - self.boundary_candidate[1] >= self.boundary_confirm_s:
                    self.boundary_side = near_side
                    self.protocol_event("boundary_hold_started", state, boundary_side=near_side,
                                        observation_axis=self.axis + 1,
                                        command_offset_rad=command_offset, actual_offset_rad=offset,
                                        execution_page_seq=execution["page_seq"],
                                        execution_received_monotonic_s=execution["received_monotonic_s"],
                                        actual_received_monotonic_s=state["state_received_monotonic"],
                                        observed_confirmation_s=now - self.boundary_candidate[1])
                    QtWidgets.QApplication.beep()
                if "boundary_hold_started" not in self.protocol_events:
                    return f"缓慢推向任选一侧边界，接近阶段剩余 {approach_end - elapsed:.1f} 秒；到边界不再加力，等待保持提示"
        if "boundary_hold_started" in self.protocol_events and "release_cue" not in self.protocol_events:
            held_s = now - self.protocol_events["boundary_hold_started"] / 1e9
            if held_s >= self.hold_duration_s:
                self.protocol_event("release_cue", state, outcome="boundary_hold_timer_finished")
                self.cue("边界保持提示满 5 秒，请立即完全松手；无需点击按钮")
            else:
                location = "已接近边界" if near_side == self.boundary_side else "目前已离开边界观测带，勿追赶"
                return f"{location}：保持现有轻推，不增加推力；剩余 {self.hold_duration_s - held_s:.1f} 秒，无需点击"
        released_s = now - self.protocol_events["release_cue"] / 1e9
        if released_s < self.release_grace_s:
            return f"立即完全松手，无需点击；{self.release_grace_s - released_s:.1f} 秒后进入计划无接触观察"
        if "observation_window_started" not in self.protocol_events:
            self.protocol_event("observation_window_started", state)
        return f"计划无接触观察 {released_s - self.release_grace_s:.1f} 秒，请保持双手离开，等待本轮结束"

    def timed_push_phase(self, state, elapsed, delta, offset):
        """单人操作只需开始；到时撤力，宽限后进入计划观察，超观测线不提前结束。"""
        push_end = self.baseline_duration_s + self.push_duration_s
        if elapsed < push_end:
            if self.alerted:
                return "请立即完全松手，等待停止确认"
            if "push_window_started" not in self.protocol_events:
                self.protocol_event("push_window_started", state)
                QtWidgets.QApplication.beep()
            phase = f"现在可轻推 {self.axes_label}，剩余 {push_end - elapsed:.1f} 秒；中途无需点击，接近行程边界不继续加力"
            notices = []
            if delta >= self.args.force_cue_n:
                notices.append(f"力增量超过 {self.args.force_cue_n:g} N 观测线")
            if abs(offset) >= self.offset_cue_rad:
                notices.append(f"偏移超过 {self.offset_cue_rad:g} rad 观测线")
            if notices:
                phase += "；观测提醒：" + "、".join(notices)
            return phase
        if "release_cue" not in self.protocol_events:
            self.protocol_event("release_cue", state)
            self.cue("20 秒手推时间结束，请立即完全松手；无需点击松手按钮")
        observation_start = push_end + self.release_grace_s
        if elapsed < observation_start:
            return f"立即完全松手，无需点击；{observation_start - elapsed:.1f} 秒后进入计划无接触观察"
        if "observation_window_started" not in self.protocol_events:
            self.protocol_event("observation_window_started", state)
        return f"计划无接触观察 {elapsed - observation_start:.1f} 秒，请保持双手离开；结束后确认实际松手情况"

    def refresh(self):
        """显示采样只用于人工提示，完整证据由正式 500 Hz 日志链记录。"""
        if self.controller is None or self.completed or self.postcheck_pending:
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
                self.running_monotonic = time.monotonic()
            elapsed = (time.monotonic() - self.running_monotonic if self.guided
                       else state["timestamp"] - self.running_timestamp)
            if elapsed < self.baseline_duration_s:
                self.force_samples.append(state["actual_TCP_force"][:3])
                phase = f"运行 {elapsed:.1f} / {self.baseline_duration_s:g} 秒：保持无接触，正在记录基线"
            else:
                if self.force_baseline is None:
                    self.force_baseline = [sum(row[i] for row in self.force_samples) / len(self.force_samples) for i in range(3)]
                    self.logger.record("manual_contact_baseline", {"force_base_n": self.force_baseline,
                                                                  "display_sample_count": len(self.force_samples)})
                delta = math.dist(state["actual_TCP_force"][:3], self.force_baseline)
                offsets = [state["actual_q"][axis - 1] - state["q_start"][axis - 1] for axis in self.args.axes]
                offset = state["actual_q"][self.axis] - state["q_start"][self.axis]
                if self.boundary_hold:
                    phase = self.boundary_hold_phase(state, elapsed, offset)
                elif self.timed_push:
                    phase = self.timed_push_phase(state, elapsed, delta, offset)
                elif self.release_monotonic is None:
                    if delta >= self.args.force_cue_n or max(map(abs, offsets)) >= self.offset_cue_rad or elapsed >= 35:
                        self.cue(f"末端力增量 {delta:.1f} N，{self.axes_label} 偏移 {offsets} rad，运行 {elapsed:.1f} 秒")
                    if self.hold_started_monotonic is not None:
                        held_s = time.monotonic() - self.hold_started_monotonic
                        if held_s >= self.hold_duration_s:
                            self.cue("5 秒保持计时结束，请完全松手并记录")
                        phase = f"保持轻推力度，勿继续加力：{held_s:.1f} / {self.hold_duration_s:g} 秒"
                    elif self.sustained and self.force_started:
                        phase = "小幅让位后，点击左上方绿色“开始保持受载”，保持力度，勿继续加力"
                    else:
                        phase = ("点“施力开始”后缓慢轻推，小幅让位后点击左上方绿色“开始保持受载”"
                                 if self.sustained else "点“施力开始”后单次缓慢推动，明显让位即松手")
                    if self.alerted:
                        phase = "立即完全松手，并点击“完全松手”"
                else:
                    released_s = time.monotonic() - self.release_monotonic
                    phase = f"松手后观察 {released_s:.1f} / 至少 20 秒，请保持无接触"
                phase += f"；末端力增量 {delta:.1f} N；" + " / ".join(
                    f"J{axis} {value:+.4f} rad" for axis, value in zip(self.args.axes, offsets))
            if int(elapsed) != self.last_second:
                print(phase, flush=True)
                self.last_second = int(elapsed)
        if state["stopping"] or state["last_error"]:
            self.cue(state["last_error"] or state["exit_reason_text"])
            phase = "请保持双手离开机械臂：" + (state["last_error"] or state["phase_name"])
        self.window.update_context({"phase": phase, "occupied": state["active"],
                                    "stop_pending": state["stopping"], "debug_log_status": self.logger.get_status()})
        snapshot = self.window.monitor.update(state, None, run_id=self.logger.get_status()["run_id"], connected=True,
                                              parameters=self.config, reference_source="fixed", space="joint")
        self.window.update_snapshot(snapshot)
        self.window.ui.label_ContactStage.setText(phase)
        if self.window.ui.pushButton_EventForceStart.isEnabled():
            self.window.ui.pushButton_EventForceStart.setEnabled(
                self.force_baseline is not None and not self.force_started and state["running"] and not self.alerted)
            self.window.ui.pushButton_EventHoldStart.setEnabled(
                self.sustained and self.force_started and self.hold_started_monotonic is None
                and self.release_monotonic is None and state["running"] and not self.alerted)
            self.window.ui.pushButton_EventReleased.setEnabled(self.force_started and self.release_monotonic is None)
        if not state["active"] and state["stopped_confirmed"]:
            self.finish_stop(state)

    def finish_stop(self, state):
        """独立复核连续静止后收尾；未通过时保留窗口和日志，等现场显式再次检查。"""
        self.controller.close()
        try:
            observation = read_robot_state(self.args.robot_ip)
            self.logger.record("manual_stop_observations", observation)
            require_stopped(observation, admittance_config.stop_speed_limit(self.config))
        except (RuntimeError, OSError, ValueError) as error:
            self.postcheck_pending = True
            message = f"独立停止复核未通过：{error}。请保持撤力，点击“停止导纳”再次复核。"
            self.logger.record("manual_postcheck_pending", {"error": str(error)})
            self.window.append_log(message)
            self.window.ui.label_ContactStage.setText(message)
            self.window.update_context({"robot_ready": False, "busy": True, "occupied": True,
                                        "stop_pending": True, "phase": message})
            print(message, flush=True)
            return
        self.postcheck_pending = False
        self.logger.record("manual_postcheck", observation)
        self.logger.finish({"status": state})
        (self.path / "manual_final_status.json").write_text(json.dumps(state, ensure_ascii=False, indent=2), encoding="utf-8")
        report = analyze(self.path)
        (self.path / "descriptive_report.json").write_text(
            json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
        self.completed = True
        self.window.ui.label_ContactStage.setText("已停止并保存描述性报告，请等待数据分析与现场评价")
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
    parser.add_argument("--axes", type=int, nargs="+", choices=range(1, 7), default=[1],
                        help="柔顺轴：1～6 中的单轴、1 3，或 1 2 3 4 5 6")
    parser.add_argument("--observation-axis", type=int, choices=range(1, 7), help="逐周期统计和边界提示轴，默认采用候选观测轴")
    parser.add_argument("--contact-point", default="空法兰末端", help="记录本轮实际接触位置，例如前臂连杆中段")
    parser.add_argument("--contact-direction", default="沿启用关节允许的方向缓慢轻推", help="记录本轮现场约定施力方向")
    parser.add_argument("--protocol", choices=("single", "sustained", "timed_push", "boundary_hold"), default="single",
                        help="single 单次施力，sustained 人工保持计时，timed_push 自动手推，boundary_hold 到边界后提示保持 5 秒")
    parser.add_argument("--preview", type=Path)
    args = parser.parse_args()
    if len(args.axes) != 1 and args.axes not in ([1, 3], [1, 2, 3, 4, 5, 6]):
        parser.error("本轮柔顺轴须为单轴、1 3，或完整六轴 1 2 3 4 5 6")
    if args.observation_axis is not None and args.observation_axis not in args.axes:
        parser.error("观测轴须属于本轮启用轴")
    if not math.isfinite(args.force_cue_n) or args.force_cue_n <= 0:
        parser.error("撤力提示值须为正有限力增量，单位 N")
    configure_high_dpi()
    app = QtWidgets.QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(False)
    window = AdmittanceWindow(settings=QtCore.QSettings("/tmp/urcontrol_joint_contact.ini", QtCore.QSettings.IniFormat))
    title = "+".join(f"J{axis}" for axis in args.axes) + " " + {
        "single": "单次手推联调", "sustained": "持续受载联调：保持 5 秒",
        "timed_push": "连续手推联调：自动引导 20 秒",
        "boundary_hold": "边界保持联调：到边界后保持现有轻推 5 秒"}[args.protocol]
    window.setWindowTitle(title + " · 本轮 60 秒 · 撤力后至少观察 20 秒")
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
        window.showNormal()
        window.resize(1600, 1000)
        app.processEvents()
        window.grab().save(str(args.preview))
        print("离线预览已保存：" + str(args.preview))
        return 0
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
