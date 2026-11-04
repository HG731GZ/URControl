"""单轴调参窗口：生成预览、冻结执行参数、显示实际反馈及本轮误差报告。"""

from copy import deepcopy
import math
import time

from PyQt5 import QtCore, QtWidgets

from admittance_plot import AdmittanceTrendWidget
from impedance_tuning_report import TuningReportWorker
from ui_impedance_tuning_window import Ui_ImpedanceTuningWindow
from UR_Utils.impedance_config import validate, snapshot
from UR_Utils.impedance_trajectory import generate_axis_tuning
from UR_Utils.impedance_step import JointStep


class ImpedanceTuningWindow(QtWidgets.QDialog):
    start_requested = QtCore.pyqtSignal(dict)
    stop_requested = QtCore.pyqtSignal()
    FIELDS = (
        ("stiffnessSpin", "JOINT_K", 1.0),
        ("dampingSpin", "JOINT_D", 1.0),
        ("torqueSpin", "TAU_MAX_NM", 1.0),
        ("slewSpin", "TAU_RATE_MAX_NM_S", 1.0),
        ("referenceSpeedSpin", "REF_MAX_QD_RAD_S", 180 / math.pi),
        ("accelerationSpin", "REF_MAX_QDD_RAD_S2", 180 / math.pi),
    )

    def __init__(self, parameters, axis=0, parent=None):
        super().__init__(parent)
        self.ui = Ui_ImpedanceTuningWindow()
        self.ui.setupUi(self)
        self.setWindowModality(QtCore.Qt.NonModal)
        self.parameters = parameters
        self.axis = axis
        self.trajectory = None
        self.generated_config = None
        self.actual_q = self.actual_qd = None
        self.context_time = 0.0
        self.ready = self.busy = self.executing = False
        self.status = {}
        self.run_id = self.previous_run_id = None
        self.samples = []
        self.last_cycle = None
        self.report_worker = None
        self.plot = AdmittanceTrendWidget(self)
        self.ui.plotLayout.addWidget(self.plot)
        self.ui.axisCombo.setCurrentIndex(axis)
        self._fill_axis()
        self.ui.axisCombo.currentIndexChanged.connect(self._change_axis)
        for name, _, _ in self.FIELDS:
            getattr(self.ui, name).valueChanged.connect(self._parameter_edited)
        self.ui.speedSpin.valueChanged.connect(self._invalidate)
        self.ui.offsetSpin.valueChanged.connect(self._invalidate)
        self.ui.holdSpin.valueChanged.connect(self._invalidate)
        self.ui.modeCombo.currentIndexChanged.connect(self._mode_changed)
        self.ui.tunedPresetButton.clicked.connect(self._load_tuned_defaults)
        self.ui.applyButton.clicked.connect(self.apply_parameters)
        self.ui.discardButton.clicked.connect(self.discard_parameters)
        self.ui.importButton.clicked.connect(self.import_preset)
        self.ui.exportButton.clicked.connect(self.export_preset)
        self.parameters.changed.connect(self._parameter_state)
        self._parameter_state()
        self.ui.generateButton.clicked.connect(self.generate)
        self.ui.executeButton.clicked.connect(self.execute)
        self.ui.stopButton.clicked.connect(self._stop)
        QtWidgets.QApplication.instance().aboutToQuit.connect(self._wait_report)
        self._mode_changed()

    def _is_step(self):
        return self.ui.modeCombo.currentIndex() == 0

    def _mode_changed(self):
        step = self._is_step()
        self.ui.speedSpin.setEnabled(not step)
        self.ui.holdSpin.setEnabled(step)
        self._invalidate()

    @property
    def config(self):
        return self.parameters.draft

    def _parameter_state(self):
        self.ui.parameterStateLabel.setText(
            f"已应用版本 {self.parameters.revision} · "
            + ("草稿有未应用修改" if self.parameters.dirty else "草稿与已应用参数一致")
        )

    def _parameter_edited(self):
        try:
            self.parameters.require_editable()
        except ValueError as exc:
            self._fill_axis()
            self._error(exc)
            return
        for name, key, scale in self.FIELDS:
            if self.sender() is getattr(self.ui, name):
                self.config[key][self.axis] = getattr(self.ui, name).value() / scale
        self._update_speed_limit()
        self.parameters.changed.emit()
        self._invalidate()

    def apply_parameters(self):
        try:
            self.parameters.apply()
            self.ui.statusLabel.setText("完整六轴参数已应用；未启动运动。")
        except ValueError as exc:
            self._error(exc)

    def discard_parameters(self):
        try:
            self.parameters.require_editable()
            self.parameters.draft = deepcopy(self.parameters.prepared)
            self._fill_axis()
            self._invalidate()
            self.parameters.changed.emit()
        except ValueError as exc:
            self._error(exc)

    def _load_tuned_defaults(self):
        try:
            self.parameters.require_editable()
            self.parameters.draft = snapshot()
            self._fill_axis()
            self._invalidate()
            self.parameters.changed.emit()
            self.ui.statusLabel.setText("默认值已载入草稿；仍需应用参数。")
        except ValueError as exc:
            self._error(exc)

    def import_preset(self):
        try:
            self.parameters.require_editable()
        except ValueError as exc:
            self._error(exc)
            return
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "导入阻抗预设", "", "JSON (*.json)")
        if path:
            try:
                self.parameters.load(path)
                self._fill_axis()
                self._invalidate()
            except (ValueError, OSError) as exc:
                self._error(exc)

    def export_preset(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "导出阻抗草稿", "impedance_preset.json", "JSON (*.json)")
        if path:
            try:
                self.parameters.save(path)
            except (ValueError, OSError) as exc:
                self._error(exc)

    def _fill_axis(self):
        for name, key, scale in self.FIELDS:
            widget = getattr(self.ui, name)
            blocker = QtCore.QSignalBlocker(widget)
            widget.setValue(self.config[key][self.axis] * scale)
            del blocker
        self._update_speed_limit()
        self.ui.guardLabel.setText(
            f"摩擦补偿为 0；跟踪误差守卫 {math.degrees(self.config['MAX_ERROR_RAD'][self.axis]):.2f}°。\n"
            f"实际速度守卫 {math.degrees(self.config['ACTUAL_MAX_QD_RAD_S'][self.axis]):.2f}°/s。"
        )

    def _update_speed_limit(self):
        """激励速度同时满足参考上限和实际速度停止阈值的预算。"""
        limit = min(self.config["REF_MAX_QD_RAD_S"][self.axis],
                    self.config["ACTUAL_MAX_QD_RAD_S"][self.axis]) * 0.85
        self.ui.speedSpin.setMaximum(math.degrees(limit))

    def _change_axis(self, axis):
        self.axis = axis
        self._fill_axis()
        self._invalidate()
        self._current_label()

    def _invalidate(self):
        self.trajectory = self.generated_config = None
        self.samples = []
        self.plot.clear()
        self.ui.trajectoryLabel.setText("设置已改变，请重新生成轨迹。")
        self.ui.reportLabel.setText("执行结束后显示本轮跟踪误差。")
        self._buttons()

    def _fresh(self):
        return (
            self.ready
            and self.actual_q is not None
            and self.actual_qd is not None
            and time.monotonic() - self.context_time < 0.25
        )

    def _buttons(self):
        locked = self.executing or self.busy or self.report_worker is not None or self.parameters.locked
        self.ui.trajectoryGroup.setEnabled(not locked)
        self.ui.parameterGroup.setEnabled(not locked)
        self.ui.generateButton.setEnabled(not locked and self._fresh())
        self.ui.executeButton.setEnabled(
            not locked and self._fresh() and self.trajectory is not None
        )
        self.ui.stopButton.setEnabled(self.executing)
        for name in ("tunedPresetButton", "applyButton", "discardButton", "importButton"):
            getattr(self.ui, name).setEnabled(not locked and not self.parameters.locked)

    def _current_label(self):
        if self.actual_q is None:
            text = "当前位置：等待新鲜机器人反馈"
        else:
            text = f"J{self.axis+1} 当前角度：{math.degrees(self.actual_q[self.axis]):.3f}°"
        self.ui.currentLabel.setText(text)

    def update_context(self, status, ready, busy, actual_q=None, actual_qd=None):
        self.status, self.ready, self.busy = (
            status,
            ready,
            busy or status.get("active", False),
        )
        self.actual_q = tuple(actual_q) if actual_q is not None else None
        self.actual_qd = tuple(actual_qd) if actual_qd is not None else None
        self.context_time = time.monotonic()
        self._current_label()
        if self.executing:
            run_id = status.get("run_id")
            if run_id is not None and run_id != self.previous_run_id:
                self.run_id = run_id
            if self.run_id == run_id and self.run_id is not None:
                self.ui.statusLabel.setText(
                    f"J{self.axis+1} · {status.get('phase')} · "
                    + (status.get("last_error") or status.get("stop_error") or "执行中")
                )
                sample = status.get("sample")
                if (
                    sample
                    and sample.get("phase") in ("RUNNING", "ENDING")
                    and sample["cycle_seq"] != self.last_cycle
                ):
                    self.last_cycle = sample["cycle_seq"]
                    self.samples.append(
                        (
                            sample["sample_time_s"],
                            math.degrees(sample["q_ref"][self.axis]),
                            math.degrees(sample["q_actual"][self.axis]),
                            None,
                        )
                    )
                    start = self.samples[0][0]
                    rows = [
                        (t - start, ref, actual, source)
                        for t, ref, actual, source in self.samples
                    ]
                    self._draw(rows, "运行曲线（界面抽样，结束后用本轮日志更新）")
                if not self.busy:
                    self.executing = False
                    self._finish(status)
        self._buttons()

    def _stationary(self):
        if not self._fresh():
            raise ValueError("没有新鲜的机器人反馈，请先连接并准备好机器人")
        if max(map(abs, self.actual_qd)) > self.config["STATIONARY_QD_RAD_S"]:
            raise ValueError("请先停止机器人，静止后再生成或执行轨迹")

    def generate(self):
        if self.executing or self.busy or self.report_worker is not None:
            return
        try:
            self._stationary()
            cfg = deepcopy(self.config)
            offset = math.radians(self.ui.offsetSpin.value())
            speed = math.radians(self.ui.speedSpin.value())
            if offset == 0:
                raise ValueError("角度偏置不能为零")
            cfg = validate(cfg)
            if self._is_step():
                trajectory = JointStep(self.actual_q, self.axis, offset, hold_s=self.ui.holdSpin.value())
                trajectory.bind(self.actual_q, cfg, trajectory.sample_hz)
            else:
                trajectory = generate_axis_tuning(self.actual_q, self.axis, offset, speed, cfg)
            self.trajectory, self.generated_config = trajectory, cfg
            self.samples = []
            start = math.degrees(trajectory.rows[0][self.axis])
            delta = self.ui.offsetSpin.value()
            if self._is_step():
                description = (f"J{self.axis+1}：{start:.3f}° → {start+delta:.3f}°；"
                               f"先保持 1 秒，再直接切换位置参考，观察 {trajectory.hold_s:g} 秒。"
                               "目标速度为零，实际速度与力矩守卫有效，不自动回位。")
            else:
                description = (f"J{self.axis+1}：{start:.3f}° → {start+delta:.3f}° → "
                               f"{start-delta:.3f}° → {start:.3f}°；时长 {trajectory.duration():.2f} s；"
                               f"最大速度 ≤ {self.ui.speedSpin.value():g}°/s；五次平滑起停。")
            self.ui.trajectoryLabel.setText(description)
            self.ui.statusLabel.setText(
                "轨迹已生成；执行时使用此起点，位置变化后需重新生成。"
            )
            self.ui.reportLabel.setText("等待执行。")
            rows = [
                (i / trajectory.sample_hz, math.degrees(q[self.axis]), None, None)
                for i, q in enumerate(trajectory.rows)
            ]
            self._draw(rows, "生成轨迹预览", preview=True)
        except Exception as exc:
            self._invalidate()
            self._error(exc)
        self._buttons()

    def execute(self):
        if (
            self.executing
            or self.busy
            or self.report_worker is not None
            or self.trajectory is None
        ):
            return
        try:
            self._stationary()
            self.trajectory.bind(
                self.actual_q, self.generated_config, self.trajectory.sample_hz
            )
            self.previous_run_id = self.status.get("run_id")
            self.run_id = None
            self.samples = []
            self.last_cycle = None
            self.executing = True
            self.ui.statusLabel.setText("正在准备单轴调参执行……")
            self.ui.reportLabel.setText("执行中；结束后从本轮日志计算完整采样误差。")
            self._buttons()
            self.start_requested.emit(
                dict(
                    reference_source="step" if self._is_step() else "trajectory",
                    config=deepcopy(self.generated_config),
                    trajectory=self.trajectory,
                    play_hz=self.trajectory.sample_hz,
                )
            )
        except Exception as exc:
            self.executing = False
            self._error(exc)
            self._buttons()

    def start_failed(self, message):
        if not self.executing:
            return
        self.executing = False
        self.ui.statusLabel.setText("启动失败：" + str(message))
        self._buttons()

    def _stop(self):
        self.ui.statusLabel.setText("已请求停止，等待停止确认……")
        self.stop_requested.emit()

    def _finish(self, status):
        complete = status.get("completed_trajectory") and not status.get("last_error")
        self.ui.statusLabel.setText(
            ("计划执行结束，已确认停止；到位效果见误差报告。" if complete else "本轮提前结束。")
            + (status.get("last_error") or status.get("stop_error") or "")
        )
        directory = status.get("log_directory")
        if not directory:
            self.ui.reportLabel.setText("本轮未产生可用日志，无法统计跟踪误差。")
            return
        self.ui.reportLabel.setText("正在读取本轮日志并计算跟踪误差……")
        worker = TuningReportWorker(
            directory, self.run_id, self.axis, self.trajectory.rows[-1][self.axis], self
        )
        self.report_worker = worker
        worker.finished.connect(self._report_finished)
        worker.start()

    def _report_finished(self):
        worker = self.report_worker
        self.report_worker = None
        if worker.error:
            self.ui.reportLabel.setText("本轮误差报告不可用：" + worker.error)
        else:
            result = worker.result
            report = result["report"]
            self._draw(result["rows"], "本轮日志曲线（按采纳身份对齐生成目标）")
            metrics = report["reference_error"]
            lines = [
                f"{'完整执行' if report['completed'] else '提前结束，仅统计已执行部分'} · "
                f"{metrics['sample_count']} 个同周期样本（°）",
                f"生效参考−实际：均方根 {metrics['rmse_deg']:.4f}°；"
                f"最大绝对误差 {metrics['max_abs_deg']:.4f}°；末次误差 {metrics['last_error_deg']:.4f}°。",
            ]
            source = report["generated_target_error"]
            if source:
                lines.append(
                    f"生成目标−实际：均方根 {source['rmse_deg']:.4f}°；"
                    f"最大绝对误差 {source['max_abs_deg']:.4f}°。"
                )
            if report["completed"]:
                lines.append(
                    f"最终目标误差：{report['final_target_error_deg']:.4f}°。"
                )
            if report.get("tracking_response"):
                tracking = report["tracking_response"]["tracking"]
                passed = tracking["source_rms_within_tolerance"]
                verdict = "证据不足" if passed is None else "满足" if passed else "未满足"
                lines.append(
                    f"运动阶段：生成目标 RMS {math.degrees(tracking['motion_source_rmse_rad']):.4f}°；"
                    f"生效参考 RMS {math.degrees(tracking['motion_reference_rmse_rad']):.4f}°；"
                    f"{verdict} {tracking['rms_tolerance_deg']:g}° 跟踪目标。"
                )
            if report.get("step_response"):
                response = report["step_response"]
                step = response["step"]
                rise = "未达到 90%" if step["rise_10_90_s"] is None else f"{step['rise_10_90_s']:.3f} s"
                settling = "未稳定进入" if step["settling_time_s"] is None else f"{step['settling_time_s']:.3f} s"
                lines[1:1] = [
                    f"阶跃：10%→90% {rise}；超调 {step['overshoot_percent']:.2f}%；"
                    f"±{math.degrees(step['settling_tolerance_rad']):.2f}° 稳定时间：{settling}。",
                    f"末 1 秒平均误差：{math.degrees(response['tail_mean_error_rad']):.4f}°；"
                    "阶跃时间指标分辨率约 10 ms。",
                ]
            if (
                not report["log_complete"]
                or report["snapshot_gaps"]
                or report["source_alignment_missing"]
            ):
                lines.append(
                    f"数据不完整：遥测缺口 {report['snapshot_gaps']}，"
                    f"目标未对齐 {report['source_alignment_missing']}，"
                    f"日志丢弃 {report['log_dropped']}；统计仅覆盖已记录样本。"
                )
            if report["environment"] == "URSim":
                lines.append(
                    "URSim 的力矩指令不驱动关节运动，本报告不能代表实机跟踪性能。"
                )
            lines.append("报告：" + result["path"])
            self.ui.reportLabel.setText("\n".join(lines))
        worker.deleteLater()
        self._buttons()

    def _draw(self, rows, title, preview=False):
        if not rows:
            return
        # 绘图使用等间隔抽取，误差统计始终使用全部日志样本。
        stride = max(1, math.ceil(len(rows) / 2500))
        display = rows[::stride]
        if display[-1] != rows[-1]:
            display.append(rows[-1])
        series = []
        definitions = [(1, "生成轨迹" if preview else "生效参考", "#2474b5", "solid")]
        if not preview:
            definitions.append((2, "实际角度", "#df7c25", "dash"))
            if any(row[3] is not None for row in display):
                definitions.append((3, "生成目标", "#71717a", "dot"))
        for column, name, color, style in definitions:
            series.append(
                dict(
                    name=name,
                    color=color,
                    style=style,
                    points=[
                        dict(
                            time=row[0],
                            value=(
                                row[column] if row[column] is not None else float("nan")
                            ),
                            valid=row[column] is not None,
                            run_id=self.run_id or 0,
                            segment=0,
                        )
                        for row in display
                    ],
                )
            )
        end = max(rows[-1][0], 0.1)
        self.plot.set_data(
            dict(
                title=f"J{self.axis+1} · {title}",
                unit="°",
                now=end,
                time_range=(0.0, end),
                series=series,
            )
        )

    def _error(self, error):
        QtWidgets.QMessageBox.warning(self, "单轴调参测试", str(error))

    def _wait_report(self):
        if self.report_worker is not None:
            self.report_worker.wait()

    def reject(self):
        self.hide()

    def closeEvent(self, event):
        event.ignore()
        self.hide()
