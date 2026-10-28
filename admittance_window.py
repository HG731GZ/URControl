"""独立导纳窗口的数据绑定；不持有机器人或 UDP 连接，也不直接提交运动。"""

from copy import deepcopy
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets

from admittance_monitor import AdmittanceMonitor
from admittance_parameters import AdmittanceParameters, FIELDS
from ui_admittance_window import Ui_AdmittanceWindow


OPERATOR_EVENTS = {
    "force_start": "施力开始", "released": "完全松手",
    "buzzing": "听到嗡鸣", "stop_requested": "请求停止",
}


class AdmittanceWindow(QtWidgets.QMainWindow):
    """只发送用户意图，控制状态与停止确认始终由主程序统一管理。"""

    start_requested = QtCore.pyqtSignal(str, str, object, int)
    stop_requested = QtCore.pyqtSignal()
    sync_requested = QtCore.pyqtSignal()
    apply_requested = QtCore.pyqtSignal()
    visibility_changed = QtCore.pyqtSignal(bool)
    logging_toggled = QtCore.pyqtSignal(bool)
    operator_event_requested = QtCore.pyqtSignal(str, str)

    def __init__(self, settings=None, parent=None):
        super().__init__(parent, QtCore.Qt.Window)
        self.ui = Ui_AdmittanceWindow()
        self.ui.setupUi(self)
        self.settings = settings if settings is not None else QtCore.QSettings("URControl", "Display")
        self.parameters = AdmittanceParameters()
        self.monitor = AdmittanceMonitor()
        self._snapshot = None
        self._refresh_count = 0
        self._binding = False
        self._context = {
            "robot_ready": False, "busy": False, "occupied": False,
            "stop_pending": False, "master_fresh": False, "synced": False,
            "phase": "未开启", "run_id": None, "robot_address": "未连接",
            "controller_joint_limits": None, "parameters_revision": 0,
            "run_metadata": {},
            "diagnostic": None,
            "debug_log_status": {
                "enabled": True, "active": False, "path": None,
                "record_count": 0, "dropped_count": 0, "pending_count": 0, "error": None,
                "logger_timing": {"schema_version": 1, "producer": {}, "writer": {},
                                  "queue_observed_high_watermark": 0, "queue_age": None},
            },
        }
        self._editors = {}
        defaults = self.parameters.draft_parameters
        for field in FIELDS:
            if not field.editable:
                continue
            value = defaults[field.key]
            if isinstance(value, str):
                controls = [getattr(self.ui, "comboBox_" + field.widget)]
                controls[0].currentIndexChanged.connect(lambda _, key=field.key: self._read_editor(key, 0))
            elif field.key.endswith("_AXES"):
                controls = [getattr(self.ui, "checkBox_" + field.widget + str(i)) for i in range(1, 7)]
                for axis, control in enumerate(controls):
                    control.toggled.connect(lambda _, key=field.key, index=axis: self._read_editor(key, index))
            else:
                names = ([field.widget + str(i) for i in range(1, 7)]
                         if isinstance(value, list) else [field.widget])
                controls = [getattr(self.ui, "doubleSpinBox_" + name) for name in names]
                for axis, control in enumerate(controls):
                    control.valueChanged.connect(lambda _, key=field.key, index=axis: self._read_editor(key, index))
            self._editors[field.key] = controls
        self.ui.pushButton_ApplyParameters.clicked.connect(self._apply_parameters)
        self.ui.pushButton_RestoreDefaults.clicked.connect(self._restore_defaults)
        self.ui.pushButton_ImportParameters.clicked.connect(self._import_parameters)
        self.ui.pushButton_ExportParameters.setMenu(self.ui.menu_ExportParameters)
        self.ui.actionExportDraft.triggered.connect(self._export_draft)
        self.ui.actionExportEffective.triggered.connect(self._export_effective)
        self.ui.pushButton_StartAdmittance.clicked.connect(self._start)
        self.ui.pushButton_StopAdmittance.clicked.connect(self.stop_requested.emit)
        self.ui.pushButton_SyncMaster.clicked.connect(self.sync_requested.emit)
        self.ui.checkBox_DebugLogging.toggled.connect(self._debug_logging_toggled)
        for name, event in (("ForceStart", "force_start"), ("Released", "released"),
                            ("Buzzing", "buzzing"), ("Stop", "stop_requested")):
            getattr(self.ui, "pushButton_Event" + name).clicked.connect(
                lambda _, key=event: self.operator_event_requested.emit(
                    key, self.ui.lineEdit_OperatorEventNote.text().strip()))
        self.ui.comboBox_ReferenceSource.currentIndexChanged.connect(self._selection_changed)
        self.ui.comboBox_AdmittanceSpace.currentIndexChanged.connect(self._selection_changed)
        self.ui.comboBox_AngleUnit.currentIndexChanged.connect(self._units_changed)
        self.ui.comboBox_LengthUnit.currentIndexChanged.connect(self._units_changed)
        self.ui.comboBox_PlotSignal.currentIndexChanged.connect(self._refresh_plot)
        self.ui.comboBox_PlotAxis.currentIndexChanged.connect(self._refresh_plot)
        self.ui.comboBox_PlotWindow.currentIndexChanged.connect(self._plot_window_changed)
        self.ui.pushButton_PausePlot.toggled.connect(self._pause_plot)
        self.ui.pushButton_ClearPlot.clicked.connect(self._clear_plot)
        self.ui.widget_TrendPlot.cursor_changed.connect(self.ui.label_PlotCursor.setText)
        for name in ("JointTargets", "TcpTargets", "MasterState", "AdmittanceState",
                     "RobotConfiguration", "Diagnostics", "ParameterDiff"):
            table = getattr(self.ui, "tableWidget_" + name)
            table.verticalHeader().setVisible(False)
            # 诊断长表按 Designer 列宽显示，避免每个计时值变化都重新扫描整列尺寸。
            resize_mode = (QtWidgets.QHeaderView.Interactive if name == "Diagnostics"
                           else QtWidgets.QHeaderView.ResizeToContents)
            table.horizontalHeader().setSectionResizeMode(resize_mode)
            table.horizontalHeader().setStretchLastSection(True)
            table.setHorizontalScrollMode(QtWidgets.QAbstractItemView.ScrollPerPixel)
        self._load_settings()
        self._write_editors()
        self._update_parameter_visibility()
        self._fill_waiting_tables()
        self._plot_window_changed()
        self._update_controls()

    @property
    def reference_source(self):
        return ("teleop", "fixed")[self.ui.comboBox_ReferenceSource.currentIndex()]

    @property
    def space(self):
        return ("joint", "cartesian")[self.ui.comboBox_AdmittanceSpace.currentIndex()]

    @property
    def angle_unit(self):
        return ("deg", "rad")[self.ui.comboBox_AngleUnit.currentIndex()]

    @property
    def length_unit(self):
        return ("mm", "m")[self.ui.comboBox_LengthUnit.currentIndex()]

    @property
    def debug_logging_enabled(self):
        return self.ui.checkBox_DebugLogging.isChecked()

    def set_release_alert(self, active):
        """显式显示撤力警示；触发后保持，仅由新轮准备流程传入 False 清除。"""
        self.ui.label_ReleaseAlert.setVisible(active)

    def _debug_logging_toggled(self, enabled):
        self._context["debug_log_status"]["enabled"] = enabled
        self._update_debug_log_status()
        self.logging_toggled.emit(enabled)

    def _load_settings(self):
        self.settings.beginGroup("admittanceWindow")
        geometry = self.settings.value("geometry")
        splitter = self.settings.value("splitter")
        if geometry is not None:
            self.restoreGeometry(geometry)
        if splitter is not None:
            self.ui.splitter_Main.restoreState(splitter)
        else:
            self.ui.splitter_Main.setSizes([610, 790])
        self.ui.comboBox_AngleUnit.setCurrentIndex(1 if self.settings.value("angleUnit", "deg") == "rad" else 0)
        self.ui.comboBox_LengthUnit.setCurrentIndex(1 if self.settings.value("lengthUnit", "mm") == "m" else 0)
        self.settings.endGroup()

    def _save_settings(self):
        self.settings.beginGroup("admittanceWindow")
        self.settings.setValue("geometry", self.saveGeometry())
        self.settings.setValue("splitter", self.ui.splitter_Main.saveState())
        self.settings.setValue("angleUnit", self.angle_unit)
        self.settings.setValue("lengthUnit", self.length_unit)
        self.settings.endGroup()

    def _write_editors(self):
        self._binding = True
        try:
            for field in FIELDS:
                value = self.parameters.draft_parameters[field.key]
                if not field.editable:
                    getattr(self.ui, "label_Value_" + field.widget).setText(self._format_parameter(value))
                elif isinstance(value, str):
                    self._editors[field.key][0].setCurrentIndex(("base", "tool_start").index(value))
                elif field.key.endswith("_AXES"):
                    for editor, number in zip(self._editors[field.key], value):
                        editor.setChecked(bool(number))
                else:
                    for editor, number in zip(self._editors[field.key], value if isinstance(value, list) else [value]):
                        editor.setValue(number)
        finally:
            self._binding = False
        self._refresh_diff()
        self._update_controls()

    def _read_editor(self, key, axis):
        """只更新实际改动的轴，避免其他轴被控件显示精度重新舍入。"""
        if self._binding:
            return
        values = deepcopy(self.parameters.draft_parameters)
        editor = self._editors[key][axis]
        if key == "CART_FRAME":
            values[key] = ("base", "tool_start")[editor.currentIndex()]
        elif key.endswith("_AXES"):
            values[key][axis] = int(editor.isChecked())
        elif isinstance(values[key], list):
            values[key][axis] = editor.value()
        else:
            values[key] = editor.value()
        self.parameters.set_draft(values)
        self._refresh_diff()
        self._update_controls()

    @staticmethod
    def _format_parameter(value):
        if value is None:
            return "尚无生效值"
        if isinstance(value, (list, tuple)):
            return " · ".join(f"{i + 1}:{number:.6g}" for i, number in enumerate(value))
        return value if isinstance(value, str) else f"{value:.6g}"

    def _refresh_diff(self):
        table = self.ui.tableWidget_ParameterDiff
        rows = []
        for entry in self.parameters.differences():
            field = entry["field"]
            applicability = "参与本轮" if field.participates(self.reference_source, self.space) else "不参与本轮"
            if entry["changed"]:
                if isinstance(entry["draft"], list):
                    changed = [str(i + 1) for i, (a, b) in enumerate(zip(entry["draft"], entry["prepared"])) if a != b]
                    state = "草稿未应用；轴 " + ", ".join(changed)
                else:
                    state = "草稿未应用"
            elif entry["effective"] is not None and entry["prepared"] != entry["effective"]:
                state = "准备值与本轮不同"
            else:
                state = "一致" if entry["effective"] is not None else "已准备"
            rows.append([field.title + " / " + field.unit,
                         self._format_parameter(entry["draft"]), self._format_parameter(entry["prepared"]),
                         self._format_parameter(entry["effective"]), state + "；" + applicability])
        self._fill_table(table, rows)
        for row, entry in enumerate(self.parameters.differences()):
            if entry["changed"]:
                for col in range(table.columnCount()):
                    table.item(row, col).setBackground(QtGui.QColor("#fff1d6"))
        self._filter_differences()

    def _filter_differences(self):
        """参数对照只显示当前场景需要调节的常用项。"""
        for row, field in enumerate(FIELDS):
            self.ui.tableWidget_ParameterDiff.setRowHidden(
                row, not field.participates(self.reference_source, self.space))

    def _update_parameter_visibility(self):
        """随所选空间显示参数页，两种来源均使用恢复刚度。"""
        tabs = self.ui.tabWidget_Parameters
        showing_diff = tabs.currentWidget() is self.ui.tab_ParameterDiff
        for space, page in (("joint", self.ui.tab_Parameters_joint),
                            ("cartesian", self.ui.tab_Parameters_cart)):
            tabs.setTabVisible(tabs.indexOf(page), self.space == space)
            if self.space == space and not showing_diff:
                tabs.setCurrentWidget(page)
        for group in ("joint", "cart"):
            getattr(self.ui, "label_FixedModel_" + group).setVisible(self.reference_source == "fixed")

    def _apply_parameters(self):
        try:
            self.parameters.apply()
        except ValueError as error:
            self.ui.label_ParameterHint.setText("参数未应用：" + str(error))
            self.append_log("参数校验失败：" + str(error))
            return
        self._refresh_diff()
        self._update_controls()
        self.append_log(f"参数已应用至下轮；准备修订号 {self.parameters.revision}")
        self.apply_requested.emit()

    def _restore_defaults(self):
        self.parameters.restore_defaults()
        self._write_editors()
        self.append_log("常用参数默认值已装入草稿；仍需应用才影响下一轮。")

    def _import_parameters(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "导入常用参数预设", "", "JSON 预设 (*.json)")
        if not path:
            return
        try:
            reference_source, space, name = self.parameters.import_preset(path)
        except (OSError, UnicodeError, ValueError) as error:
            QtWidgets.QMessageBox.warning(self, "预设导入失败", str(error))
            return
        self.ui.comboBox_ReferenceSource.setCurrentIndex(("teleop", "fixed").index(reference_source))
        self.ui.comboBox_AdmittanceSpace.setCurrentIndex(("joint", "cartesian").index(space))
        self._write_editors()
        self.append_log(f"预设“{name}”已装入草稿；仍需应用。")

    def _export_draft(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "导出常用参数预设", "导纳调参预设.json", "JSON 预设 (*.json)")
        if not path:
            return
        try:
            self.parameters.export_preset(path, Path(path).stem, self.reference_source, self.space)
        except (OSError, ValueError) as error:
            QtWidgets.QMessageBox.warning(self, "草稿预设导出失败", str(error))
            return
        self.append_log("已导出草稿预设：" + path)

    def _export_effective(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "导出本轮完整生效记录（不可作为预设导入）", "导纳运行记录.json", "JSON 运行记录 (*.json)")
        if not path:
            return
        try:
            self.parameters.export_effective(path, self._context["run_metadata"])
        except (OSError, ValueError) as error:
            QtWidgets.QMessageBox.warning(self, "生效记录导出失败", str(error))
            return
        self.append_log("已导出完整生效记录：" + path)

    def _start(self):
        self.start_requested.emit(self.reference_source, self.space,
                                  self.parameters.prepared_copy(), self.parameters.revision)

    def _selection_changed(self):
        self._refresh_diff()
        self._update_parameter_visibility()
        self._update_controls()

    def update_context(self, context):
        """接收主程序的业务状态；视图的使能不代替最终控制判断。"""
        self._context.update(deepcopy(context))
        self._update_controls()

    def set_effective(self, parameters, revision=None):
        """生效值仅接受后端已确认的完整参数副本。"""
        self.parameters.set_effective(parameters, revision)
        self._refresh_diff()
        self._update_controls()
        self.update_context({})

    def clear_effective(self):
        """进入新一轮前清除旧生效值，避免与新一轮元数据混配。"""
        self.parameters.effective_parameters = None
        self.parameters.effective_revision = None
        self._context["diagnostic"] = None
        self._refresh_diff()
        self._update_controls()
        self.update_context({})

    def _update_controls(self):
        occupied = self._context["occupied"] or self._context["stop_pending"]
        idle = self._context["robot_ready"] and not self._context["busy"] and not occupied
        teleop = self.reference_source == "teleop"
        ready_source = not teleop or (self._context["master_fresh"] and self._context["synced"])
        parameters = (self.parameters.effective_parameters if occupied and self.parameters.effective_parameters is not None
                      else self.parameters.prepared_parameters)
        diagnostic_axis = parameters["DIAGNOSTIC_AXIS"]
        diagnostic_mode = parameters["DIAGNOSTIC_MODE"]
        supported_selection = self.reference_source == "fixed" and self.space == "joint"
        for field in FIELDS:
            for editor in self._editors[field.key]:
                editor.setEnabled(not occupied and field.participates(self.reference_source, self.space))
        for name in ("ApplyParameters", "RestoreDefaults", "ImportParameters"):
            getattr(self.ui, "pushButton_" + name).setEnabled(not occupied)
        self.ui.comboBox_ReferenceSource.setEnabled(not occupied)
        self.ui.comboBox_AdmittanceSpace.setEnabled(not occupied)
        self.ui.pushButton_StartAdmittance.setEnabled(
            idle and ready_source and not self.parameters.has_unapplied_changes
            and supported_selection)
        self.ui.pushButton_SyncMaster.setEnabled(idle and teleop and self._context["master_fresh"])
        self.ui.pushButton_StopAdmittance.setEnabled(occupied)
        self.ui.actionExportDraft.setEnabled(not occupied)
        self.ui.actionExportEffective.setEnabled(self.parameters.effective_parameters is not None)
        self.ui.label_RunPhase.setText("阶段：" + str(self._context["phase"]))
        run_id = self._context["run_id"]
        self.ui.label_RunIdentity.setText(f"本轮：{run_id if run_id is not None else '未启动'} · 准备修订号：{self.parameters.revision} · 生效修订号：{self.parameters.effective_revision if self.parameters.effective_revision is not None else '未确认'}")
        self.ui.label_RobotConnection.setText("机器人：" + str(self._context["robot_address"]) + (" · 已就绪" if self._context["robot_ready"] else " · 未就绪"))
        if occupied:
            hint = "本轮参数已固定；停止确认后可编辑。"
        elif self.parameters.has_unapplied_changes:
            hint = "草稿尚未应用；请先应用参数再启动。"
        else:
            hint = "参数已应用；修改后请重新应用。"
        self.ui.label_ParameterHint.setText(hint)
        self.ui.label_DiagnosticMode.setVisible(True)
        if diagnostic_mode == "fixed_joint_hold":
            text = (f"固定关节保持；J{diagnostic_axis} 观测，外力不驱动目标，"
                    f"限时 {parameters['DIAGNOSTIC_DURATION_S']:g} 秒。")
        elif diagnostic_mode == "preset_joint_motion":
            text = (f"预设微动；J{parameters['DIAGNOSTIC_MOTION_AXIS']} 运动，J{diagnostic_axis} 观测，"
                    "外力不驱动目标。")
        else:
            enabled_axes = [index + 1 for index, enabled in enumerate(parameters["JOINT_AXES"]) if enabled]
            axes_text = "、".join(f"J{axis}" for axis in enabled_axes) or "未选择"
            text = f"受限单轴导纳；当前启用 {axes_text}，J{diagnostic_axis} 观测。"
            if enabled_axes not in ([1], [3]):
                text += " 需只选 J1 或 J3 中的一个轴。"
            text += (f" 限时 {parameters['DIAGNOSTIC_DURATION_S']:g} 秒，"
                     f"最大偏移 ±{parameters['SINGLE_AXIS_MAX_OFFSET_RAD']:g} rad。")
        text += f" 伺服 {parameters['SERVO_DT_S'] * 1000:g} ms / 模型 {parameters['MODEL_DT_S'] * 1000:g} ms。"
        if not supported_selection:
            text += " 本轮仅开放定点柔顺与关节空间；遥操作和末端空间尚未验收周期预算。"
        self.ui.label_DiagnosticMode.setText(text)
        self._update_debug_log_status()

    def _update_debug_log_status(self):
        """用后端状态同步日志开关与文件信息，不把开关保存到用户设置。"""
        status = self._context["debug_log_status"]
        checkbox = self.ui.checkBox_DebugLogging
        previous = checkbox.blockSignals(True)
        checkbox.setChecked(status["enabled"])
        checkbox.blockSignals(previous)
        if not status["enabled"]:
            if status["pending_count"] and not status["error"]:
                state = "已关闭，正在写入剩余记录"
            else:
                state = "已关闭" if status["path"] else "已关闭，尚未创建日志"
        elif status["error"]:
            state = "记录已中断"
        elif status["active"]:
            if self._context["occupied"] or self._context["stop_pending"]:
                state = "正在记录"
            else:
                state = "正在收尾，写入剩余记录" if status["pending_count"] else "正在收尾"
        else:
            state = "已开启，等待启动导纳"
        if status["error"]:
            state += "；日志错误：" + status["error"]
        text = (f"{state} · 已写入 {status['record_count']} 条"
                f" · 待写 {status['pending_count']} 条 · 丢弃 {status['dropped_count']} 条")
        timing = status["logger_timing"]
        queue_age = timing["queue_age"]
        queue_delay = "未观测" if queue_age is None else f"{queue_age['max_s'] * 1000:.3f} ms"
        text += (f" · 队列观测峰值 {timing['queue_observed_high_watermark']} 条"
                 f" · 最大提交至出队 {queue_delay}")
        if status["path"]:
            text += "\n日志：" + str(status["path"])
        self.ui.label_DebugLogStatus.setText(text)
        self.ui.label_DebugLogStatus.setToolTip(text)
        events_enabled = (self._context["occupied"] and self._context["run_id"] is not None
                          and status["enabled"] and status["active"] and status["error"] is None)
        for name in ("ForceStart", "Released", "Buzzing", "Stop"):
            getattr(self.ui, "pushButton_Event" + name).setEnabled(events_enabled)

    def operator_event_result(self, text):
        """显示主程序实际提交结果，入队成功不宣称已经完成写盘。"""
        self.ui.label_OperatorEventStatus.setText(text)

    @staticmethod
    def _fill_table(table, rows):
        """复用表格项，只修改变化文字，避免每帧重建全部 Qt 对象。"""
        if table.rowCount() != len(rows):
            table.setRowCount(len(rows))
        for row, values in enumerate(rows):
            for column, value in enumerate(values):
                text = str(value)
                item = table.item(row, column)
                if item is None:
                    item = QtWidgets.QTableWidgetItem(text)
                    item.setToolTip(text)
                    table.setItem(row, column, item)
                elif item.text() != text:
                    item.setText(text)
                    item.setToolTip(text)

    def _fill_waiting_tables(self):
        self._fill_table(self.ui.tableWidget_JointTargets, [[f"J{i}"] + ["未启动"] * 5 for i in range(1, 7)])
        self._fill_table(self.ui.tableWidget_TcpTargets, [[axis] + ["等待 TCP 配置"] * 3 + ["等待反馈"] for axis in ("x", "y", "z", "rx", "ry", "rz")])
        for name in ("MasterState", "AdmittanceState", "RobotConfiguration", "Diagnostics"):
            self._fill_table(getattr(self.ui, "tableWidget_" + name), [["等待数据", "尚无有效样本", "只读显示"]])

    def update_snapshot(self, snapshot):
        """显示监视器组合的同一次快照；保持数值刷新与曲线重绘分频。"""
        self._snapshot = snapshot
        self._refresh_tables()
        self._refresh_count += 1
        if self._refresh_count % 2 == 0:
            self._refresh_plot()

    def _refresh_tables(self):
        if self._snapshot is None:
            return
        rows = self.monitor.display_rows(self._snapshot, angle_unit=self.angle_unit, length_unit=self.length_unit)
        for key, name in (("joint_rows", "JointTargets"), ("tcp_rows", "TcpTargets"),
                          ("master_rows", "MasterState"), ("admittance_rows", "AdmittanceState"),
                          ("robot_configuration_rows", "RobotConfiguration"), ("diagnostic_rows", "Diagnostics")):
            self._fill_table(getattr(self.ui, "tableWidget_" + name), rows[key])
        self.ui.label_StateFreshness.setText(str(self._snapshot["freshness"]))
        self.ui.label_SourceTime.setText(str(self._snapshot["source_time"]))
        diag = self._snapshot["diagnostics"]
        cfg = self._snapshot["parameters"]

        def milliseconds(value):
            return "未观测" if value is None else f"{value * 1000:.3f} ms"

        heartbeat_age = diag["heartbeat_age_us"]
        prefix = "本轮" if self._snapshot["active"] else "最近一轮"
        self.ui.label_HostBudgetSummary.setText(
            f"{prefix}主机心跳最大间隔 {milliseconds(diag['heartbeat_send_max_interval_s'])}；"
            f"机器人当前心跳年龄 {milliseconds(None if heartbeat_age is None else heartbeat_age / 1000000)} / "
            f"软件阈值 {milliseconds(cfg['ADMITTANCE_HEARTBEAT_TIMEOUT_S'])}；"
            f"最大唤醒迟到 {milliseconds(diag['wake_max_lateness_s'])}；"
            f"最长持续未排空 {milliseconds(diag['rtde_backlog_max_duration_s'])}。"
            "两端时基独立，上述差值不能作为已验证的停止余量。")

    def _units_changed(self):
        self._refresh_tables()
        self._refresh_plot()

    def _refresh_plot(self):
        signal = ("joint_targets", "joint_errors", "tcp_targets", "delta_q", "delta_x",
                  "drive", "actual_qd", "master_received")[self.ui.comboBox_PlotSignal.currentIndex()]
        data = self.monitor.plot_series(signal, self.ui.comboBox_PlotAxis.currentIndex(),
                                        angle_unit=self.angle_unit, length_unit=self.length_unit)
        self.ui.widget_TrendPlot.set_data(data)

    def _plot_window_changed(self):
        self.ui.widget_TrendPlot.set_time_window((5, 15, 30, 60)[self.ui.comboBox_PlotWindow.currentIndex()])

    def _pause_plot(self, paused):
        self.ui.widget_TrendPlot.set_paused(paused)
        self.ui.pushButton_PausePlot.setText("恢复显示" if paused else "暂停显示")

    def _clear_plot(self):
        self.monitor.clear()
        self.ui.widget_TrendPlot.clear()
        self.append_log("显示曲线已清空；控制积分状态和采集记录不变。")

    def append_log(self, text):
        stamp = QtCore.QDateTime.currentDateTime().toString("HH:mm:ss")
        self.ui.plainTextEdit_AdmittanceLog.appendPlainText(f"[{stamp}] {text}")

    def showEvent(self, event):
        super().showEvent(event)
        self.monitor.set_visible(True)
        self.visibility_changed.emit(True)

    def hideEvent(self, event):
        self._save_settings()
        self.monitor.set_visible(False)
        self.visibility_changed.emit(False)
        super().hideEvent(event)

    def closeEvent(self, event):
        event.ignore()
        self.hide()
