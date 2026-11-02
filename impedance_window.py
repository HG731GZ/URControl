"""非模态阻抗参数/轨迹窗口；只发信号，不拥有机器人连接。"""

from copy import deepcopy
import json
from pathlib import Path
from PyQt5 import QtCore, QtWidgets
from ui_impedance_window import Ui_ImpedanceWindow
from impedance_parameters import ImpedanceParameters
from impedance_monitor import ImpedanceMonitor
from UR_Utils.impedance_trajectory import JointTrajectory
from UR_Utils.impedance_config import snapshot, validate
from impedance_tuning_window import ImpedanceTuningWindow


class ImpedanceWindow(QtWidgets.QWidget):
    start_requested = QtCore.pyqtSignal(dict)
    stop_requested = QtCore.pyqtSignal()
    sync_requested = QtCore.pyqtSignal()
    move_start_requested = QtCore.pyqtSignal(list)
    FIELDS = [
        ("JOINT_K", "K 刚度"),
        ("JOINT_D", "D 阻尼"),
        ("TAU_MAX_NM", "最大指令力矩 N·m"),
        ("TAU_RATE_MAX_NM_S", "力矩变化率 N·m/s"),
        ("REF_MAX_QD_RAD_S", "参考速度 rad/s"),
        ("REF_MAX_QDD_RAD_S2", "参考加速度 rad/s²"),
    ]

    def __init__(self, parent=None):
        super().__init__(parent, QtCore.Qt.Window)
        self.ui = Ui_ImpedanceWindow()
        self.ui.setupUi(self)
        self._draft_valid = True
        self.parameters = ImpedanceParameters()
        self.trajectory = None
        self._active = False
        self._ready = False
        self._status = {}
        self.tuning_window = None
        self._current_q = self._current_qd = None
        self.monitor = ImpedanceMonitor(self)
        self.ui.plotLayout.addWidget(self.monitor)
        table = self.ui.parameterTable
        table.setRowCount(len(self.FIELDS))
        table.setColumnCount(6)
        table.setHorizontalHeaderLabels([f"J{i}" for i in range(1, 7)])
        table.setVerticalHeaderLabels([label for _, label in self.FIELDS])
        table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        table.itemChanged.connect(self._edited)
        actions = {
            "load": self.load,
            "generate": self.generate,
            "preview": self.preview,
            "apply": self.apply,
            "restore": self.restore,
            "import": self.import_preset,
            "export": self.export_preset,
            "export_run": self.export_run,
            "start": self.start,
            "stop": self.stop_requested.emit,
            "sync": self.sync_requested.emit,
            "move_start": self.move_start,
        }
        for name, callback in actions.items():
            getattr(self.ui, name + "Button").clicked.connect(callback)
        self.ui.axisCombo.currentIndexChanged.connect(
            lambda axis: setattr(self.monitor, "axis", axis)
        )
        self.ui.playHz.valueChanged.connect(self.describe)
        self.ui.sourceCombo.currentIndexChanged.connect(self.describe)
        self._fill()
        self.describe()

    def _fill(self):
        self._draft_valid = True
        table = self.ui.parameterTable
        table.blockSignals(True)
        for r, (key, _) in enumerate(self.FIELDS):
            for j, v in enumerate(self.parameters.draft[key]):
                table.setItem(r, j, QtWidgets.QTableWidgetItem(f"{v:g}"))
        table.blockSignals(False)

    def _edited(self):
        try:
            for r, (key, _) in enumerate(self.FIELDS):
                self.parameters.draft[key] = [
                    float(self.ui.parameterTable.item(r, j).text()) for j in range(6)
                ]
        except ValueError:
            self._draft_valid = False
            self.ui.startButton.setEnabled(False)
            return
        self._draft_valid = True
        self.describe()

    def _error(self, error):
        QtWidgets.QMessageBox.warning(self, "关节阻抗", str(error))

    def load(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "加载六轴轨迹", "", "CSV (*.csv)"
        )
        if not path:
            return
        try:
            self.trajectory = JointTrajectory.load(path)
            self.ui.playHz.setValue(self.trajectory.sample_hz)
            self.describe()
        except Exception as exc:
            self._error(exc)

    def generate(self):
        try:
            if self.tuning_window and (
                self.tuning_window.executing or self.tuning_window.report_worker
            ):
                self.tuning_window.show()
                self.tuning_window.raise_()
                return
            if not self._draft_valid:
                raise ValueError("请先修正阻抗参数表中的无效数字")
            cfg = validate(self.parameters.draft)
            if self.tuning_window is None:
                self.tuning_window = ImpedanceTuningWindow(
                    cfg, self.ui.axisCombo.currentIndex(), self
                )
                self.tuning_window.start_requested.connect(self._start_tuning)
                self.tuning_window.stop_requested.connect(self.stop_requested.emit)
            elif cfg != self.tuning_window.config and not self.tuning_window.executing:
                self.tuning_window.config = cfg
                self.tuning_window._fill_axis()
                self.tuning_window._invalidate()
            self.tuning_window.update_context(
                self._status,
                self._ready,
                self._active,
                self._current_q,
                self._current_qd,
            )
            self.tuning_window.show()
            self.tuning_window.raise_()
        except Exception as exc:
            self._error(exc)

    def _start_tuning(self, kwargs):
        self.parameters.draft = deepcopy(kwargs["config"])
        self.parameters.apply()
        self.parameters.effective = deepcopy(self.parameters.prepared)
        self.tuning_window.config = deepcopy(kwargs["config"])
        self._fill()
        self.describe()
        self.start_requested.emit(kwargs)

    def tuning_start_failed(self, message):
        if self.tuning_window:
            self.tuning_window.start_failed(message)

    def preview(self):
        if self.trajectory is None:
            return
        from matplotlib.figure import Figure
        from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
        import numpy as np

        self._preview = QtWidgets.QDialog(self)
        self._preview.setWindowTitle("轨迹预览（文件坐标）")
        layout = QtWidgets.QVBoxLayout(self._preview)
        fig = Figure(figsize=(9, 5))
        canvas = FigureCanvasQTAgg(fig)
        layout.addWidget(canvas)
        ax = fig.subplots()
        q = np.array(self.trajectory.rows)
        t = np.arange(len(q)) / self.ui.playHz.value()
        for j in range(6):
            ax.plot(t, q[:, j], label=f"J{j+1}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Joint angle / offset (rad)")
        ax.legend()
        fig.tight_layout()
        self._preview.resize(900, 500)
        self._preview.show()

    def apply(self):
        try:
            for r, (key, _) in enumerate(self.FIELDS):
                self.parameters.draft[key] = [
                    float(self.ui.parameterTable.item(r, j).text()) for j in range(6)
                ]
            self.parameters.apply()
            self.describe()
        except Exception as exc:
            self._error(exc)

    def restore(self):
        self.parameters.draft = snapshot()
        self._fill()
        self.describe()

    def import_preset(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "导入阻抗预设", "", "JSON (*.json)"
        )
        if path:
            try:
                self.parameters.load(path)
                self._fill()
                self.describe()
            except Exception as exc:
                self._error(exc)

    def export_preset(self):
        if not self._draft_valid:
            self._error("草稿包含无效数字，请先修正")
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "导出阻抗草稿", "impedance_preset.json", "JSON (*.json)"
        )
        if path:
            try:
                self.parameters.save(path)
            except Exception as exc:
                self._error(exc)

    def export_run(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "导出本轮状态", "impedance_run.json", "JSON (*.json)"
        )
        if path:
            Path(path).write_text(
                json.dumps(
                    dict(
                        record_type="joint_impedance_run",
                        status=self._status,
                        parameters=self.parameters.effective,
                    ),
                    ensure_ascii=False,
                    indent=2,
                )
            )

    def move_start(self):
        if self.trajectory and self.trajectory.coordinate == "absolute":
            self.move_start_requested.emit(list(self.trajectory.rows[0]))

    def start(self):
        if self.parameters.dirty or self._active or not self._draft_valid:
            return
        try:
            source = (
                "trajectory" if self.ui.sourceCombo.currentIndex() == 0 else "teleop"
            )
            if source == "trajectory":
                self.trajectory.validate_speed(
                    self.parameters.prepared, self.ui.playHz.value()
                )
            self.parameters.effective = deepcopy(self.parameters.prepared)
            self.start_requested.emit(
                dict(
                    reference_source=source,
                    config=deepcopy(self.parameters.prepared),
                    trajectory=self.trajectory if source == "trajectory" else None,
                    play_hz=self.ui.playHz.value(),
                )
            )
        except Exception as exc:
            self._error(exc)

    def describe(self):
        t = self.trajectory
        if t:
            self.ui.trajectoryLabel.setText(
                f"{t.coordinate} · {len(t.rows)} 行 · 名义 {t.sample_hz:g} Hz · "
                f"{self.ui.playHz.value()/t.sample_hz:.2f} 倍 · {t.duration(self.ui.playHz.value()):.2f} s"
                + (" · 参数修改待应用" if self.parameters.dirty else " · 参数已应用")
            )
        self.ui.startButton.setEnabled(
            self._ready
            and not self._active
            and self._draft_valid
            and not self.parameters.dirty
            and (self.ui.sourceCombo.currentIndex() == 1 or t is not None)
        )
        self.ui.stopButton.setEnabled(self._active)

    def update_status(
        self, status, ready=False, busy=False, actual_q=None, actual_qd=None
    ):
        self._status = status
        self._current_q, self._current_qd = actual_q, actual_qd
        self._active = status.get("active", False) or busy
        self._ready = ready
        phase = status.get("phase", "IDLE")
        error = status.get("last_error") or status.get("stop_error") or ""
        sample = status.get("sample") or {}
        diagnostics = (
            f"源年龄 {status.get('source_age_s',0)*1000:.1f} ms · "
            f"最大循环 {sample.get('max_interval_s',0)*1000:.3f} ms · "
            f"参考提交/采纳 {status.get('reference_seq',0)}/{sample.get('adopted_reference_seq',0)} · "
            f"轨迹行 {status.get('trajectory_row_position',0):.1f}"
        )
        self.ui.statusLabel.setText(
            f'{phase}  {error}\n{diagnostics}\n日志：{status.get("log_directory") or "未开始"}'
        )
        for name in (
            "load",
            "generate",
            "apply",
            "restore",
            "import",
            "move_start",
            "sync",
        ):
            getattr(self.ui, name + "Button").setEnabled(not self._active)
        if self.tuning_window and self.tuning_window.executing:
            self.ui.generateButton.setEnabled(True)
        for widget in (self.ui.parameterTable, self.ui.sourceCombo, self.ui.playHz):
            widget.setEnabled(not self._active)
        if self.tuning_window:
            self.tuning_window.update_context(status, ready, busy, actual_q, actual_qd)
        self.monitor.update_status(status)
        self.describe()

    def closeEvent(self, event):
        event.ignore()
        self.hide()
