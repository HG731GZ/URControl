"""非模态阻抗轨迹窗口；只发信号，不拥有机器人连接。"""

from copy import deepcopy
import json
from pathlib import Path
from PyQt5 import QtCore, QtWidgets
from ui_impedance_window import Ui_ImpedanceWindow
from impedance_monitor import ImpedanceMonitor
from UR_Utils.impedance_trajectory import JointTrajectory


class ImpedanceWindow(QtWidgets.QWidget):
    start_requested = QtCore.pyqtSignal(dict)
    stop_requested = QtCore.pyqtSignal()
    tuning_requested = QtCore.pyqtSignal()
    move_start_requested = QtCore.pyqtSignal(list)

    def __init__(self, parameters, parent=None):
        super().__init__(parent, QtCore.Qt.Window)
        self.ui = Ui_ImpedanceWindow()
        self.ui.setupUi(self)
        self.parameters = parameters
        self.trajectory = None
        self._active = False
        self._ready = False
        self._status = {}
        self._can_stop = False
        self._current_q = self._current_qd = None
        self.monitor = ImpedanceMonitor(self)
        self.ui.plotLayout.addWidget(self.monitor)
        actions = {
            "load": self.load,
            "generate": self.tuning_requested.emit,
            "preview": self.preview,
            "export_run": self.export_run,
            "start": self.start,
            "stop": self.stop_requested.emit,
            "move_start": self.move_start,
        }
        for name, callback in actions.items():
            getattr(self.ui, name + "Button").clicked.connect(callback)
        self.ui.axisCombo.currentIndexChanged.connect(
            lambda axis: setattr(self.monitor, "axis", axis)
        )
        self.ui.playHz.valueChanged.connect(self.describe)
        self.parameters.changed.connect(self.describe)
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
        if self.parameters.dirty or self._active or not self._ready:
            return
        try:
            self.trajectory.validate_playback(
                self.parameters.prepared, self.ui.playHz.value()
            )
            self.start_requested.emit(
                dict(
                    reference_source="trajectory",
                    config=deepcopy(self.parameters.prepared),
                    trajectory=self.trajectory,
                    play_hz=self.ui.playHz.value(),
                )
            )
        except Exception as exc:
            self._error(exc)

    def describe(self):
        self.ui.parameterLabel.setText(
            f"参数：已应用版本 {self.parameters.revision}"
            + (" · 有未应用修改，请在单关节窗口处理" if self.parameters.dirty else "")
        )
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
            and not self.parameters.dirty
            and t is not None
        )
        self.ui.stopButton.setEnabled(self._can_stop)

    def update_status(
        self, status, ready=False, busy=False, actual_q=None, actual_qd=None, can_stop=False
    ):
        self._status = status
        self._can_stop = can_stop
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
        for name in ("load", "move_start"):
            getattr(self.ui, name + "Button").setEnabled(not self._active and (ready or name == "load"))
        self.ui.playHz.setEnabled(not self._active)
        self.monitor.update_status(status)
        self.describe()

    def closeEvent(self, event):
        event.ignore()
        self.hide()
