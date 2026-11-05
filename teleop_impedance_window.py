"""遥操作阻抗显示与操作请求；不连接设备、不拥有控制参数。"""

import math
from pathlib import Path
import time

from PyQt5 import QtCore, QtWidgets

from ui_teleop_impedance_window import Ui_TeleopImpedanceWindow
from UR_Utils.impedance_reference import JointTargetFrame
from UR_Utils.ur5e_visualizer import UR5eDualVisualizer


def master_target(snapshot, config, now=None):
    """显示与操作边界共用新鲜 JOINT_TRACK 及绝对位置契约。"""
    now = time.monotonic() if now is None else now
    if snapshot['last_error']:
        raise ValueError('主端接收错误：' + snapshot['last_error'])
    command = snapshot['command']
    if command is None:
        raise ValueError('尚未收到主端目标')
    if command.mode != 1:
        raise ValueError('主端当前不是关节跟踪模式')
    stamp = snapshot['received_monotonic']
    if stamp is None:
        raise ValueError('主端目标没有接收时刻')
    age = now - stamp
    if age < 0:
        raise ValueError('主端接收时刻来自未来')
    if age > config['SOURCE_TIMEOUT_S']:
        raise ValueError(f'主端目标已过期（最近接收 {age*1000:.0f} ms）')
    frame = JointTargetFrame(1, 'teleop', snapshot['frame_count'], tuple(command.q_arm), stamp,
                             qd_rad_s=command.qd_arm)
    frame.check(1, now, config)
    return frame


class TeleopImpedanceWindow(QtWidgets.QWidget):
    start_requested = QtCore.pyqtSignal()
    stop_requested = QtCore.pyqtSignal()
    sync_requested = QtCore.pyqtSignal()
    visibility_changed = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent, QtCore.Qt.Window)
        self.ui = Ui_TeleopImpedanceWindow()
        self.ui.setupUi(self)
        model = Path(__file__).resolve().parent / 'UR_Utils/universal_robots_ur5e/ur5e.xml'
        self.visualizer = UR5eDualVisualizer(str(model), camera_azimuth_deg=-120, camera_elevation_deg=21)
        self.ui.viewLayout.addWidget(self.visualizer.widget)
        self._master = self._actual = None
        self.visualizer.set_visibility(actual=False, virtual=False)
        self.ui.startButton.clicked.connect(self.start_requested.emit)
        self.ui.stopButton.clicked.connect(self.stop_requested.emit)
        self.ui.syncButton.clicked.connect(self.sync_requested.emit)

    def update_context(self, *, snapshot, actual_q, config, revision, dirty,
                       reason, phase, can_sync, can_start, can_stop, result, now=None):
        now = time.monotonic() if now is None else now
        self.ui.parameterLabel.setText(f'参数：已应用版本 {revision}' + (' · 参数有未应用修改，请在单关节窗口处理' if dirty else ''))
        self.ui.statusLabel.setText('状态：' + (reason or phase))
        self.ui.startButton.setEnabled(can_start)
        self.ui.syncButton.setEnabled(can_sync)
        self.ui.stopButton.setEnabled(can_stop)
        self.ui.resultLabel.setText(result)
        try:
            target = master_target(snapshot, config, now)
            q_master = target.q_rad
            self.ui.sourceLabel.setText(f'主端：关节跟踪 · 最近接收 {(now-target.source_monotonic_s)*1000:.0f} ms · 帧 {target.source_seq}')
        except ValueError as exc:
            q_master = None
            self.ui.sourceLabel.setText('主端：' + str(exc))
        for name, q in (('master', q_master), ('actual', actual_q)):
            for j in range(6):
                getattr(self.ui, f'{name}Angle{j}').setText('—' if q is None else f'{math.degrees(q[j]):.4f}')
        if q_master is not None and tuple(q_master) != self._master:
            self._master = tuple(q_master)
            self.visualizer.update_virtual(self._master)
        if actual_q is not None and tuple(actual_q) != self._actual:
            self._actual = tuple(actual_q)
            self.visualizer.update_actual(self._actual)
        self.visualizer.set_visibility(actual=self._actual is not None, virtual=self._master is not None)
        actual_label = '新鲜' if actual_q is not None else ('已过期' if self._actual is not None else '无数据')
        master_label = '新鲜' if q_master is not None else ('已过期' if self._master is not None else '无数据')
        self.ui.legendLabel.setText(f'实体：从端实际（{actual_label}） · 半透明：主端目标（{master_label}） · 主从为异步显示')

    def clear_robot(self):
        """断开或切换机器人时清除旧实际姿态。"""
        self._actual = None
        self.visualizer.set_visibility(actual=False, virtual=self._master is not None)
        for j in range(6):
            getattr(self.ui, f'actualAngle{j}').setText('—')

    def showEvent(self, event):
        super().showEvent(event)
        self.visibility_changed.emit()

    def hideEvent(self, event):
        super().hideEvent(event)
        self.visibility_changed.emit()

    def closeEvent(self, event):
        event.ignore()
        self.hide()
