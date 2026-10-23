"""仅预览界面，不导入主程序、不连接设备、不创建采集文件。"""

import argparse
import math
import sys
from types import SimpleNamespace

from PyQt5 import QtCore, QtWidgets

from window_display import WindowDisplayController, configure_high_dpi
from window_layouts import create_layout, saved_layout, switch_layout


class PreviewWindow(QtWidgets.QMainWindow):
    """两套主界面的独立预览外壳。"""

    def __init__(self, settings=None):
        super().__init__()
        self.settings = (settings if settings is not None
                         else QtCore.QSettings('URControl', 'Display'))
        self.mode = saved_layout(self.settings)
        self.ui = create_layout(self, self.mode)
        self.display = WindowDisplayController(self, self.ui, self.settings)
        self._admittance_window = None
        self._configure_preview()
        self.ui.pushButton_Admittance.clicked.connect(self.open_admittance)

    def _configure_preview(self):
        self.setWindowTitle('URControl · 离线界面预览')
        label = (self.ui.label_ClassicTitle if self.display.classic
                 else self.ui.label_Subtitle)
        label.setText('离线界面预览 · 不连接设备')
        self.ui.actionLayoutModern.triggered.connect(lambda: self.change_layout('modern'))
        self.ui.actionLayoutClassic.triggered.connect(lambda: self.change_layout('classic'))

    def open_admittance(self):
        if self._admittance_window is None:
            self._admittance_window = create_admittance_preview(settings=self.settings)
        self._admittance_window.show()
        self._admittance_window.raise_()
        self._admittance_window.activateWindow()

    def change_layout(self, mode):
        if mode == self.mode:
            return
        self.setUpdatesEnabled(False)
        try:
            self.display.dispose()
            self.ui = switch_layout(self, self.ui, mode)
            self.mode = mode
            self.settings.setValue('layoutMode', mode)
            self.display = WindowDisplayController(self, self.ui, self.settings)
            self._configure_preview()
        finally:
            self.setUpdatesEnabled(True)


def create_admittance_preview(scenario='teleop-joint', settings=None):
    """用固定演示样本展示四种场景，不初始化任何硬件对象。"""
    from admittance_window import AdmittanceWindow

    source, space = scenario.split('-')
    window = AdmittanceWindow(settings=settings)
    window.setWindowTitle('导纳调参与监视 · 演示数据 · 不连接设备')
    window.ui.comboBox_ReferenceSource.setCurrentIndex(('teleop', 'free').index(source))
    window.ui.comboBox_AdmittanceSpace.setCurrentIndex(('joint', 'cartesian').index(space))
    config = window.parameters.prepared_copy()
    if space == 'cartesian':
        config['CART_FRAME'] = 'tool_start'
    window.parameters.set_draft(config)
    window.parameters.apply()
    window._write_editors()
    window.set_effective(config, window.parameters.revision)
    configuration = {
        'tcp_offset': [0.0, 0.0, 0.12, 0.0, 0.0, 0.0],
        'payload': 1.2, 'payload_cog': [0.0, 0.0, 0.045],
        'payload_inertia': [0.002, 0.002, 0.001, 0.0, 0.0, 0.0], 'timestamp': 42.0,
    }
    base = [0.0, -1.15, 1.45, -1.8, -1.57, 0.1]
    status = None
    for index in range(160):
        now = 1000.0 + index * 0.05
        movement = 0.025 * math.sin(index * 0.06)
        actual = [value + movement * (1 if axis % 2 == 0 else -0.6) for axis, value in enumerate(base)]
        final = [value + 0.002 for value in actual]
        nominal = [value - 0.006 for value in final]
        master = [value + 0.003 for value in nominal]
        status = {
            'active': True, 'reference_source': source, 'space': space, 'phase_name': '运行中（演示）',
            'parameters': config, 'timestamp': 42.0 + index * 0.05,
            'state_received_monotonic': now - 0.004, 'state_receive_count': 1000 + index * 5,
            'q_master': master if source == 'teleop' else None,
            'q_master_submitted': [value + 0.001 for value in master] if source == 'teleop' else None,
            'q_nom': nominal if source == 'teleop' else None, 'q_cmd': final,
            'q_start': base if source == 'free' else None,
            'actual_q': actual, 'actual_qd': [0.005 * math.cos(index * 0.06)] * 6,
            'actual_TCP_pose': [0.35, -0.21, 0.48, 0.01, 3.10, 0.02],
            'submitted_source_seq': 1000 + index * 5 if source == 'teleop' else None,
            'adopted_source_seq': 999 + index * 5 if source == 'teleop' else None,
            'source_seq': 999 + index * 5 if source == 'teleop' else None,
            'robot_configuration': configuration, 'drive': [1.2, -0.8, 0.4, 0.08, -0.03, 0.02],
            'delta_q': [0.006] * 6 if source == 'teleop' and space == 'joint' else None,
            'delta_x': [0.001, 0.002, -0.001, 0.003, 0.0, 0.002] if source == 'teleop' and space == 'cartesian' else None,
            'v_free': [0.004, 0.003, -0.002, 0.001, 0.0, 0.001] if source == 'free' else None,
            'cycle_count': 6000 + index * 25, 'runtime_state': 2, 'safety_status_bits': 1,
            'exit_reason': 0, 'exit_reason_text': '未退出', 'last_error': None,
            'reference_send_count': 1000 + index * 5 if source == 'teleop' else 0,
            'heartbeat_send_count': 500 + index * 2,
            'last_reference_sent_monotonic': now - 0.006 if source == 'teleop' else None,
            'last_heartbeat_sent_monotonic': now - 0.01,
        }
        udp = {'command': SimpleNamespace(mode=1, q_arm=[value + 0.0015 for value in master],
                                           q_gripper=[0.4, 0.2, 0.3]),
               'frame_count': 1001 + index * 5, 'received_monotonic': now - 0.003,
               'received_time': 1780000000.0 + index * 0.05, 'sender_address': ('192.0.2.10', 5000),
               'bind_address': ('0.0.0.0', 5001), 'last_error': None}
        snapshot = window.monitor.update(status, udp, run_id=7, connected=True, now=now,
                                         parameter_revision=window.parameters.revision,
                                         gripper={'requested_target': {'open': 0.4, 'speed': 100, 'force': 50},
                                                  'sent_target': {'open': 0.4}, 'target_sent_monotonic': now - 0.02,
                                                  'feedback': {'open': 0.4, 'real_pos': 1600, 'current': 12},
                                                  'feedback_received_monotonic': now - 0.01,
                                                  'feedback_count': 200 + index, 'following_thread_active': source == 'teleop',
                                                  'last_error': None})
    window.update_context({'robot_ready': True, 'occupied': True, 'master_fresh': True,
                           'synced': source == 'teleop', 'phase': status['phase_name'], 'run_id': 7,
                           'robot_address': '192.0.2.20（演示地址）',
                           'controller_joint_limits': config['JOINT_POSITION_LIMIT_RAD'],
                           'run_metadata': {'run_id': 7, 'reference_source': source, 'space': space,
                                            'robot_configuration': configuration, 'start_time': '演示数据',
                                            'end_time': None}})
    window.update_snapshot(snapshot)
    window._refresh_plot()
    window.ui.label_OperationHint.setText('演示数据 · 固定离线样本 · 不连接设备；--scenario 可选四种场景。')
    window.append_log('固定演示数据：' + ('遥操作导纳' if source == 'teleop' else '主动柔顺') + ' / ' + ('关节空间' if space == 'joint' else '末端空间'))
    def stop_preview():
        status['active'] = False
        status['phase_name'] = '已停止（演示）'
        status['exit_reason_text'] = '用户停止演示'
        window.update_context({'occupied': False, 'robot_ready': False, 'phase': status['phase_name']})
        window.update_snapshot(window.monitor.update(status, udp, run_id=7, connected=True,
                                                     now=1008.1, parameter_revision=window.parameters.revision))
        window.append_log('演示已停止；再次运行预览可重新建立固定样本。')

    window.stop_requested.connect(stop_preview)
    return window


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='URControl 离线界面预览，不连接设备')
    parser.add_argument('--admittance', action='store_true', help='预览独立导纳窗口')
    parser.add_argument('--scenario', choices=('teleop-joint', 'teleop-cartesian', 'free-joint', 'free-cartesian'),
                        default='teleop-joint', help='固定演示场景')
    args = parser.parse_args()
    configure_high_dpi()
    app = QtWidgets.QApplication(sys.argv)
    window = create_admittance_preview(args.scenario) if args.admittance else PreviewWindow()
    window.show()
    sys.exit(app.exec_())
