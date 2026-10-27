"""仅预览界面，不导入主程序、不连接设备、不创建采集或调试日志文件。"""

import argparse
from copy import deepcopy
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


def create_admittance_preview(scenario='diagnostic-joint', settings=None):
    """用离线演示样本展示柔顺场景或固定保持诊断，不初始化硬件。"""
    import numpy as np

    from admittance_window import AdmittanceWindow
    from UR_Utils.ur5e_kinematics import UR5eKinematics
    from UR_Utils.ur_pose_math import matrix_to_rotvec, rotvec_to_matrix

    source, space = scenario.split('-')
    diagnostic = source == 'diagnostic'
    preset = source == 'preset'
    if diagnostic or preset:
        source = 'fixed'
    window = AdmittanceWindow(settings=settings)
    window.setWindowTitle('导纳调参与监视 · 演示数据 · 不连接设备')
    window.ui.comboBox_ReferenceSource.setCurrentIndex(('teleop', 'fixed').index(source))
    window.ui.comboBox_AdmittanceSpace.setCurrentIndex(('joint', 'cartesian').index(space))
    config = window.parameters.prepared_copy()
    if space == 'cartesian':
        config['CART_FRAME'] = 'tool_start'
    config['DIAGNOSTIC_AXIS'] = 3
    config['DIAGNOSTIC_MODE'] = 'fixed_joint_hold' if diagnostic else ('preset_joint_motion' if preset else 'admittance')
    window.parameters.set_draft(config)
    window.parameters.apply()
    # 离线预览需要演示专用配置；不写入 Python 配置文件或用户预设。
    window.parameters.prepared_parameters = deepcopy(config)
    window.parameters.draft_parameters = deepcopy(config)
    window._write_editors()
    window.set_effective(config, window.parameters.revision)
    configuration = {
        'tcp_offset': [0.0, 0.0, 0.12, 0.0, 0.0, 0.0],
        'payload': 1.2, 'payload_cog': [0.0, 0.0, 0.045],
        'payload_inertia': [0.002, 0.002, 0.001, 0.0, 0.0, 0.0], 'timestamp': 42.0,
    }
    base = [0.0, -1.15, 1.45, -1.8, -1.57, 0.1]
    kinematics = UR5eKinematics(tcp_offset=configuration['tcp_offset'][:3],
                               tcp_rotation=configuration['tcp_offset'][3:], load_geometry=False)
    start_pose = kinematics.forward_kinematics(base)[3]
    compliance_rotation = rotvec_to_matrix(start_pose[3:]) if config['CART_FRAME'] == 'tool_start' else np.eye(3)
    previous_actual = base
    status = None
    for index in range(160):
        now = 1000.0 + index * 0.05
        elapsed = index * 0.05
        movement = 0.025 * math.sin(index * 0.06)
        nominal = [value + movement * (1 if axis % 2 == 0 else -0.6)
                   for axis, value in enumerate(base)] if source == 'teleop' else list(base)
        response = (0.0 if elapsed < 0.5 else 0.025 * (1.0 - math.exp(-(elapsed - 0.5) / 0.6)))
        if elapsed >= 3.0:
            response = 0.025 * (1.0 - math.exp(-2.5 / 0.6)) * math.exp(-(elapsed - 3.0) / 0.8)
        delta_q = [response * (1 if axis % 2 == 0 else -0.6) for axis in range(6)]
        if diagnostic or preset:
            delta_q = [0.0] * 6
        if preset:
            delta_q[config['DIAGNOSTIC_MOTION_AXIS'] - 1] = 0.5 * config['DIAGNOSTIC_MOTION_AMPLITUDE_RAD'] * (1.0 - math.cos(2.0 * math.pi * elapsed / config['DIAGNOSTIC_MOTION_PERIOD_S']))
        final = [value + offset for value, offset in zip(nominal, delta_q)]
        actual = [value + offset * 0.98 for value, offset in zip(nominal, delta_q)]
        actual_qd = [(value - previous) / 0.05 for value, previous in zip(actual, previous_actual)]
        previous_actual = actual
        nominal_pose = kinematics.forward_kinematics(nominal)[3]
        final_pose = kinematics.forward_kinematics(final)[3]
        delta_x = np.concatenate((
            compliance_rotation.T @ (np.asarray(final_pose[:3]) - np.asarray(nominal_pose[:3])),
            compliance_rotation.T @ matrix_to_rotvec(rotvec_to_matrix(final_pose[3:]) @ rotvec_to_matrix(nominal_pose[3:]).T),
        )).tolist()
        master = [value + 0.003 for value in nominal]
        status = {
            'active': True, 'reference_source': source, 'space': space, 'phase_name': '运行中（演示）',
            'parameters': config, 'timestamp': 42.0 + index * 0.05,
            'state_received_monotonic': now - 0.004, 'state_receive_count': 1000 + index * 5,
            'q_master': master if source == 'teleop' else None,
            'q_master_submitted': [value + 0.001 for value in master] if source == 'teleop' else None,
            'q_nom': nominal, 'q_cmd': final,
            'q_start': base if source == 'fixed' else None,
            'actual_q': actual, 'actual_qd': actual_qd,
            'actual_TCP_pose': kinematics.forward_kinematics(actual)[3].tolist(),
            'submitted_source_seq': 1000 + index * 5 if source == 'teleop' else None,
            'adopted_source_seq': 999 + index * 5 if source == 'teleop' else None,
            'source_seq': 999 + index * 5 if source == 'teleop' else None,
            'robot_configuration': configuration,
            'drive': [1.2, -0.8, 0.4, 0.08, -0.03, 0.02] if not diagnostic and 0.5 <= elapsed < 3.0 else [0.0] * 6,
            'delta_q': delta_q if space == 'joint' else None,
            'delta_x': delta_x if space == 'cartesian' else None,
            'cycle_count': 6000 + index * 25, 'runtime_state': 2, 'safety_status_bits': 1,
            'exit_reason': 0, 'exit_reason_text': '未退出', 'last_error': None,
            'speed_scaling': 1.0, 'target_speed_fraction': 1.0, 'scaling_ok': True,
            'reference_send_count': 1000 + index * 5 if source == 'teleop' else 0,
            'heartbeat_send_count': 500 + index * 2,
            'last_reference_sent_monotonic': now - 0.006 if source == 'teleop' else None,
            'last_heartbeat_sent_monotonic': now - 0.01,
            'heartbeat_echo': 500 + index * 2, 'heartbeat_age_us': 10000,
            'heartbeat_max_interval_us': 22000, 'heartbeat_send_interval_s': 0.02,
            'heartbeat_send_max_interval_s': 0.021, 'heartbeat_send_lateness_s': 0.0002,
            'heartbeat_send_max_lateness_s': 0.001, 'heartbeat_deadline_miss_count': 0,
            'processing_duration_s': 0.0012, 'processing_max_duration_s': 0.0024,
            'rtde_processing_duration_s': 0.0008, 'primary_processing_duration_s': 0.0002,
            'packet_prepare_duration_s': 0.00003, 'packet_prepare_max_duration_s': 0.00008,
            'send_duration_s': 0.00002, 'send_max_duration_s': 0.00006,
            'receive_duration_s': 0.00004, 'receive_max_duration_s': 0.00008,
            'decode_duration_s': 0.00010, 'decode_max_duration_s': 0.00015,
            'log_submit_duration_s': 0.00001, 'log_submit_max_duration_s': 0.00004,
            'last_wake_monotonic_s': now - 0.002,
            'wake_deadline_monotonic_s': now - 0.0021,
            'wake_lateness_s': 0.0001, 'wake_max_lateness_s': 0.0007,
            'rtde_buffered_bytes': 0, 'rtde_buffered_max_bytes': 1024,
            'state_timestamp_interval_s': 0.002, 'state_host_interval_s': 0.0021,
            'state_progress_lag_s': 0.0, 'state_progress_max_lag_s': 0.0015,
            'state_last_progress_monotonic_s': now - 0.004,
            'state_progress_rate': 0.95, 'state_progress_window_host_s': 1.0,
            'state_progress_window_robot_s': 0.95,
            'rtde_socket_drained': True, 'rtde_last_drained_monotonic_s': now - 0.004,
            'rtde_drain_count': 1000 + index * 5,
            'rtde_backlog_duration_s': 0.0, 'rtde_backlog_max_duration_s': 0.003,
            'communication_process_pid': 4242, 'process_worker_failed': False,
            'process_status_published_monotonic_s': now - 0.006,
            'process_status_received_monotonic_s': now - 0.004,
            'process_status_age_s': 0.006, 'process_bridge_dropped_count': 0,
            'host_timing': {
                'processing': {
                    'count': 1000 + index * 5, 'last_wall_s': 0.0012, 'last_cpu_s': 0.0008,
                    'max_wall_s': 0.0024, 'max_cpu_s': 0.0015, 'max_wall_cpu_s': 0.0013,
                    'last_started_monotonic_s': now - 0.002,
                    'last_finished_monotonic_s': now - 0.0008,
                    'max_wall_started_monotonic_s': now - 1.0,
                    'max_wall_finished_monotonic_s': now - 0.9976,
                },
            },
            'next_heartbeat_deadline_monotonic_s': now + 0.01,
            'next_reference_deadline_monotonic_s': now + 0.004 if source == 'teleop' else None,
            'diagnostic': None,
        }
        sample_time = config['DIAGNOSTIC_SETTLE_S'] + config['ADMITTANCE_BIAS_DURATION_S'] + elapsed
        sequence = 1000 + index * 12
        frame = 2000 + index * 25
        identity = {
            'run_id': 7, 'page_seq': index * 3 + 2, 'model_sample_seq': sequence,
            'snapshot_time_s': sample_time, 'phase': 2,
            'received_monotonic_s': now - 0.004, 'robot_timestamp_s': status['timestamp'],
        }
        execution = {
            **identity, 'generated_seq': sequence + 1, 'submitted_seq': sequence + 1,
            'adopted_seq': sequence, 'frame': frame, 'based_on_seq': sequence - 1,
            'start_frame': frame, 'end_frame': frame + 1, 'gap_count': 0, 'limit_reason': 0,
            'servoj_start_interval_s': 0.002, 'servoj_max_interval_s': 0.002,
            'q_cmd': final, 'qd_cmd': actual_qd, 'q_nom': nominal, 'delta_q': delta_q,
        }
        timing = {
            **identity, 'page_seq': index * 3 + 1, 'snapshot_time_s': sample_time - 0.008,
            'received_monotonic_s': now - 0.012,
            'generated_seq': sequence - 2, 'submitted_seq': sequence - 2,
            'adopted_seq': sequence - 3, 'frame': frame - 4, 'based_on_seq': sequence - 4,
            'start_frame': frame - 4, 'end_frame': frame - 3, 'gap_count': 0, 'limit_reason': 0,
            'sample_time_s': sample_time - 0.012, 'generated_time_s': sample_time - 0.010,
            'submitted_time_s': sample_time - 0.010, 'adopted_time_s': sample_time - 0.008,
            'target_age_s': 0.002, 'target_max_age_s': 0.004,
            'dt_model_actual_s': 0.004, 'dt_model_max_s': 0.004,
            'servoj_start_interval_s': 0.002, 'servoj_max_interval_s': 0.002,
            'dt_base_s': 0.002, 'dt_model_design_s': 0.004,
            'bias_std_nm': [0.015] * 6, 'bias_sample_count': 125, 'bias_ready': True,
            'compute_duration_s': 0.002, 'compute_max_duration_s': 0.002,
            'servoj_duration_s': 0.002, 'servoj_duration_max_s': 0.002,
            'publish_duration_s': 0.0001, 'publish_max_duration_s': 0.0002,
        }
        load = {
            **identity, 'page_seq': index * 3, 'model_sample_seq': sequence - 4,
            'snapshot_time_s': sample_time - 0.016, 'received_monotonic_s': now - 0.020,
            'sample_time_s': sample_time - 0.018, 'generated_seq': sequence - 4,
            'submitted_seq': sequence - 4, 'limit_reason': 0,
            'tau_raw_nm': [0.12, -0.08, 0.06, 0.02, -0.01, 0.01],
            'bias_nm': [0.10, -0.07, 0.05, 0.02, -0.01, 0.01],
            'filtered_nm': [0.02, -0.01, 0.01, 0.0, 0.0, 0.0], 'drive': status['drive'],
            'dt_model_actual_s': 0.004, 'dt_model_max_s': 0.004,
            'compute_duration_s': 0.002, 'bias_sample_count': 125, 'bias_ready': True,
            'load_duration_s': 0.00004, 'publish_duration_s': 0.0001,
        }
        status.update(telemetry_layout=3, page_type='execution', page_seq=identity['page_seq'],
                      continuous={'execution': execution, 'timing': timing}, load=load)
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
    debug_log_status = {
        'enabled': True, 'active': True,
        'path': '演示日志/admittance_run7.jsonl（仅展示，不创建文件）',
        'record_count': 1737, 'dropped_count': 0, 'pending_count': 0, 'error': None,
        'logger_timing': {
            'schema_version': 1, 'producer': {}, 'writer': {},
            'queue_observed_high_watermark': 4,
            'queue_age': {'count': 1737, 'last_s': 0.0004, 'max_s': 0.0024,
                          'max_seq': 1002, 'max_kind': 'state',
                          'max_dequeued_monotonic_s': now - 1.0,
                          'last_dequeued_monotonic_s': now - 0.0004},
        },
    }
    window.update_context({'robot_ready': True, 'occupied': True, 'master_fresh': True,
                           'synced': source == 'teleop', 'phase': status['phase_name'], 'run_id': 7,
                           'robot_address': '192.0.2.20（演示地址）',
                           'controller_joint_limits': config['JOINT_POSITION_LIMIT_RAD'],
                           'diagnostic': status['diagnostic'],
                           'debug_log_status': debug_log_status,
                           'run_metadata': {'run_id': 7, 'reference_source': source, 'space': space,
                                            'schema_version': 3, 'backend': 'urscript',
                                            'model': config['DIAGNOSTIC_MODE'] if diagnostic or preset else 'mass_damper_spring',
                                            'reference_behavior': 'moving_reference' if source == 'teleop' else 'fixed_anchor',
                                            'source_codes': {1: 'teleop', 2: 'free', 3: 'fixed'},
                                            'robot_configuration': configuration, 'start_time': '演示数据',
                                            'end_time': None}})
    window.update_snapshot(snapshot)
    window.ui.comboBox_PlotWindow.setCurrentIndex(1)
    window.ui.comboBox_PlotSignal.setCurrentIndex(3 if space == 'joint' else 4)
    window._refresh_plot()
    window.ui.label_OperationHint.setText('离线演示：不连接设备、不创建日志文件。' + window.ui.label_OperationHint.text())
    window.append_log('离线演示数据：' + ('J3 保持诊断' if diagnostic else ('预设单轴微动' if preset else ('遥操作导纳' if source == 'teleop' else '定点柔顺（撤力回位）'))) + ' / ' + ('关节空间' if space == 'joint' else '末端空间'))
    window.append_log('演示趋势：目标始终保持启动锚点，力矩不驱动目标。' if diagnostic else
                      ('演示趋势：单轴按预设余弦微动，外力不参与生成；生成与采用身份独立显示。' if preset else
                       '演示趋势：0.5 s 开始受力，3 s 撤力后偏移回零；日志开关仅切换演示状态。'))
    if diagnostic or preset:
        window.ui.tabWidget_Monitor.setCurrentWidget(window.ui.tab_Diagnostics)
        QtCore.QTimer.singleShot(0, lambda: window.ui.scrollArea_Monitor.ensureWidgetVisible(window.ui.tabWidget_Monitor))

    def toggle_preview_logging(enabled):
        debug_log_status['enabled'] = enabled
        debug_log_status['active'] = status['active']
        window.update_context({'debug_log_status': debug_log_status})

    def stop_preview():
        status['active'] = False
        status['phase_name'] = '已停止（演示）'
        status['exit_reason_text'] = '用户停止演示'
        debug_log_status['active'] = False
        window.update_context({'occupied': False, 'robot_ready': False, 'phase': status['phase_name'],
                               'debug_log_status': debug_log_status})
        window.update_snapshot(window.monitor.update(status, udp, run_id=7, connected=True,
                                                     now=1008.1, parameter_revision=window.parameters.revision))
        window.append_log('演示已停止；再次运行预览可重新建立固定样本。')

    window.logging_toggled.connect(toggle_preview_logging)
    window.stop_requested.connect(stop_preview)

    def preview_operator_event(event, note):
        from admittance_window import OPERATOR_EVENTS

        window.operator_event_result(f'演示人工事件“{OPERATOR_EVENTS[event]}”：未创建或保存日志。备注：{note or "空白，请补充位置与方向"}')
        if event == 'stop_requested':
            stop_preview()

    window.operator_event_requested.connect(preview_operator_event)
    return window


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='URControl 离线界面预览，不连接设备')
    parser.add_argument('--admittance', action='store_true', help='预览独立导纳窗口')
    parser.add_argument('--scenario', choices=('teleop-joint', 'teleop-cartesian', 'fixed-joint', 'fixed-cartesian', 'diagnostic-joint', 'preset-joint'),
                        default='diagnostic-joint', help='固定演示场景；未开放场景仅可离线显示')
    args = parser.parse_args()
    configure_high_dpi()
    app = QtWidgets.QApplication(sys.argv)
    window = create_admittance_preview(args.scenario) if args.admittance else PreviewWindow()
    window.show()
    sys.exit(app.exec_())
