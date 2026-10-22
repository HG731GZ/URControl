import os
import sys
import time

import numpy as np
from rtde.rtde import RTDEException
from PyQt5 import QtCore
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QApplication, QMainWindow

import NetWorkSet
from DataCollector import DataCollector
from GripperController import GripperController, GRIPPER_SPEED_DEFAULT, GRIPPER_FORCE_DEFAULT
from RealSenseCamera import Camera, CameraError
from window_display import WindowDisplayController, configure_high_dpi
from window_layouts import create_layout, saved_layout, switch_layout
from UR_Utils.URDashboardClient import URDashboardClient
from UR_Utils.URRealtimeClient import URRealtimeClient
from UR_Utils.URRTDEController import URRTDEController
from UR_Utils import admittance_config
from UR_Utils.URAdmittanceController import PHASE_NAMES, EXIT_REASONS
from UR_Utils.URRTDETorqueClient import URRTDETorqueClient
from UR_Utils.URTcpClient import URTcpError
from UR_Utils.URUdpClient import URUDPClient, UDPControlMode
from UR_Utils.ur5e_visualizer import UR5eDualVisualizer

UR_REAL_IP = '192.168.3.15'
UR_SIM_IP = '127.0.0.1'
JOINT_JOG_SPEED_RAD_S = 0.1
TCP_JOG_LINEAR_SPEED_M_S = 0.05
TCP_JOG_ANGULAR_SPEED_RAD_S = 0.5
CAMERA_RESOLUTION = (1280, 720)
CAMERA_FPS = 30
LOCAL_IP = NetWorkSet.get_local_ip()
UDP_LOCAL_PORT = 5005
UDP_REMOTE_PORT = 6005
UDP_REMOTE_IP = '192.168.3.5'
UDP_COMMAND_TIMEOUT_S = admittance_config.MASTER_FRAME_TIMEOUT_S
ROBOT_STATE_TIMEOUT_S = 0.5
ROBOT_MODE_RUNNING = 7
UR_HOME_RAD = [-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]
UR_RTDE_FREQ_HZ = 500
UR_485_PORT = 54321
DATA_COLLECT_FREQ = 20.0
DATA_DIRECTORY = 'data'
SESSION_NAME_FORMAT = 'URCollect_%Y%m%d_%H%M%S'


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self._layout_settings = QtCore.QSettings('URControl', 'Display')
        self._layout_mode = saved_layout(self._layout_settings)
        self._ui = create_layout(self, self._layout_mode)
        self.robot_ip = UR_REAL_IP
        self.connected_ip = None
        self.dashboard_client = None
        self.realtime_client = None
        self.rtde_controller = None
        self.torque_client = None
        self.gripper = None
        self.robot_state = None
        self.robot_ready = False
        self.control_source = 'idle'
        self.admittance_mode = 'off'
        self._admittance_reference_source = None
        self._admittance_message = '未开启'
        self._admittance_runs = []
        self._admittance_run = None
        self._last_collected_admittance_time = None
        self._control_stop_failed = False
        self._udp_synced = False
        self._udp_sync_pending = False
        self._move_target = None
        self._jog_velocity = None
        self._jog_mode = None
        self._last_collected_udp_time = None
        self._last_collected_robot_time = None
        self.initialize_devices()
        self.initialize_collector()
        self.create_timers()
        self.initialize_visualizer()
        self.display_controller = WindowDisplayController(self, self._ui, self._layout_settings)
        self.bind_signals()
        self.bind_layout_actions()
        self.lineEdit_IP.setText(self.robot_ip)
        self.update_connection_label()
        self.update_buttons()
        self.status_timer.start(100)
        self.robot_view_timer.start(10)
        self.camera_timer.start(50)

    def bind_layout_actions(self):
        self.actionLayoutModern.triggered.connect(lambda: self.on_change_layout('modern'))
        self.actionLayoutClassic.triggered.connect(lambda: self.on_change_layout('classic'))

    def on_change_layout(self, mode):
        if mode == self._layout_mode:
            return
        self.setUpdatesEnabled(False)
        try:
            self.display_controller.dispose()
            self._ui = switch_layout(self, self._ui, mode)
            self._layout_mode = mode
            self._layout_settings.setValue('layoutMode', mode)
            self.display_controller = WindowDisplayController(self, self._ui, self._layout_settings)
            self.bind_layout_actions()
            self.update_connection_label()
            self.update_buttons()
        finally:
            self.setUpdatesEnabled(True)

    def initialize_devices(self):
        self.udp_client = URUDPClient(bind_host='0.0.0.0', bind_port=UDP_LOCAL_PORT)
        self.udp_client.start()
        self.camera_1 = Camera('d405', resolution=CAMERA_RESOLUTION, fps=CAMERA_FPS, rotation=180)
        self.camera_2 = Camera('d435i', resolution=(640, 480), fps=CAMERA_FPS, rotation=90)

    def initialize_collector(self):
        self.collector = DataCollector(
            base_dir=DATA_DIRECTORY, session_name=time.strftime(SESSION_NAME_FORMAT),
            write_mode='realtime', async_write=True,
            extra_metadata={
                'admittance_parameters': admittance_config.snapshot(),
                'collection_hz': DATA_COLLECT_FREQ,
                'master_not_applicable': '主动柔顺期间不写入主端目标组',
                'admittance_phase_codes': PHASE_NAMES,
                'admittance_exit_codes': EXIT_REASONS,
                'admittance_runs': [],
            })
        for name, columns, units in (
            ('UR_TCP_POSE', ['x', 'y', 'z', 'rx', 'ry', 'rz'], ['m'] * 3 + ['rad'] * 3),
            ('UR_JOINT', [f'q{i}' for i in range(1, 7)], ['rad'] * 6),
            ('MASTER_GRIPPER', ['position'], ['0..1=无量纲开度；>1=设备原始位置']),
            ('UR_GRIPPER', ['open', 'current'], ['无量纲开度，0..1', '设备原始电流单位']),
            ('MASTER_JOINT', [f'q{i}' for i in range(1, 7)], ['rad'] * 6),
            ('ADMITTANCE_STATE', ['run', 'reference_source', 'space', 'phase', 'exit_reason',
                                  'source_seq', 'adopted_source_seq', 'cycle', 'robot_time'],
             ['编号', '1=teleop;2=free', '1=joint;2=cartesian', '阶段编号', '退出原因编号',
              '帧序号;NaN=不适用', '帧序号;NaN=不适用', '周期', 's']),
            ('ADMITTANCE_Q_CMD', [f'q{i}' for i in range(1, 7)], ['rad'] * 6),
            ('ADMITTANCE_Q_ACTUAL', [f'q{i}' for i in range(1, 7)], ['rad'] * 6),
            ('ADMITTANCE_Q_MASTER', [f'q{i}' for i in range(1, 7)], ['rad'] * 6),
            ('ADMITTANCE_Q_NOM', [f'q{i}' for i in range(1, 7)], ['rad'] * 6),
            ('ADMITTANCE_Q_START', [f'q{i}' for i in range(1, 7)], ['rad'] * 6),
            ('ADMITTANCE_JOINT_DRIVE', [f'tau{i}' for i in range(1, 7)], ['Nm'] * 6),
            ('ADMITTANCE_CART_DRIVE', ['fx', 'fy', 'fz', 'tx', 'ty', 'tz'], ['N'] * 3 + ['Nm'] * 3),
            ('ADMITTANCE_DELTA_Q', [f'dq{i}' for i in range(1, 7)], ['rad'] * 6),
            ('ADMITTANCE_DELTA_X', ['dx', 'dy', 'dz', 'drx', 'dry', 'drz'], ['m'] * 3 + ['rad'] * 3),
            ('ADMITTANCE_FREE_QD', [f'qd{i}' for i in range(1, 7)], ['rad/s'] * 6),
            ('ADMITTANCE_FREE_XD', ['vx', 'vy', 'vz', 'wx', 'wy', 'wz'], ['m/s'] * 3 + ['rad/s'] * 3),
        ):
            self.collector.register_numeric(name, columns, units=units)
        for index, camera in enumerate((self.camera_1, self.camera_2), 1):
            if camera is not None:
                self.collector.register_image(
                    f'CAMERA_{index}', camera_id=camera.device_name,
                    storage='video', video_fps=DATA_COLLECT_FREQ)

    def create_timers(self):
        for name, callback in (
            ('status_timer', self.update_robot_status),
            ('robot_view_timer', self.update_robot_view),
            ('jog_timer', self.update_jog),
            ('udp_control_timer', self.update_udp_control),
            ('camera_timer', self.update_cameras),
            ('collection_timer', self.collect_sample),
        ):
            timer = QtCore.QTimer(self)
            timer.setTimerType(Qt.PreciseTimer)
            timer.timeout.connect(callback)
            setattr(self, name, timer)

    def initialize_visualizer(self):
        mjcf_path = os.path.join(os.path.dirname(__file__), 'UR_Utils/universal_robots_ur5e/ur5e.xml')
        self.robot_visualizer = UR5eDualVisualizer(
            mjcf_path, camera_azimuth_deg=-120, camera_elevation_deg=21)
        self.robot_view = self.robot_visualizer.widget
        self.verticalLayout_RobotViewport.addWidget(self.robot_view)

    def bind_signals(self):
        for name, callback in (
            ('IP', self.on_set_ip), ('ConnectUR', self.on_connect_robot),
            ('URPowerOn', self.on_power_on), ('URBrakeRelease', self.on_brake_release),
            ('Shutdown', self.on_power_off), ('Stop', self.on_stop_control),
            ('URScriptMoveJ', self.on_move_joint), ('RTDEUDP', self.on_start_udp),
            ('StopRTDE', self.on_stop_control), ('HOME', self.on_home),
            ('UDPSync', self.on_sync_udp), ('Collect', self.on_collect),
            ('Admittance', self.on_admittance),
        ):
            getattr(self, f'pushButton_{name}').clicked.connect(callback)
        self.horizontalSlider_SpeedSlider.valueChanged.connect(self.on_speed_changed)
        self.checkBox_URSim.stateChanged.connect(self.on_simulator_changed)
        self.checkBox_FTSwitch.stateChanged.connect(self.update_torque_subscription)
        self.bind_jog_buttons()
        self.bind_joint_validation()

    def update_connection_label(self):
        state = f'已连接 {self.connected_ip}' if self.connected_ip else '未连接'
        self.label_IP_now.setText(self.connected_ip or '未连接')
        self.label_IP_now.setToolTip(f'目标 {self.robot_ip} | {state}')
        self.lineEdit_IP.setToolTip(f'待连接目标：{self.robot_ip}，修改后点击“修改IP”应用')

    def on_set_ip(self):
        robot_ip = self.lineEdit_IP.text().strip()
        if not NetWorkSet.is_valid_ipv4(robot_ip):
            self.append_message('请输入合法的 IP 地址')
            return
        if robot_ip != self.robot_ip:
            try:
                self.disconnect_robot()
            except (RuntimeError, OSError, RTDEException, URTcpError) as exc:
                self.append_message(f'未切换 IP，需先确认机器人停止：{exc}')
                self.lineEdit_IP.setText(self.robot_ip)
                self.checkBox_URSim.blockSignals(True)
                self.checkBox_URSim.setChecked(self.robot_ip == UR_SIM_IP)
                self.checkBox_URSim.blockSignals(False)
                return
            self.robot_ip = robot_ip
        self.checkBox_URSim.blockSignals(True)
        self.checkBox_URSim.setChecked(self.robot_ip == UR_SIM_IP)
        self.checkBox_URSim.blockSignals(False)
        self.update_connection_label()

    def on_simulator_changed(self):
        self.lineEdit_IP.setText(UR_SIM_IP if self.checkBox_URSim.isChecked() else UR_REAL_IP)
        self.on_set_ip()

    def on_connect_robot(self):
        try:
            self.connect_robot()
        except (URTcpError, OSError, RuntimeError, RTDEException) as exc:
            self.append_message(f'机器人连接失败：{exc}')
            if not self._control_stop_failed:
                try:
                    self.disconnect_robot()
                except (RuntimeError, OSError, RTDEException, URTcpError) as stop_exc:
                    self.append_message(f'连接清理中停止未确认：{stop_exc}')

    def connect_robot(self):
        self.disconnect_robot()
        self.dashboard_client = URDashboardClient(self.robot_ip)
        self.append_message(self.dashboard_client.connect())
        self.realtime_client = URRealtimeClient(self.robot_ip)
        self.realtime_client.connect()
        self.connected_ip = self.robot_ip
        # 模拟器没有夹爪串口转发服务。
        if self.robot_ip != UR_SIM_IP:
            try:
                self.gripper = GripperController(
                    port=f'{self.robot_ip}:{UR_485_PORT}', slave_id=1,
                    connection_type='tcp', debug=False)
                self.gripper.start(interval=0.05)
            except OSError as exc:
                self.append_message(f'夹爪连接失败：{exc}')
        self.update_connection_label()
        self.update_robot_status()

    def disconnect_robot(self):
        self.stop_control()
        # 确认运动退出后才能释放连接；停止失败时保留当前控制后端。
        for name in ('rtde_controller', 'torque_client', 'gripper',
                     'realtime_client', 'dashboard_client'):
            client = getattr(self, name)
            if client is not None:
                client.close()
                setattr(self, name, None)
        self.connected_ip = None
        self.robot_state = None
        self.robot_ready = False
        self.update_connection_label()
        self.update_buttons()

    def on_stop_control(self):
        try:
            self.stop_control()
            return True
        except (RuntimeError, OSError, RTDEException, URTcpError) as exc:
            self.append_message(f'停止尚未确认，控制入口保持锁定：{exc}')
            return False

    def stop_control(self, *, stop_script=False):
        """先关闭指令来源，确认执行后端停止后才解除运动互斥。"""
        self.jog_timer.stop()
        self.udp_control_timer.stop()
        self.control_source = 'stopping'
        self._jog_velocity = None
        self._jog_mode = None
        self._udp_synced = False
        self._udp_sync_pending = False
        self._move_target = None
        if self.gripper is not None:
            self.gripper.pause()
        try:
            if self.rtde_controller is not None:
                self.rtde_controller.stop(stop_script=stop_script)
        except (RuntimeError, OSError, RTDEException, URTcpError):
            self._control_stop_failed = True
            self._admittance_message = '停止尚未确认，请检查机器人状态'
            self.update_buttons()
            raise
        self._control_stop_failed = False
        if self.admittance_mode != 'off':
            status = self.rtde_controller.get_status()['admittance']
            self.finish_admittance(status['exit_reason_text'] or '导纳已停止')
        self.control_source = 'idle'
        self.set_joint_targets_readonly(False)
        self.update_buttons()

    def on_power_on(self):
        self.append_message(self.dashboard_client.power_on())

    def on_brake_release(self):
        self.append_message(self.dashboard_client.brake_release())

    def on_power_off(self):
        try:
            self.stop_control(stop_script=True)
        except (RuntimeError, OSError, RTDEException, URTcpError) as exc:
            self.append_message(f'未下电，需先确认机器人停止：{exc}')
            return
        for name in ('torque_client', 'rtde_controller'):
            client = getattr(self, name)
            if client is not None:
                client.close()
                setattr(self, name, None)
        self.robot_ready = False
        self.append_message(self.dashboard_client.power_off())
        self.update_buttons()

    def get_robot_snapshot(self):
        if self.realtime_client is None:
            return None
        updated_at = self.realtime_client.get_last_update_time()
        if updated_at is None or time.time() - updated_at > ROBOT_STATE_TIMEOUT_S:
            return None
        if self.realtime_client.get_last_error() is not None:
            return None
        return self.realtime_client.get_latest_state()

    def get_udp_snapshot(self):
        return self.get_udp_snapshot_and_count()[0]

    def get_udp_snapshot_and_count(self):
        """一次读取指令与来源帧身份，重复提交缓存不产生新帧。"""
        command, source_seq = self.udp_client.get_latest_and_count()
        if self.udp_client.get_last_error() is not None:
            return None, source_seq
        if command is None or time.time() - command.recv_time > UDP_COMMAND_TIMEOUT_S:
            return None, source_seq
        return command, source_seq

    def update_robot_status(self):
        state = self.get_robot_snapshot()
        was_ready = self.robot_ready
        self.robot_ready = (state is not None and state.robot_mode == ROBOT_MODE_RUNNING
                            and state.safety_mode in (1, 2))
        if was_ready and not self.robot_ready:
            self.on_stop_control()
            if self.rtde_controller is not None and not self._control_stop_failed:
                self.rtde_controller.close()
                self.rtde_controller = None
        # 仅在进入可运行状态时创建，失败后通过显式重连重试。
        if self.robot_ready and not was_ready and self.rtde_controller is None:
            try:
                self.rtde_controller = URRTDEController(
                    self.connected_ip, frequency=UR_RTDE_FREQ_HZ,
                    use_safety_check=False, auto_start_on_command=False)
                self.on_speed_changed()
            except (RuntimeError, OSError) as exc:
                self.append_message(f'RTDE 初始化失败：{exc}')
        self.update_motion_status(state)
        self.update_admittance_status()
        self.update_torque_subscription()
        self.update_buttons()
        mode = f'机器人模式 {state.robot_mode}' if state else '无实时状态'
        ready = 'RTDE 就绪' if self.rtde_controller is not None else 'RTDE 未就绪'
        self.statusbar.showMessage(
            f'UDP {LOCAL_IP}:{self.udp_client.bind_port} | {self.label_IP_now.toolTip()} | '
            f'{mode} | {ready} | 控制来源 {self.control_source}')

    def update_buttons(self):
        connected = self.connected_ip is not None
        ready = connected and self.robot_ready
        rtde_ready = ready and self.rtde_controller is not None
        idle = self.control_source == 'idle' and self.admittance_mode == 'off'
        for name in ('Shutdown', 'URPowerOn', 'URBrakeRelease', 'Stop', 'StopRTDE'):
            getattr(self, f'pushButton_{name}').setEnabled(connected)
        for name in ('HOME', 'URScriptMoveJ'):
            getattr(self, f'pushButton_{name}').setEnabled(rtde_ready and idle)
        command = self.get_udp_snapshot()
        joint_command = command is not None and command.mode == UDPControlMode.JOINT_TRACK
        self.pushButton_UDPSync.setEnabled(rtde_ready and idle and joint_command)
        self.pushButton_RTDEUDP.setEnabled(rtde_ready and idle and joint_command)
        active = self.admittance_mode != 'off'
        self.pushButton_Admittance.setEnabled(active or (rtde_ready and idle))
        self.pushButton_Admittance.setText('停止导纳' if active else '开启导纳')
        self.radioButton_AdmittanceJoint.setEnabled(idle)
        self.radioButton_AdmittanceCartesian.setEnabled(idle)
        self.label_AdmittanceStatus.setText(self._admittance_message)
        self.horizontalSlider_SpeedSlider.setEnabled(rtde_ready)
        for button in self.jog_buttons:
            button.setEnabled(rtde_ready and (idle or self.control_source == 'jog'))

    def update_motion_status(self, state):
        """点位运动完成后解锁入口，同步结果仅供下一次遥操作导纳使用。"""
        if self.control_source != 'moveJ' or state is None:
            return
        reached = np.max(np.abs(np.asarray(state.q_actual) - self._move_target)) <= 0.01
        stationary = np.max(np.abs(state.qd_actual)) <= admittance_config.STATIONARY_MAX_QD_RAD_S
        if reached and stationary:
            was_sync = self._udp_sync_pending
            if not self.on_stop_control():
                return
            self._udp_synced = was_sync
            if was_sync:
                self.append_message('主从同步完成；保持主端静止，可开启遥操作导纳')

    def on_admittance(self):
        if self.admittance_mode != 'off':
            self.on_stop_control()
            return
        if self.control_source != 'idle' or not self.robot_ready or self.rtde_controller is None:
            self.append_message('请先停止当前运动，并确认机器人就绪')
            return
        command, source_seq = self.get_udp_snapshot_and_count()
        if self.udp_client.get_last_error() is not None:
            self._admittance_message = 'UDP 接收异常，请先排查通信'
            self.append_message(self._admittance_message)
            self.update_buttons()
            return
        if command is not None and command.mode != UDPControlMode.JOINT_TRACK:
            self._admittance_message = '主端未启用关节跟随'
            self.append_message(self._admittance_message)
            self.update_buttons()
            return
        reference_source = 'teleop' if command is not None else 'free'
        state = self.get_robot_snapshot()
        if state is None or np.max(np.abs(state.qd_actual)) > admittance_config.STATIONARY_MAX_QD_RAD_S:
            self.append_message('导纳偏置采样前请保持机器人静止、无外部接触')
            return
        if reference_source == 'teleop':
            aligned = np.max(np.abs(np.asarray(command.q_arm) - state.q_actual)) <= admittance_config.TELEOP_SYNC_TOLERANCE_RAD
            if not self._udp_synced or not aligned:
                self.append_message('请保持主端静止，先点击“UDP 同步”并等待完成，再开启遥操作导纳')
                return
        space = 'joint' if self.radioButton_AdmittanceJoint.isChecked() else 'cartesian'
        if self.gripper is not None:
            self.gripper.pause()
        try:
            accepted = self.rtde_controller.start_admittance(
                reference_source, space,
                q_master=None if command is None else command.q_arm,
                source_seq=None if command is None else source_seq)
        except (RuntimeError, OSError, RTDEException, URTcpError, ValueError) as exc:
            self.append_message(f'导纳启动失败：{exc}')
            return
        if not accepted:
            self.append_message(self.rtde_controller.get_status()['last_error'])
            return
        self.admittance_mode = space
        self._admittance_reference_source = reference_source
        self.control_source = 'udp' if reference_source == 'teleop' else 'admittance_free'
        self._udp_synced = False
        self._last_collected_admittance_time = None
        self._admittance_message = '导纳启动中：静止采样偏置，请勿接触机器人'
        self._admittance_run = {
            'id': len(self._admittance_runs) + 1,
            'started_at': time.time(), 'reference_source': reference_source, 'space': space,
            'parameters': admittance_config.snapshot(),
            'units_and_coordinates': admittance_config.units(reference_source, space),
            'master_applicable': reference_source == 'teleop',
        }
        self._admittance_runs.append(self._admittance_run)
        self.collector.update_metadata({'admittance_runs': self._admittance_runs})
        self.set_joint_targets_readonly(True)
        if reference_source == 'teleop':
            self.udp_control_timer.start(round(1000 / admittance_config.TELEOP_REFERENCE_HZ))
        self.append_message(self._admittance_message)
        self.update_buttons()

    def update_admittance_status(self):
        if self.admittance_mode == 'off' or self.rtde_controller is None:
            return
        status = self.rtde_controller.get_status()['admittance']
        if self._admittance_run is not None and status.get('robot_configuration'):
            if 'robot_configuration' not in self._admittance_run:
                self._admittance_run['robot_configuration'] = status['robot_configuration']
                self._admittance_run['parameters'] = status['parameters']
                self.collector.update_metadata({'admittance_runs': self._admittance_runs})
        if status['stopped_confirmed'] and not status['active']:
            self.finish_admittance(status['last_error'] or status['exit_reason_text'] or '导纳已停止')
            return
        if status['last_error']:
            self._admittance_message = f"导纳异常：{status['last_error']}"
        elif status['running']:
            space = '关节' if self.admittance_mode == 'joint' else '末端'
            source = '遥操作导纳' if self._admittance_reference_source == 'teleop' else '主动柔顺'
            self._admittance_message = f'{space}{source}'
        else:
            self._admittance_message = f"导纳：{status['phase_name']}"

    def finish_admittance(self, reason):
        """机器人确认退出后收尾，保留最后遥测用于本轮记录。"""
        self.udp_control_timer.stop()
        if self.gripper is not None:
            self.gripper.pause()
        if self._admittance_run is not None:
            status = self.rtde_controller.get_status()['admittance']
            self.collect_admittance_sample(time.time(), status)
            self._admittance_run.update(ended_at=time.time(), exit_reason=reason,
                                        exit_code=status['exit_reason'])
            self.collector.update_metadata({'admittance_runs': self._admittance_runs})
            self._admittance_run = None
        self._admittance_message = reason
        self.admittance_mode = 'off'
        self._admittance_reference_source = None
        self.control_source = 'idle'
        self._control_stop_failed = False
        self.set_joint_targets_readonly(False)
        self.append_message(reason)

    def on_jog_pressed(self, mode, index, direction):
        if self.admittance_mode != 'off' or self._control_stop_failed:
            return
        if not self.on_stop_control():
            return
        if not self.rtde_controller.start():
            self.append_message(self.rtde_controller.get_status()['last_error'])
            return
        velocity = [0.0] * 6
        if mode == 'joint':
            speed = JOINT_JOG_SPEED_RAD_S
        else:
            speed = TCP_JOG_LINEAR_SPEED_M_S if index <= 3 else TCP_JOG_ANGULAR_SPEED_RAD_S
        velocity[index - 1] = direction * speed
        self._jog_velocity = velocity
        self._jog_mode = mode
        self.control_source = 'jog'
        self.set_joint_targets_readonly(True)
        self.jog_timer.start(2)

    def on_jog_released(self):
        if self.control_source == 'jog':
            self.on_stop_control()

    def update_jog(self):
        if self.control_source != 'jog':
            return
        if self._jog_mode == 'joint':
            self.rtde_controller.speedJ(
                qd=self._jog_velocity, time_s=self.rtde_controller.dt, acceleration=0.1)
        else:
            frame = 'tool' if self._jog_mode == 'tcp_tool' else 'base_add'
            self.rtde_controller.speedL(
                xd=self._jog_velocity, time_s=self.rtde_controller.dt, frame=frame)

    def on_move_joint(self):
        q_rad = [np.deg2rad(self.get_valid_qtarget_degree(i, commit=True)) for i in range(1, 7)]
        self.move_joint(q_rad)

    def on_home(self):
        self.move_joint(UR_HOME_RAD)

    def move_joint(self, q_rad):
        if self.admittance_mode != 'off' or self._control_stop_failed:
            return False
        if not self.on_stop_control():
            return False
        try:
            self.rtde_controller.moveJ(q_rad, speed=0.1, asynchronous=True)
        except (RuntimeError, OSError, RTDEException, ValueError) as exc:
            self.append_message(f'关节目标运动未启动：{exc}')
            return False
        self._move_target = np.asarray(q_rad, dtype=float)
        self.control_source = 'moveJ'
        self.update_buttons()
        return True

    def on_start_udp(self):
        if self.admittance_mode != 'off' or self._control_stop_failed:
            return
        if not self.on_stop_control():
            return
        command = self.get_udp_snapshot()
        if command is None or command.mode != UDPControlMode.JOINT_TRACK:
            self.append_message('没有有效的 UDP 关节跟踪指令')
            return
        if not self.rtde_controller.start():
            self.append_message(self.rtde_controller.get_status()['last_error'])
            return
        self.control_source = 'udp'
        self.set_joint_targets_readonly(True)
        self.udp_control_timer.start(10)
        self.update_buttons()

    def update_udp_control(self):
        if self.control_source != 'udp':
            return
        command, source_seq = self.get_udp_snapshot_and_count()
        if command is None or command.mode != UDPControlMode.JOINT_TRACK:
            self.on_stop_control()
            self.append_message('UDP 跟随已停止：关闭、断流或不支持的控制模式')
            return
        try:
            self.rtde_controller.track_joint(command.q_arm, dq_max=1, source_seq=source_seq)
        except (RuntimeError, OSError, RTDEException, ValueError) as exc:
            self.on_stop_control()
            self.append_message(f'UDP 目标提交失败：{exc}')
            return
        admittance_running = (self.admittance_mode == 'off' or
                              self.rtde_controller.get_status()['admittance']['running'])
        if self.gripper is not None and admittance_running:
            self.gripper.set_target_position(command.q_gripper[0])
            if self.gripper.is_paused:
                self.gripper.resume()

    def on_sync_udp(self):
        if self.admittance_mode != 'off' or self._control_stop_failed:
            return
        command = self.get_udp_snapshot()
        if command is None or command.mode != UDPControlMode.JOINT_TRACK:
            return
        if not self.move_joint(command.q_arm):
            return
        self._udp_sync_pending = True
        if self.gripper is not None:
            self.gripper.move(command.q_gripper[0], speed=GRIPPER_SPEED_DEFAULT,
                              force=GRIPPER_FORCE_DEFAULT)
            self.gripper.resume()

    def on_speed_changed(self):
        speed_percent = self.horizontalSlider_SpeedSlider.value()
        self.label_SpeedSlider.setText(f'限速: {speed_percent}%')
        if self.rtde_controller is not None:
            self.rtde_controller.set_speed_slider(speed_percent / 100)

    def update_torque_subscription(self):
        enabled = self.checkBox_FTSwitch.isChecked() and self.robot_ready
        if enabled and self.torque_client is None:
            try:
                self.torque_client = URRTDETorqueClient(self.connected_ip, UR_RTDE_FREQ_HZ)
                self.torque_client.start_reader_thread()
            except (OSError, RTDEException, RuntimeError, ValueError) as exc:
                self.checkBox_FTSwitch.setChecked(False)
                self.append_message(f'力矩订阅失败：{exc}')
        elif not enabled and self.torque_client is not None:
            self.torque_client.close()
            self.torque_client = None

    def update_robot_view(self):
        command = self.get_udp_snapshot()
        if command is None:
            for index in range(1, 9):
                getattr(self, f'lineEdit_UDP{index}').clear()
        else:
            self.lineEdit_UDP1.setText(UDPControlMode.cn_name(command.mode))
            for index, angle_rad in enumerate(command.q_arm, 2):
                getattr(self, f'lineEdit_UDP{index}').setText(f'{np.rad2deg(angle_rad):.4f}')
            self.lineEdit_UDP8.setText(f'{command.q_gripper[0]:.4f}')
            if self.checkBox_UDPVisual.isChecked() and command.mode == UDPControlMode.JOINT_TRACK:
                self.robot_visualizer.update_virtual(np.append(command.q_arm, 0.0))
        if self.realtime_client is not None:
            self.robot_state = self.get_robot_snapshot()
            if self.robot_state is not None:
                for i in range(6):
                    # 实时关节角
                    line_edit = getattr(self, f"lineEdit_QA{i + 1}")
                    line_edit.setText(f"{self.robot_state.q_actual[i] * 180 / np.pi:.3f}")
                    # 末端位姿
                    line_edit = getattr(self, f"lineEdit_TA{i + 1}")
                    if i < 3:
                        line_edit.setText(f"{self.robot_state.tcp_pose[i] * 1000:.3f}")
                    else:
                        line_edit.setText(f"{self.robot_state.tcp_pose[i] * 180 / np.pi:.3f}")
                    # 末端广义力
                    if self.checkBox_FTSwitch.checkState() != Qt.Checked:
                        line_edit = getattr(self, f"lineEdit_Fex{i + 1}")
                        line_edit.setText(f"{self.robot_state.tcp_force[i] :.3f}")

                    # 目标关节角
                    line_edit = getattr(self, f"lineEdit_QT{i + 1}")
                    if line_edit.isReadOnly():
                        line_edit.setText(f"{self.robot_state.fields['q_target'][i] * 180 / np.pi:.3f}")

                # 更新可视化：真实机械臂=当前关节角，虚拟机械臂=目标关节角
                q_actual_7 = np.append(self.robot_state.q_actual, 0.0)
                self.robot_visualizer.update_actual(q_actual_7)
                if self.checkBox_UDPVisual.checkState() != Qt.Checked:
                    q_target_deg = [self.get_valid_qtarget_degree(i + 1) for i in range(6)]
                    q_target_rad = np.deg2rad(q_target_deg)
                    q_target_7 = np.append(q_target_rad, 0.0)
                    self.robot_visualizer.update_virtual(q_target_7)

        # 夹钳实时反馈
        if self.gripper is not None:
            fb = self.gripper.feedback
            self.lineEdit_Clamp1.setText(f"{fb.position}")
            self.lineEdit_Clamp2.setText(f"{fb.current}")
            # 打包发送当前数据到主端
        if (self.gripper is not None) and (self.robot_state is not None):
            fb = self.gripper.feedback
            self.udp_client.send_to((UDP_REMOTE_IP, UDP_REMOTE_PORT), self.robot_state.q_actual, UDPControlMode.OFF,
                                       [fb.open, fb.current, fb.position])

        if self.torque_client is not None:
            sample = self.torque_client.get_latest()
            if sample is not None:
                for index, torque_nm in enumerate(sample[1], 1):
                    getattr(self, f'lineEdit_Fex{index}').setText(f'{torque_nm:.3f}')
            error = self.torque_client.get_last_error()
            if error is not None:
                self.checkBox_FTSwitch.setChecked(False)
                self.append_message(f'力矩读取失败：{error}')

    def get_camera_frames(self):
        frames = []
        for index in (1, 2):
            camera = getattr(self, f'camera_{index}')
            frame = None
            if camera is not None:
                try:
                    frame = camera.get_rgb_frame()
                except CameraError as exc:
                    self.append_message(f'相机 {index} 取帧失败：{exc}')
                    camera.close()
                    setattr(self, f'camera_{index}', None)
            frames.append(frame)
        return frames

    def update_cameras(self):
        for index, frame in enumerate(self.get_camera_frames(), 1):
            label = getattr(self, f'label_camera{index}')
            if frame is None:
                label.setText(f'相机 {index} 未连接')
            else:
                label.setPixmap(self.rgb_to_pixmap(frame.image))

    def on_collect(self):
        if self.collector.episode_active:
            self.collection_timer.stop()
            self.collector.end_episode()
            self.pushButton_Collect.setText('开始采集')
        else:
            self._last_collected_udp_time = None
            self._last_collected_robot_time = None
            self._last_collected_admittance_time = None
            self.collector.start_episode()
            self.collection_timer.start(round(1000 / DATA_COLLECT_FREQ))
            self.pushButton_Collect.setText('采集结束')

    def collect_sample(self):
        timestamp = time.time()
        command = self.get_udp_snapshot()
        state = self.get_robot_snapshot()
        feedback = self.gripper.feedback if self.gripper is not None else None
        frames = self.get_camera_frames()
        # 每个采样周期使用同一数值时间戳，未更新的数据源不重复入库。
        if (self._admittance_reference_source != 'free' and command is not None
                and command.recv_time != self._last_collected_udp_time):
            self.collector.push_numeric('MASTER_GRIPPER', [command.q_gripper[0]], timestamp)
            self.collector.push_numeric('MASTER_JOINT', command.q_arm, timestamp)
            self._last_collected_udp_time = command.recv_time
        if state is not None and state.time != self._last_collected_robot_time:
            self.collector.push_numeric('UR_TCP_POSE', state.tcp_pose, timestamp)
            self.collector.push_numeric('UR_JOINT', state.q_actual, timestamp)
            self._last_collected_robot_time = state.time
        if feedback is not None and not self.gripper.is_paused:
            self.collector.push_numeric('UR_GRIPPER', [feedback.open, feedback.current], timestamp)
        if self.admittance_mode != 'off':
            self.collect_admittance_sample(timestamp, self.rtde_controller.get_status()['admittance'])
        for index, frame in enumerate(frames, 1):
            if frame is not None:
                # 保留相机后台接收该帧的时间，避免用本次取帧时间覆盖来源时间。
                self.collector.push_image(f'CAMERA_{index}', frame.image, timestamp=frame.received_time)
        self.collector.step()

    def collect_admittance_sample(self, timestamp, status):
        """按机器人时间去重保存同一遥测包；不适用的主端字段留空。"""
        if not self.collector.episode_active or status['timestamp'] is None:
            return
        sample_key = (status['timestamp'], status['phase'])
        if sample_key == self._last_collected_admittance_time:
            return
        source = status['reference_source']
        space = status['space']
        teleop = source == 'teleop'
        self.collector.push_numeric('ADMITTANCE_STATE', [
            self._admittance_run['id'], 1 if teleop else 2, 1 if space == 'joint' else 2,
            status['phase'], status['exit_reason'],
            status['source_seq'] if teleop and status['source_seq'] is not None else float('nan'),
            status['adopted_source_seq'] if teleop and status['adopted_source_seq'] is not None else float('nan'),
            status['cycle_count'], status['timestamp'],
        ], timestamp)
        groups = {
            'ADMITTANCE_Q_CMD': status['q_cmd'],
            'ADMITTANCE_Q_ACTUAL': status['actual_q'],
            'ADMITTANCE_JOINT_DRIVE' if space == 'joint' else 'ADMITTANCE_CART_DRIVE': status['drive'],
        }
        if teleop:
            groups.update(ADMITTANCE_Q_MASTER=status['q_master'], ADMITTANCE_Q_NOM=status['q_nom'])
            groups['ADMITTANCE_DELTA_Q' if space == 'joint' else 'ADMITTANCE_DELTA_X'] = (
                status['delta_q'] if space == 'joint' else status['delta_x'])
        else:
            groups['ADMITTANCE_Q_START'] = status['q_start']
            groups['ADMITTANCE_FREE_QD' if space == 'joint' else 'ADMITTANCE_FREE_XD'] = status['v_free']
        for name, values in groups.items():
            if values is not None:
                self.collector.push_numeric(name, values, timestamp)
        self._last_collected_admittance_time = sample_key

    @staticmethod
    def rgb_to_pixmap(image: np.ndarray) -> QPixmap:
        h, w, ch = image.shape
        bytes_per_line = ch * w

        qimg = QImage(
            image.data,
            w,
            h,
            bytes_per_line,
            QImage.Format_RGB888
        )

        pixmap = QPixmap.fromImage(qimg)
        return pixmap

    def append_message(self, message):
        if message is not None:
            message = time.strftime("%H:%M:%S", time.localtime()) + ': ' + message
            self.plainTextEdit_DashboardMessage.appendPlainText(message)

    def bind_jog_buttons(self):
        self.jog_buttons = []
        for prefix, mode in (("J", "joint"), ("T", "tcp_tool"), ("TW", "tcp_base")):
            for direction_name, direction in (("Up", 1), ("Down", -1)):
                for index in range(1, 7):
                    button = getattr(self, f"pushButton_{prefix}{direction_name}{index}")
                    self.jog_buttons.append(button)
                    button.pressed.connect(
                        lambda m=mode, i=index, d=direction: self.on_jog_pressed(m, i, d))
                    button.released.connect(self.on_jog_released)

    def bind_joint_validation(self):
        for i in range(1, 7):
            getattr(self, f"lineEdit_QT{i}").editingFinished.connect(
                lambda idx=i: self.validate_qtarget_input(idx))

    def validate_qtarget_input(self, index: int) -> None:
        line_edit = getattr(self, f"lineEdit_QT{index}")
        if line_edit.isReadOnly():
            return
        self.get_valid_qtarget_degree(index, commit=True)

    def get_actual_q_degree(self, index: int) -> float:
        actual_edit = getattr(self, f"lineEdit_QA{index}")
        actual_text = actual_edit.text().strip()
        try:
            return float(actual_text)
        except ValueError:
            return 0.0

    def get_valid_qtarget_degree(self, index: int, commit: bool = False) -> float:
        target_edit = getattr(self, f"lineEdit_QT{index}")
        actual_value = self.get_actual_q_degree(index)
        actual_text = f"{actual_value:.3f}"

        target_text = target_edit.text().strip()
        try:
            target_value = float(target_text)
        except ValueError:
            if commit:
                target_edit.setText(actual_text)
            return actual_value

        if not (-360.0 <= target_value <= 360.0):
            if commit:
                target_edit.setText(actual_text)
            return actual_value

        if commit:
            target_edit.setText(f"{target_value:.3f}")

        return target_value

    def set_joint_targets_readonly(self, flag: bool) -> None:
        for i in range(6):
            line_edit = getattr(self, f"lineEdit_QT{i + 1}")
            line_edit.setReadOnly(flag)

    def closeEvent(self, event):
        for timer in (self.status_timer, self.robot_view_timer, self.jog_timer,
                      self.udp_control_timer, self.camera_timer, self.collection_timer):
            timer.stop()
        try:
            self.disconnect_robot()
        except (RuntimeError, OSError, RTDEException, URTcpError) as exc:
            self.append_message(f'关闭取消，需先确认机器人停止：{exc}')
            self.status_timer.start(100)
            self.robot_view_timer.start(10)
            self.camera_timer.start(50)
            if self.collector.episode_active:
                self.collection_timer.start(round(1000 / DATA_COLLECT_FREQ))
            event.ignore()
            return
        self.udp_client.stop()
        for camera in (self.camera_1, self.camera_2):
            if camera is not None:
                camera.close()
        self.collector.close()
        self.robot_visualizer.close()
        event.accept()


if __name__ == '__main__':
    configure_high_dpi()
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
