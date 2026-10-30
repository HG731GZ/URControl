import os
import sys
import time
import threading
from copy import deepcopy

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
from ui_classic_window import Ui_MainWindow
from admittance_window import AdmittanceWindow, OPERATOR_EVENTS
from admittance_lifecycle import AdmittanceLifecycleWorker, AdmittanceLogFinishWorker
from admittance_logger import AdmittanceLogger
from UR_Utils.URDashboardClient import URDashboardClient
from UR_Utils.URRealtimeClient import URRealtimeClient
from UR_Utils.URRTDEController import URRTDEController
from UR_Utils import admittance_config
from UR_Utils.URAdmittanceController import PHASE_NAMES, EXIT_REASONS, idle_admittance_status
from UR_Utils.URRTDETorqueClient import URRTDETorqueClient
from UR_Utils.URCompensatedTorqueReader import URCompensatedTorqueReader
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
# 主窗口勾选“力矩”后的显示来源；修改后重启程序生效。
# True：raw（actual_current_as_torque）；False：UR 内部补偿力矩（get_joint_torques）。
# 补偿值尚未减去导纳启动偏置，也未经过滤波或死区；本开关只影响显示。
SHOW_RAW_JOINT_TORQUES = False
UR_485_PORT = 54321
DATA_COLLECT_FREQ = 20.0
DATA_DIRECTORY = 'data'
SESSION_NAME_FORMAT = 'URCollect_%Y%m%d_%H%M%S'
ADMITTANCE_SOURCE_CODES = {1: 'teleop', 2: 'free', 3: 'fixed'}
ADMITTANCE_RECORD_VERSION = 3
ADMITTANCE_HOST_TIMING_FIELDS = (
    'state_receive_count', 'state_received_monotonic', 'reference_send_count',
    'last_reference_sent_monotonic', 'heartbeat_send_count', 'last_heartbeat_sent_monotonic',
    'heartbeat_send_interval_s', 'heartbeat_send_max_interval_s',
    'heartbeat_send_lateness_s', 'heartbeat_send_max_lateness_s', 'heartbeat_deadline_miss_count',
    'processing_duration_s', 'processing_max_duration_s',
    'rtde_processing_duration_s', 'primary_processing_duration_s',
    'wake_lateness_s', 'wake_max_lateness_s',
    'rtde_buffered_bytes', 'rtde_buffered_max_bytes',
    'state_timestamp_interval_s', 'state_host_interval_s',
    'state_progress_lag_s', 'state_progress_max_lag_s', 'state_last_progress_monotonic_s',
)
ADMITTANCE_ROBOT_HEARTBEAT_FIELDS = ('heartbeat_echo', 'heartbeat_age_us', 'heartbeat_max_interval_us')


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self._display_settings = QtCore.QSettings('URControl', 'Display')
        self._ui = Ui_MainWindow()
        self._ui.setupUi(self)
        vars(self).update(vars(self._ui))
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
        self.admittance_window = None
        self._admittance_worker = None
        self._admittance_worker_result = None
        self._admittance_stop_event = None
        self._admittance_stop_pending = False
        self._admittance_after_stop = None
        self._admittance_handoff_complete = False
        self._admittance_parameters = None
        self._last_admittance_status = None
        self._last_admittance_run_id = None
        # 日常持续运行默认不写高频调试日志，现场分析时在导纳窗口显式开启。
        self._admittance_logging_enabled = False
        self._admittance_logger = None
        self._admittance_log_finish_workers = []
        self._admittance_reported_log_error = None
        self._last_collected_admittance_time = None
        self._last_collected_submitted_seq = None
        self._last_collected_master_count = None
        self._control_stop_failed = False
        self._udp_synced = False
        self._udp_sync_pending = False
        self._udp_sync_parameters = None
        self._move_target = None
        self._jog_velocity = None
        self._jog_mode = None
        self._last_collected_udp_time = None
        self._last_collected_robot_time = None
        self._ui_timing = {}
        self.initialize_devices()
        self.initialize_collector()
        self.create_timers()
        self.initialize_visualizer()
        self.display_controller = WindowDisplayController(self, self._ui, self._display_settings)
        self.bind_signals()
        self.lineEdit_IP.setText(self.robot_ip)
        self.update_connection_label()
        self.update_buttons()
        self.status_timer.start(100)
        self.robot_view_timer.start(10)
        self.camera_timer.start(50)

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
                'admittance_record_version': ADMITTANCE_RECORD_VERSION,
                'admittance_source_codes': ADMITTANCE_SOURCE_CODES,
                'master_not_applicable': '定点柔顺期间不写入主端目标组，code 2 仅供识别历史 free 记录',
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
             ['编号', '1=teleop;2=free(历史);3=fixed', '1=joint;2=cartesian', '阶段编号', '退出原因编号',
              '帧序号;NaN=不适用', '帧序号;NaN=不适用', '周期', 's']),
            ('ADMITTANCE_PAGE_IDENTITY', ['run', 'robot_run', 'page_type', 'page_seq',
                                          'model_sample_seq', 'snapshot_time_s', 'received_monotonic_s'],
             ['编号', '机器人本轮身份', '1=anchor;2=load;3=execution;4=timing', '冻结页序号',
              '模型采样编号', 's；脚本冻结时刻', 's；主机接收时刻']),
            ('ADMITTANCE_Q_CMD', [f'q{i}' for i in range(1, 7)], ['rad'] * 6),
            ('ADMITTANCE_Q_ACTUAL', [f'q{i}' for i in range(1, 7)], ['rad'] * 6),
            ('ADMITTANCE_QD_ACTUAL', [f'qd{i}' for i in range(1, 7)], ['rad/s'] * 6),
            ('ADMITTANCE_Q_MASTER', [f'q{i}' for i in range(1, 7)], ['rad'] * 6),
            ('ADMITTANCE_Q_SUBMITTED', [f'q{i}' for i in range(1, 7)], ['rad'] * 6),
            ('ADMITTANCE_SUBMISSION', ['run', 'source_seq', 'observed_monotonic'],
             ['编号', '本机有效接收计数', 's；主机观察时刻']),
            ('ADMITTANCE_MASTER_RECEIVED', ['run', 'frame_count', 'received_monotonic',
                                          'mode'] + [f'q{i}' for i in range(1, 7)] +
                                         ['gripper', 'gripper_raw2', 'gripper_raw3'],
             ['编号', '本机有效接收计数', 's', '模式编号'] + ['rad；非关节跟踪时NaN'] * 6 +
             ['协议开度或原始位置', '附加原始量', '附加原始量']),
            ('ADMITTANCE_HOST_TIMING', list(ADMITTANCE_HOST_TIMING_FIELDS),
             ['计数', 's', '计数', 's；未发送时NaN', '计数', 's；未发送时NaN',
              's', 's', 's', 's', '累计错过心跳周期数', 's', 's', 's', 's',
              's', 's', '字节', '字节', 's', 's', 's；相对推进差，非网络年龄',
              's；相对推进差，非网络年龄', 's；主机单调时刻']),
            ('ADMITTANCE_ROBOT_HEARTBEAT', list(ADMITTANCE_ROBOT_HEARTBEAT_FIELDS),
             ['心跳回显计数', 'us', 'us']),
            ('ADMITTANCE_Q_NOM', [f'q{i}' for i in range(1, 7)], ['rad'] * 6),
            ('ADMITTANCE_Q_START', [f'q{i}' for i in range(1, 7)], ['rad'] * 6),
            ('ADMITTANCE_JOINT_DRIVE', [f'tau{i}' for i in range(1, 7)], ['Nm'] * 6),
            ('ADMITTANCE_CART_DRIVE', ['fx', 'fy', 'fz', 'tx', 'ty', 'tz'], ['N'] * 3 + ['Nm'] * 3),
            ('ADMITTANCE_DELTA_Q', [f'dq{i}' for i in range(1, 7)], ['rad'] * 6),
            ('ADMITTANCE_DELTA_X', ['dx', 'dy', 'dz', 'drx', 'dry', 'drz'], ['m'] * 3 + ['rad'] * 3),
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
            ('admittance_monitor_timer', self.refresh_admittance_window),
        ):
            timer = QtCore.QTimer(self)
            timer.setTimerType(Qt.PreciseTimer)
            timer.timeout.connect(lambda n=name, cb=callback: self.run_timed_ui_callback(n, cb))
            setattr(self, name, timer)

    def run_timed_ui_callback(self, name, callback):
        """记录实际界面回调的墙钟和线程 CPU；不把事件循环等待算成回调计算。"""
        if self._admittance_run is None:
            callback()
            return
        started, cpu_started = time.monotonic(), time.thread_time()
        try:
            callback()
        finally:
            wall, cpu = time.monotonic() - started, time.thread_time() - cpu_started
            previous = self._ui_timing.get(name)
            self._ui_timing[name] = {
                'count': 1 if previous is None else previous['count'] + 1,
                'wall_s': wall, 'cpu_s': cpu,
                'max_wall_s': wall if previous is None else max(previous['max_wall_s'], wall),
                'max_cpu_s': cpu if previous is None else max(previous['max_cpu_s'], cpu),
                'started_monotonic': started, 'ended_monotonic': started + wall,
            }
            if previous is None or wall > previous['max_wall_s']:
                self._ui_timing[name].update(
                    max_wall_cpu_s=cpu, max_wall_started_monotonic_s=started,
                    max_wall_finished_monotonic_s=started + wall)
            else:
                self._ui_timing[name].update({key: previous[key] for key in (
                    'max_wall_cpu_s', 'max_wall_started_monotonic_s', 'max_wall_finished_monotonic_s')})

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
            if self.defer_until_admittance_stopped(self.on_set_ip):
                return
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
        if self.defer_until_admittance_stopped(self.on_connect_robot):
            return
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
        if not self.disconnect_robot():
            return
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
                self.append_message(f'夹爪未连接，继续使用机械臂：{exc}')
        self.update_connection_label()
        self.update_robot_status()

    def disconnect_robot(self):
        if self.defer_until_admittance_stopped(self.disconnect_robot):
            return False
        self.stop_control()
        # 确认运动退出后才能释放连接；停止失败时保留当前控制后端。
        for name in ('torque_client', 'rtde_controller', 'gripper',
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
        return True

    def on_stop_control(self):
        if self.admittance_mode != 'off':
            self.request_admittance_stop()
            return False
        try:
            self.stop_control()
            return True
        except (RuntimeError, OSError, RTDEException, URTcpError) as exc:
            self.append_message(f'停止尚未确认，控制入口保持锁定：{exc}')
            return False

    def stop_control(self, *, stop_script=False):
        """先关闭指令来源，确认执行后端停止后才解除运动互斥。"""
        if self.admittance_mode != 'off':
            self.request_admittance_stop()
            return False
        self.jog_timer.stop()
        self.udp_control_timer.stop()
        self.control_source = 'stopping'
        self._jog_velocity = None
        self._jog_mode = None
        self._udp_synced = False
        self._udp_sync_pending = False
        self._move_target = None
        self._udp_sync_parameters = None
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
        self.control_source = 'idle'
        self.set_joint_targets_readonly(False)
        self.update_buttons()

    def on_power_on(self):
        self.append_message(self.dashboard_client.power_on())

    def on_brake_release(self):
        self.append_message(self.dashboard_client.brake_release())

    def on_power_off(self):
        if self.defer_until_admittance_stopped(self.on_power_off):
            return
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

    def get_gripper_feedback(self):
        return None if self.gripper is None else self.gripper.get_feedback()

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

    def get_udp_snapshot_and_count(self, timeout_s=UDP_COMMAND_TIMEOUT_S):
        """一次读取指令与来源帧身份，重复提交缓存不产生新帧。"""
        snapshot = self.udp_client.get_monitor_snapshot()
        command, source_seq = snapshot['command'], snapshot['frame_count']
        if snapshot['last_error'] is not None:
            return None, source_seq
        if command is None or time.monotonic() - snapshot['received_monotonic'] > timeout_s:
            return None, source_seq
        return command, source_seq

    def update_robot_status(self):
        state = self.get_robot_snapshot()
        was_ready = self.robot_ready
        self.robot_ready = (state is not None and state.robot_mode == ROBOT_MODE_RUNNING
                            and state.safety_mode in (1, 2))
        if was_ready and not self.robot_ready:
            if self.admittance_mode != 'off':
                self.defer_until_admittance_stopped(self.release_unready_controller, automatic=True)
            else:
                self.on_stop_control()
                self.release_unready_controller()
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
        self.record_admittance_application_state()
        self.report_admittance_log_error()
        self.update_torque_subscription()
        self.update_buttons()
        mode = f'机器人模式 {state.robot_mode}' if state else '无实时状态'
        ready = 'RTDE 就绪' if self.rtde_controller is not None else 'RTDE 未就绪'
        gripper_status = '夹爪在线' if self.get_gripper_feedback() is not None else '夹爪未连接/无反馈'
        feedback_error = self.udp_client.get_monitor_snapshot()['send_last_error']
        feedback_status = ' | 主端反馈发送失败，正在重试' if feedback_error is not None else ''
        self.statusbar.showMessage(
            f'UDP {LOCAL_IP}:{self.udp_client.bind_port} | {self.label_IP_now.toolTip()} | '
            f'{mode} | {ready} | {gripper_status} | 控制来源 {self.control_source}{feedback_status}')

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
        self.pushButton_Admittance.setEnabled(True)
        hidden = (self.admittance_mode != 'off' and
                  (self.admittance_window is None or not self.admittance_window.isVisible()))
        self.label_AdmittanceStatus.setText(
            self._admittance_message + ('；监视窗口已隐藏，控制继续' if hidden else ''))
        self.horizontalSlider_SpeedSlider.setEnabled(rtde_ready and self._admittance_worker is None)
        for button in self.jog_buttons:
            button.setEnabled(rtde_ready and (idle or self.control_source == 'jog'))
        self.update_admittance_window_context()

    def update_motion_status(self, state):
        """点位运动完成后解锁入口，同步结果仅供下一次遥操作导纳使用。"""
        if self.control_source != 'moveJ' or state is None:
            return
        reached = np.max(np.abs(np.asarray(state.q_actual) - self._move_target)) <= 0.01
        stationary_limit = (admittance_config.STATIONARY_MAX_QD_RAD_S if self._udp_sync_parameters is None
                            else self._udp_sync_parameters['STATIONARY_MAX_QD_RAD_S'])
        stationary = np.max(np.abs(state.qd_actual)) <= stationary_limit
        if reached and stationary:
            was_sync = self._udp_sync_pending
            if not self.on_stop_control():
                return
            self._udp_synced = was_sync
            if was_sync:
                self.append_message('主从同步完成；保持主端静止，可开启遥操作导纳')

    def release_unready_controller(self):
        """机器人失去就绪状态后，仅在停止确认后释放原控制器。"""
        if not self.robot_ready and self.rtde_controller is not None and not self._control_stop_failed:
            if self.torque_client is not None:
                self.torque_client.close()
                self.torque_client = None
            self.rtde_controller.close()
            self.rtde_controller = None

    def on_admittance(self):
        """启动导纳按钮打开同一窗口；首次默认六轴固定参考，实际启动由窗口按钮发起。"""
        if self.admittance_window is None:
            window = AdmittanceWindow(settings=self._display_settings, parent=self)
            self.admittance_window = window
            window.start_requested.connect(self.start_admittance_run)
            window.stop_requested.connect(self.request_admittance_stop)
            window.sync_requested.connect(self.sync_admittance_master)
            window.apply_requested.connect(self.on_admittance_parameters_applied)
            window.visibility_changed.connect(self.on_admittance_visibility)
            window.logging_toggled.connect(self.on_admittance_logging_toggled)
            window.operator_event_requested.connect(self.record_admittance_operator_event)
            if self._admittance_parameters is not None:
                revision = self._admittance_runs[-1]['parameter_revision']
                window.set_effective(self._admittance_parameters, revision)
        self.update_admittance_window_context()
        self.admittance_window.show()
        self.admittance_window.raise_()
        self.admittance_window.activateWindow()
        self.refresh_admittance_window()

    def on_admittance_visibility(self, visible):
        if self._admittance_logger is not None:
            self._admittance_logger.record('monitor_visibility', {'visible': visible})
        self.admittance_window.monitor.set_visible(visible)
        if visible:
            self.admittance_monitor_timer.start(50)
        else:
            self.admittance_monitor_timer.stop()
        self.update_buttons()

    def on_admittance_parameters_applied(self):
        self.append_message(f'导纳准备参数已应用，修订号 {self.admittance_window.parameters.revision}')
        self.update_admittance_window_context()

    def admittance_log_status(self):
        """开关属于本次应用会话；上一轮写盘期间仍可准备下一轮的选择。"""
        status = ({'active': False, 'path': None, 'record_count': 0, 'dropped_count': 0,
                   'pending_count': 0, 'error': None,
                   'logger_timing': {'schema_version': 1, 'producer': {}, 'writer': {},
                                     'queue_observed_high_watermark': 0, 'queue_age': None}}
                  if self._admittance_logger is None else self._admittance_logger.get_status())
        status['enabled'] = self._admittance_logging_enabled
        return status

    def on_admittance_logging_toggled(self, enabled):
        """只切换诊断记录；运行中的控制参数和运动状态不随开关变化。"""
        self._admittance_logging_enabled = enabled
        if self._admittance_logger is not None and self._admittance_run is not None:
            status = (self.rtde_controller.get_status()['admittance']
                      if self._admittance_handoff_complete else self._last_admittance_status)
            self._admittance_logger.set_enabled(enabled, context={
                'run_metadata': deepcopy(self._admittance_run), 'controller_status': status,
            })
        self.append_message('导纳调试日志已开启' if enabled else '导纳调试日志已关闭')
        self.update_admittance_window_context()

    def record_admittance_operator_event(self, event, note):
        """标注人为观察与最近收到的机器人样本；停止意图仍交由原停止入口处理。"""
        label = OPERATOR_EVENTS[event]
        event_monotonic_ns = time.monotonic_ns()
        event_wall_time_ns = time.time_ns()
        log_status = self.admittance_log_status()
        recorded = False
        if (self._admittance_run is not None and log_status['enabled']
                and log_status['active'] and log_status['error'] is None):
            status = (self.rtde_controller.get_status()['admittance']
                      if self._admittance_handoff_complete else None)
            pages = {} if status is None else {
                **status['continuous'], 'load': status['load'],
            }
            recorded = self._admittance_logger.record('operator_event', {
                'event': event, 'label': label, 'note': note,
                'event_monotonic_ns': event_monotonic_ns, 'event_wall_time_ns': event_wall_time_ns,
                'run_id': self._admittance_run['run_id'],
                'association': '点击时最近收到的机器人状态；主机事件与机器人样本异步，不代表同周期',
                'latest_robot_state': {
                    'timestamp': None if status is None else status['timestamp'],
                    'state_receive_count': None if status is None else status['state_receive_count'],
                    'state_received_monotonic': None if status is None else status['state_received_monotonic'],
                    'derived_valid': None if status is None else status['derived_valid'],
                    'lifecycle_valid': None if status is None else status['lifecycle_valid'],
                    'telemetry_layout': None if status is None else status['telemetry_layout'],
                    'page_type': None if status is None else status['page_type'],
                    'page_seq': None if status is None else status['page_seq'],
                    'model_sample_seq': None if status is None else status['model_sample_seq'],
                    'independent_pages': {
                        name: None if page is None else {key: page[key] for key in (
                            'run_id', 'page_seq', 'model_sample_seq',
                            'snapshot_time_s', 'received_monotonic_s')}
                        for name, page in pages.items()
                    },
                },
            })
        message = (f'人工事件“{label}”已提交日志队列；待文件写入。' if recorded else
                   f'人工事件“{label}”未记录：本轮日志不可用、已关闭或队列已满。')
        if not note:
            message += ' 备注为空，请在后续记录中补充接触位置与施力方向。'
        self.admittance_window.operator_event_result(message)
        self.append_message(message)
        if event == 'stop_requested':
            self.request_admittance_stop()

    def report_admittance_log_error(self):
        """写盘失败独立报告，保留机械臂控制及停止确认流程。"""
        status = self.admittance_log_status()
        if status['error'] is not None and status['error'] != self._admittance_reported_log_error:
            self._admittance_reported_log_error = status['error']
            self.append_message(f"导纳日志记录失败：{status['error']}；路径：{status['path']}")

    def record_admittance_application_state(self):
        """以界面状态周期补充独立通道上下文，不替代逐包 RTDE 日志。"""
        if self._admittance_run is None or not self._admittance_logging_enabled:
            return
        udp = self.udp_client.get_monitor_snapshot()
        command = udp['command']
        udp['command'] = (None if command is None else {
            'q_arm': list(command.q_arm), 'mode': command.mode,
            'q_gripper': list(command.q_gripper), 'received_time': command.recv_time,
        })
        self._admittance_logger.record('application_state', {
            'sample_source': '主界面状态定时器；目标 10 Hz，各通道独立采样',
            'control_source': self.control_source, 'robot_ready': self.robot_ready,
            'stop_pending': self._admittance_stop_pending,
            'handoff_complete': self._admittance_handoff_complete,
            'udp_synced': self._udp_synced, 'udp_sync_pending': self._udp_sync_pending,
            'speed_slider_percent': self.horizontalSlider_SpeedSlider.value(),
            'collection_active': self.collector.episode_active,
            'monitor_visible': self.admittance_window is not None and self.admittance_window.isVisible(),
            'ui_timing': deepcopy(self._ui_timing),
            'udp': udp,
            'gripper': None if self.gripper is None else self.gripper.get_monitor_snapshot(),
        })

    def on_admittance_log_finished(self, worker):
        """后台排空完成后报告路径，按本轮身份更新采集元数据。"""
        result = worker.result
        for run in self._admittance_runs:
            if run['id'] == worker.summary['run_id']:
                run['debug_log'] = result
                break
        self.collector.update_metadata({'admittance_runs': self._admittance_runs})
        if result['error'] is not None:
            self.append_message(f"导纳日志写盘失败：{result['error']}；路径：{result['path']}")
        elif result['path'] is not None:
            self.append_message(f"导纳日志已保存：{result['path']}（{result['record_count']} 条，丢弃 {result['dropped_count']} 条）")
        self._admittance_log_finish_workers.remove(worker)
        worker.deleteLater()
        self.update_admittance_window_context()

    def update_admittance_window_context(self):
        window = self.admittance_window
        if window is None:
            return
        parameters = window.parameters.prepared_parameters
        command, _ = self.get_udp_snapshot_and_count(parameters['MASTER_FRAME_TIMEOUT_S'])
        state = self.get_robot_snapshot()
        fresh_master = command is not None and command.mode == UDPControlMode.JOINT_TRACK
        aligned = (fresh_master and state is not None and
                   np.max(np.abs(np.asarray(command.q_arm) - state.q_actual)) <=
                   parameters['TELEOP_SYNC_TOLERANCE_RAD'])
        occupied = self.admittance_mode != 'off'
        run = self._admittance_run or (self._admittance_runs[-1] if self._admittance_runs else None)
        window.update_context({
            'robot_ready': self.robot_ready and self.rtde_controller is not None,
            'busy': self.control_source != 'idle', 'occupied': occupied,
            'stop_pending': self._admittance_stop_pending,
            'master_fresh': fresh_master, 'synced': self._udp_synced and aligned,
            'phase': self._admittance_message, 'run_id': self._last_admittance_run_id,
            'robot_address': self.connected_ip or self.robot_ip,
            'controller_joint_limits': (None if self.rtde_controller is None else
                                        self.rtde_controller.get_status()['joint_position_limit']),
            'parameters_revision': None if run is None else run['parameter_revision'],
            'run_metadata': deepcopy(run),
            'debug_log_status': self.admittance_log_status(),
            'speed_slider_percent': self.horizontalSlider_SpeedSlider.value(),
        })

    def refresh_admittance_window(self):
        """显示采样独立于控制与采集，只在独立窗口可见时执行。"""
        window = self.admittance_window
        if window is None or not window.isVisible():
            return
        self.update_admittance_window_context()
        now = time.monotonic()
        realtime = None
        if self.realtime_client is not None:
            state = self.realtime_client.get_latest_state()
            received_time = self.realtime_client.get_last_update_time()
            realtime_error = self.realtime_client.get_last_error()
            if state is not None and received_time is not None:
                realtime = {
                    'timestamp': state.time, 'actual_q': state.q_actual,
                    'actual_qd': state.qd_actual, 'actual_TCP_pose': state.tcp_pose,
                    'received_monotonic': now - (time.time() - received_time),
                    'tcp_force': state.tcp_force,
                    'speed_scaling': state.fields['speed_scaling'],
                    'requested_speed_percent': self.horizontalSlider_SpeedSlider.value(),
                    'last_error': None if realtime_error is None else str(realtime_error),
                }
        ft_sensor = None
        if self.torque_client is not None:
            torque = self.torque_client.get_latest()
            if torque is not None:
                ft_sensor = {'name': ('原始关节力矩订阅' if SHOW_RAW_JOINT_TORQUES else 'UR 内部补偿关节力矩'),
                             'timestamp': torque[0],
                             'values': torque[1], 'received_monotonic': None}
        status = self._last_admittance_status
        if (self.admittance_mode != 'off' and self._admittance_handoff_complete
                and self.rtde_controller is not None):
            status = self.rtde_controller.get_status()['admittance']
            self._last_admittance_status = status
        window.update_context({'diagnostic': None if status is None else status['diagnostic']})
        run = self._admittance_run or (self._admittance_runs[-1] if self._admittance_runs else None)
        snapshot = window.monitor.update(
            idle_admittance_status() if status is None else status,
            self.udp_client.get_monitor_snapshot(),
            run_id=self._last_admittance_run_id, connected=self.connected_ip is not None,
            realtime=realtime, gripper=None if self.gripper is None else self.gripper.get_monitor_snapshot(),
            ft_sensor=ft_sensor, now=now,
            parameters=window.parameters.prepared_parameters,
            parameter_revision=None if run is None else run['parameter_revision'],
            reference_source=window.reference_source, space=window.space,
            realtime_timeout_s=ROBOT_STATE_TIMEOUT_S,
            context={
                'feedback_destination': (UDP_REMOTE_IP, UDP_REMOTE_PORT),
                'udp_synced': self._udp_synced, 'udp_sync_pending': self._udp_sync_pending,
                'gripper_following': (self.control_source == 'udp' and self.gripper is not None
                                      and self.get_gripper_feedback() is not None and not self.gripper.is_paused),
                'requested_speed_percent': self.horizontalSlider_SpeedSlider.value(),
            })
        window.update_snapshot(snapshot)

    def start_admittance_run(self, reference_source, space, config, revision):
        """先在界面线程占用本轮控制权，再异步交接现有控制连接。"""
        window = self.admittance_window
        if window.parameters.has_unapplied_changes:
            self.append_message('请先应用尚未提交的参数草稿')
            return
        if (self.control_source != 'idle' or self.admittance_mode != 'off' or
                self._control_stop_failed or not self.robot_ready or self.rtde_controller is None):
            self.append_message('请先停止当前运动，并确认机器人就绪')
            return
        try:
            parameters = admittance_config.validate_run_config(reference_source, space, config=config)
        except ValueError as exc:
            self.append_message(f'导纳参数无效：{exc}')
            return
        state = self.get_robot_snapshot()
        if state is None or np.max(np.abs(state.qd_actual)) > parameters['STATIONARY_MAX_QD_RAD_S']:
            self.append_message('导纳偏置采样前请保持机器人静止、无外部接触')
            return
        command, source_seq = self.get_udp_snapshot_and_count(parameters['MASTER_FRAME_TIMEOUT_S'])
        if reference_source == 'teleop':
            if command is None or command.mode != UDPControlMode.JOINT_TRACK:
                self.append_message('遥操作导纳要求新鲜的主端关节跟踪指令')
                return
            aligned = np.max(np.abs(np.asarray(command.q_arm) - state.q_actual)) <= parameters['TELEOP_SYNC_TOLERANCE_RAD']
            if not self._udp_synced or not aligned:
                self.append_message('请保持主端静止，先同步主从并等待完成，再启动遥操作导纳')
                return
        self._admittance_stop_event = threading.Event()
        # 每轮单独统计界面回调；实际 OpenGL 绘制发生在后续 Qt 绘制事件中。
        self._ui_timing.clear()
        self._admittance_stop_pending = False
        self._admittance_handoff_complete = False
        self._last_admittance_status = None
        self._admittance_parameters = None
        window.clear_effective()
        self.admittance_mode = space
        self._admittance_reference_source = reference_source
        self.control_source = 'admittance_starting'
        self._udp_synced = False
        self._last_collected_admittance_time = None
        self._last_collected_submitted_seq = None
        self._last_collected_master_count = None
        self._admittance_message = '导纳启动中：静止采样偏置，请勿接触机器人'
        if parameters['DIAGNOSTIC_MODE'] != 'admittance':
            self._admittance_message = (
                f"{parameters['DIAGNOSTIC_MODE']}，观测J{parameters['DIAGNOSTIC_AXIS']}，限时{parameters['DIAGNOSTIC_DURATION_S']:g}秒；"
                "启动中：静止采样偏置，请勿接触机器人")
        self._last_admittance_run_id = len(self._admittance_runs) + 1
        self._admittance_run = {
            'record_version': ADMITTANCE_RECORD_VERSION,
            'id': self._last_admittance_run_id, 'parameter_revision': revision,
            'run_id': self._last_admittance_run_id, 'robot_address': self.connected_ip,
            'started_at': time.time(), 'reference_source': reference_source, 'space': space,
            'reference_behavior': 'moving_reference' if reference_source == 'teleop' else 'fixed_anchor',
            'model': parameters['DIAGNOSTIC_MODE'] if parameters['DIAGNOSTIC_MODE'] != 'admittance' else 'mass_damper_spring',
            'backend': 'urscript',
            'source_codes': deepcopy(ADMITTANCE_SOURCE_CODES),
            'prepared_parameters': deepcopy(parameters), 'parameters': None,
            'units_and_coordinates': None, 'master_applicable': reference_source == 'teleop',
            'debug_logging_enabled_at_start': self._admittance_logging_enabled,
            'hold_diagnostic': {
                'enabled': parameters['DIAGNOSTIC_MODE'] != 'admittance',
                'mode': parameters['DIAGNOSTIC_MODE'],
                'motion_axis': parameters['DIAGNOSTIC_MOTION_AXIS'],
                'axis': parameters['DIAGNOSTIC_AXIS'],
                'duration_s': parameters['DIAGNOSTIC_DURATION_S'],
                'settle_s': parameters['DIAGNOSTIC_SETTLE_S'],
                'model_design_s': parameters['MODEL_DT_S'],
                'servo_design_s': parameters['SERVO_DT_S'],
                'state_hz': parameters['DIAGNOSTIC_STATE_HZ'],
                'purpose': '按显式类型执行保持或预设微动；外力不驱动诊断目标，不验证柔顺闭环',
            },
        }
        self._admittance_logger = AdmittanceLogger(enabled=self._admittance_logging_enabled)
        self._admittance_reported_log_error = None
        self._admittance_logger.start({
            **deepcopy(self._admittance_run), 'python_version': sys.version,
            'units_and_coordinates': admittance_config.units(reference_source, space, config=parameters),
            'state_sampling': '每个实际收到的专用 RTDE 输出包；配置频率不代表保证的采样率',
        })
        self._admittance_runs.append(self._admittance_run)
        self.collector.update_metadata({'admittance_runs': self._admittance_runs})
        self.set_joint_targets_readonly(True)
        if self.gripper is not None:
            self.gripper.pause()
        self.update_buttons()
        self.append_message(self._admittance_message)
        self.dispatch_admittance_worker('start', {
            'reference_source': reference_source, 'space': space, 'config': deepcopy(parameters),
            'q_master': command.q_arm if reference_source == 'teleop' else None,
            'source_seq': source_seq if reference_source == 'teleop' else None,
        })

    def dispatch_admittance_worker(self, operation, start_kwargs=None):
        """同一时刻只有一个启停等待任务；完成收尾仍在界面线程。"""
        worker = AdmittanceLifecycleWorker(
            self.rtde_controller, operation, start_kwargs=start_kwargs,
            stop_event=self._admittance_stop_event, log_sink=self._admittance_logger, parent=self)
        self._admittance_worker = worker
        self._admittance_worker_result = None
        worker.completed.connect(self.on_admittance_worker_result)
        worker.finished.connect(self.on_admittance_worker_finished)
        worker.start()

    def on_admittance_worker_result(self, result):
        self._admittance_worker_result = result

    def on_admittance_worker_finished(self):
        worker = self._admittance_worker
        result = self._admittance_worker_result
        self._admittance_worker = None
        self._admittance_worker_result = None
        worker.deleteLater()
        status = result['status']
        self._admittance_logger.record('lifecycle_result', deepcopy(result))
        if result['operation'] == 'start':
            self._admittance_handoff_complete = result['parameters_accepted']
            if self._admittance_handoff_complete:
                self.record_admittance_status(status)
            if result['error']:
                self.append_message(f"导纳启动失败：{result['error']}")
            if self._admittance_stop_pending:
                self.dispatch_admittance_worker('stop')
                return
            if not result['success']:
                if result['error'] is not None or (status is not None and status['active']):
                    self._control_stop_failed = True
                    self.control_source = 'stopping'
                    self._admittance_message = result['error'] or '导纳启动未完成，需显式停止'
                else:
                    self.finish_admittance(result['error'] or '导纳启动已取消', status)
            else:
                self.control_source = 'udp' if self._admittance_reference_source == 'teleop' else 'admittance_fixed'
                if self._admittance_reference_source == 'teleop':
                    self.udp_control_timer.start(max(1, round(1000 / self._admittance_parameters['TELEOP_REFERENCE_HZ'])))
        elif result['success']:
            if self._admittance_handoff_complete:
                self.record_admittance_status(status)
                self.finish_admittance(status['last_error'] or status['exit_reason_text'] or '导纳已停止', status)
            else:
                self.finish_admittance('导纳启动已取消')
        else:
            self._admittance_stop_pending = False
            self._control_stop_failed = True
            self._admittance_message = f"停止尚未确认：{result['error']}"
            self.append_message(self._admittance_message)
        self.update_buttons()

    def defer_until_admittance_stopped(self, action, *, automatic=False):
        """断开、下电和退出均在同一停止确认之后执行。"""
        if self.admittance_mode == 'off':
            return False
        if not automatic or self._admittance_after_stop is None:
            self._admittance_after_stop = action
        self.request_admittance_stop()
        return True

    def request_admittance_stop(self):
        """停止目标生产并保留跨越启动交接阶段的同一个停止事件。"""
        if self.admittance_mode == 'off':
            return
        first_request = not self._admittance_stop_pending
        self.udp_control_timer.stop()
        self.jog_timer.stop()
        self.control_source = 'stopping'
        self._udp_synced = False
        if self.gripper is not None:
            self.gripper.pause()
        self._admittance_stop_event.set()
        self.rtde_controller.request_admittance_stop()
        self._admittance_stop_pending = True
        self._admittance_message = '导纳停止中，正在等待机器人确认'
        if first_request:
            self._admittance_logger.record('application_stop_requested', {
                'control_source': self.control_source, 'phase': self._admittance_message,
            })
            self.append_message(self._admittance_message)
        self.update_buttons()
        if self._admittance_worker is None:
            self.dispatch_admittance_worker('stop')

    def record_admittance_status(self, status):
        """后端接受参数后立即保存，不等待机器人 TCP 配置到达。"""
        self._last_admittance_status = status
        run = self._admittance_run
        if run is None:
            return
        changed = False
        if run['parameters'] is None:
            self._admittance_parameters = deepcopy(status['parameters'])
            run['parameters'] = deepcopy(status['parameters'])
            run['units_and_coordinates'] = admittance_config.units(
                run['reference_source'], run['space'], config=status['parameters'])
            self.admittance_window.set_effective(status['parameters'], run['parameter_revision'])
            changed = True
        if status['robot_configuration'] is not None and 'robot_configuration' not in run:
            run['robot_configuration'] = deepcopy(status['robot_configuration'])
            changed = True
        if status['controller_version'] is not None and 'controller_version' not in run:
            run['controller_version'] = deepcopy(status['controller_version'])
            changed = True
        if status['q_start'] is not None and 'q_start' not in run:
            # 锚点只取机器人已确认的本轮固定名义参考，后续实际运动不能改写。
            run['q_start'] = list(status['q_start'])
            changed = True
        # 收发点累计的最坏值随快照保存；20 Hz 采集只记录统计快照，不反推逐帧时序。
        run['timing_diagnostics'] = {
            key: status[key] for key in ADMITTANCE_HOST_TIMING_FIELDS + ADMITTANCE_ROBOT_HEARTBEAT_FIELDS
        }
        run['timing_diagnostics']['host_timing'] = deepcopy(status['host_timing'])
        run['continuous_pages'] = deepcopy({**status['continuous'], 'load': status['load']})
        if changed:
            self._admittance_logger.record('run_metadata_updated', deepcopy(run))
            self.collector.update_metadata({'admittance_runs': self._admittance_runs})

    def update_admittance_status(self):
        if (self.admittance_mode == 'off' or self.rtde_controller is None or
                not self._admittance_handoff_complete or self._admittance_worker is not None):
            return
        status = self.rtde_controller.get_status()['admittance']
        self.record_admittance_status(status)
        if status['stopped_confirmed'] and not status['active'] and not self._control_stop_failed:
            self.finish_admittance(status['last_error'] or status['exit_reason_text'] or '导纳已停止', status)
        elif status['last_error']:
            self._admittance_message = f"导纳异常：{status['last_error']}"
        elif status['running']:
            space = '关节' if self.admittance_mode == 'joint' else '末端'
            source = '遥操作导纳' if self._admittance_reference_source == 'teleop' else '定点柔顺'
            self._admittance_message = (
                f"{self._admittance_parameters['DIAGNOSTIC_MODE']}：运行中，观测J{self._admittance_parameters['DIAGNOSTIC_AXIS']}"
                if self._admittance_parameters['DIAGNOSTIC_MODE'] != 'admittance' else f'{space}{source}：运行中')
        else:
            self._admittance_message = f"导纳：{status['phase_name']}"

    def finish_admittance(self, reason, status=None):
        """机器人确认退出后收尾；未交接即取消时不借用上一轮样本。"""
        self.udp_control_timer.stop()
        if self.gripper is not None:
            self.gripper.pause()
        if self._admittance_run is not None:
            if status is not None and self._admittance_handoff_complete:
                self.record_admittance_status(status)
                self.collect_admittance_sample(time.time(), status)
            self._admittance_run.update(ended_at=time.time(), exit_reason=reason,
                                        exit_code=None if status is None else status['exit_reason'])
            self._admittance_logger.record('application_run_finished', {
                'reason': reason, 'controller_status': deepcopy(status),
            })
            self._admittance_run['debug_log'] = self._admittance_logger.get_status()
            log_worker = AdmittanceLogFinishWorker(
                self._admittance_logger, self._admittance_run, parent=self)
            self._admittance_log_finish_workers.append(log_worker)
            log_worker.finished.connect(lambda worker=log_worker: self.on_admittance_log_finished(worker))
            log_worker.start()
            self.collector.update_metadata({'admittance_runs': self._admittance_runs})
            self._admittance_run = None
        self._admittance_message = reason
        self.admittance_mode = 'off'
        self._admittance_reference_source = None
        self._admittance_stop_pending = False
        self.control_source = 'idle'
        self._control_stop_failed = False
        self.set_joint_targets_readonly(False)
        self.append_message(reason)
        action = self._admittance_after_stop
        self._admittance_after_stop = None
        if action is not None:
            QtCore.QTimer.singleShot(0, action)

    def sync_admittance_master(self):
        """窗口同步使用本次准备参数，普通 UDP 跟随保留原有策略。"""
        if self.admittance_window.reference_source != 'teleop':
            return
        self.on_sync_udp(config=self.admittance_window.parameters.prepared_copy())

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
        timeout_s = (UDP_COMMAND_TIMEOUT_S if self.admittance_mode == 'off' else
                     self._admittance_parameters['MASTER_FRAME_TIMEOUT_S'])
        command, source_seq = self.get_udp_snapshot_and_count(timeout_s)
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
        if self.get_gripper_feedback() is not None and admittance_running:
            self.gripper.set_target_position(command.q_gripper[0])
            if self.gripper.is_paused:
                self.gripper.resume()

    def on_sync_udp(self, checked=False, *, config=None):
        if (self.admittance_mode != 'off' or self._control_stop_failed or
                self.control_source != 'idle' or not self.robot_ready or self.rtde_controller is None):
            return
        if config is None:
            config = (admittance_config.snapshot() if self.admittance_window is None else
                      self.admittance_window.parameters.prepared_copy())
        command, _ = self.get_udp_snapshot_and_count(config['MASTER_FRAME_TIMEOUT_S'])
        if command is None or command.mode != UDPControlMode.JOINT_TRACK:
            return
        if not self.move_joint(command.q_arm):
            return
        self._udp_sync_pending = True
        self._udp_sync_parameters = deepcopy(config)
        if self.get_gripper_feedback() is not None:
            self.gripper.move(command.q_gripper[0], speed=GRIPPER_SPEED_DEFAULT,
                              force=GRIPPER_FORCE_DEFAULT)
            self.gripper.resume()

    def on_speed_changed(self):
        speed_percent = self.horizontalSlider_SpeedSlider.value()
        self.label_SpeedSlider.setText(f'限速: {speed_percent}%')
        if self.rtde_controller is not None and self._admittance_worker is None:
            self.rtde_controller.set_speed_slider(speed_percent / 100)

    def update_torque_subscription(self):
        """按全局配置选择显示来源，两种来源均由后台读取。"""
        source_text = ('raw：实际电流换算关节力矩，保留重力、摩擦等分量。'
                       if SHOW_RAW_JOINT_TORQUES else
                       'UR 内部补偿关节力矩：未减导纳启动偏置，未滤波、未过死区。无有效当前测量时留空。')
        self.checkBox_FTSwitch.setToolTip(source_text + ' 取消勾选显示 TCP 广义力。')
        enabled = self.checkBox_FTSwitch.isChecked() and self.robot_ready
        if not SHOW_RAW_JOINT_TORQUES:
            enabled = enabled and self.rtde_controller is not None
        if enabled and self.torque_client is None:
            try:
                self.torque_client = (URRTDETorqueClient(self.connected_ip, UR_RTDE_FREQ_HZ)
                                      if SHOW_RAW_JOINT_TORQUES else
                                      URCompensatedTorqueReader(self.rtde_controller))
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
        fb = self.get_gripper_feedback()
        if fb is not None:
            self.lineEdit_Clamp1.setText(f"{fb.position}")
            self.lineEdit_Clamp2.setText(f"{fb.current}")
        else:
            text = '未连接' if self.gripper is None else '无反馈'
            self.lineEdit_Clamp1.setText(text)
            self.lineEdit_Clamp2.setText(text)
        # 机械臂反馈不依赖夹爪；不可用的夹爪量用 NaN 标记，不伪装为真实零值。
        if self.robot_state is not None:
            gripper_values = [float('nan')] * 3 if fb is None else [fb.open, fb.current, fb.position]
            self.udp_client.queue_send_to(
                (UDP_REMOTE_IP, UDP_REMOTE_PORT), self.robot_state.q_actual,
                UDPControlMode.OFF, gripper_values, max_age=ROBOT_STATE_TIMEOUT_S)

        if self.checkBox_FTSwitch.isChecked():
            sample = None if self.torque_client is None else self.torque_client.get_latest()
            for index in range(1, 7):
                text = '' if sample is None else f'{sample[1][index - 1]:.3f}'
                getattr(self, f'lineEdit_Fex{index}').setText(text)
            error = None if self.torque_client is None else self.torque_client.get_last_error()
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
            self._last_collected_submitted_seq = None
            self._last_collected_master_count = None
            self.collector.start_episode()
            self.collection_timer.start(round(1000 / DATA_COLLECT_FREQ))
            self.pushButton_Collect.setText('采集结束')

    def collect_sample(self):
        timestamp = time.time()
        command = self.get_udp_snapshot()
        state = self.get_robot_snapshot()
        feedback = self.get_gripper_feedback()
        frames = self.get_camera_frames()
        # 每个采样周期使用同一数值时间戳，未更新的数据源不重复入库。
        if (self._admittance_reference_source != 'fixed' and command is not None
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
        if self.admittance_mode != 'off' and self._admittance_handoff_complete:
            self.collect_admittance_sample(timestamp, self.rtde_controller.get_status()['admittance'])
        for index, frame in enumerate(frames, 1):
            if frame is not None:
                # 保留相机后台接收该帧的时间，避免用本次取帧时间覆盖来源时间。
                self.collector.push_image(f'CAMERA_{index}', frame.image, timestamp=frame.received_time)
        self.collector.step()

    def collect_admittance_sample(self, timestamp, status):
        """按机器人时间去重保存同一遥测包；不适用的主端字段留空。"""
        if not self.collector.episode_active or self._admittance_run is None:
            return
        run_id = self._admittance_run['id']
        # 主端接收和主机提交按各自计数去重，不伪装成机器人同时采样。
        udp = self.udp_client.get_monitor_snapshot()
        command = udp['command']
        if (status['reference_source'] == 'teleop' and command is not None
                and udp['frame_count'] != self._last_collected_master_count):
            joint_values = command.q_arm if command.mode == UDPControlMode.JOINT_TRACK else [float('nan')] * 6
            self.collector.push_numeric('ADMITTANCE_MASTER_RECEIVED', [
                run_id, udp['frame_count'], udp['received_monotonic'], command.mode,
                *joint_values, *command.q_gripper], timestamp)
            self._last_collected_master_count = udp['frame_count']
        if (status['reference_source'] == 'teleop' and status['q_master_submitted'] is not None and
                status['submitted_source_seq'] != self._last_collected_submitted_seq):
            self.collector.push_numeric('ADMITTANCE_Q_SUBMITTED', status['q_master_submitted'], timestamp)
            self.collector.push_numeric('ADMITTANCE_SUBMISSION', [
                run_id, status['submitted_source_seq'], time.monotonic()], timestamp)
            self._last_collected_submitted_seq = status['submitted_source_seq']
        # 启动前只读基线也有机器人时间，只有本轮周期证据成立后才写机器人导纳组。
        if status['timestamp'] is None or status['cycle_count'] is None:
            return
        sample_key = (status['timestamp'], status['phase'])
        if sample_key == self._last_collected_admittance_time:
            return
        source = status['reference_source']
        space = status['space']
        teleop = source == 'teleop'
        self.collector.push_numeric('ADMITTANCE_STATE', [
            self._admittance_run['id'], 1 if teleop else 3, 1 if space == 'joint' else 2,
            status['phase'], status['exit_reason'],
            status['source_seq'] if teleop and status['source_seq'] is not None else float('nan'),
            status['adopted_source_seq'] if teleop and status['adopted_source_seq'] is not None else float('nan'),
            status['cycle_count'], status['timestamp'],
        ], timestamp)
        page_name = status['page_type']
        page = (status['load'] if page_name == 'load' else
                status['continuous'][page_name] if page_name in ('execution', 'timing') else None)
        if page is not None:
            self.collector.push_numeric('ADMITTANCE_PAGE_IDENTITY', [
                self._admittance_run['id'], page['run_id'],
                {'load': 2, 'execution': 3, 'timing': 4}[page_name], page['page_seq'],
                page['model_sample_seq'], page['snapshot_time_s'], page['received_monotonic_s'],
            ], timestamp)
        groups = {
            'ADMITTANCE_Q_CMD': status['q_cmd'],
            'ADMITTANCE_Q_ACTUAL': status['actual_q'],
            'ADMITTANCE_QD_ACTUAL': status['actual_qd'],
            'ADMITTANCE_Q_NOM': status['q_nom'],
            'ADMITTANCE_JOINT_DRIVE' if space == 'joint' else 'ADMITTANCE_CART_DRIVE': status['drive'],
            'ADMITTANCE_DELTA_Q' if space == 'joint' else 'ADMITTANCE_DELTA_X': (
                status['delta_q'] if space == 'joint' else status['delta_x']),
        }
        if teleop:
            groups['ADMITTANCE_Q_MASTER'] = status['q_master']
        else:
            groups['ADMITTANCE_Q_START'] = status['q_start']
        for name, values in groups.items():
            if values is not None:
                self.collector.push_numeric(name, values, timestamp)
        self.collector.push_numeric('ADMITTANCE_HOST_TIMING', [
            float('nan') if status[key] is None else status[key]
            for key in ADMITTANCE_HOST_TIMING_FIELDS], timestamp)
        self.collector.push_numeric('ADMITTANCE_ROBOT_HEARTBEAT', [
            float('nan') if status[key] is None else status[key]
            for key in ADMITTANCE_ROBOT_HEARTBEAT_FIELDS], timestamp)
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
            if self._admittance_logger is not None:
                self._admittance_logger.record('application_message', {'message': str(message)})
            message = time.strftime("%H:%M:%S", time.localtime()) + ': ' + message
            self.plainTextEdit_DashboardMessage.appendPlainText(message)
            if self.admittance_window is not None:
                self.admittance_window.append_log(message)

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
        if self.defer_until_admittance_stopped(self.close):
            event.ignore()
            return
        for timer in (self.status_timer, self.robot_view_timer, self.jog_timer,
                      self.udp_control_timer, self.camera_timer, self.collection_timer,
                      self.admittance_monitor_timer):
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
            if self.admittance_window is not None and self.admittance_window.isVisible():
                self.admittance_monitor_timer.start(50)
            event.ignore()
            return
        self.udp_client.stop()
        for camera in (self.camera_1, self.camera_2):
            if camera is not None:
                camera.close()
        # 机械臂已停止后再等待剩余日志落盘，退出时不遗留写盘线程。
        for worker in list(self._admittance_log_finish_workers):
            worker.wait()
            for run in self._admittance_runs:
                if run['id'] == worker.summary['run_id']:
                    run['debug_log'] = worker.result
                    break
        self.collector.update_metadata({'admittance_runs': self._admittance_runs})
        self.collector.close()
        self.robot_visualizer.close()
        if self.admittance_window is not None:
            self.admittance_window.close()
            self.admittance_window.deleteLater()
        event.accept()


if __name__ == '__main__':
    configure_high_dpi()
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
