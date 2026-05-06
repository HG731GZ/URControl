# PyQt5 QtWebEngine bundles libexpat which shadows Python's pyexpat symbols.
# Force pyexpat to load first so meshcat can import it without conflict.
import xml.etree.ElementTree  # noqa: F401

import sys
import os
import time
import numpy as np
from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout
from PyQt5.QtCore import QUrl, Qt
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtGui import QImage, QPixmap

from DataCollector import DataCollector
from ui_main_window import Ui_MainWindow
from UR_Utils.URDashboardClient import URDashboardClient
from UR_Utils.URScriptClient import URScriptClient
from UR_Utils.URRealtimeClient import URRealtimeClient
from UR_Utils.URRTDEController import URRTDEController
from UR_Utils.URUdpClient import URUDPClient, UDPControlMode
from GripperController import GripperController, GRIPPER_SPEED_DEFAULT, GRIPPER_FORCE_DEFAULT
from RealSenseCamera import Camera, CameraError

import NetWorkSet
import threading
from UR_Utils.ur5e_visualizer import UR5eDualVisualizer

QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)

UR_REAL_IP = '192.168.3.15'
UR_SIM_IP = '127.0.0.1'
UR_J_SPEED_UI = 0.1  # 关节控制按钮的速度
UR_TCP_SPEED_UI = 0.01  # 末端控制按钮的速度
CAMERA_RESOLUTION = (640, 480)  # 相机分辨率，RGB和深度统一设定
CAMERA_FPS = 30  # 相机帧率
LOCAL_IP = NetWorkSet.get_local_ip()
UDP_LOCAL_PORT = 5005  # UDP本机端口
UDP_REMOTE_PORT = 6005  # UDP远端端口
UDP_REMOTE_IP = '192.168.3.5'  # UDP远端IP
UR_HOME = [-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0]  # 预设零位
UR_RTDE_FREQ = 500  # RTDE频率
UR_485_PORT = 54321  # UR485的转发端口
DATA_COLLECT_FREQ = 20.0  # 数采频率


class UI_MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()

        # UR相关
        self.URIP = UR_REAL_IP
        self.URDashboardClient = None
        self.URScriptClient = None
        self.URRealtimeClient = None
        self.URRTDEController = None
        self.GripperController = None
        self.UR_J_Control_Speed = UR_J_SPEED_UI  # 关节控制按钮的速度
        self.UR_TCP_Control_Speed = UR_TCP_SPEED_UI  # 末端控制按钮的速度

        # 深度相机
        self.Camera1 = Camera('d435i', resolution=CAMERA_RESOLUTION, fps=CAMERA_FPS)
        time.sleep(0.2)
        self.Camera2 = Camera('d455', resolution=CAMERA_RESOLUTION, fps=CAMERA_FPS)

        # 数采
        self.DataCollector = DataCollector(session_name='test1')
        self.DataCollector.register_numeric('TCP_POSE')
        self.DataCollector.register_numeric('GRIPPER')
        if self.Camera1 is not None:
            self.DataCollector.register_image('CAMERA_1', camera_id=self.Camera1.device_name)
        if self.Camera2 is not None:
            self.DataCollector.register_image('CAMERA_2', camera_id=self.Camera2.device_name)
        self.timer_DataCollect = QtCore.QTimer(self)

        # 窗口控件
        self.setupUi(self)
        self.timer_URStatus = QtCore.QTimer(self)
        self.timer_URStatus.start(100)

        self.timer_URStatus_RT = QtCore.QTimer(self)

        self.timer_CameraUpdate = QtCore.QTimer(self)
        self.timer_CameraUpdate.start(50)

        self.timer_URRTDEControl_UI = QtCore.QTimer(self)

        self.timer_URUDPControl = QtCore.QTimer(self)
        self.timer_URUDPControl.setTimerType(Qt.PreciseTimer)

        # 创建可视化界面（Meshcat服务器在此启动）
        mjcf_path = os.path.join(os.path.dirname(__file__), 'UR_Utils/universal_robots_ur5e', 'ur5e.xml')
        self.viz_visual = UR5eDualVisualizer(mjcf_path)

        self.URVisual = QWebEngineView(self.widget_WebView)
        layout = self.widget_WebView.layout()
        if layout is None:
            layout = QVBoxLayout(self.widget_WebView)
            layout.setContentsMargins(0, 0, 0, 0)

        layout.addWidget(self.URVisual)
        self.URVisual.load(QUrl(self.viz_visual.url))

        self.lineEdit_IP.setText(self.URIP)
        self.label_IP_now.setText(self.URIP)

        # 控件事件
        self.pushButton_IP.clicked.connect(self.on_IP_Button_Clicked)
        self.pushButton_ConnectUR.clicked.connect(self.on_Connect_UR_Button)
        self.pushButton_URPowerOn.clicked.connect(self.on_URPowerOn_Button)
        self.pushButton_URBrakeRelease.clicked.connect(self.on_URBrakeRelease_Button)
        self.pushButton_Shutdown.clicked.connect(self.on_URShutdown_Button)
        self.pushButton_Stop.clicked.connect(self.on_URStop_Button)
        self.pushButton_URScriptMoveJ.clicked.connect(self.on_URScriptMoveJ_Button)
        self.pushButton_RTDEUDP.clicked.connect(self.on_RTDEUDP_Button)
        self.pushButton_StopRTDE.clicked.connect(self.on_RTDEStop_Button)
        self.pushButton_HOME.clicked.connect(self.on_HOME_Button)
        self.pushButton_UDPSync.clicked.connect(self.on_UDPSync_Button)
        self.pushButton_Collect.clicked.connect(self.on_Collect_Button)
        self.horizontalSlider_SpeedSlider.valueChanged.connect(self.on_SpeedSliderValueChanged)

        self.timer_URStatus.timeout.connect(self.on_timerURStatus_timeout)
        self.timer_URStatus_RT.timeout.connect(self.on_timerURStatus_RT_timeout)
        self.timer_URRTDEControl_UI.timeout.connect(self.on_timerURRTDE_UI_timeout)
        self.timer_URUDPControl.timeout.connect(self.on_timerURUDPControl_timeout)
        self.timer_CameraUpdate.timeout.connect(self.on_timerCameraUpdate_timeout)
        self.timer_DataCollect.timeout.connect(self.on_timerDataCollect_timeout)
        self.control_button_events_connect()
        self.lineedits_qtarget_bind_validation()

        # UDP 外源控制
        self.ur_udp_client = URUDPClient(bind_host='0.0.0.0', bind_port=UDP_LOCAL_PORT)
        self.ur_udp_client.start()

        # 遥操作控制状态（timer 回调时读取，避免每次按下按钮重复 connect）
        self._control_dq = None
        self._control_mode = None

    # 控件事件函数
    def on_IP_Button_Clicked(self):
        if NetWorkSet.is_valid_ipv4(self.lineEdit_IP.text()):
            self.URIP = self.lineEdit_IP.text()
            self.label_IP_now.setText(self.URIP)
        else:
            self.label_IP_now.setText("请输入合法的IP")

    def on_Connect_UR_Button(self):
        self.URDashboardClient = URDashboardClient(self.URIP, auto_connect=False)
        self.message_append_to_textbox(self.URDashboardClient.connect())
        self.URScriptClient = URScriptClient(self.URIP, auto_connect=True)
        self.URRealtimeClient = URRealtimeClient(self.URIP, auto_connect=True)
        self.timer_URStatus_RT.start(30)
        if self.URDashboardClient.robot_mode() == 'Robotmode: RUNNING':
            self.URRTDEController = URRTDEController(self.URIP, frequency=UR_RTDE_FREQ, use_safety_check=False)

        # 创建夹钳控制器连接 (TCP 串口服务器)
        if self.GripperController is None:
            try:
                self.GripperController = GripperController(
                    port=self.URIP + f":{UR_485_PORT}", slave_id=1, connection_type="tcp", debug=False)
                self.GripperController.start(interval=0.05)
                self.message_append_to_textbox("夹钳控制器已连接并启动")
            except Exception as e:
                self.message_append_to_textbox(f"夹钳控制器连接失败: {e}")

    def on_URPowerOn_Button(self):
        self.message_append_to_textbox(self.URDashboardClient.power_on())

    def on_URBrakeRelease_Button(self):
        self.message_append_to_textbox(self.URDashboardClient.brake_release())

    def on_URShutdown_Button(self):
        self.message_append_to_textbox(self.URDashboardClient.power_off())

    def on_URStop_Button(self):
        self.URScriptClient.stopj(a=10)

    def on_Collect_Button(self):
        if self.pushButton_Collect.text() == '开始采集':
            self.pushButton_Collect.setText('采集结束')
            self.DataCollector.start_episode()
            self.timer_DataCollect.start(int(1000 / DATA_COLLECT_FREQ))
        else:
            self.pushButton_Collect.setText('开始采集')
            self.DataCollector.end_episode()
            self.timer_DataCollect.stop()

    def on_RTControl_Button_Pressed(self, control_mode, index, direction):
        control_delta = [0, 0, 0, 0, 0, 0]
        if control_mode == 'joint':
            control_delta[index - 1] = direction * self.UR_J_Control_Speed
        elif control_mode in ('tcp_tool', 'tcp_base'):
            tcp_speed = self.UR_TCP_Control_Speed if index <= 3 else self.UR_TCP_Control_Speed * 10
            control_delta[index - 1] = direction * tcp_speed
        else:
            return

        self._control_dq = control_delta
        self._control_mode = control_mode
        self.URRTDEController.start()
        self.timer_URRTDEControl_UI.start(2)
        self.lineedits_qtarget_setreadonly(True)

    def on_RTControl_Button_Released(self):
        # 这里直接停的时候会顿一下，暂时不知道怎么处理
        self.URRTDEController.stop()
        self.timer_URRTDEControl_UI.stop()
        self._control_dq = None
        self._control_mode = None
        self.lineedits_qtarget_setreadonly(False)

    def on_URScriptMoveJ_Button(self):
        q = [np.deg2rad(self.get_valid_qtarget_degree(i + 1, commit=True)) for i in range(6)]
        self.URScriptClient.movej(q, v=0.1)

    def on_HOME_Button(self):
        self.URScriptClient.movej(UR_HOME, v=0.1)

    def on_RTDEUDP_Button(self):
        self.URRTDEController.start()
        self.timer_URUDPControl.start(10)

    def on_RTDEStop_Button(self):
        self.URRTDEController.stop()
        self.timer_URUDPControl.stop()

    def on_UDPSync_Button(self):
        udp_err = self.ur_udp_client.get_last_error()
        if udp_err is not None:
            for i in range(1, 9):
                getattr(self, f"lineEdit_UDP{i}").setText("UDP ERR")
        else:
            cmd = self.ur_udp_client.get_latest()
            if cmd is not None:
                self.URScriptClient.movej(cmd.q_arm)
                self.GripperController.move(cmd.q_gripper[0], speed=GRIPPER_SPEED_DEFAULT, force=GRIPPER_FORCE_DEFAULT)

    def on_SpeedSliderValueChanged(self):
        self.label_SpeedSlider.setText(f'限速: {self.horizontalSlider_SpeedSlider.value()}%')
        if self.URRTDEController is not None:
            self.URRTDEController.set_speed_slider(self.horizontalSlider_SpeedSlider.value() / 100)

    def on_timerURStatus_timeout(self):
        message = ''
        udp_info = f"UDP: {LOCAL_IP}:{self.ur_udp_client.bind_port} | "
        if self.URDashboardClient is not None:
            robot_status = self.URDashboardClient.robot_mode()
            if robot_status is not None:
                message = message + robot_status + ";"
                self.pushButton_Shutdown.setEnabled(True)
                self.pushButton_URPowerOn.setEnabled(True)
                self.pushButton_URBrakeRelease.setEnabled(True)
                self.pushButton_Stop.setEnabled(True)
                self.pushButton_RTDEUDP.setEnabled(True)
                self.pushButton_StopRTDE.setEnabled(True)
                self.pushButton_HOME.setEnabled(True)
            if self.URRTDEController is not None:
                message = message + "\t RTDE Connected;"
                if self.URRTDEController.is_running():  # 需要注意如果线程暂停了这个地方还是会显示true
                    message = message + "\t RTDE Running;"
            self.statusbar.showMessage(udp_info + message)
        else:
            self.statusbar.showMessage(udp_info + 'UR未连接')

    def on_timerURStatus_RT_timeout(self):
        # UDP 外源数据显示
        udp_err = self.ur_udp_client.get_last_error()
        if udp_err is not None:
            self.pushButton_UDPSync.setEnabled(False)
            for i in range(1, 9):
                getattr(self, f"lineEdit_UDP{i}").setText("UDP ERR")
        else:
            cmd = self.ur_udp_client.get_latest()
            if cmd is not None:
                self.pushButton_UDPSync.setEnabled(True)
                mode_cn = UDPControlMode.cn_name(cmd.mode)
                self.lineEdit_UDP1.setText(mode_cn)
                for i in range(6):
                    getattr(self, f"lineEdit_UDP{i + 2}").setText(f"{cmd.q_arm[i] * 180 / np.pi:.4f}")
                self.lineEdit_UDP8.setText(f"{cmd.q_gripper[0]:.4f}")
                if self.checkBox_UDPVisual.checkState() == Qt.Checked:
                    q_target_rad = cmd.q_arm
                    q_target_7 = np.append(q_target_rad, 0.0)
                    self.viz_visual.update_virtual(q_target_7)
            else:
                for i in range(1, 9):
                    getattr(self, f"lineEdit_UDP{i}").setText("")

        if self.URRealtimeClient is not None:
            state = self.URRealtimeClient.get_latest_state()
            if state is not None:
                for i in range(6):
                    # 实时关节角
                    line_edit = getattr(self, f"lineEdit_QA{i + 1}")
                    line_edit.setText(f"{state.q_actual[i] * 180 / np.pi:.3f}")
                    # TCP
                    line_edit = getattr(self, f"lineEdit_TA{i + 1}")
                    if i < 3:
                        line_edit.setText(f"{state.tcp_pose[i] * 1000:.3f}")
                    else:
                        line_edit.setText(f"{state.tcp_pose[i] * 180 / np.pi:.3f}")
                    # TCP Force
                    line_edit = getattr(self, f"lineEdit_Fex{i + 1}")
                    line_edit.setText(f"{state.tcp_force[i] :.3f}")

                    # 目标关节角
                    line_edit = getattr(self, f"lineEdit_QT{i + 1}")
                    if line_edit.isReadOnly():
                        line_edit.setText(f"{state.fields['q_target'][i] * 180 / np.pi:.3f}")

                # 更新可视化：真实机械臂=当前关节角，虚拟机械臂=目标关节角
                q_actual_7 = np.append(state.q_actual, 0.0)
                self.viz_visual.update_actual(q_actual_7)
                if self.checkBox_UDPVisual.checkState() != Qt.Checked:
                    q_target_deg = [self.get_valid_qtarget_degree(i + 1) for i in range(6)]
                    q_target_rad = np.deg2rad(q_target_deg)
                    q_target_7 = np.append(q_target_rad, 0.0)
                    self.viz_visual.update_virtual(q_target_7)
            else:
                print('RealTime接口接收异常！')
        else:
            print('RealTime接口异常！')
        # 夹钳实时反馈
        if self.GripperController is not None:
            fb = self.GripperController.feedback
            self.lineEdit_Clamp1.setText(f"{fb.position}")
            self.lineEdit_Clamp2.setText(f"{fb.current}")

    def on_timerURRTDE_UI_timeout(self):
        if self._control_dq is None or self._control_mode is None:
            return
        if self._control_mode == 'tcp_tool':
            # self.URRTDEController.move_tcp_delta(delta_pose=self._control_dq, dq_max=10, frame="tool")
            self.URRTDEController.speedL(xd=self._control_dq, time_s=1, frame='tool')
        elif self._control_mode == 'tcp_base':
            # self.URRTDEController.move_tcp_delta(delta_pose=self._control_dq, dq_max=10, frame="base_add")
            self.URRTDEController.speedL(xd=self._control_dq, time_s=1, frame='base_add')
        elif self._control_mode == 'joint':
            # self.URRTDEController.move_joint_delta(delta_q=self._control_dq, dq_max=10)
            self.URRTDEController.speedJ(qd=self._control_dq, time_s=1, acceleration=0.1)

    # 遥操作定时器
    def on_timerURUDPControl_timeout(self):
        udp_err = self.ur_udp_client.get_last_error()
        if udp_err is not None:
            for i in range(1, 9):
                getattr(self, f"lineEdit_UDP{i}").setText("UDP ERR")
        else:
            cmd = self.ur_udp_client.get_latest()
            if cmd is not None:
                mode_cn = UDPControlMode.cn_name(cmd.mode)
                if cmd.mode == 1:  # 关节跟踪
                    self.URRTDEController.track_joint(cmd.q_arm, dq_max=0.5)
                if self.GripperController is not None:
                    self.GripperController.set_target_position(cmd.q_gripper[0])

    # 相机显示定时器
    def on_timerCameraUpdate_timeout(self):

        if self.Camera1 is not None:
            try:
                camera1_RGB_frame = self.Camera1.get_rgb_frame()
                rgb_image = camera1_RGB_frame.image
                pixmap = self.convert_image_to_QImage(rgb_image)
                self.label_camera1.setPixmap(pixmap)
            except CameraError as exc:
                print(f"Camera1 取帧失败: {exc}")
                self.Camera1.close()
                self.Camera1 = None
        else:
            self.label_camera1.setText("No Camera1")

        if self.Camera2 is not None:
            try:
                camera2_RGB_frame = self.Camera2.get_rgb_frame()
                rgb_image = camera2_RGB_frame.image
                pixmap = self.convert_image_to_QImage(rgb_image)
                self.label_camera2.setPixmap(pixmap)
            except CameraError as exc:
                print(f"Camera2 取帧失败: {exc}")
                self.Camera2.close()
                self.Camera2 = None
        else:
            self.label_camera2.setText("No Camera2")

    # 数采定时器
    def on_timerDataCollect_timeout(self):

        if self.URRealtimeClient is not None:
            state = self.URRealtimeClient.get_latest_state()
            if state is not None:
                self.DataCollector.push_numeric('TCP_POSE', state.tcp_pose)

        if self.GripperController is not None:
            fb = self.GripperController.feedback
            self.DataCollector.push_numeric('GRIPPER', [fb.open, fb.current])

        if self.Camera1 is not None:
            try:
                camera1_RGB_frame = self.Camera1.get_rgb_frame()
                rgb_image = camera1_RGB_frame.image
                self.DataCollector.push_image('CAMERA_1', rgb_image)
            except CameraError as exc:
                print(f"Camera1 取帧失败: {exc}")
                self.Camera1.close()
                self.Camera1 = None

        if self.Camera2 is not None:
            try:
                camera2_RGB_frame = self.Camera2.get_rgb_frame()
                rgb_image = camera2_RGB_frame.image
                self.DataCollector.push_image('CAMERA_2', rgb_image)
            except CameraError as exc:
                print(f"Camera2 取帧失败: {exc}")
                self.Camera2.close()
                self.Camera2 = None

        self.DataCollector.step()

    # 其他辅助函数

    @staticmethod
    def convert_image_to_QImage(image: np.ndarray) -> QPixmap:
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

    def message_append_to_textbox(self, message):
        if message is not None:
            message = time.strftime("%H:%M:%S", time.localtime()) + ': ' + message
            self.plainTextEdit_DashboardMessage.appendPlainText(message)

    def control_button_events_connect(self):
        for i in range(1, 7):
            getattr(self, f"pushButton_JUp{i}").pressed.connect(
                lambda checked=False, idx=i: self.on_RTControl_Button_Pressed('joint', idx, 1))
            getattr(self, f"pushButton_JDown{i}").pressed.connect(
                lambda checked=False, idx=i: self.on_RTControl_Button_Pressed('joint', idx, -1))
            getattr(self, f"pushButton_JUp{i}").released.connect(self.on_RTControl_Button_Released)
            getattr(self, f"pushButton_JDown{i}").released.connect(self.on_RTControl_Button_Released)

            getattr(self, f"pushButton_TUp{i}").pressed.connect(
                lambda checked=False, idx=i: self.on_RTControl_Button_Pressed('tcp_tool', idx, 1))
            getattr(self, f"pushButton_TDown{i}").pressed.connect(
                lambda checked=False, idx=i: self.on_RTControl_Button_Pressed('tcp_tool', idx, -1))
            getattr(self, f"pushButton_TUp{i}").released.connect(self.on_RTControl_Button_Released)
            getattr(self, f"pushButton_TDown{i}").released.connect(self.on_RTControl_Button_Released)

            getattr(self, f"pushButton_TWUp{i}").pressed.connect(
                lambda checked=False, idx=i: self.on_RTControl_Button_Pressed('tcp_base', idx, 1))
            getattr(self, f"pushButton_TWDown{i}").pressed.connect(
                lambda checked=False, idx=i: self.on_RTControl_Button_Pressed('tcp_base', idx, -1))
            getattr(self, f"pushButton_TWUp{i}").released.connect(self.on_RTControl_Button_Released)
            getattr(self, f"pushButton_TWDown{i}").released.connect(self.on_RTControl_Button_Released)

    def lineedits_qtarget_bind_validation(self):
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

    def lineedits_qtarget_setreadonly(self, flag: bool) -> None:
        for i in range(6):
            line_edit = getattr(self, f"lineEdit_QT{i + 1}")
            line_edit.setReadOnly(flag)

    def closeEvent(self, event):
        self.timer_URStatus.stop()
        self.timer_URStatus_RT.stop()
        self.timer_CameraUpdate.stop()
        self.timer_URRTDEControl_UI.stop()
        self.ur_udp_client.stop()
        if self.URRTDEController is not None:
            self.URRTDEController.shutdown()
        if self.URRealtimeClient is not None:
            self.URRealtimeClient.close()
        if self.URScriptClient is not None:
            self.URScriptClient.close()
        if self.URDashboardClient is not None:
            self.URDashboardClient.close()
        if self.GripperController is not None:
            self.GripperController.close()
        if self.Camera1 is not None:
            self.Camera1.close()
        if self.Camera2 is not None:
            self.Camera2.close()
        if hasattr(self, 'viz_visual'):
            self.viz_visual.close()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = UI_MainWindow()
    window.show()
    sys.exit(app.exec_())
