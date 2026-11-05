"""现场监护的单轮 UDP 联调，复用正式阻抗集成层与窗口，不启动相机或夹爪。"""

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import signal
import socket
import sys
import threading
import time

from PyQt5 import QtCore, QtWidgets

from admittance_manual import read_robot_state, require_stopped
from impedance_lifecycle import ImpedanceIntegration
from ui_classic_window import Ui_MainWindow
from UR_Utils.URRealtimeMonitor import URRealtimeMonitor
from UR_Utils.URRTDEController import URRTDEController
from UR_Utils.URUdpClient import URUDPClient
from UR_Utils.ur5e_visualizer import UR5eDualVisualizer
from window_display import configure_high_dpi


class ManualRecord:
    def __init__(self, path, plan):
        self.path, self.data = path, plan
        self.update_metadata({})

    def update_metadata(self, values):
        self.data.update(deepcopy(values))
        self.path.write_text(json.dumps(self.data, ensure_ascii=False, indent=2), encoding='utf-8')


class ManualWindow(ImpedanceIntegration, QtWidgets.QMainWindow):
    """使用真实反馈和正式集成层；自动操作仅限命令行明确选定的这一轮。"""

    def __init__(self, args, center, before):
        super().__init__()
        self.args, self.center = args, center
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.pushButton_Impedance = self.ui.pushButton_Impedance
        self.pushButton_TeleopImpedance = self.ui.pushButton_TeleopImpedance
        self.setWindowTitle('遥操作阻抗 · 现场单轮联调 · 不连接相机或夹爪')
        self.connected_ip = args.robot_ip
        self.control_source, self.admittance_mode = 'idle', 'off'
        self._control_stop_failed = self._udp_synced = False
        self.robot_ready = True
        self.gripper = None
        self.udp_control_timer = QtCore.QTimer(self)
        self.jog_timer = QtCore.QTimer(self)
        self.realtime_client = URRealtimeMonitor(args.robot_ip)
        self.realtime_client.connect()
        self.rtde_controller = URRTDEController(args.robot_ip, frequency=500,
                                               use_safety_check=True, use_rtde_safety_check=True, auto_start_on_command=False)
        self.envelope_offset = .09 if args.axis == 1 else .07
        self.envelope_stage = 'sync'
        self.envelope_ready_time = None
        planned = []
        endpoints = ((args.axis-1, -self.envelope_offset), (args.axis-1, self.envelope_offset)) if args.scenario == 'envelope' else ((0, -.05), (0, -.02), (0, .02), (0, .04), (0, .08))
        for axis, offset in endpoints:
            target = list(center)
            target[axis] += offset
            check = self.rtde_controller.check_joint_safety(target)
            if not check['local_hard_limit_ok'] or not check['rtde_isJointsWithinSafetyLimits'] or check['rtde_error']:
                self.rtde_controller.close()
                self.realtime_client.close()
                raise RuntimeError('联调目标未通过本地及控制器安全检查：' + str(check))
            planned.append(target)
        if args.scenario == 'combined':
            for sign in (-1, 1):
                target = [q + sign*.01*(1 if j%2 == 0 else -1) for j,q in enumerate(center)]
                check = self.rtde_controller.check_joint_safety(target)
                if not check['local_hard_limit_ok'] or not check['rtde_isJointsWithinSafetyLimits'] or check['rtde_error']:
                    self.rtde_controller.close()
                    self.realtime_client.close()
                    raise RuntimeError('组合目标未通过控制器安全检查：' + str(check))
                planned.append(target)
        self.udp_client = URUDPClient(bind_host='127.0.0.1', bind_port=args.port)
        self.udp_client.start()
        self.collector = ManualRecord(args.output, dict(scenario=args.scenario,
            workspace_center=center, preflight=before, udp_hz=args.hz, started_at=time.time(), axis=args.axis,
            controller_safety_checked_targets=planned,
            validation_scope='正式窗口、ImpedanceIntegration、URRTDEController、UDP、控制进程及机器人脚本；不含主程序相机/夹爪'))
        self.init_impedance()
        self.ui.pushButton_TeleopImpedance.clicked.connect(self.on_teleop_impedance)
        self.ui.pushButton_Impedance.clicked.connect(self.on_impedance)
        self.ui.pushButton_Stop.setEnabled(True)
        self.ui.pushButton_StopRTDE.setEnabled(True)
        self.ui.pushButton_Stop.clicked.connect(self.request_impedance_stop)
        self.ui.pushButton_StopRTDE.clicked.connect(self.request_impedance_stop)
        model = Path(__file__).parent / 'UR_Utils/universal_robots_ur5e/ur5e.xml'
        self.robot_visualizer = UR5eDualVisualizer(str(model), camera_azimuth_deg=-120, camera_elevation_deg=21)
        self.ui.verticalLayout_RobotViewport.addWidget(self.robot_visualizer.widget)
        self.robot_visualizer.update_actual(center)
        self.robot_visualizer.update_virtual(center)
        self.started = self.running_since = self.finished_since = None
        self.previous_phase = None
        self.peak_displacement = [0.0]*6
        self.peak_speed = [0.0]*6
        self.terminal_requested = False
        self.hidden_once = self.reopened = False
        self.sender_stop = threading.Event()
        self.sender_error = None
        self.sender = None
        QtCore.QTimer.singleShot(1500, self._begin_source)
        self.on_impedance_tuning()
        self.impedance_tuning_window.apply_parameters()
        self.impedance_tuning_window.hide()
        self.on_teleop_impedance()
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._observe)
        self.timer.start(20)
        self.launch_deadline = time.monotonic()+12

    def append_message(self, message):
        print(message, flush=True)

    def get_gripper_feedback(self):
        return None

    def get_robot_snapshot(self):
        snapshot = self.realtime_client.get_snapshot()
        if snapshot['error'] or snapshot['state'] is None or time.monotonic()-snapshot['received_monotonic'] > 0.25:
            return None
        return snapshot['state']

    def _begin_source(self):
        """首轮模型加载完成后才启动本轮发送，不把加载停顿带入通信时序。"""
        self.sender = threading.Thread(target=self._send, daemon=True, name="ManualUDPSource")
        self.sender.start()

    def _send(self):
        """仅通过本机 UDP 发送目标；不调用运动后端的目标提交接口。"""
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                next_send = time.monotonic()
                mode_event_sent = False
                while not self.sender_stop.is_set():
                    elapsed = 0 if self.running_since is None else time.monotonic()-self.running_since
                    qd = [0.0]*6
                    q = list(self.center)
                    q[0] += 0.02
                    if self.args.scenario == 'envelope':
                        q[0] = self.center[0]
                        q[self.args.axis-1] += -self.envelope_offset if self.envelope_stage == 'sync' else self.envelope_offset
                    elif self.args.scenario.startswith('sync'):
                        q[0] = self.center[0] - .05
                        if self.started is not None and time.monotonic()-self.started > .1:
                            q[0] = self.center[0] + .04
                    elif self.running_since is not None and self.args.scenario in ('follow','hidden','global-stop'):
                        q[0] = self.center[0] + .02*math.cos(2*math.pi*elapsed/4)
                        qd[0] = -.02*(math.pi/2)*math.sin(math.pi*elapsed/2)
                    if self.args.scenario == 'latest':
                        offset = .08 if elapsed < .10 else (.02 if elapsed < .12 else -.01)
                        q[0] = self.center[0] + offset
                    elif self.args.scenario == 'combined':
                        offset = .01*math.cos(2*math.pi*elapsed/4)
                        q = [value + offset*(1 if j%2 == 0 else -1) for j,value in enumerate(self.center)]
                        speed = -.01*(math.pi/2)*math.sin(math.pi*elapsed/2)
                        qd = [speed*(1 if j%2 == 0 else -1) for j in range(6)]
                    if self.args.scenario == 'mode' and elapsed >= 1 and not mode_event_sent:
                        sock.sendto(URUDPClient.pack_command(q,0,[0.0]*3,qd),('127.0.0.1',self.args.port))
                        mode_event_sent = True
                    stream_paused = (self.args.scenario == 'dropout' and elapsed >= 1) or (self.args.scenario == 'sync-dropout' and elapsed >= .15)
                    if not stream_paused:
                        sock.sendto(URUDPClient.pack_command(q,1,[0.0]*3,qd),('127.0.0.1',self.args.port))
                    next_send += 1/self.args.hz
                    delay = next_send-time.monotonic()
                    if delay < -0.1:
                        raise RuntimeError('人工 UDP 发送调度超期')
                    self.sender_stop.wait(max(0,delay))
        except Exception as exc:
            self.sender_error = str(exc)

    def _observe(self):
        try:
            self._observe_round()
        except Exception as exc:
            self.append_message('联调中止：'+str(exc))
            self.collector.update_metadata({'manual_error':str(exc)})
            self.terminal_requested = True
            self.request_impedance_stop()

    def _observe_round(self):
        now = time.monotonic()
        state = self.get_robot_snapshot()
        if state is None:
            raise RuntimeError('人工监督反馈失效')
        self.robot_ready = state.robot_mode == 7 and state.safety_mode == 1
        displacement = [abs(a-b) for a,b in zip(state.q_actual,self.center)]
        self.peak_displacement = [max(a,b) for a,b in zip(self.peak_displacement,displacement)]
        self.peak_speed = [max(a,abs(b)) for a,b in zip(self.peak_speed,state.qd_actual)]
        if max(displacement) > 0.10 and not self.terminal_requested:
            self.terminal_requested = True
            self.request_impedance_stop()
            self.collector.update_metadata({'workspace_stop':'实际偏移达到 0.10 rad，保留已确认工作区内的制动余量'})
        if self.sender_error and not self.terminal_requested:
            raise RuntimeError(self.sender_error)
        if self.started is None and self.terminal_requested:
            self.close()
            return
        if self.started is None:
            snapshot = self.udp_client.get_monitor_snapshot()
            if snapshot['command'] is not None and self._impedance_worker is None:
                self.started = now
                if self.args.scenario.startswith('sync') or self.args.scenario == 'envelope':
                    self.sync_teleop_impedance()
                    if self.args.scenario != 'envelope':
                        self.running_since = now
                else:
                    self.start_teleop_impedance()
            elif now > self.launch_deadline:
                raise RuntimeError('未收到首个本机 UDP 目标')
            return
        if self.args.scenario == 'envelope' and self.envelope_stage != 'tracking' and not self.impedance_busy():
            if not self._teleop_result.startswith('定位完成'):
                self.terminal_requested = True
            elif self.envelope_stage == 'sync':
                self.collector.update_metadata({'positioned_q':list(state.q_actual)})
                self.envelope_stage = 'ready'
                self.envelope_ready_time = now
                return
            elif now-self.envelope_ready_time < .10:
                return
            else:
                self.envelope_stage = 'tracking'
                self.start_teleop_impedance()
                return
        status = self.rtde_controller.get_impedance_status()
        phase = self._teleop_sync['phase'] if self._teleop_sync else status['phase']
        if phase != self.previous_phase:
            self.previous_phase = phase
            self.append_message(f'阶段：{phase}；错误：{status.get("last_error")}')
        if status.get('running') and self.running_since is None:
            self.running_since = now
        if self.running_since is not None:
            elapsed = now-self.running_since
            if self.args.scenario == 'hidden':
                if elapsed >= .5 and not self.hidden_once:
                    self.teleop_impedance_window.hide()
                    self.hidden_once = True
                    self.refresh_impedance()
                    self.collector.update_metadata({'hidden_entry_text':self.pushButton_TeleopImpedance.text()})
                if elapsed >= 1.5 and not self.reopened:
                    self.on_teleop_impedance()
                    self.reopened = True
            if self.args.scenario == 'sync-cancel' and elapsed >= .15 and not self.terminal_requested:
                self.terminal_requested = True
                self.stop_teleop_impedance()
            if elapsed >= self.args.duration and not self.terminal_requested:
                self.terminal_requested = True
                if self.args.scenario in ('global-stop','latest'):
                    self.ui.pushButton_Stop.click()
                else:
                    self.stop_teleop_impedance()
        if now-self.started > 15+self.args.duration and not self.terminal_requested:
            self.terminal_requested = True
            self.request_impedance_stop()
        if phase == 'STOP_UNCONFIRMED':
            self.timer.stop()
            self.collector.update_metadata({'stop_unconfirmed':status})
            self.append_message('停止未确认；窗口和连接保留，请现场处理后显式停止。')
            return
        if not self.impedance_busy():
            if self.finished_since is None:
                self.finished_since = now
                self.collector.update_metadata({'terminal':status, 'operation_result':self._teleop_result})
            # 停止后继续来包一秒，观察是否保持空闲。
            if now-self.finished_since >= 1:
                self.collector.update_metadata({'peak_displacement_rad':self.peak_displacement,
                    'peak_actual_speed_rad_s':self.peak_speed,'no_auto_restart':self.control_source=='idle',
                    'final_q':list(state.q_actual),'completed_at':time.time()})
                self.teleop_impedance_window.grab().save(str(self.args.output.with_suffix('.png')))
                self.close()

    def closeEvent(self, event):
        if self.defer_until_impedance_stopped(self.close):
            event.ignore()
            return
        self.timer.stop()
        self._impedance_timer.stop()
        self.sender_stop.set()
        if self.sender is not None:
            self.sender.join(1)
        self.udp_client.stop()
        self.rtde_controller.close()
        from UR_Utils.control_stop import confirm_stopped
        stopped = confirm_stopped(self.args.robot_ip, deadline=time.monotonic()+4)
        self.collector.update_metadata({"final_stop_confirmation": stopped})
        self.realtime_client.close()
        self.teleop_impedance_window.hide()
        self.teleop_impedance_window.visualizer.close()
        self.impedance_tuning_window.hide()
        self.robot_visualizer.close()
        event.accept()
        QtWidgets.QApplication.instance().quit()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot-ip',required=True)
    parser.add_argument('--workspace-state',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--scenario',required=True,choices=('follow','dropout','mode','hidden','global-stop','envelope','sync','sync-cancel','sync-dropout','latest','combined'))
    parser.add_argument('--duration',type=float,default=4)
    parser.add_argument('--hz',type=float,default=200)
    parser.add_argument('--port',type=int,default=5005)
    parser.add_argument('--axis',type=int,choices=(1,2),default=1,help='包络场景目标轴：J1 跨 0.18 rad，J2 跨 0.14 rad；其他场景使用 J1')
    args=parser.parse_args()
    if not 0 < args.duration <= 10 or not 1 <= args.hz <= 2000:
        raise ValueError('单轮时长须为 0～10 秒，UDP 频率须为 1～2000 Hz')
    if args.output.exists():
        raise FileExistsError('本轮记录已存在，请使用新的记录路径')
    workspace=json.loads(args.workspace_state.read_text())
    if workspace['robot_address'] != args.robot_ip:
        raise ValueError('本轮地址与已确认工作区不一致')
    center=workspace['samples'][-1]['actual_q']
    before=read_robot_state(args.robot_ip)
    require_stopped(before,0.01)
    if max(abs(a-b) for a,b in zip(before['samples'][-1]['actual_q'],center)) > .10:
        raise ValueError('当前位置已超出本联调入口允许的启动偏移')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    configure_high_dpi()
    app=QtWidgets.QApplication(sys.argv)
    window=ManualWindow(args,center,before)
    for sig in (signal.SIGINT,signal.SIGTERM):
        signal.signal(sig,lambda *_: window.close())
    window.show()
    print('现场单轮开始：'+args.scenario+'；已确认每关节 ±0.15 rad；实际偏移 0.10 rad 请求停止。',flush=True)
    return app.exec_()


if __name__=='__main__':
    raise SystemExit(main())
