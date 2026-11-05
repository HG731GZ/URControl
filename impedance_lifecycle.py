"""阻抗共享参数、窗口归属、后台交接及停止确认；隐藏窗口不停止控制。"""

from copy import deepcopy
import threading
import time

from PyQt5 import QtCore

from impedance_parameters import ImpedanceParameters
from impedance_window import ImpedanceWindow
from impedance_tuning_window import ImpedanceTuningWindow
from teleop_impedance_window import TeleopImpedanceWindow, master_target
from UR_Utils.impedance_sources import UDPImpedanceSource
from UR_Utils.URImpedanceController import idle_impedance_status
from UR_Utils.control_stop import confirm_stopped


class ImpedanceWorker(QtCore.QThread):
    def __init__(self, controller, operation, kwargs=None, parent=None):
        super().__init__(parent)
        self.controller = controller
        self.operation = operation
        self.kwargs = kwargs or {}
        self.error = None

    def run(self):
        try:
            if self.operation == 'start':
                self.controller.start_impedance(**self.kwargs)
            elif self.operation == 'sync':
                self.controller.moveJ(self.kwargs['q'], speed=0.1, asynchronous=True)
            elif self.operation == 'sync_stop':
                self.controller.stop(stop_script=True)
                cfg = self.kwargs['config']
                confirm_stopped(self.controller.robot_ip,
                                deadline=time.monotonic() + cfg['STOP_TIMEOUT_S'],
                                speed_limit=cfg['STATIONARY_QD_RAD_S'], window=cfg['STOP_CONFIRM_S'])
            else:
                self.controller.stop(stop_script=True)
        except Exception as exc:
            self.error = str(exc)


class ImpedanceIntegration:
    def init_impedance(self):
        self._impedance_runs = []
        self.impedance_parameters = ImpedanceParameters(self._parameters_locked)
        self.impedance_window = None
        self.teleop_impedance_window = None
        self.impedance_tuning_window = None
        self._impedance_worker = None
        self._impedance_adapter = None
        self._impedance_after_stop = None
        self._impedance_event = None
        self._impedance_owned = False
        self._impedance_origin = None
        self._impedance_notice = None
        self._teleop_sync = None
        self._teleop_result = '可以直接开启并连续追赶最新目标；同步为可选定位。'
        self._teleop_robot_identity = None
        self._impedance_timer = QtCore.QTimer(self)
        self._impedance_timer.timeout.connect(self.refresh_impedance)
        self._impedance_timer.start(50)

    def impedance_busy(self):
        return (self._impedance_owned or self._teleop_sync is not None
                or self._impedance_worker is not None
                or (self.rtde_controller is not None
                    and self.rtde_controller.get_impedance_status()['active']))

    def _parameters_locked(self):
        """编辑边界实时读取执行权，避免定时刷新间隙允许修改。"""
        return (self.impedance_busy() or self.control_source != 'idle'
                or self.admittance_mode != 'off' or self._control_stop_failed)

    def on_impedance(self):
        if self.impedance_window is None:
            self.impedance_window = ImpedanceWindow(self.impedance_parameters, self)
            self.impedance_window.start_requested.connect(lambda args: self.start_impedance_run(args, 'trajectory'))
            self.impedance_window.stop_requested.connect(self.request_impedance_stop)
            self.impedance_window.tuning_requested.connect(self.on_impedance_tuning)
            self.impedance_window.move_start_requested.connect(self.move_impedance_start)
        self.refresh_impedance()
        self.impedance_window.show()
        self.impedance_window.raise_()

    def on_impedance_tuning(self):
        if self.impedance_tuning_window is None:
            self.impedance_tuning_window = ImpedanceTuningWindow(self.impedance_parameters, parent=self)
            self.impedance_tuning_window.start_requested.connect(lambda args: self.start_impedance_run(args, 'tuning'))
            self.impedance_tuning_window.stop_requested.connect(self.request_impedance_stop)
        self.refresh_impedance()
        self.impedance_tuning_window.show()
        self.impedance_tuning_window.raise_()

    def on_teleop_impedance(self):
        if self.teleop_impedance_window is None:
            window = self.teleop_impedance_window = TeleopImpedanceWindow(self)
            window.start_requested.connect(self.start_teleop_impedance)
            window.sync_requested.connect(self.sync_teleop_impedance)
            window.stop_requested.connect(self.stop_teleop_impedance)
            window.visibility_changed.connect(self.refresh_impedance)
        self.teleop_impedance_window.show()
        self.teleop_impedance_window.raise_()
        self.refresh_impedance()

    def _idle_reason(self, state):
        if self.impedance_busy() or self.control_source != 'idle' or self.admittance_mode != 'off':
            return '执行权占用：' + self.control_source
        if self._control_stop_failed:
            return '上一轮停止尚未确认'
        if not self.robot_ready or self.rtde_controller is None:
            return '机器人及控制连接未就绪'
        if state is None:
            return '机器人反馈失效'
        return ''

    def _teleop_conditions(self, snapshot, state):
        cfg = self.impedance_parameters.prepared
        reason = self._idle_reason(state)
        if not reason:
            try:
                master_target(snapshot, cfg)
                if state.robot_mode != 7 or state.safety_mode != 1:
                    raise ValueError('要求机器人 RUNNING、NORMAL')
                if max(map(abs, state.qd_actual)) > cfg['STATIONARY_QD_RAD_S']:
                    raise ValueError('机器人尚未静止，不能接管')
            except ValueError as exc:
                reason = str(exc)
        start_reason = reason
        if not start_reason and self.impedance_parameters.dirty:
            start_reason = '参数有未应用修改，请在单关节窗口处理'
        return reason, start_reason

    def _teleop_message(self, message):
        self._teleop_result = message
        self.append_message('遥操作阻抗：' + message)
        if self.teleop_impedance_window:
            self.teleop_impedance_window.ui.resultLabel.setText(message)

    def start_teleop_impedance(self):
        self.start_impedance_run(dict(reference_source='teleop'), 'teleop')

    def sync_teleop_impedance(self):
        snapshot = self.udp_client.get_monitor_snapshot()
        state = self.get_robot_snapshot()
        reason, _ = self._teleop_conditions(snapshot, state)
        if reason:
            self._teleop_message(reason)
            return
        cfg = deepcopy(self.impedance_parameters.prepared)
        try:
            target = master_target(snapshot, cfg)
        except ValueError as exc:
            self._teleop_message(str(exc))
            return
        self._teleop_sync = dict(q=list(target.q_rad), frame=target.source_seq,
                                control_event_seq=snapshot['control_event_seq'],
                                revision=self.impedance_parameters.revision, config=cfg,
                                phase='SYNC_STARTING', reason='从端正在移动到已冻结目标')
        self.control_source = 'impedance_sync'
        self._impedance_origin = 'teleop'
        self.udp_control_timer.stop()
        self.jog_timer.stop()
        if self.gripper is not None:
            self.gripper.pause()
        self._teleop_message(f'开始一次定位：主端帧 {target.source_seq}，参数版本 {self.impedance_parameters.revision}')
        self._launch_impedance_worker('sync', {'q': list(target.q_rad)})
        self.refresh_impedance()

    def stop_teleop_impedance(self):
        if self._teleop_sync is not None:
            self._request_sync_stop('已取消本窗口定位')
        elif self._impedance_origin == 'teleop' and self.impedance_busy():
            self.request_impedance_stop()

    def _request_sync_stop(self, reason):
        sync = self._teleop_sync
        sync['reason'] = reason
        sync['phase'] = 'SYNC_STOPPING'
        if self._impedance_worker is None:
            self._launch_impedance_worker('sync_stop', {'config': sync['config']})
        self._teleop_result = reason + '；等待停止确认'

    def _refresh_sync(self, snapshot, state):
        sync = self._teleop_sync
        if sync['phase'] not in ('SYNC_STARTING', 'SYNC_MOVING'):
            return
        try:
            master_target(snapshot, sync['config'])
            if snapshot['control_event_seq'] != sync['control_event_seq']:
                raise ValueError(snapshot['control_event_reason'])
            if state is None or not self.robot_ready:
                raise ValueError('机器人反馈失效或状态不再就绪')
        except ValueError as exc:
            self._request_sync_stop(str(exc))
            return
        if (sync['phase'] == 'SYNC_MOVING'
                and max(abs(a-b) for a, b in zip(state.q_actual, sync['q'])) <= 0.01
                and max(map(abs, state.qd_actual)) <= sync['config']['STATIONARY_QD_RAD_S']):
            self._request_sync_stop('定位完成')

    def move_impedance_start(self, q):
        if self._idle_reason(self.get_robot_snapshot()):
            return
        try:
            self.rtde_controller.moveJ(q, speed=0.15, acceleration=0.3, asynchronous=True)
            self.control_source = 'moveJ'
            self._move_target = q
        except Exception as exc:
            self.append_message(str(exc))

    def start_impedance_run(self, kwargs, origin='trajectory'):
        state = self.get_robot_snapshot()
        reason = self._idle_reason(state)
        snapshot = None
        if origin == 'teleop':
            snapshot = self.udp_client.get_monitor_snapshot()
            _, reason = self._teleop_conditions(snapshot, state)
        if reason:
            self._start_failed(origin, reason)
            return
        try:
            if origin == 'tuning':
                self.impedance_parameters.draft = deepcopy(kwargs['config'])
                self.impedance_parameters.apply()
            config = self.impedance_parameters.freeze()
            kwargs = dict(kwargs, config=config, parameter_revision=self.impedance_parameters.effective_revision)
            if origin == 'teleop':
                kwargs['initial_reference'] = master_target(snapshot, config)
                event_seq = snapshot['control_event_seq']
                kwargs['source_started'] = lambda backend: self._start_udp_adapter(backend, event_seq)
        except ValueError as exc:
            self._start_failed(origin, str(exc))
            return
        self._impedance_event = threading.Event()
        kwargs['stop_event'] = self._impedance_event
        self.udp_control_timer.stop()
        self.jog_timer.stop()
        if self.gripper is not None:
            self.gripper.pause()
        self.control_source = 'impedance'
        self._impedance_origin = origin
        self._impedance_owned = True
        if origin == 'teleop':
            self._teleop_result = '正在准备遥操作，从实际位置连续追赶最新目标'
        self._launch_impedance_worker('start', kwargs)
        self.refresh_impedance()

    def _start_udp_adapter(self, backend, control_event_seq):
        """在后台交接中续供来源，覆盖 PREPARING/ARMED 阶段。"""
        self._impedance_adapter = UDPImpedanceSource(self.udp_client, backend, control_event_seq)
        self._impedance_adapter.start()

    def _start_failed(self, origin, message):
        if origin == 'teleop':
            self._teleop_message('启动失败：' + message)
        else:
            self.append_message('阻抗启动失败：' + message)
        if origin == 'tuning' and self.impedance_tuning_window:
            self.impedance_tuning_window.start_failed(message)

    def _launch_impedance_worker(self, operation, kwargs=None):
        worker = ImpedanceWorker(self.rtde_controller, operation, kwargs, self)
        self._impedance_worker = worker
        worker.finished.connect(self._impedance_worker_finished)
        worker.start()

    def _impedance_worker_finished(self):
        worker = self._impedance_worker
        self._impedance_worker = None
        if worker.operation == 'sync':
            if worker.error or self._teleop_sync['phase'] == 'SYNC_STOPPING':
                self._request_sync_stop(worker.error or self._teleop_sync['reason'])
            else:
                self._teleop_sync['phase'] = 'SYNC_MOVING'
        elif worker.operation == 'sync_stop':
            if worker.error:
                self._teleop_sync['phase'] = 'STOP_UNCONFIRMED'
                self._teleop_message('定位停止未确认：' + worker.error)
            else:
                sync = self._teleop_sync
                self._teleop_sync = None
                self.control_source = 'idle'
                self._teleop_message(sync['reason'] + '；已确认程序退出和连续静止')
                self._impedance_runs.append(dict(source='teleop_sync', target=sync['q'],
                                                source_seq=sync['frame'], parameter_revision=sync['revision'],
                                                parameters=sync['config'], stopped_confirmed=True,
                                                result=sync['reason']))
                self.collector.update_metadata({'impedance_runs': self._impedance_runs})
                self._after_impedance_stop()
        else:
            if worker.error:
                self._start_failed(self._impedance_origin, worker.error)
            if worker.operation == 'start':
                backend = self.rtde_controller._impedance
                if backend is not None:
                    status = backend.get_status()
                    self._impedance_runs.append(dict(run_id=backend.run_id, parameters=backend.parameters,
                                                    parameter_revision=self.impedance_parameters.effective_revision,
                                                    source=backend.reference_source, origin=self._impedance_origin,
                                                    log_directory=status.get('log_directory')))
                    self.collector.update_metadata({'impedance_runs': self._impedance_runs})
        worker.deleteLater()
        self.refresh_impedance()

    def request_impedance_stop(self):
        if self._teleop_sync is not None:
            self._request_sync_stop('已请求停止定位')
            return
        if self._impedance_event:
            self._impedance_event.set()
        if self.rtde_controller:
            self.rtde_controller.request_impedance_stop()
        if self._impedance_adapter:
            self._impedance_adapter.close()
            self._impedance_adapter = None
        if self.gripper is not None:
            self.gripper.pause()
        if (self._impedance_worker is None and self.rtde_controller
                and self.rtde_controller.get_impedance_status()['active']):
            self._launch_impedance_worker('stop')

    def defer_until_impedance_stopped(self, action):
        if not self.impedance_busy():
            return False
        self._impedance_after_stop = action
        self.request_impedance_stop()
        return True

    def _after_impedance_stop(self):
        action = self._impedance_after_stop
        self._impedance_after_stop = None
        if action:
            QtCore.QTimer.singleShot(0, action)

    def refresh_impedance(self):
        status = (self.rtde_controller.get_impedance_status() if self.rtde_controller else idle_impedance_status())
        state = self.get_robot_snapshot()
        snapshot = self.udp_client.get_monitor_snapshot()
        error = status.get('stop_error') or status.get('last_error')
        notice = (status.get('run_id'), error)
        if self._impedance_owned and error and notice != self._impedance_notice:
            self._impedance_notice = notice
            if self._impedance_origin == 'teleop':
                self._teleop_message(status['phase'] + '：' + error)
            else:
                self.append_message('阻抗：' + status['phase'] + '：' + error)
        if self._teleop_sync is not None:
            self._refresh_sync(snapshot, state)
        if self._impedance_owned and not status['active'] and self._impedance_worker is None:
            self._impedance_owned = False
            self.control_source = 'idle'
            self._udp_synced = False
            if self._impedance_runs and self._impedance_runs[-1].get('run_id') == status.get('run_id'):
                self._impedance_runs[-1]['final_status'] = status
                self.collector.update_metadata({'impedance_runs': self._impedance_runs})
            if self._impedance_adapter:
                self._impedance_adapter.close()
                self._impedance_adapter = None
            message = f"结束：{status.get('last_error') or status['phase']}；日志 {status.get('log_directory')}"
            if self._impedance_origin == 'teleop':
                self._teleop_message(message)
            else:
                self.append_message('阻抗' + message)
            if status['stopped_confirmed']:
                self._after_impedance_stop()
        owned = self.impedance_busy()
        ready = not self._idle_reason(state)
        actual_q = state.q_actual if state else None
        actual_qd = state.qd_actual if state else None
        if self.impedance_window:
            self.impedance_window.update_status(status, ready=ready,
                                               busy=self.impedance_parameters.locked,
                                               actual_q=actual_q, actual_qd=actual_qd,
                                               can_stop=owned and self._impedance_origin in ("trajectory", "tuning") and self._teleop_sync is None)
        if self.impedance_tuning_window:
            self.impedance_tuning_window.update_context(status, ready, self.impedance_parameters.locked, actual_q, actual_qd)
        if self._teleop_robot_identity != self.connected_ip:
            self._teleop_robot_identity = self.connected_ip
            if self.teleop_impedance_window:
                self.teleop_impedance_window.clear_robot()
        if self.teleop_impedance_window and self.teleop_impedance_window.isVisible():
            sync_reason, start_reason = self._teleop_conditions(snapshot, state)
            own_run = owned and self._impedance_origin == 'teleop'
            phase = self._teleop_sync['phase'] if self._teleop_sync else status['phase']
            reason = start_reason
            if own_run:
                reason = (self._teleop_sync['reason'] if self._teleop_sync
                          else status.get('stop_error') or status.get('last_error') or phase)
            self.teleop_impedance_window.update_context(
                snapshot=snapshot, actual_q=actual_q, config=self.impedance_parameters.prepared,
                revision=self.impedance_parameters.revision, dirty=self.impedance_parameters.dirty,
                reason=reason, phase='可以直接开启；同步可选',
                can_sync=not sync_reason, can_start=not start_reason, can_stop=own_run,
                result=self._teleop_result)
        if status.get('running') and status.get('source') == 'teleop':
            command = snapshot['command']
            if (command and command.mode == 1 and not snapshot['last_error']
                    and snapshot['received_monotonic'] is not None
                    and time.monotonic() - snapshot['received_monotonic'] < self.impedance_parameters.effective['SOURCE_TIMEOUT_S']
                    and self.get_gripper_feedback() is not None):
                self.gripper.set_target_position(command.q_gripper[0])
                if self.gripper.is_paused:
                    self.gripper.resume()
        elif owned and self.gripper is not None:
            self.gripper.pause()
        teleop_active = owned and self._impedance_origin == 'teleop'
        hidden = teleop_active and (self.teleop_impedance_window is None or not self.teleop_impedance_window.isVisible())
        if hidden:
            entry_text = '遥操作阻抗运行中（窗口已隐藏）'
        elif teleop_active:
            phase = self._teleop_sync['phase'] if self._teleop_sync else status['phase']
            entry_text = '遥操作阻抗 · ' + phase
        else:
            entry_text = '遥操作阻抗'
        self.pushButton_TeleopImpedance.setText(entry_text)
        self.pushButton_TeleopImpedance.setToolTip('关闭子窗口仅隐藏，控制继续。' + self._teleop_result)
        other_hidden = owned and not teleop_active and (self.impedance_window is None or not self.impedance_window.isVisible())
        self.pushButton_Impedance.setText('阻抗运行中（窗口已隐藏）' if other_hidden else '关节阻抗')
