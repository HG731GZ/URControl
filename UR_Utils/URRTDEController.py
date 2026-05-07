from __future__ import annotations

import time
import threading
from typing import Optional, Sequence, Literal

import numpy as np

import rtde_control
import rtde_io
import rtde_receive
from UR_Utils.ur_pose_math import (
    apply_pose_delta,
    is_pose_reached,
    rate_limit_tcp_pose,
    twist_to_base,
)


class URRTDEController:
    """
    基于 ur_rtde 的 UR5e 遥操作控制封装。

    暴露接口：
    1. track_joint(q, dq_max)：闭环跟踪关节角 q，并限制每个关节速度 dq_max
    2. track_tcp_pose(pose, v_max, w_max)：闭环跟踪 TCP pose，并限制末端速度/角速度
    3. move_joint_delta(delta_q, dq_max)：调用时读取当前实际关节角 q_actual，目标为 q_actual + delta_q
    4. move_tcp_delta(delta_pose, dx_max, dq_max, frame)：以当前实际 TCP pose 为基准生成新目标，并用 servoL 限速跟踪
    5. moveL(delta_pose, speed, acceleration, frame)：以当前实际 TCP pose 为基准执行线性运动
    6. speedJ(qd, acceleration, time_s)：关节速度控制
    7. speedL(xd, acceleration, time_s, frame)：TCP 速度控制
    8. set_speed_slider(speed)：设置控制柜 Speed Slider

    pose 格式：
        [x, y, z, rx, ry, rz]
        x/y/z 单位 m，rx/ry/rz 为旋转向量，单位 rad。

    重要行为：
        - move_joint_delta() 每次调用都会立即读取实际关节角，并生成新的目标关节角。
        - move_tcp_delta() 每次调用都会立即读取实际 TCP pose，并以该实际位姿为增量基准。
        - 关节目标由控制线程用 servoJ 按 dq_max 进行限速跟踪。
        - TCP 目标由控制线程用 servoL 跟踪，可按平动速度模长和角速度模长限速。
        - speedJ()/speedL() 会启动速度 watchdog；超时未刷新速度命令时自动 speedStop()。
        - 如果 auto_start_on_command=True，则首次调用 track_*()/move_*_delta() 时会自动 start()。

    注意：
        - RTDEControlInterface 不是线程安全的，因此本类用 _rtde_c_lock 串行化所有 rtde_c 调用。
        - 外部不要直接调用 self.rtde_c.servoJ / servoL / moveL / speedJ / speedL /
          moveJ / getInverseKinematics 等控制接口。
    """

    def __init__(
        self,
        robot_ip: str,
        frequency: float = 500.0,
        default_dq_max: float | Sequence[float] = 1,
        lookahead_time: float = 0.08,
        gain: float = 300.0,
        servo_speed: float = 1,
        servo_acceleration: float = 0.5,
        servo_stop_acc: float = 10.0,
        joint_tolerance: float = 1e-3,
        use_safety_check: bool = True,
        use_rtde_safety_check: bool = False,
        joint_position_limit: float | Sequence[float] = 2.0 * np.pi,
        auto_start_on_command: bool = True,
        speed_watchdog_timeout: Optional[float] = 0.2,
        verbose: bool = False,
    ):
        self.robot_ip = robot_ip
        self.frequency = float(frequency)
        self.dt = 1.0 / self.frequency

        self.lookahead_time = float(lookahead_time)
        self.gain = float(gain)
        self.servo_speed = float(servo_speed)
        self.servo_acceleration = float(servo_acceleration)
        self.servo_stop_acc = float(servo_stop_acc)
        self.joint_tolerance = float(joint_tolerance)
        self.use_safety_check = bool(use_safety_check)
        # 默认不再调用 ur_rtde 的 isJointsWithinSafetyLimits/isPoseWithinSafetyLimits
        # 作为目标预检查。该接口会受 URSim/PolyScope 当前安全配置、TCP、
        # 安全平面和姿态偏差限制影响，可能把常用 home_q 判为 False。
        # 如果确实需要调用 UR 控制器内部安全检查，可显式设为 True。
        self.use_rtde_safety_check = bool(use_rtde_safety_check)
        self._joint_position_limit = self._parse_joint_position_limit(joint_position_limit)
        self.auto_start_on_command = bool(auto_start_on_command)
        self.speed_watchdog_timeout = self._parse_optional_positive_float(
            speed_watchdog_timeout,
            "speed_watchdog_timeout",
        )

        self.rtde_c = rtde_control.RTDEControlInterface(robot_ip, self.frequency)
        self.rtde_r = rtde_receive.RTDEReceiveInterface(robot_ip, self.frequency, [], verbose)
        self.rtde_io = rtde_io.RTDEIOInterface(robot_ip)

        # RTDE 连接建立后，立即保存一次初始实际状态。
        # 用于首次状态查询时兜底。
        self._initial_actual_q: Optional[np.ndarray] = None
        self._initial_actual_tcp_pose: Optional[np.ndarray] = None

        try:
            q0 = np.asarray(self.rtde_r.getActualQ(), dtype=float)
            pose0 = np.asarray(self.rtde_r.getActualTCPPose(), dtype=float)

            if q0.shape == (6,) and np.all(np.isfinite(q0)):
                self._initial_actual_q = q0.copy()

            if pose0.shape == (6,) and np.all(np.isfinite(pose0)):
                self._initial_actual_tcp_pose = pose0.copy()

        except Exception:
            # 初始化阶段读取失败不直接报错，后续 start() 里还会再读取一次。
            pass

        self._lock = threading.Lock()
        self._rtde_c_lock = threading.RLock()
        self._running = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._dq_max = self._parse_dq_max(default_dq_max)

        self._target_q: Optional[np.ndarray] = None

        self._cmd_seq = 0
        self._last_error: Optional[str] = None

        self._last_actual_q: Optional[np.ndarray] = (
            None if self._initial_actual_q is None else self._initial_actual_q.copy()
        )
        self._last_commanded_q: Optional[np.ndarray] = (
            None if self._initial_actual_q is None else self._initial_actual_q.copy()
        )
        self._last_target_q: Optional[np.ndarray] = (
            None if self._initial_actual_q is None else self._initial_actual_q.copy()
        )

        self._last_actual_tcp_pose: Optional[np.ndarray] = (
            None if self._initial_actual_tcp_pose is None else self._initial_actual_tcp_pose.copy()
        )
        self._last_target_tcp_pose: Optional[np.ndarray] = (
            None if self._initial_actual_tcp_pose is None else self._initial_actual_tcp_pose.copy()
        )

        # TCP 增量命令在外部 API 调用时先生成 target_pose；
        # servoL 和可选安全检查统一放在控制线程里调用，避免多个线程同时访问 RTDEControlInterface。
        self._target_tcp_pose: Optional[np.ndarray] = None
        self._target_tcp_dx_max: Optional[float] = None
        self._target_tcp_dq_max: Optional[float] = None
        self._pending_tcp_cmd_seq: Optional[int] = None
        self._pending_tcp_servo_deadline: Optional[float] = None

        self._target_reached: bool = True

        self._loop_count = 0
        self._servo_count = 0
        self._last_servo_time: Optional[float] = None
        self._last_status_time: float = 0.0

        self._lifecycle_lock = threading.RLock()
        self._is_shutdown = False

        self._speed_watchdog_stop = threading.Event()
        self._speed_watchdog_thread: Optional[threading.Thread] = None
        self._speed_watchdog_active = False
        self._speed_watchdog_deadline: Optional[float] = None
        self._last_speed_command_kind: Optional[str] = None

    # ------------------------------------------------------------------
    # 对外接口
    # ------------------------------------------------------------------

    def start(self, force_reupload: bool = True) -> bool:
        """
        启动或恢复 RTDE 控制线程。

        这个函数被设计成“可重复调用且无风险”：
            1. 如果当前没有运行控制线程，则恢复/上传 RTDE control script 并启动线程；
            2. 如果当前已经在运行，默认也会先安全停止旧线程，再 reuploadScript()，最后重启线程；
            3. 如果之前被外部 URScript 顶掉，下一次直接调用 start() 即可恢复；
            4. 不会销毁 RTDE 连接。

        参数：
            force_reupload:
                True:
                    默认行为。每次 start() 都会执行一次安全恢复流程：
                    stop old thread -> servoStop -> reuploadScript -> start new thread。
                    适合你的使用习惯：需要遥操作时 start()，被外部脚本顶掉后再次 start()。
                False:
                    如果控制线程已经正常运行，则直接返回 True，不重启线程。
                    适合确认已经运行但不希望产生任何短暂停顿的场景。

        返回：
            True  启动/恢复成功；
            False 启动/恢复失败，错误信息可通过 get_status()["last_error"] 查看。
        """
        with self._lifecycle_lock:
            if self._is_shutdown:
                raise RuntimeError("Controller has been shutdown. Please create a new object.")

            self._cancel_speed_watchdog()

            thread_alive = self._thread is not None and self._thread.is_alive()

            with self._lock:
                last_error = self._last_error

            # 不强制恢复时，如果当前线程已经正常运行，则 start() 幂等返回。
            if thread_alive and last_error is None and not force_reupload:
                return True

            # 只要需要恢复，就先停止旧控制线程。
            if thread_alive:
                self._running.clear()
                self._thread.join(timeout=2.0)
                if self._thread.is_alive():
                    with self._lock:
                        self._last_error = "Old control thread did not stop within timeout"
                    return False
                self._thread = None

            # 停止当前 servo。RTDE control script 已被外部脚本顶掉时，这里可能报错，忽略即可。
            try:
                with self._rtde_c_lock:
                    try:
                        self.rtde_c.speedStop(self.servo_stop_acc)
                    except Exception:
                        pass
                    self.rtde_c.servoStop(self.servo_stop_acc)
            except Exception:
                pass

            # 恢复机器人端 RTDE control script。
            if force_reupload:
                try:
                    with self._rtde_c_lock:
                        ok = self.rtde_c.reuploadScript()
                    if not ok:
                        with self._lock:
                            self._last_error = "reuploadScript() returned False"
                        return False
                except Exception as exc:
                    with self._lock:
                        self._last_error = f"reuploadScript() failed: {repr(exc)}"
                    return False

            # 用当前实际关节角重置内部命令状态，避免恢复后突然跳变。
            try:
                q_now = np.asarray(self.rtde_r.getActualQ(), dtype=float)
                pose_now = np.asarray(self.rtde_r.getActualTCPPose(), dtype=float)
            except Exception as exc:
                with self._lock:
                    self._last_error = f"Failed to read actual state before start: {repr(exc)}"
                return False

            with self._lock:
                if self._initial_actual_q is None:
                    self._initial_actual_q = q_now.copy()

                if self._initial_actual_tcp_pose is None:
                    self._initial_actual_tcp_pose = pose_now.copy()
                self._target_q = None
                self._last_actual_q = q_now.copy()
                self._last_commanded_q = q_now.copy()
                self._last_target_q = q_now.copy()
                self._last_actual_tcp_pose = pose_now.copy()
                self._last_target_tcp_pose = pose_now.copy()
                self._target_tcp_pose = None
                self._target_tcp_dx_max = None
                self._target_tcp_dq_max = None
                self._pending_tcp_cmd_seq = None
                self._pending_tcp_servo_deadline = None
                self._target_reached = True
                self._last_error = None
                self._cmd_seq += 1

            # 启动新的控制线程。
            self._running.set()
            self._thread = threading.Thread(
                target=self._control_loop,
                name="URRTDEControlLoop",
                daemon=True,
            )
            self._thread.start()
            time.sleep(max(0.02, 5.0 * self.dt))

            return True

    def stop(self, stop_script: bool = False) -> None:
        """
        停止 RTDE 控制线程，但不销毁 RTDE 连接。

        stop_script=False:
            普通暂停。

        stop_script=True:
            同时停止机器人端 RTDE control script。
            适合接下来要发送外部 URScript 的场景。
        """
        with self._lifecycle_lock:
            self._cancel_speed_watchdog()
            self._running.clear()

            if self._thread is not None:
                self._thread.join(timeout=2.0)
                self._thread = None

            try:
                with self._rtde_c_lock:
                    try:
                        self.rtde_c.speedStop(self.servo_stop_acc)
                    except Exception:
                        pass
                    self.rtde_c.servoStop(self.servo_stop_acc)
            except Exception:
                pass

            if stop_script:
                try:
                    with self._rtde_c_lock:
                        self.rtde_c.stopScript()
                except Exception:
                    pass

            try:
                q_now = np.asarray(self.rtde_r.getActualQ(), dtype=float)
                pose_now = np.asarray(self.rtde_r.getActualTCPPose(), dtype=float)
            except Exception:
                q_now = None
                pose_now = None

            with self._lock:
                self._target_reached = True
                self._target_q = None

                if q_now is not None:
                    self._last_commanded_q = q_now.copy()
                    self._last_target_q = q_now.copy()
                    self._last_actual_q = q_now.copy()

                if pose_now is not None:
                    self._last_actual_tcp_pose = pose_now.copy()
                    self._last_target_tcp_pose = pose_now.copy()

                self._target_tcp_pose = None
                self._target_tcp_dx_max = None
                self._target_tcp_dq_max = None
                self._pending_tcp_cmd_seq = None
                self._pending_tcp_servo_deadline = None
                self._last_error = None

    def is_running(self) -> bool:
        """返回控制线程是否正在运行。"""
        return self._thread is not None and self._thread.is_alive()

    def shutdown(self) -> None:
        """
        最终关闭控制器。
        程序退出时才调用。
        调用后，这个对象不建议再 start()。
        """
        with self._lifecycle_lock:
            if self._is_shutdown:
                return

            self.stop(stop_script=True)
            self._speed_watchdog_stop.set()
            if self._speed_watchdog_thread is not None:
                self._speed_watchdog_thread.join(timeout=1.0)
                self._speed_watchdog_thread = None

            try:
                with self._rtde_c_lock:
                    self.rtde_c.disconnect()
            except Exception:
                pass

            try:
                self.rtde_r.disconnect()
            except Exception:
                pass

            try:
                self.rtde_io.disconnect()
            except Exception:
                pass

            self._is_shutdown = True

    def close(self) -> None:
        """为了兼容 with 写法，close 仍然表示最终关闭。"""
        self.shutdown()

    def track_joint(
        self,
        q: Sequence[float],
        dq_max: Optional[float | Sequence[float]] = None,
    ) -> None:
        """
        闭环跟踪输入关节角 q。

        参数：
            q:
                目标关节角，长度 6，单位 rad。
            dq_max:
                关节限速。
                可以是一个标量，例如 0.5，表示每个关节最大 0.5 rad/s；
                也可以是长度 6 的列表，例如 [0.5, 0.5, 0.4, 0.8, 0.8, 1.0]。
        """
        q = self._parse_vec6(q, "q")
        new_dq_max = self._parse_dq_max(dq_max) if dq_max is not None else None

        self._ensure_started_for_command()

        with self._lock:
            self._target_q = q.copy()
            self._target_tcp_pose = None
            self._target_tcp_dx_max = None
            self._target_tcp_dq_max = None
            self._pending_tcp_cmd_seq = None
            self._pending_tcp_servo_deadline = None
            self._last_target_q = q.copy()
            self._last_target_tcp_pose = None
            if new_dq_max is not None:
                self._dq_max = new_dq_max
            self._target_reached = False
            self._cmd_seq += 1
            self._last_error = None

    def move_joint_delta(
        self,
        delta_q: Sequence[float],
        dq_max: Optional[float | Sequence[float]] = None,
    ) -> np.ndarray:
        """
        关节增量控制。

        每次调用本函数时立即读取当前实际关节角：
            q_actual = getActualQ()
            q_target = q_actual + delta_q

        也就是说，快速循环调用时，目标会随着机器人实际运动持续向前滚动；
        如果两次调用间隔较大，则机器人会先到达上一次目标并等待下一次命令。

        返回：
            本次调用生成的目标关节角 q_target，单位 rad。
        """
        delta_q = self._parse_vec6(delta_q, "delta_q")
        new_dq_max = self._parse_dq_max(dq_max) if dq_max is not None else None

        self._ensure_started_for_command()

        q_now = np.asarray(self.rtde_r.getActualQ(), dtype=float)
        target_q = q_now + delta_q

        with self._lock:
            self._target_q = target_q.copy()
            self._target_tcp_pose = None
            self._target_tcp_dx_max = None
            self._target_tcp_dq_max = None
            self._pending_tcp_cmd_seq = None
            self._pending_tcp_servo_deadline = None
            self._last_actual_q = q_now.copy()
            self._last_target_q = target_q.copy()
            self._last_target_tcp_pose = None
            if new_dq_max is not None:
                self._dq_max = new_dq_max
            self._target_reached = False
            self._cmd_seq += 1
            self._last_error = None

        return target_q.copy()

    def track_tcp_pose(
        self,
        pose: Sequence[float],
        v_max: Optional[float] = None,
        w_max: Optional[float] = None,
    ) -> None:
        """
        闭环跟踪输入 TCP pose。

        这个接口接受 base frame 下的绝对目标位姿，不做增量计算。控制线程会复用
        move_tcp_delta() 的 servoL 路径，并通过 v_max / w_max 做末端限速。

        参数：
            pose:
                目标 TCP 位姿，长度 6，[x, y, z, rx, ry, rz]。
                平移单位 m，旋转向量单位 rad。
            v_max:
                TCP 平动速度上限，单位 m/s。限制整体平动速度向量模长，
                而不是分别限制 vx/vy/vz。None 表示不限制平动速度。
            w_max:
                TCP 角速度上限，单位 rad/s。限制整体角速度向量模长，
                而不是分别限制 wx/wy/wz。None 表示不限制角速度。
        """
        target_pose = self._parse_vec6(pose, "pose")
        v_max = self._parse_optional_positive_float(v_max, "v_max")
        w_max = self._parse_optional_positive_float(w_max, "w_max")

        self._ensure_started_for_command()

        q_now = np.asarray(self.rtde_r.getActualQ(), dtype=float)
        pose_actual = np.asarray(self.rtde_r.getActualTCPPose(), dtype=float)

        with self._lock:
            self._target_q = None
            self._target_tcp_pose = target_pose.copy()
            self._target_tcp_dx_max = v_max
            self._target_tcp_dq_max = w_max
            self._last_actual_q = q_now.copy()
            self._last_actual_tcp_pose = pose_actual.copy()
            self._last_target_q = None
            self._last_target_tcp_pose = target_pose.copy()
            self._target_reached = False
            self._cmd_seq += 1
            self._pending_tcp_cmd_seq = self._cmd_seq
            self._pending_tcp_servo_deadline = time.monotonic() + max(0.2, 20.0 * self.dt)
            self._last_error = None

    def move_tcp_delta(
        self,
        delta_pose: Sequence[float],
        dx_max: Optional[float] = None,
        dq_max: Optional[float] = None,
        frame: Literal["tool", "base_add"] = "base_add",
    ) -> np.ndarray:
        """
        末端增量控制。

        每次调用本函数时立即读取当前实际 TCP pose，并以该实际位姿为基准：
            pose_actual = getActualTCPPose()
            target_pose = pose_actual +/⊕ delta_pose

        frame="base_add"：
            target_pose = actual_pose + delta_pose
            对 xyz 小位移直观；旋转向量只建议小角度使用。

        frame="tool"：
            target_pose = poseTrans(actual_pose, delta_pose)
            即 delta_pose 表示当前 TCP 坐标系下的增量。
            若 delta_pose 的旋转部分为 0，则目标姿态会保持 actual_pose 的姿态。
            控制线程随后会直接把这个 base frame 下的 target_pose 交给 servoL。

        参数：
            delta_pose:
                长度 6，[dx, dy, dz, drx, dry, drz]。
                平移单位 m，旋转单位 rad。
            dx_max:
                TCP 平动速度上限，单位 m/s。限制的是整体平动速度向量模长，
                而不是分别限制 vx/vy/vz。None 表示不限制平动速度。
            dq_max:
                TCP 角速度上限，单位 rad/s。限制的是整体角速度向量模长，
                而不是分别限制 wx/wy/wz。None 表示不限制角速度。
            frame:
                "base_add" 或 "tool"。

        返回：
            本次调用生成的目标 TCP 位姿 target_pose。
        """
        if frame not in ("tool", "base_add"):
            raise ValueError("frame must be 'tool' or 'base_add'")

        delta_pose = self._parse_vec6(delta_pose, "delta_pose")
        dx_max = self._parse_optional_positive_float(dx_max, "dx_max")
        dq_max = self._parse_optional_positive_float(dq_max, "dq_max")

        self._ensure_started_for_command()

        q_now = np.asarray(self.rtde_r.getActualQ(), dtype=float)
        pose_actual = np.asarray(self.rtde_r.getActualTCPPose(), dtype=float)

        # 不在外部 API 线程里调用 RTDEControlInterface。
        # 这里仅基于当前实际 pose 生成 servoL 需要的 base frame target_pose。
        target_pose = apply_pose_delta(
            pose_now=pose_actual,
            delta_pose=delta_pose,
            frame=frame,
        )

        with self._lock:
            self._target_q = None
            self._target_tcp_pose = target_pose.copy()
            self._target_tcp_dx_max = dx_max
            self._target_tcp_dq_max = dq_max
            self._last_actual_q = q_now.copy()
            self._last_actual_tcp_pose = pose_actual.copy()
            self._last_target_q = None
            self._last_target_tcp_pose = target_pose.copy()
            self._target_reached = False
            self._cmd_seq += 1
            self._pending_tcp_cmd_seq = self._cmd_seq
            self._pending_tcp_servo_deadline = time.monotonic() + max(0.2, 20.0 * self.dt)
            self._last_error = None

        return target_pose.copy()

    def moveL(
        self,
        delta_pose: Sequence[float],
        speed: float = 0.25,
        acceleration: float = 1.2,
        frame: Literal["tool", "base_add"] = "base_add",
        asynchronous: bool = False,
    ) -> np.ndarray:
        """
        以当前实际 TCP pose 为基准执行一次 moveL。

        frame="base_add"：
            target_pose = actual_pose + delta_pose

        frame="tool"：
            target_pose = poseTrans(actual_pose, delta_pose)
            即 delta_pose 表示当前 TCP 坐标系下的位姿增量。

        返回：
            本次调用生成并下发的目标 TCP 位姿 target_pose。
        """
        if frame not in ("tool", "base_add"):
            raise ValueError("frame must be 'tool' or 'base_add'")

        delta_pose = self._parse_vec6(delta_pose, "delta_pose")
        speed = float(speed)
        acceleration = float(acceleration)
        asynchronous = bool(asynchronous)

        if speed <= 0.0 or not np.isfinite(speed):
            raise ValueError("speed must be a positive finite scalar")
        if acceleration <= 0.0 or not np.isfinite(acceleration):
            raise ValueError("acceleration must be a positive finite scalar")

        self._prepare_direct_motion_command()

        pose_actual = np.asarray(self.rtde_r.getActualTCPPose(), dtype=float)
        target_pose = apply_pose_delta(
            pose_now=pose_actual,
            delta_pose=delta_pose,
            frame=frame,
        )

        with self._rtde_c_lock:
            ok = self.rtde_c.moveL(
                target_pose.tolist(),
                speed,
                acceleration,
                asynchronous,
            )

        if not ok:
            raise RuntimeError("moveL() returned False")

        self._record_direct_motion_state(
            actual_q=np.asarray(self.rtde_r.getActualQ(), dtype=float),
            actual_tcp_pose=np.asarray(self.rtde_r.getActualTCPPose(), dtype=float),
            target_tcp_pose=target_pose,
            reached=(not asynchronous),
        )
        return target_pose.copy()

    def speedJ(
        self,
        qd: Sequence[float],
        acceleration: float = 0.5,
        time_s: float = 0.0,
    ) -> bool:
        """
        直接调用 ur_rtde.speedJ()。

        参数：
            qd:
                长度 6 的关节速度，单位 rad/s。
            acceleration:
                关节加速度，单位 rad/s^2。
            time_s:
                函数返回前阻塞时间，单位 s。0 表示沿用 ur_rtde 默认行为。
                遥操作连续速度控制建议保持 0，并依赖本类的 speed watchdog 做超时停止。

        返回：
            ur_rtde.speedJ() 的返回值，True 表示成功。

        注意：
            speedJ 的 time_s 在实机上可能只表现为接口阻塞时间，不保证到时自动刹车。
            本类会在下发 speedJ 前启动速度 watchdog：如果超过
            speed_watchdog_timeout 没有新的 speedJ/speedL 命令刷新 deadline，
            后台线程会主动调用 speedStop()。
        """
        qd_arr = self._parse_vec6(qd, "qd")
        acceleration = float(acceleration)
        time_s = float(time_s)

        if acceleration <= 0.0 or not np.isfinite(acceleration):
            raise ValueError("acceleration must be a positive finite scalar")
        if time_s < 0.0 or not np.isfinite(time_s):
            raise ValueError("time_s must be a non-negative finite scalar")

        self._prepare_direct_motion_command()

        self._arm_speed_watchdog("speedJ")

        with self._rtde_c_lock:
            ok = bool(self.rtde_c.speedJ(qd_arr.tolist(), acceleration, time_s))

        # if not ok:
        #     raise RuntimeError("speedJ() returned False")
        # 实机上这个地方会抛出异常，但不影响使用，所以注释掉

        self._record_direct_motion_state(
            actual_q=np.asarray(self.rtde_r.getActualQ(), dtype=float),
            actual_tcp_pose=np.asarray(self.rtde_r.getActualTCPPose(), dtype=float),
            reached=False,
        )
        return ok

    def speedL(
        self,
        xd: Sequence[float],
        acceleration: float = 0.25,
        time_s: float = 0.0,
        frame: Literal["tool", "base_add"] = "base_add",
    ) -> np.ndarray:
        """
        以当前实际 TCP pose 为基准执行一次 speedL。

        frame="base_add"：
            xd 直接解释为 base 坐标系下的 TCP 速度 [vx, vy, vz, wx, wy, wz]。

        frame="tool"：
            xd 解释为当前 TCP 坐标系下的速度，并转换到 base 坐标系后再下发。

        返回：
            实际下发给 ur_rtde.speedL() 的 base 坐标系速度向量。

        注意：
            speedL 的 time_s 在实机上可能只表现为接口阻塞时间，不保证到时自动刹车。
            本类会在下发 speedL 前启动速度 watchdog：如果超过
            speed_watchdog_timeout 没有新的 speedJ/speedL 命令刷新 deadline，
            后台线程会主动调用 speedStop()。
        """
        if frame not in ("tool", "base_add"):
            raise ValueError("frame must be 'tool' or 'base_add'")

        xd = self._parse_vec6(xd, "xd")
        acceleration = float(acceleration)
        time_s = float(time_s)

        if acceleration <= 0.0 or not np.isfinite(acceleration):
            raise ValueError("acceleration must be a positive finite scalar")
        if time_s < 0.0 or not np.isfinite(time_s):
            raise ValueError("time_s must be a non-negative finite scalar")

        self._prepare_direct_motion_command()

        pose_actual = np.asarray(self.rtde_r.getActualTCPPose(), dtype=float)
        xd_base = twist_to_base(
            pose_now=pose_actual,
            twist=xd,
            frame=frame,
        )

        self._arm_speed_watchdog("speedL")

        with self._rtde_c_lock:
            ok = bool(self.rtde_c.speedL(xd_base.tolist(), acceleration, time_s))

        # if not ok:
        #     raise RuntimeError("speedL() returned False")
        # 实机上这个地方会抛出异常，但不影响使用，所以注释掉

        self._record_direct_motion_state(
            actual_q=np.asarray(self.rtde_r.getActualQ(), dtype=float),
            actual_tcp_pose=np.asarray(self.rtde_r.getActualTCPPose(), dtype=float),
            reached=False,
        )
        return xd_base.copy()

    def set_speed_slider(self, speed: float) -> bool:
        """
        设置机器人控制柜上的 Speed Slider。

        参数：
            speed:
                0 到 1 之间的比例值；1 表示 100%。

        说明：
            该接口走 RTDE IO 通道，与 RTDE control script 分离。
            通常可以在 start() 前后、以及机器人运动过程中随时调整。
        """
        speed = float(speed)
        if not np.isfinite(speed):
            raise ValueError("speed must be a finite scalar")
        if speed < 0.0 or speed > 1.0:
            raise ValueError("speed must be within [0, 1]")

        ok = bool(self.rtde_io.setSpeedSlider(speed))
        if not ok:
            raise RuntimeError(f"setSpeedSlider({speed}) returned False")
        return ok

    def get_actual_q(self) -> np.ndarray:
        """读取当前实际关节角，单位 rad。"""
        return np.asarray(self.rtde_r.getActualQ(), dtype=float)

    def get_actual_tcp_pose(self) -> np.ndarray:
        """读取当前实际 TCP pose：[x, y, z, rx, ry, rz]。"""
        return np.asarray(self.rtde_r.getActualTCPPose(), dtype=float)

    def get_target_tcp_pose(self, prefer_rtde: bool = False) -> np.ndarray:
        """
        读取目标 TCP pose。

        prefer_rtde=False 时，返回本类内部记录的最后一个目标 TCP pose；
        若尚未生成过目标 TCP pose，则返回当前实际 TCP pose。

        prefer_rtde=True 时，优先尝试 rtde_receive.getTargetTCPPose()；
        如果当前 ur_rtde 版本不支持或读取失败，则回退到内部目标 TCP pose。
        """
        if prefer_rtde:
            try:
                return np.asarray(self.rtde_r.getTargetTCPPose(), dtype=float)
            except Exception:
                pass

        with self._lock:
            pose = None if self._last_target_tcp_pose is None else self._last_target_tcp_pose.copy()

        if pose is not None:
            return pose

        return np.asarray(self.rtde_r.getActualTCPPose(), dtype=float)

    def get_target_q(self, prefer_rtde: bool = False) -> np.ndarray:
        """
        读取目标关节角。

        prefer_rtde=False 时，返回本类内部记录的最后一个目标关节角；
        若尚未生成过目标关节角，则返回当前实际关节角。

        prefer_rtde=True 时，优先尝试 rtde_receive.getTargetQ()；
        如果当前 ur_rtde 版本不支持或读取失败，则回退到内部目标关节角。
        """
        if prefer_rtde:
            try:
                return np.asarray(self.rtde_r.getTargetQ(), dtype=float)
            except Exception:
                pass

        with self._lock:
            q = None if self._last_target_q is None else self._last_target_q.copy()

        if q is not None:
            return q

        return np.asarray(self.rtde_r.getActualQ(), dtype=float)

    def get_status(self) -> dict:
        """读取控制器内部状态。"""
        with self._lock:
            return {
                "target_reached": self._target_reached,
                "last_error": self._last_error,
                "last_actual_q": None if self._last_actual_q is None else self._last_actual_q.copy(),
                "last_commanded_q": None if self._last_commanded_q is None else self._last_commanded_q.copy(),
                "last_target_q": None if self._last_target_q is None else self._last_target_q.copy(),
                "last_actual_tcp_pose": None if self._last_actual_tcp_pose is None else self._last_actual_tcp_pose.copy(),
                "last_target_tcp_pose": None if self._last_target_tcp_pose is None else self._last_target_tcp_pose.copy(),
                "target_tcp_dx_max": self._target_tcp_dx_max,
                "target_tcp_dq_max": self._target_tcp_dq_max,
                "pending_tcp_cmd_seq": self._pending_tcp_cmd_seq,
                "cmd_seq": self._cmd_seq,
                "loop_count": self._loop_count,
                "servo_count": self._servo_count,
                "last_servo_time": self._last_servo_time,
                "is_running": self.is_running(),
                "use_safety_check": self.use_safety_check,
                "use_rtde_safety_check": self.use_rtde_safety_check,
                "speed_watchdog_timeout": self.speed_watchdog_timeout,
                "speed_watchdog_active": self._speed_watchdog_active,
                "speed_watchdog_deadline": self._speed_watchdog_deadline,
                "last_speed_command_kind": self._last_speed_command_kind,
            }


    def debug_status_string(self) -> str:
        """返回一行便于打印的调试状态。"""
        st = self.get_status()
        return (
            f"running={st['is_running']}, "
            f"reached={st['target_reached']}, error={st['last_error']}, "
            f"cmd_seq={st['cmd_seq']}, loop={st['loop_count']}, servo={st['servo_count']}, "
            f"rtde_safety={st['use_rtde_safety_check']}, "
            f"speed_wd={st['speed_watchdog_active']}, "
            f"tcp_pending={st['pending_tcp_cmd_seq']}, "
            f"actual_q={None if st['last_actual_q'] is None else np.round(st['last_actual_q'], 4)}, "
            f"cmd_q={None if st['last_commanded_q'] is None else np.round(st['last_commanded_q'], 4)}, "
            f"target_q={None if st['last_target_q'] is None else np.round(st['last_target_q'], 4)}"
        )

    # ------------------------------------------------------------------
    # 控制循环
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        try:
            q_cmd = np.asarray(self.rtde_r.getActualQ(), dtype=float)
        except Exception as exc:
            self._set_error_and_stop(f"Failed to read initial q: {repr(exc)}")
            return

        tcp_cmd_pose: Optional[np.ndarray] = None
        last_control_kind: Optional[str] = None
        last_cmd_seq = -1

        while self._running.is_set():
            t_start = None

            try:
                with self._lock:
                    self._loop_count += 1

                snap = self._snapshot_command()
                target_q = snap["target_q"]
                target_pose = snap["target_tcp_pose"]
                if target_pose is not None:
                    control_kind = "tcp"
                elif target_q is not None:
                    control_kind = "joint"
                else:
                    control_kind = None

                # 没有 active target 时不要反复 initPeriod()/waitPeriod()，
                # 否则 UR 端脚本虽然显示 Running，但没有任何有效 servo 命令，
                # 也容易影响下一次切入 servoJ 的节拍。
                if control_kind is None:
                    if last_control_kind is not None:
                        with self._rtde_c_lock:
                            self.rtde_c.servoStop(self.servo_stop_acc)
                    last_control_kind = None
                    tcp_cmd_pose = None
                    now = time.time()
                    if now - self._last_status_time > 0.05:
                        actual_q = np.asarray(self.rtde_r.getActualQ(), dtype=float)
                        self._update_internal_state(
                            actual_q=actual_q,
                            commanded_q=actual_q,
                            target_q=actual_q,
                            reached=True,
                        )
                        self._last_status_time = now
                    time.sleep(self.dt)
                    continue

                # 目标持续流入时保持 q_cmd 连续。如果每个新目标都从 actual_q 重新起步，
                # servoJ 命令序列会出现跳变，在较高 dq_max 下容易触发 UR 的加速度检查。
                if last_control_kind is None or last_control_kind != control_kind:
                    q_cmd = np.asarray(self.rtde_r.getActualQ(), dtype=float)
                    if control_kind == "tcp":
                        tcp_cmd_pose = np.asarray(self.rtde_r.getActualTCPPose(), dtype=float)

                is_new_command = (
                    snap["cmd_seq"] != last_cmd_seq
                    or last_control_kind != control_kind
                )

                if last_control_kind is not None and last_control_kind != control_kind:
                    with self._rtde_c_lock:
                        self.rtde_c.servoStop(self.servo_stop_acc)

                if control_kind == "joint":
                    with self._rtde_c_lock:
                        # initPeriod()/waitPeriod() 配合使用，让 servoJ 周期与 RTDE 控制频率对齐，
                        # 而不是以 Python 最快速度循环。
                        t_start = self.rtde_c.initPeriod()

                    active_target_q = target_q.copy()

                    if is_new_command:
                        self._check_joint_hard_limits(active_target_q, "target_q")
                        if self.use_rtde_safety_check:
                            with self._rtde_c_lock:
                                if not self.rtde_c.isJointsWithinSafetyLimits(active_target_q.tolist()):
                                    raise RuntimeError(
                                        f"RTDE target_q safety check failed: {active_target_q.tolist()}"
                                    )

                    last_cmd_seq = snap["cmd_seq"]

                elif control_kind == "tcp":
                    with self._rtde_c_lock:
                        # servoL 与 servoJ 使用相同的 RTDE 周期节拍。
                        # t_start 是周期句柄，由下方的 waitPeriod() 消耗。
                        t_start = self.rtde_c.initPeriod()

                    last_cmd_seq = snap["cmd_seq"]

                    if is_new_command and self.use_rtde_safety_check:
                        with self._rtde_c_lock:
                            if not self.rtde_c.isPoseWithinSafetyLimits(target_pose.tolist()):
                                raise RuntimeError(
                                    f"RTDE target_pose safety check failed: {target_pose.tolist()}"
                                )

                    if tcp_cmd_pose is None:
                        tcp_cmd_pose = np.asarray(self.rtde_r.getActualTCPPose(), dtype=float)

                    tcp_cmd_pose = rate_limit_tcp_pose(
                        pose_cmd=tcp_cmd_pose,
                        pose_target=target_pose,
                        dx_max=snap["target_tcp_dx_max"],
                        dq_max=snap["target_tcp_dq_max"],
                        dt=self.dt,
                    )

                    with self._rtde_c_lock:
                        ok = self.rtde_c.servoL(
                            tcp_cmd_pose.tolist(),
                            self.servo_speed,
                            self.servo_acceleration,
                            self.dt,
                            self.lookahead_time,
                            self.gain,
                        )

                    if not ok:
                        if self._should_retry_tcp_servo(
                            cmd_seq=snap["cmd_seq"],
                            pending_tcp_cmd_seq=snap["pending_tcp_cmd_seq"],
                            pending_tcp_servo_deadline=snap["pending_tcp_servo_deadline"],
                        ):
                            continue
                        raise RuntimeError("servoL() returned False")

                    with self._lock:
                        self._servo_count += 1
                        self._last_servo_time = time.time()
                        if self._pending_tcp_cmd_seq == snap["cmd_seq"]:
                            self._pending_tcp_cmd_seq = None
                            self._pending_tcp_servo_deadline = None

                    actual_q = np.asarray(self.rtde_r.getActualQ(), dtype=float)
                    actual_pose = np.asarray(self.rtde_r.getActualTCPPose(), dtype=float)
                    reached = is_pose_reached(actual_pose, target_pose)

                    self._update_internal_state(
                        actual_q=actual_q,
                        commanded_q=actual_q,
                        target_q=actual_q,
                        reached=reached,
                        actual_tcp_pose=actual_pose,
                        target_tcp_pose=target_pose,
                    )

                    last_control_kind = control_kind
                    continue

                else:
                    raise RuntimeError(f"Unsupported control kind: {control_kind}")

                # Python 侧关节速度限幅。
                q_cmd = self._rate_limit_q(
                    q_cmd=q_cmd,
                    q_target=active_target_q,
                    dq_max=snap["dq_max"],
                    dt=self.dt,
                )

                # 真正下发给 servoJ 的每一帧命令也做一次轻量检查。
                # 这里检查的是本周期实际要发送的 q_cmd，而不是远处最终目标，
                # 避免常用目标被 RTDE 目标预检查误判后完全不动。
                self._check_joint_hard_limits(q_cmd, "q_cmd")

                # servoJ 底层闭环。
                with self._rtde_c_lock:
                    if self.use_rtde_safety_check:
                        if not self.rtde_c.isJointsWithinSafetyLimits(q_cmd.tolist()):
                            raise RuntimeError(f"RTDE q_cmd safety check failed: {q_cmd.tolist()}")

                    self.rtde_c.servoJ(
                        q_cmd.tolist(),
                        self.servo_speed,
                        self.servo_acceleration,
                        self.dt,
                        self.lookahead_time,
                        self.gain,
                    )

                with self._lock:
                    self._servo_count += 1
                    self._last_servo_time = time.time()

                actual_q = np.asarray(self.rtde_r.getActualQ(), dtype=float)
                actual_pose = np.asarray(self.rtde_r.getActualTCPPose(), dtype=float)
                reached = bool(np.max(np.abs(active_target_q - actual_q)) < self.joint_tolerance)

                self._update_internal_state(
                    actual_q=actual_q,
                    commanded_q=q_cmd,
                    target_q=active_target_q,
                    reached=reached,
                    actual_tcp_pose=actual_pose,
                )

                last_control_kind = control_kind

            except Exception as exc:
                # 例如外部 URScript 顶掉了 RTDE control script 时，
                # 这里会捕获 “RTDE control script is not running”。
                # 此时不要在后台无限报错，而是退出线程，等待下一次 start() 或命令自动恢复。
                self._set_error_and_stop(repr(exc))
                try:
                    with self._rtde_c_lock:
                        self.rtde_c.servoStop(self.servo_stop_acc)
                except Exception:
                    pass
                last_control_kind = None
                self._running.clear()
                break

            finally:
                if t_start is not None:
                    try:
                        with self._rtde_c_lock:
                            # 等待本 RTDE 控制周期的剩余时间。
                            self.rtde_c.waitPeriod(t_start)
                    except Exception:
                        time.sleep(self.dt)

        try:
            with self._rtde_c_lock:
                self.rtde_c.servoStop(self.servo_stop_acc)
        except Exception:
            pass

    # ------------------------------------------------------------------
    # 内部工具函数
    # ------------------------------------------------------------------


    def check_joint_safety(self, q: Sequence[float]) -> dict:
        """
        手动检查一个关节目标的安全判定结果，便于排查 URSim/PolyScope 的安全限制。

        返回字段：
            local_hard_limit_ok:
                本类的本地硬关节范围检查结果，默认每个关节绝对值不超过 2π。
            rtde_isJointsWithinSafetyLimits:
                UR 控制器内部 isJointsWithinSafetyLimits(q) 的结果。
                它会受当前安全配置、TCP、安全平面、姿态偏差限制等影响。
        """
        q_arr = self._parse_vec6(q, "q")
        local_ok = bool(np.all(np.abs(q_arr) <= self._joint_position_limit))
        result = {
            "q": q_arr.copy(),
            "local_hard_limit_ok": local_ok,
            "joint_position_limit": self._joint_position_limit.copy(),
            "rtde_isJointsWithinSafetyLimits": None,
            "rtde_error": None,
        }

        try:
            with self._rtde_c_lock:
                result["rtde_isJointsWithinSafetyLimits"] = bool(
                    self.rtde_c.isJointsWithinSafetyLimits(q_arr.tolist())
                )
        except Exception as exc:
            result["rtde_error"] = repr(exc)

        return result

    def check_pose_safety(self, pose: Sequence[float]) -> dict:
        """手动检查一个 TCP pose 在 UR 控制器内部安全检查中的结果。"""
        pose_arr = self._parse_vec6(pose, "pose")
        result = {
            "pose": pose_arr.copy(),
            "rtde_isPoseWithinSafetyLimits": None,
            "rtde_error": None,
        }
        try:
            with self._rtde_c_lock:
                result["rtde_isPoseWithinSafetyLimits"] = bool(
                    self.rtde_c.isPoseWithinSafetyLimits(pose_arr.tolist())
                )
        except Exception as exc:
            result["rtde_error"] = repr(exc)
        return result

    def _ensure_started_for_command(self) -> None:
        """
        确保控制线程已启动。

        auto_start_on_command=True 时，首次调用控制命令会自动 start(force_reupload=True)。
        如果之前 RTDE control script 被外部 URScript 顶掉导致线程退出，下一次控制命令也会自动恢复。
        """
        if self._is_shutdown:
            raise RuntimeError("Controller has been shutdown. Please create a new object.")

        thread_alive = self._thread is not None and self._thread.is_alive()
        with self._lock:
            last_error = self._last_error

        if thread_alive and last_error is None:
            return

        if not self.auto_start_on_command:
            raise RuntimeError("Control thread is not running. Call start() before sending commands.")

        if not self.start(force_reupload=True):
            raise RuntimeError(f"Failed to start URRTDEController: {self.get_status()['last_error']}")

    def _snapshot_command(self) -> dict:
        with self._lock:
            return {
                "dq_max": self._dq_max.copy(),
                "target_q": None if self._target_q is None else self._target_q.copy(),
                "target_tcp_pose": None if self._target_tcp_pose is None else self._target_tcp_pose.copy(),
                "target_tcp_dx_max": self._target_tcp_dx_max,
                "target_tcp_dq_max": self._target_tcp_dq_max,
                "pending_tcp_cmd_seq": self._pending_tcp_cmd_seq,
                "pending_tcp_servo_deadline": self._pending_tcp_servo_deadline,
                "cmd_seq": self._cmd_seq,
                "loop_count": self._loop_count,
                "servo_count": self._servo_count,
                "last_servo_time": self._last_servo_time,
                "is_running": self.is_running(),
            }

    def _arm_speed_watchdog(self, command_kind: str) -> None:
        if self.speed_watchdog_timeout is None:
            return

        self._ensure_speed_watchdog_thread()
        with self._lock:
            self._speed_watchdog_active = True
            self._speed_watchdog_deadline = time.monotonic() + self.speed_watchdog_timeout
            self._last_speed_command_kind = command_kind

    def _cancel_speed_watchdog(self) -> None:
        with self._lock:
            self._speed_watchdog_active = False
            self._speed_watchdog_deadline = None
            self._last_speed_command_kind = None

    def _ensure_speed_watchdog_thread(self) -> None:
        if self.speed_watchdog_timeout is None:
            return

        with self._lock:
            thread_alive = (
                self._speed_watchdog_thread is not None
                and self._speed_watchdog_thread.is_alive()
            )
            if thread_alive:
                return

            self._speed_watchdog_stop.clear()
            thread = threading.Thread(
                target=self._speed_watchdog_loop,
                name="URRTDESpeedWatchdog",
                daemon=True,
            )
            self._speed_watchdog_thread = thread

        thread.start()

    def _speed_watchdog_loop(self) -> None:
        poll_s = 0.01

        while not self._speed_watchdog_stop.is_set():
            with self._lock:
                active = self._speed_watchdog_active
                deadline = self._speed_watchdog_deadline

            if not active or deadline is None:
                self._speed_watchdog_stop.wait(poll_s)
                continue

            now = time.monotonic()
            remaining = deadline - now
            if remaining > 0.0:
                self._speed_watchdog_stop.wait(min(poll_s, remaining))
                continue

            with self._lock:
                if (
                    not self._speed_watchdog_active
                    or self._speed_watchdog_deadline != deadline
                ):
                    continue
                self._speed_watchdog_active = False
                self._speed_watchdog_deadline = None

            try:
                with self._rtde_c_lock:
                    self.rtde_c.speedStop(self.servo_stop_acc)
            except Exception as exc:
                with self._lock:
                    self._last_error = f"speed watchdog speedStop failed: {repr(exc)}"

    def _prepare_direct_motion_command(self) -> None:
        """
        为 moveL/speedJ/speedL 这类直接 RTDE 运动命令做准备。

        如果控制线程正在运行，先停掉，避免和后台 servo 控制竞争同一个 RTDE script。
        """
        if self._is_shutdown:
            raise RuntimeError("Controller has been shutdown. Please create a new object.")

        self._cancel_speed_watchdog()

        if self.is_running():
            self.stop(stop_script=False)

        with self._lock:
            self._target_q = None
            self._target_tcp_pose = None
            self._target_tcp_dx_max = None
            self._target_tcp_dq_max = None
            self._pending_tcp_cmd_seq = None
            self._pending_tcp_servo_deadline = None
            self._target_reached = False
            self._last_error = None
            self._cmd_seq += 1

    def _record_direct_motion_state(
        self,
        actual_q: np.ndarray,
        actual_tcp_pose: np.ndarray,
        target_q: Optional[np.ndarray] = None,
        target_tcp_pose: Optional[np.ndarray] = None,
        reached: bool = False,
    ) -> None:
        actual_q = np.asarray(actual_q, dtype=float)
        actual_tcp_pose = np.asarray(actual_tcp_pose, dtype=float)

        with self._lock:
            self._last_actual_q = actual_q.copy()
            self._last_commanded_q = actual_q.copy()
            self._last_target_q = None if target_q is None else np.asarray(target_q, dtype=float).copy()
            self._last_actual_tcp_pose = actual_tcp_pose.copy()
            if target_tcp_pose is not None:
                self._last_target_tcp_pose = np.asarray(target_tcp_pose, dtype=float).copy()
            self._target_reached = bool(reached)

    def _update_internal_state(
        self,
        actual_q: np.ndarray,
        commanded_q: np.ndarray,
        target_q: np.ndarray,
        reached: bool,
        actual_tcp_pose: Optional[np.ndarray] = None,
        target_tcp_pose: Optional[np.ndarray] = None,
    ) -> None:
        with self._lock:
            self._last_actual_q = actual_q.copy()
            self._last_commanded_q = commanded_q.copy()
            self._last_target_q = target_q.copy()
            if actual_tcp_pose is not None:
                self._last_actual_tcp_pose = np.asarray(actual_tcp_pose, dtype=float).copy()
            if target_tcp_pose is not None:
                self._last_target_tcp_pose = np.asarray(target_tcp_pose, dtype=float).copy()
            self._target_reached = reached

    def _set_error_and_stop(self, error: str) -> None:
        with self._lock:
            self._last_error = error
            self._target_q = None
            self._target_tcp_pose = None
            self._target_tcp_dx_max = None
            self._target_tcp_dq_max = None
            self._pending_tcp_cmd_seq = None
            self._pending_tcp_servo_deadline = None
            self._cmd_seq += 1

    @staticmethod
    def _should_retry_tcp_servo(
        cmd_seq: int,
        pending_tcp_cmd_seq: Optional[int],
        pending_tcp_servo_deadline: Optional[float],
    ) -> bool:
        if pending_tcp_cmd_seq != cmd_seq or pending_tcp_servo_deadline is None:
            return False

        return time.monotonic() < pending_tcp_servo_deadline


    def _check_joint_hard_limits(self, q: np.ndarray, name: str) -> None:
        """
        本地轻量关节安全检查。

        注意：这里故意不调用 isJointsWithinSafetyLimits()，因为该接口是 UR 控制器
        当前安全配置下的综合检查，在 URSim 或安全平面/TCP 配置不一致时可能把
        合法关节目标判为 False。真正需要该检查时请设置 use_rtde_safety_check=True。
        """
        if not self.use_safety_check:
            return

        q_arr = np.asarray(q, dtype=float)
        if q_arr.shape != (6,):
            raise RuntimeError(f"{name} must be a 6-dimensional vector, got shape {q_arr.shape}")
        if not np.all(np.isfinite(q_arr)):
            raise RuntimeError(f"{name} contains non-finite value: {q_arr.tolist()}")
        if np.any(np.abs(q_arr) > self._joint_position_limit):
            raise RuntimeError(
                f"{name} exceeds local joint position limit: "
                f"q={q_arr.tolist()}, limit={self._joint_position_limit.tolist()}"
            )

    @staticmethod
    def _rate_limit_q(
        q_cmd: np.ndarray,
        q_target: np.ndarray,
        dq_max: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        err = q_target - q_cmd
        max_step = dq_max * dt
        step = np.clip(err, -max_step, max_step)
        return q_cmd + step

    @staticmethod
    def _parse_vec6(x: Sequence[float], name: str) -> np.ndarray:
        arr = np.asarray(x, dtype=float)
        if arr.shape != (6,):
            raise ValueError(f"{name} must be a 6-dimensional vector, got shape {arr.shape}")
        if not np.all(np.isfinite(arr)):
            raise ValueError(f"{name} contains non-finite value")
        return arr

    @staticmethod
    def _parse_dq_max(dq_max: float | Sequence[float]) -> np.ndarray:
        arr = np.asarray(dq_max, dtype=float)

        if arr.shape == ():
            arr = np.full(6, float(arr), dtype=float)

        if arr.shape != (6,):
            raise ValueError(f"dq_max must be scalar or 6-dimensional vector, got shape {arr.shape}")

        if not np.all(np.isfinite(arr)):
            raise ValueError("dq_max contains non-finite value")

        if np.any(arr <= 0):
            raise ValueError("dq_max must be positive")

        return arr

    @staticmethod
    def _parse_optional_positive_float(value: Optional[float], name: str) -> Optional[float]:
        if value is None:
            return None

        value = float(value)
        if not np.isfinite(value):
            raise ValueError(f"{name} must be finite or None")
        if value <= 0.0:
            raise ValueError(f"{name} must be positive or None")

        return value

    @staticmethod
    def _parse_joint_position_limit(limit: float | Sequence[float]) -> np.ndarray:
        arr = np.asarray(limit, dtype=float)

        if arr.shape == ():
            arr = np.full(6, float(arr), dtype=float)

        if arr.shape != (6,):
            raise ValueError(
                f"joint_position_limit must be scalar or 6-dimensional vector, got shape {arr.shape}"
            )

        if not np.all(np.isfinite(arr)):
            raise ValueError("joint_position_limit contains non-finite value")

        if np.any(arr <= 0):
            raise ValueError("joint_position_limit must be positive")

        return arr

    def __enter__(self) -> "URRTDEController":
        if not self.start():
            raise RuntimeError(f"Failed to start URRTDEController: {self.get_status()['last_error']}")
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
