"""唯一通信进程内的阻抗控制后端；闭环只在 URScript 内执行。"""

import sys
from importlib.metadata import version as package_version
import time
from pathlib import Path
from UR_Utils import impedance_config as config
from UR_Utils import impedance_telemetry as protocol
from UR_Utils.impedance_sources import TrajectorySource
from UR_Utils.rtde_script_transport import ScriptRTDE, ScriptClient
from UR_Utils.control_stop import confirm_stopped
from impedance_logger import ImpedanceLogger


def idle_impedance_status():
    return dict(
        active=False,
        phase="IDLE",
        running=False,
        stopped_confirmed=True,
        last_error=None,
        sample=None,
        q_source=None,
        run_id=None,
        log_directory=None,
    )


class URImpedanceController:
    def __init__(
        self,
        ip,
        parameters,
        run_id,
        source,
        trajectory,
        play_hz,
        directory,
        stop_event,
        service,
        parameter_revision=None,
    ):
        self.ip, self.parameters, self.run_id = ip, config.validate(parameters), run_id
        self.source, self.trajectory, self.play_hz = source, trajectory, play_hz
        self.stop_event, self.service = stop_event, service
        self.status = {
            **idle_impedance_status(),
            "active": True,
            "phase": "PREPARING",
            "stopped_confirmed": False,
            "run_id": run_id,
            "source": source,
            "log_directory": str(directory),
        }
        self.frame = None
        self.conn = self.script_client = None
        self.uploaded = False
        self.stop_deadline = None
        self.logger = ImpedanceLogger(
            directory,
            dict(
                record_type="joint_impedance_run",
                schema_version=4 if source == "teleop" else 3,
                teleop_velocity_input={
                    "source": "UDP J1-J6",
                    "unit": "rad/s",
                    "master_filter": "none",
                    "receiver_filter": "none",
                    "input_limit": ("REF_MAX_QD_RAD_S" if self.parameters["TELEOP_REF_MODE"] == config.POSITION_VELOCITY
                                    else "zero"),
                    "reference_model": self.parameters["TELEOP_REF_MODE"],
                } if source == "teleop" else None,
                parameters=self.parameters,
                parameter_revision=parameter_revision,
                run_id=run_id,
                source=source,
                robot_ip=ip,
                play_hz=play_hz,
                python_version=sys.version,
                dependencies={
                    name: package_version(name)
                    for name in ("numpy", "UrRtde", "ur-rtde")
                },
                environment="URSim" if ip in ("localhost", "127.0.0.1") else "robot",
                time_definition=(
                    "snapshot: robot same-cycle; source: host monotonic, join by adopted_reference_seq"
                    + ("; standard.actual_TCP_force: Fx/Fy/Fz (N), Mx/My/Mz (N*m); "
                       "standard.timestamp = sample.robot_timestamp_s at snapshot observation"
                       if source == "teleop" else "")
                ),
            ),
        )

    def request_stop(self, error=None):
        if error and not self.status["last_error"]:
            self.status["last_error"] = error
        if self.stop_deadline is None:
            self.stop_deadline = time.monotonic() + self.parameters["STOP_TIMEOUT_S"]
            self.logger.record(
                "stop_requested", {"error": error, "deadline": self.stop_deadline}
            )
        self.stop_event.set()
        self.status.update(phase="STOPPING", running=False)

    def run(self):
        cfg = self.parameters
        try:
            self._execute()
        except Exception as exc:
            self.request_stop(str(exc))
        finally:
            # 停止监督独立于故障数据通道，旧的 RTDE 时间戳不参与静止确认。
            if self.uploaded:
                self.request_stop()
                try:
                    confirm_stopped(
                        self.ip,
                        deadline=self.stop_deadline,
                        speed_limit=cfg["STATIONARY_QD_RAD_S"],
                        window=cfg["STOP_CONFIRM_S"],
                        record=self.logger.record,
                    )
                    self.status.update(
                        active=False, phase="STOPPED_CONFIRMED", stopped_confirmed=True
                    )
                except Exception as exc:
                    self.status.update(
                        active=True, phase="STOP_UNCONFIRMED", stop_error=str(exc)
                    )
            else:
                self.status.update(
                    active=False, phase="CANCELLED", stopped_confirmed=True
                )
            self.status["running"] = False
            if self.conn is not None:
                self.conn.disconnect()
            if self.script_client is not None:
                self.script_client.close()
            self.logger.record("terminal", self.status.copy())
            self.logger.finish(self.status)
            self.service(self)

    def _execute(self):
        cfg = self.parameters
        self.service(self)
        if self.stop_event.is_set():
            return
        conn = self.conn = ScriptRTDE(self.ip, buffer_limit_bytes=1048576)
        conn.connect()
        version = conn.get_controller_version()
        if version[0] != 5 or version[1] < 26:
            raise RuntimeError(f"要求 PolyScope 5.26+，实际 {version}")
        packet = conn.send_input_setup(protocol.INPUT_NAMES, protocol.INPUT_TYPES)
        standard_fields = protocol.TELEOP_STANDARD if self.source == "teleop" else protocol.STANDARD
        output_fields = {**standard_fields, **protocol.OUTPUT_REGISTERS}
        conn.send_output_setup(list(output_fields), list(output_fields.values()), frequency=500)
        if not conn.send_start():
            raise RuntimeError("RTDE 拒绝同步")
        baseline = conn.receive()
        if baseline is None:
            raise RuntimeError("没有预检反馈")
        if (
            baseline.runtime_state != 1
            or baseline.robot_mode != 7
            or baseline.safety_mode != 1
        ):
            raise RuntimeError("要求程序已退出、机器人 RUNNING、NORMAL")
        if max(map(abs, baseline.actual_qd)) > cfg["STATIONARY_QD_RAD_S"]:
            raise RuntimeError("预检要求机器人静止")
        if any(
            abs(a - b) > 1e-6
            for a, b in zip(baseline.tcp_offset, cfg["EXPECTED_TCP_OFFSET"])
        ):
            raise RuntimeError("实际 TCP 与本轮配置不匹配")
        if baseline.target_speed_fraction != 1:
            raise RuntimeError("要求速度滑条及速度缩放 100%")
        anchor = list(baseline.actual_q)
        # 上传前初始化输入，收到机器人就绪锚点后才允许进入运行。
        self.logger.update_context(
            {
                "controller_version": version,
                "preflight_anchor": anchor,
                "payload": baseline.payload,
                "tcp_offset": baseline.tcp_offset,
            }
        )
        for name in protocol.INPUT_NAMES:
            setattr(packet, name, 0)
        setattr(packet, "input_int_register_24", 2)
        setattr(packet, "input_int_register_25", self.run_id)
        self._send(packet)
        self.service(self)
        if self.stop_event.is_set():
            return
        script = config.render_script(cfg, self.run_id, self.source)
        (self.logger.directory / "control.script").write_text(script)
        self.script_client = ScriptClient(
            self.ip, port=30001, timeout=1, auto_connect=True
        )
        self.uploaded = True  # 部分上传也可能已经启动程序，需要停止监督。
        self.script_client.send_script(script)
        start = time.monotonic()
        last_feedback = start
        next_target = next_heartbeat = start
        last_timestamp = baseline.timestamp
        last_cycle = -1
        last_phase = None
        backlog_since = None
        reference_seq = heartbeat = 0
        last_adopted = 0
        player = None
        ready = False
        seen_running = False
        end_since = settled_since = None
        while True:
            now = time.monotonic()
            self.service(self)
            if self.stop_event.is_set():
                self.request_stop()
                setattr(packet, "input_int_register_26", 2)
                self._send(packet)
                self._drain_terminal()
                return
            if self.logger.error:
                raise RuntimeError("日志写盘失败：" + self.logger.error)
            # 有界排空接收队列，避免把历史积压反馈当作新鲜状态。
            for _ in range(32):
                state = conn.receive_once()
                if state is None:
                    break
                if state.timestamp <= last_timestamp:
                    continue
                last_timestamp = state.timestamp
                last_feedback = now
                self.logger.record(
                    "standard",
                    {name: getattr(state, name) for name in standard_fields},
                )
                if self.source == "teleop" and state.output_int_register_24 == self.run_id:
                    adopted = state.output_int_register_47
                    if adopted > last_adopted:
                        self.logger.record("adoption", dict(reference_seq=adopted,
                                                           robot_timestamp_s=state.timestamp))
                        last_adopted = adopted
                sample = protocol.decode(state, self.run_id)
                if ready and (
                    state.safety_mode != 1
                    or state.robot_mode != 7
                    or state.target_speed_fraction != 1
                    or (seen_running and state.speed_scaling < 0.99)
                ):
                    raise RuntimeError("机器人模式/安全状态/速度缩放改变")
                if sample is not None and sample["cycle_seq"] != last_cycle:
                    last_cycle = sample["cycle_seq"]
                    self.status.update(
                        sample=sample,
                        phase=sample["phase"],
                        running=sample["phase"] in ("RUNNING", "ENDING"),
                    )
                    if sample["phase"] != last_phase:
                        self.logger.record(
                            "state_transition",
                            dict(
                                previous=last_phase,
                                current=sample["phase"],
                                cycle_seq=sample["cycle_seq"],
                            ),
                        )
                        last_phase = sample["phase"]
                    self.logger.record("sample", sample)
                    if sample.get("final"):
                        self.status["final_statistics_received"] = True
                    if sample["fault"]:
                        raise RuntimeError(sample["fault"])
                    if not ready and sample["phase"] == "ARMED":
                        anchor = sample["q_actual"]
                        self.logger.update_context({"anchor": anchor})
                        if self.source in ("trajectory", "step"):
                            bound = self.trajectory.bind(anchor, cfg, self.play_hz)
                            self.trajectory.save(
                                self.logger.directory / (
                                    "step.json" if self.source == "step" else "trajectory.csv"
                                )
                            )
                            self.logger.update_context(
                                {
                                    "file_hz": self.trajectory.sample_hz,
                                }
                            )
                            player = TrajectorySource(
                                bound,
                                self.play_hz,
                                self.run_id,
                                cfg["PLAYBACK_MAX_LAG_S"],
                                self.source,
                            )
                        ready = True
                    if sample["phase"] in ("RUNNING", "ENDING"):
                        seen_running = True
                if seen_running and state.runtime_state != 2:
                    raise RuntimeError("运行程序中断或暂停")
            if conn.last_receive_drained:
                backlog_since = None
            elif backlog_since is None:
                backlog_since = now
            elif now - backlog_since > cfg["FEEDBACK_TIMEOUT_S"]:
                raise RuntimeError("RTDE 反馈缓冲持续积压，不能确认当前状态")
            if now - last_feedback > cfg["FEEDBACK_TIMEOUT_S"]:
                raise RuntimeError("RTDE 反馈停滞")
            if not seen_running and now - start > cfg["START_TIMEOUT_S"]:
                raise RuntimeError("阻抗启动握手超时")
            error = self.script_client.poll_script_error(
                deadline=now + 0.001, max_read_bytes=65536, buffer_limit_bytes=65536
            )
            if error:
                raise RuntimeError(error)
            if now >= next_heartbeat:
                heartbeat += 1
                setattr(packet, "input_int_register_27", heartbeat)
                next_heartbeat = now + 0.02
            if ready and now >= next_target:
                if player is not None:
                    self.frame = player.sample(now, seen_running)
                frame = self.frame
                if frame is None:
                    raise RuntimeError("没有有效首目标")
                age = frame.check(self.run_id, time.monotonic(), cfg)
                if (
                    self.source != "teleop"
                    and not seen_running
                    and max(abs(a - b) for a, b in zip(frame.q_rad, anchor))
                    > cfg["START_TOLERANCE_RAD"]
                ):
                    raise RuntimeError("首目标未与实际锚点同步")
                reference_seq += 1
                ints = [
                    2,
                    self.run_id,
                    1,
                    heartbeat,
                    {"teleop": 1, "trajectory": 2, "step": 3}[self.source],
                    frame.source_seq,
                    reference_seq,
                    1,
                    int(frame.end_of_source),
                    reference_seq,
                ]
                for n, v in enumerate(ints, 24):
                    setattr(packet, f"input_int_register_{n}", v)
                for n, v in enumerate(
                    (*frame.q_rad, age, frame.trajectory_row_position), 24
                ):
                    setattr(packet, f"input_double_register_{n}", v)
                qd_used = None
                if self.source == "teleop":
                    qd_used = [0.0] * 6
                    if cfg["TELEOP_REF_MODE"] == config.POSITION_VELOCITY:
                        qd_used = [max(-limit, min(limit, speed))
                                   for speed, limit in zip(frame.qd_rad_s, cfg["REF_MAX_QD_RAD_S"])]
                    for n, speed in enumerate(qd_used, 32):
                        setattr(packet, f"input_double_register_{n}", speed)
                self.status.update(
                    q_source=list(frame.q_rad),
                    source_age_s=age,
                    reference_seq=reference_seq,
                    heartbeat_seq=heartbeat,
                    trajectory_row_position=frame.trajectory_row_position,
                )
                self.logger.record(
                    "reference",
                    dict(
                        reference_seq=reference_seq,
                        source_seq=frame.source_seq,
                        q_source=frame.q_rad,
                        qd_source_raw=frame.qd_rad_s,
                        qd_source_used=qd_used,
                        source_monotonic_s=frame.source_monotonic_s,
                        source_age_s=age,
                        row_position=frame.trajectory_row_position,
                    ),
                )
                next_target = now + 0.01
                if frame.end_of_source:
                    if end_since is None:
                        end_since = now
                    sample = self.status["sample"]
                    settled = (
                        sample
                        and max(
                            abs(a - b) for a, b in zip(frame.q_rad, sample["q_ref"])
                        )
                        < 0.0005
                        and max(map(abs, sample["qd_actual"]))
                        < cfg["STATIONARY_QD_RAD_S"]
                    )
                    if settled:
                        if settled_since is None:
                            settled_since = now
                        if now - settled_since >= cfg["END_HOLD_S"]:
                            self.status["completed_trajectory"] = True
                            self.request_stop()
                    else:
                        settled_since = None
                    if now - end_since > cfg["END_SETTLE_TIMEOUT_S"]:
                        raise RuntimeError("末点收敛超时")
            # 目标以 100 Hz 发布，心跳独立为 50 Hz；只发送完整配方。
            if now >= getattr(self, "_next_send", 0):
                send_started = time.monotonic()
                if self.source == "teleop" and reference_seq:
                    # 实际交给套接字前按包内源身份重算年龄，重复提交沿用原始收包时刻。
                    packet.input_double_register_30 = frame.check(self.run_id, send_started, cfg)
                self._send(packet)
                if self.source == "teleop" and reference_seq:
                    self.logger.record("rtde_send", dict(
                        reference_seq=reference_seq, source_seq=frame.source_seq,
                        send_start_monotonic_s=send_started, send_done_monotonic_s=time.monotonic()))
                self._next_send = now + 0.01
            time.sleep(0.0005)

    def _drain_terminal(self):
        deadline = min(self.stop_deadline, time.monotonic() + 0.15)
        while time.monotonic() < deadline:
            state = self.conn.receive_once()
            if state is not None:
                sample = protocol.decode(state, self.run_id)
                if sample and sample.get("final"):
                    self.status["sample"] = sample
                    self.status["final_statistics_received"] = True
                    self.logger.record("sample", sample)
                    return
            time.sleep(0.0005)

    def _send(self, packet):
        self.conn.send_prepared(
            self.conn.prepare_inputs(packet),
            deadline=time.monotonic() + self.parameters["SEND_TIMEOUT_S"],
        )
