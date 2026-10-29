"""组合已有只读遥测，保留目标层级、采样身份和有限显示缓存。"""

from collections import deque
from copy import deepcopy
import math
import time

import numpy as np

from UR_Utils import admittance_config, admittance_telemetry
from UR_Utils.ur_pose_math import matrix_to_rotvec, rotvec_to_matrix


STATE_TEXT = {
    "fresh": "新鲜", "stale": "过期", "previous": "上一轮", "waiting": "等待数据",
    "not_started": "未启动", "inapplicable": "不适用", "unmatched": "待匹配",
    "waiting_tcp": "等待 TCP 配置", "unknown_age": "未提供接收时间",
}

TIMING_FIELDS = (
    ("heartbeat_echo", "机器人心跳回显", "序号；机器人实际观察值"),
    ("heartbeat_age_us", "机器人当前心跳年龄", "μs；机器人周期累计"),
    ("heartbeat_max_interval_us", "机器人最大心跳观察间隔", "μs；本轮已观察更新间隔"),
    ("heartbeat_send_interval_s", "主机最近心跳发送间隔", "s；实际发送点单调时钟"),
    ("heartbeat_send_max_interval_s", "主机最大心跳发送间隔", "s；本轮最坏间隔"),
    ("heartbeat_send_lateness_s", "主机最近心跳发送迟到", "s；相对绝对截止时刻"),
    ("heartbeat_send_max_lateness_s", "主机最大心跳发送迟到", "s；相对绝对截止时刻"),
    ("heartbeat_deadline_miss_count", "累计错过心跳周期数", "个周期；逐次累计迟到跨过的完整周期"),
    ("processing_duration_s", "主机最近轮处理耗时", "s；收发线程实际测量"),
    ("processing_max_duration_s", "主机最大轮处理耗时", "s；收发线程实际测量"),
    ("rtde_processing_duration_s", "RTDE 最近批处理耗时", "s；读取及解析预算"),
    ("primary_processing_duration_s", "Primary 最近批处理耗时", "s；读取及解析预算"),
    ("packet_prepare_duration_s", "主机最近报文准备耗时", "s；输入快照和序列化"),
    ("packet_prepare_max_duration_s", "主机最大报文准备耗时", "s；本轮最大值"),
    ("send_duration_s", "主机最近发送耗时", "s；实际发送调用"),
    ("send_max_duration_s", "主机最大发送耗时", "s；本轮最大值"),
    ("send_warning_count", "单包发送观测超期次数", "次；组包至发送返回超过本轮观测阈值，不等于发送失败"),
    ("receive_duration_s", "主机最近接收耗时", "s；单次缓冲接收调用"),
    ("receive_max_duration_s", "主机最大接收耗时", "s；本轮最大值"),
    ("decode_duration_s", "主机最近状态解析耗时", "s；状态解析及事件提交"),
    ("decode_max_duration_s", "主机最大状态解析耗时", "s；本轮最大值"),
    ("log_submit_duration_s", "主机最近日志提交耗时", "s；入队耗时，不是写盘耗时"),
    ("log_submit_max_duration_s", "主机最大日志提交耗时", "s；本轮最大值"),
    ("last_wake_monotonic_s", "最近收发线程唤醒时刻", "s；主机单调时钟"),
    ("wake_deadline_monotonic_s", "最近收发线程唤醒期限", "s；主机单调时钟"),
    ("wake_lateness_s", "最近收发线程唤醒迟到", "s；相对所安排期限"),
    ("wake_max_lateness_s", "最大收发线程唤醒迟到", "s；本轮最大值"),
    ("rtde_buffered_bytes", "RTDE 当前未解析缓冲", "字节；读取后尚未解析的状态"),
    ("rtde_buffered_max_bytes", "RTDE 未解析缓冲高水位", "字节；本轮最大值"),
    ("state_timestamp_interval_s", "最近状态机器人时间间隔", "s；相邻状态机器人时间差"),
    ("state_host_interval_s", "最近状态主机接收间隔", "s；相邻状态接收单调时间差"),
    ("state_progress_lag_s", "当前未排空窗口推进差", "s；自最近明确排空起的主机与机器人推进差，仅观测，不是停止条件"),
    ("state_progress_max_lag_s", "最大未排空窗口推进差", "s；本轮最大值，不将跨时钟推进差等同于网络积压"),
    ("state_last_progress_monotonic_s", "最近状态时间推进的主机时刻", "s；主机单调时钟"),
    ("state_progress_rate", "机器人与主机时间推进比", "机器人时间推进 / 主机时间推进；用于观察 URSim 时钟速度"),
    ("state_progress_window_host_s", "推进比窗口的主机时间", "s；该统计窗口内主机单调时间推进"),
    ("state_progress_window_robot_s", "推进比窗口的机器人时间", "s；同一统计窗口内机器人 timestamp 推进"),
    ("rtde_socket_drained", "RTDE 本次确认已排空", "仅当 socket 当前不可读且用户缓冲为零时成立"),
    ("rtde_last_drained_monotonic_s", "RTDE 最近确认排空时刻", "s；主机单调时钟"),
    ("rtde_drain_count", "RTDE 明确排空观测次数", "次；独立于机器人时间推进速度"),
    ("rtde_backlog_duration_s", "RTDE 当前持续未排空时间", "s；与 timestamp 停滞分别按状态超时阈值停止"),
    ("rtde_backlog_max_duration_s", "RTDE 最长持续未排空时间", "s；本轮最大值，不由跨时钟累计差推算"),
    ("communication_process_pid", "导纳通信进程 PID", "本机独立通信进程；未启动时未提供"),
    ("process_worker_failed", "导纳通信进程工作失败", "失败后仍需确认机器人退出与静止"),
    ("process_status_published_monotonic_s", "通信进程快照生成时刻", "s；子进程使用本机单调时钟"),
    ("process_status_received_monotonic_s", "主程序收到进程快照时刻", "s；父进程桥接使用同一本机单调时钟"),
    ("process_status_age_s", "通信进程快照年龄", "s；取快照时刻减去子进程生成时刻，不是机器人网络年龄"),
    ("process_bridge_dropped_count", "进程日志桥接丢弃记录数", "条；独立于日志写盘队列丢弃计数"),
    ("next_heartbeat_deadline_monotonic_s", "下一心跳发送期限", "s；主机单调时钟"),
    ("next_reference_deadline_monotonic_s", "下一参考发送期限", "s；主机单调时钟"),
)

HOST_TIMING_LABELS = {
    "input_snapshot_wait": "输入参考锁等待", "input_snapshot_copy": "输入参考快照",
    "input_fields": "输入字段填充", "input_serialize": "输入配方序列化",
    "packet_total": "单包准备至发送结束",
    "packet_prepare": "报文准备", "socket_writable_wait": "等待套接字可写",
    "socket_send": "套接字发送", "send": "完整报文发送", "rtde_read": "RTDE 读取",
    "receive": "单包接收与解包", "rtde_processing": "RTDE 批处理",
    "rtde_unpack": "RTDE 解包", "decode": "状态解码",
    "state_publish": "状态发布", "state_lock_wait": "等待状态锁",
    "state_lock_hold": "持有状态锁", "log_submit": "日志提交",
    "primary": "Primary 处理", "processing": "收发轮处理", "wake_wait": "唤醒等待",
    "owner_service": "进程指令与状态交接",
}


def _observation(value, state, label, source, unit, timestamp=None, age_s=None):
    """值、来源与有效性一同传递，缺失值不以零代替。"""
    return {"value": None if value is None else list(value), "state": state, "label": label,
            "source": source, "unit": unit, "timestamp": timestamp, "age_s": age_s}


def _age(now, received):
    return None if received is None else max(0.0, now - received)


def _freshness(value, age, timeout, connected=True):
    if value is None:
        return "waiting"
    if not connected:
        return "stale"
    if age is None:
        return "unknown_age"
    return "fresh" if age <= timeout else "stale"


def _text(value):
    if value is None:
        return "未提供"
    if isinstance(value, (list, tuple)):
        return ", ".join(_text(item) for item in value)
    if isinstance(value, float):
        return f"{value:.5g}"
    return str(value)


class AdmittanceMonitor:
    """不持有通信对象；仅由 GUI 线程输入各通道的独立快照。"""

    def __init__(self):
        self.robot_samples = deque(maxlen=1200)
        self.udp_samples = deque(maxlen=1200)
        self.snapshot = None
        self._visible = True
        self._segment = {"robot": 0, "udp": 0}
        self._valid = {"robot": False, "udp": False}
        self._last_robot_key = None
        self._last_udp_key = None
        self._run_id = None
        self._robot_stream = None
        self._rate_history = {}
        self._kinematics = None
        self._kinematics_run = None
        self._kinematics_configuration = None

    def set_visible(self, visible):
        """隐藏和重新显示均建立断点，保留已有曲线与显示草稿。"""
        visible = bool(visible)
        if visible != self._visible:
            self._break("robot")
            self._break("udp")
            self._rate_history.clear()
        self._visible = visible

    def clear(self):
        """只清显示缓存，不修改任何控制状态或采集记录。"""
        self.robot_samples.clear()
        self.udp_samples.clear()
        self._break("robot")
        self._break("udp")

    def _break(self, stream):
        self._segment[stream] += 1
        self._valid[stream] = False

    def _rate(self, key, count, now):
        if count is None:
            return None
        history = self._rate_history.setdefault(key, deque(maxlen=1200))
        if history and (count < history[-1][1] or now <= history[-1][0]):
            history.clear()
        history.append((now, count))
        while len(history) > 2 and history[1][0] < now - 1.0:
            history.popleft()
        elapsed = now - history[0][0]
        return None if elapsed < 0.1 else (count - history[0][1]) / elapsed

    def _estimate_tcp(self, item, run_id, configuration):
        """每轮只使用该轮 TCP 配置，本地正解不参与控制。"""
        if item["value"] is None:
            return _observation(None, item["state"], item["label"] + " TCP 模型估算",
                                "本地 UR5e 模型估算", "pose", item["timestamp"], item["age_s"])
        if configuration is None:
            return _observation(None, "waiting_tcp", item["label"] + " TCP 模型估算",
                                "本地 UR5e 模型估算", "pose", item["timestamp"], item["age_s"])
        offset = configuration["tcp_offset"]
        if self._kinematics_run != run_id or self._kinematics_configuration != offset:
            from UR_Utils.ur5e_kinematics import UR5eKinematics
            self._kinematics = UR5eKinematics(tcp_offset=offset[:3], tcp_rotation=offset[3:],
                                             load_geometry=False)
            self._kinematics_run = run_id
            self._kinematics_configuration = list(offset)
        pose = self._kinematics.forward_kinematics(item["value"])[3]
        return _observation(pose, item["state"], item["label"] + " TCP 模型估算",
                            "本地 UR5e 模型估算（本轮 TCP）", "pose", item["timestamp"], item["age_s"])

    def update(self, status, udp_snapshot, *, run_id=None, connected=False, realtime=None,
               gripper=None, ft_sensor=None, now=None, parameters=None, parameter_revision=None,
               reference_source=None, space=None, context=None, realtime_timeout_s=0.5):
        """接收同次专用状态及独立主端快照，所有输入与结果保持 SI 单位。"""
        now = time.monotonic() if now is None else now
        if status is None:
            status = {key: None for key in (
                "parameters", "reference_source", "space", "timestamp", "state_received_monotonic",
                "q_master", "q_master_submitted", "q_nom", "q_cmd", "q_start", "actual_q", "actual_qd",
                "actual_TCP_pose", "submitted_source_seq", "adopted_source_seq", "source_seq",
                "robot_configuration", "drive", "delta_q", "delta_x", "cycle_count",
                "runtime_state", "safety_status_bits", "exit_reason", "exit_reason_text", "last_error",
                "last_reference_sent_monotonic", "last_heartbeat_sent_monotonic",
                "telemetry_layout", "page_type", "page_seq", "load",
                "speed_scaling", "target_speed_fraction", "scaling_ok")}
            status.update({key: None for key, _, _ in TIMING_FIELDS})
            status.update(active=False, phase_name="未开启", state_receive_count=0,
                          reference_send_count=0, heartbeat_send_count=0,
                          continuous={"execution": None, "timing": None}, host_timing={})
        cfg = status["parameters"] if status["parameters"] is not None else parameters
        if cfg is None:
            cfg = admittance_config.snapshot()
        active = status["active"]
        monitoring_cfg = parameters if not active and parameters is not None else cfg
        source = status["reference_source"] if status["parameters"] is not None else reference_source
        space = status["space"] if status["parameters"] is not None else space
        if run_id != self._run_id:
            self._break("robot")
            self._break("udp")
            self._rate_history.clear()
            self._run_id = run_id
        robot_age = _age(now, status["state_received_monotonic"])
        robot_state = _freshness(status["timestamp"], robot_age,
                                 cfg["ADMITTANCE_STATE_TIMEOUT_S"], connected)
        target_state = robot_state if active else ("previous" if status["q_cmd"] is not None else "not_started")
        timestamp = status["timestamp"]
        execution = status["continuous"]["execution"]
        load = status["load"]

        def page_target(page, key, label, unit="rad"):
            if page is None:
                return _observation(None, "waiting" if active else "not_started", label,
                                    "等待完整冻结页", unit)
            age = _age(now, page["received_monotonic_s"])
            state = _freshness(page[key], age, cfg["ADMITTANCE_STATE_TIMEOUT_S"], connected)
            if not active:
                state = "previous"
            source_text = (f"独立冻结页 {page['page_seq']}；模型采样 {page['model_sample_seq']}；"
                           f"脚本时刻 {page['snapshot_time_s']:.6f} s；接收年龄不代表采样年龄")
            return _observation(page[key], state, label, source_text, unit,
                                page["snapshot_time_s"], age)

        def target(key, label, applicable=True, unit="rad"):
            state = "inapplicable" if not applicable else target_state
            value = status[key] if applicable else None
            if applicable and value is None and active:
                state = "unmatched" if key == "q_master" and timestamp is not None else "waiting"
            return _observation(value, state, label, "专用 RTDE 同次状态", unit, timestamp, robot_age)

        joint = {
            "q_master_adopted": target("q_master", "机器人已采用主端目标", source == "teleop"),
            "q_nom": page_target(execution, "q_nom", "固定名义参考" if source == "fixed" else "连续限速后的名义参考"),
            "q_cmd": page_target(execution, "q_cmd", "执行器已采用关节目标"),
            "qd_cmd": page_target(execution, "qd_cmd", "执行器已采用关节速度", "rad/s"),
            "q_start": target("q_start", "固定启动锚点（机器人确认）", source == "fixed"),
        }
        if source == "fixed":
            joint["q_start"]["source"] = "本轮已确认的固定锚点上下文，不随当前采样改变"
        live = status if active or realtime is None else realtime
        live_age = robot_age if live is status else _age(now, realtime["received_monotonic"])
        live_state = robot_state if live is status else _freshness(
            realtime["timestamp"], live_age, realtime_timeout_s, connected)
        if live is realtime and "last_error" in realtime and realtime["last_error"] is not None:
            live_state = "stale"
        if not active and realtime is None and status["actual_q"] is not None:
            live_state = "previous" if connected else "stale"
        live_source = "专用 RTDE 同次状态" if live is status else "已有实时反馈（独立采样）"
        joint["actual_q"] = _observation(live["actual_q"], live_state, "实际关节角", live_source,
                                         "rad", live["timestamp"], live_age)
        joint["actual_qd"] = _observation(live["actual_qd"], live_state, "实际关节速度", live_source,
                                          "rad/s", live["timestamp"], live_age)
        errors = {}
        for key, label, left, right, applicable in (
            ("nominal_following", "主端采用与名义参考差（独立快照）", "q_master_adopted", "q_nom", source == "teleop"),
            ("command_correction", "已采用命令相对名义参考的修正", "q_cmd", "q_nom", True),
            ("tracking", "执行采用与实际反馈差（异步）", "q_cmd", "actual_q", True),
        ):
            value = None
            left_item, right_item = joint[left], joint[right]
            if applicable and left_item["value"] is not None and right_item["value"] is not None:
                value = np.asarray(left_item["value"]) - np.asarray(right_item["value"])
            state = left_item["state"] if applicable else "inapplicable"
            if value is None and active and applicable:
                state = "waiting"
            elif value is not None and right_item["state"] != "fresh":
                state = right_item["state"]
            difference_source = (left_item["source"] if key == "command_correction" else
                                 "独立冻结执行页与最新反馈/参考相减；不是同周期跟踪误差")
            errors[key] = _observation(value, state, label, difference_source, "rad",
                                      left_item["timestamp"], left_item["age_s"])

        command = None if udp_snapshot is None else udp_snapshot["command"]
        udp_age = None if udp_snapshot is None else _age(now, udp_snapshot["received_monotonic"])
        udp_state = _freshness(command, udp_age, monitoring_cfg["MASTER_FRAME_TIMEOUT_S"])
        if udp_snapshot is not None and udp_snapshot["last_error"] is not None:
            udp_state = "stale"
        received = _observation(None if source == "fixed" or command is None or command.mode != 1 else command.q_arm,
                                "inapplicable" if source == "fixed" or command is not None and command.mode != 1 else udp_state,
                                "主端最新接收目标", "UDP 有效接收", "rad",
                                None if udp_snapshot is None else udp_snapshot["received_time"], udp_age)
        submitted = _observation(status["q_master_submitted"] if source == "teleop" else None,
                                 target_state if source == "teleop" else "inapplicable",
                                 "主机已提交目标", "主机 submit_reference 保存值", "rad", None, None)
        joint.update(q_master_received=received, q_master_submitted=submitted)
        sync_error = None
        if source == "teleop" and received["state"] == "fresh" and joint["actual_q"]["state"] == "fresh":
            sync_error = np.asarray(received["value"]) - np.asarray(joint["actual_q"]["value"])
        sync_state = "inapplicable" if source == "fixed" else ("fresh" if sync_error is not None else "waiting")
        errors["sync"] = _observation(sync_error, sync_state,
                                      "主从同步误差", "UDP 与实际反馈独立采样", "rad")

        configuration = status["robot_configuration"]
        tcp = {key: self._estimate_tcp(joint[key], run_id,
               None if key == "q_master_received" and not active else configuration)
               for key in ("q_master_adopted", "q_nom", "q_cmd", "q_master_received", "q_start")}
        tcp["actual_TCP_pose"] = _observation(live["actual_TCP_pose"], live_state,
                                               "机器人反馈实际 TCP", live_source, "pose",
                                               live["timestamp"], live_age)
        model_difference = None
        if tcp["q_cmd"]["value"] is not None and status["actual_TCP_pose"] is not None:
            estimated = np.asarray(tcp["q_cmd"]["value"])
            actual = np.asarray(status["actual_TCP_pose"])
            rotation = matrix_to_rotvec(rotvec_to_matrix(estimated[3:]) @ rotvec_to_matrix(actual[3:]).T)
            model_difference = np.concatenate((estimated[:3] - actual[:3], rotation))
        errors["tcp_model_difference"] = _observation(model_difference,
            "waiting" if model_difference is None else target_state,
            "已采用 TCP 模型估算与反馈差（异步，含模型差异）", "独立执行页关节正解与最新实际 TCP；姿态为相对旋转", "pose", timestamp, robot_age)
        admittance = {
            "drive": page_target(load, "drive", "导纳驱动力（独立载荷页）", unit="joint_load" if space == "joint" else "wrench"),
            "delta_q": page_target(execution, "delta_q", "执行页关节导纳偏移") if space == "joint" else target("delta_q", "关节导纳偏移", False),
            "delta_x": target("delta_x", "末端导纳偏移", space == "cartesian", "pose"),
        }
        rates = {
            "udp_receive_hz": self._rate("udp", None if udp_snapshot is None else udp_snapshot["frame_count"], now),
            "state_receive_hz": self._rate("state", status["state_receive_count"], now),
            "reference_send_hz": self._rate("reference", status["reference_send_count"], now),
            "heartbeat_send_hz": self._rate("heartbeat", status["heartbeat_send_count"], now),
            "cycle_count_hz": self._rate("cycles", status["cycle_count"], now),
        }
        master = {
            "command": command, "state": udp_state, "age_s": udp_age,
            "frame_count": None if udp_snapshot is None else udp_snapshot["frame_count"],
            "received_monotonic": None if udp_snapshot is None else udp_snapshot["received_monotonic"],
            "received_time": None if udp_snapshot is None else udp_snapshot["received_time"],
            "sender_address": None if udp_snapshot is None else udp_snapshot["sender_address"],
            "bind_address": None if udp_snapshot is None else udp_snapshot["bind_address"],
            "last_error": None if udp_snapshot is None else udp_snapshot["last_error"],
            "submitted_source_seq": status["submitted_source_seq"] if source == "teleop" else None,
            "adopted_source_seq": status["adopted_source_seq"] if source == "teleop" else None,
            "source_seq": status["source_seq"] if source == "teleop" else None,
            "sync_max_abs_rad": None if sync_error is None else float(np.max(np.abs(sync_error))),
            "sync_tolerance_rad": monitoring_cfg["TELEOP_SYNC_TOLERANCE_RAD"],
            "gripper": deepcopy(gripper),
        }
        diagnostics = {key: deepcopy(status[key]) for key in (
            "runtime_state", "safety_status_bits", "cycle_count", "exit_reason", "exit_reason_text", "last_error",
            "state_receive_count", "state_received_monotonic", "reference_send_count", "heartbeat_send_count",
            "last_reference_sent_monotonic", "last_heartbeat_sent_monotonic",
                "telemetry_layout", "page_type", "page_seq", "load",
                "speed_scaling", "target_speed_fraction", "scaling_ok")}
        diagnostics.update({key: status[key] for key, _, _ in TIMING_FIELDS})
        diagnostics["host_timing"] = deepcopy(status["host_timing"])
        diagnostics.update(ft_sensor=deepcopy(ft_sensor), realtime=deepcopy(realtime),
                           state_age_s=robot_age, state_timeout_s=cfg["ADMITTANCE_STATE_TIMEOUT_S"],
                           continuous=deepcopy(status["continuous"]), load=deepcopy(load),
                           telemetry_layout=status["telemetry_layout"], page_type=status["page_type"],
                           page_seq=status["page_seq"], speed_scaling=status["speed_scaling"],
                           target_speed_fraction=status["target_speed_fraction"], scaling_ok=status["scaling_ok"])
        diagnostics["statistics"] = deepcopy(status.get("statistics"))
        diagnostics["statistics_final_received"] = status.get("statistics_final_received", False)
        diagnostics["statistics_validation_errors"] = deepcopy(status.get("statistics_validation_errors", []))
        snapshot = {
            "phase": status["phase_name"], "freshness": STATE_TEXT[live_state] +
                ("" if live_age is None else f" / 年龄 {live_age:.3f} s"),
            "run_identity": f"本轮 {run_id if run_id is not None else '未启动'} / 参数修订 {parameter_revision if parameter_revision is not None else '未应用'}",
            "source_time": f"机器人时间 {_text(live['timestamp'])} s；" + (
                "主端不适用" if source == "fixed" else f"UDP 年龄 {_text(udp_age)} s（独立采样）") +
                ("；执行页尚未收到" if execution is None else
                 f"；执行采用 {execution['adopted_seq']} / 帧 {execution['frame']}，页 {execution['page_seq']}；采用/反馈差为异步显示"),
            "run_id": run_id, "reference_source": source, "space": space, "active": active,
            "joint": joint, "tcp": tcp, "errors": errors, "admittance": admittance,
            "master": master, "configuration": deepcopy(configuration), "diagnostics": diagnostics,
            "rates": rates, "parameters": deepcopy(cfg), "now": now,
            "context": deepcopy(context),
        }
        self.snapshot = snapshot
        if self._visible:
            self._append_samples(snapshot, status, udp_snapshot, live, live_state)
        return snapshot

    def _append_samples(self, snapshot, status, udp_snapshot, live, live_state):
        stream = "admittance" if snapshot["active"] else "realtime"
        if stream != self._robot_stream:
            self._break("robot")
            self._robot_stream = stream
        robot_key = (stream, snapshot["run_id"], live["timestamp"])
        if live_state != "fresh":
            if self._valid["robot"]:
                self._break("robot")
        elif robot_key != self._last_robot_key:
            sample_time = status["state_received_monotonic"] if snapshot["active"] else snapshot["now"]
            if not snapshot["active"] and snapshot["diagnostics"]["realtime"] is not None:
                sample_time = snapshot["diagnostics"]["realtime"]["received_monotonic"]
            self.robot_samples.append({"time": sample_time, "run_id": snapshot["run_id"],
                "segment": self._segment["robot"], "valid": True, "space": snapshot["space"],
                "joint": deepcopy(snapshot["joint"]), "tcp": deepcopy(snapshot["tcp"]),
                "errors": deepcopy(snapshot["errors"]), "admittance": deepcopy(snapshot["admittance"])})
            self._last_robot_key = robot_key
            self._valid["robot"] = True
        received = snapshot["joint"]["q_master_received"]
        if received["state"] != "fresh":
            if self._valid["udp"]:
                self._break("udp")
        else:
            key = (snapshot["run_id"], udp_snapshot["frame_count"], udp_snapshot["received_monotonic"])
            if key != self._last_udp_key:
                self.udp_samples.append({"time": udp_snapshot["received_monotonic"], "run_id": snapshot["run_id"],
                    "segment": self._segment["udp"], "valid": True, "space": snapshot["space"],
                    "joint": {"q_master_received": deepcopy(received)}})
                self._last_udp_key = key
                self._valid["udp"] = True

    @staticmethod
    def _unit_factor(unit, axis, angle_unit, length_unit):
        angle = (180.0 / math.pi, "°") if angle_unit == "deg" else (1.0, "rad")
        length = (1000.0, "mm") if length_unit == "mm" else (1.0, "m")
        if unit == "rad":
            return angle
        if unit == "rad/s":
            return angle[0], angle[1] + "/s"
        if unit in ("pose", "twist"):
            scale, name = length if axis < 3 else angle
            return scale, name + ("/s" if unit == "twist" else "")
        if unit == "joint_load":
            return 1.0, "N·m"
        if unit == "wrench":
            return 1.0, "N" if axis < 3 else "N·m"
        return 1.0, unit

    def display_rows(self, snapshot, angle_unit="deg", length_unit="mm"):
        """仅在展示边界转换单位；旋转向量和连续关节角不折返。"""
        def cell(item, axis):
            if item["value"] is None:
                return STATE_TEXT[item["state"]]
            factor, unit = self._unit_factor(item["unit"], axis, angle_unit, length_unit)
            suffix = "" if item["state"] == "fresh" else f"（{STATE_TEXT[item['state']]}）"
            return f"{item['value'][axis] * factor:.4f} {unit}{suffix}"

        def vector(item):
            if item["value"] is None:
                return STATE_TEXT[item["state"]]
            return "; ".join(cell(item, axis) for axis in range(len(item["value"])))

        joint = snapshot["joint"]
        tcp = snapshot["tcp"]
        master = snapshot["master"]
        diag = snapshot["diagnostics"]
        cfg = snapshot["parameters"]
        diagnostic_mode = cfg["DIAGNOSTIC_MODE"]
        hold_diagnostic = diagnostic_mode == "fixed_joint_hold"
        preset_motion = diagnostic_mode == "preset_joint_motion"
        command = master["command"]
        fixed = snapshot["reference_source"] == "fixed"
        rows = {
            "joint_rows": [[f"J{i + 1}"] + [cell(joint[key], i) for key in
                ("q_master_adopted", "q_nom", "q_cmd", "actual_q")] + [cell(snapshot["errors"]["tracking"], i)] for i in range(6)],
            "tcp_rows": [[name] + [cell(tcp[key], i) for key in
                ("q_master_adopted", "q_nom", "q_cmd", "actual_TCP_pose")]
                for i, name in enumerate(("x", "y", "z", "旋转向量 rx", "旋转向量 ry", "旋转向量 rz"))],
            "master_rows": [
                ["主端接收状态", STATE_TEXT[master["state"]], f"接收年龄 {_text(master['age_s'])} s"],
                ["发送方 / 本地绑定", f"{_text(master['sender_address'])} / {_text(master['bind_address'])}", "有效 UDP 数据包"],
                ["协议模式", "未提供" if command is None else {0: "关闭", 1: "关节跟踪", 2: "关节增量", 3: "末端增量", 4: "末端速度"}[command.mode], "UDP 模式字段"],
                ["协议原始机械臂载荷", "未提供" if command is None else _text(command.q_arm), "仅关节跟踪模式表示关节目标；其他模式按协议解释"],
                ["最新接收关节目标", vector(joint["q_master_received"]), "UDP；非关节跟踪时不适用"],
                ["最新接收 TCP 模型估算", vector(tcp["q_master_received"]), "仅运行中采用本轮 TCP；不是机器人回传目标"],
                ["主机已提交目标", vector(joint["q_master_submitted"]), "提交不表示机器人已采用"],
                ["有效接收计数", _text(master["frame_count"]), "本机计数；不是主端发送序号"],
                ["已提交完整序号", "不适用" if fixed else _text(master["submitted_source_seq"]), "主机保存值"],
                ["已采用线上序号 / 完整序号", "不适用" if fixed else f"{_text(master['adopted_source_seq'])} / {_text(master['source_seq'])}", "线上取模序号 / 匹配的本机接收计数"],
                ["接收至采用计数进度", "不适用" if fixed else ("未匹配" if master["source_seq"] is None or master["frame_count"] is None else _text(master["frame_count"] - master["source_seq"])), "相同本机计数域；不是真实丢包率"],
                ["同步每轴误差", vector(snapshot["errors"]["sync"]), "UDP 与实际状态独立采样"],
                ["最大同步误差 / 容差", "不适用" if fixed else f"{_text(master['sync_max_abs_rad'])} / {_text(master['sync_tolerance_rad'])} rad", snapshot["source_time"]],
                ["主端夹爪三通道", "未提供" if command is None else _text(command.q_gripper), "UDP 原始夹爪通道"],
                ["夹爪开度输入 / 附加原始量", "未提供" if command is None else f"{_text(command.q_gripper[0])} / {_text(command.q_gripper[1:])}", "第一通道用于已有跟随；后两通道未用于接收方向控制"],
                ["主端接收墙时 / 单调时刻", f"{_text(master['received_time'])} / {_text(master['received_monotonic'])}", "主机时间；不代表单向网络延迟"],
                ["主端机械参数 / 编码器零偏 / 控制增益", "主端未提供", "从端映射后的 q_arm 不是原始编码器值"],
                ["UDP 错误", _text(master["last_error"]), "接收客户端"],
            ],
            "admittance_rows": [], "robot_configuration_rows": [], "diagnostic_rows": [],
        }
        for item in list(snapshot["errors"].values()) + list(snapshot["admittance"].values()) + [joint["q_nom"], joint["qd_cmd"], joint["actual_qd"], joint["q_start"], tcp["q_start"]]:
            rows["admittance_rows"].append([item["label"], vector(item), item["source"]])
        prefix = "JOINT" if snapshot["space"] == "joint" else "CART"
        rows["admittance_rows"].extend([
            ["柔顺坐标", cfg["CART_FRAME"], "tool_start 为启动时固定工具方向，中心在当前 TCP"],
            ["控制模型", "固定关节保持诊断（不积分）" if hold_diagnostic else
             ("预设单轴无接触运动（外力不驱动）" if preset_motion else "质量—阻尼—刚度"),
             "六轴目标保持启动锚点；所选轴仅观测" if hold_diagnostic else
             (f"J{cfg['DIAGNOSTIC_MOTION_AXIS']} 预设轨迹；J{cfg['DIAGNOSTIC_AXIS']} 观测" if preset_motion else
              ("固定启动参考，撤力回位" if fixed else "随主端移动的名义参考"))],
            ["虚拟质量 / 惯量 M", _text(cfg[prefix + "_MASS"]), "关节 kg·m²；末端前三轴 kg、后三轴 kg·m²"],
            ["阻尼 D", _text(cfg[prefix + "_DAMPING"]), "关节 N·m·s/rad；末端 N·s/m、N·m·s/rad"],
            ["恢复刚度 K", _text(cfg[prefix + "_STIFFNESS"]), "关节 N·m/rad；末端 N/m、N·m/rad"],
            ["目标载荷", _text(cfg["JOINT_TARGET_TORQUE_NM"] if snapshot["space"] == "joint" else cfg["CART_TARGET_WRENCH"]), "定点柔顺必须全为 0；关节 N·m，末端 N/N·m" if fixed else "关节 N·m；末端平移 N、转动 N·m"],
            ["连续死区", _text(cfg["JOINT_DEADBAND_NM"] if snapshot["space"] == "joint" else cfg["CART_DEADBAND"]), "原始载荷与导纳驱动力含义不同"],
        ])
        if "ADMITTANCE_RTDE_SEND_TIMEOUT_S" in cfg:
            rows["admittance_rows"].extend([
                ["主机单包观测 / 退出期限",
                 f"{cfg['ADMITTANCE_RTDE_SEND_WARNING_S'] * 1000:g} / {cfg['ADMITTANCE_RTDE_SEND_TIMEOUT_S'] * 1000:g} ms",
                 "组包与发送共用期限；观测超期累计，退出期限超期终止"],
            ])
        # 显示实际生效配置：运动轴与固定轴的守卫分别列出，避免把 J1 放宽量误认为六轴共用。
        if "SINGLE_AXIS_NON_MOTION_ACTUAL_MAX_QD_RAD_S" in cfg:
            speeds, margins = admittance_config.actual_guard_limits(cfg)
            moving_axes = ([cfg["DIAGNOSTIC_MOTION_AXIS"] - 1] if preset_motion else
                           [axis for axis, enabled in enumerate(cfg["JOINT_AXES"])
                            if enabled and not hold_diagnostic])
            prefix = "DIAGNOSTIC_MOTION" if preset_motion else "SINGLE_AXIS"
            extent = cfg["DIAGNOSTIC_MOTION_AMPLITUDE_RAD"] if preset_motion else cfg["SINGLE_AXIS_MAX_OFFSET_RAD"]
            symmetric = not preset_motion or cfg["DIAGNOSTIC_MOTION_PROFILE"] == "bidirectional_trapezoid"
            anchored_guard = cfg.get("ACTUAL_GUARD_VERSION") == 2
            envelopes = ([[-margin, margin] for margin in margins] if anchored_guard else
                         [[-margins[axis] - (extent if symmetric and axis in moving_axes else 0.),
                           margins[axis] + (extent if axis in moving_axes else 0.)] for axis in range(6)])
            rows["admittance_rows"].extend([
                ["允许运动轴", ", ".join(f"J{axis + 1}" for axis in moving_axes) or "无（固定保持）", "其余轴命令保持各自固定锚点"],
                ["各轴实际速度守卫 J1–J6", _text(speeds), "rad/s；运动轴与非运动轴独立，不是命令速度"],
                ["各轴实际锚点偏移上限 J1–J6" if anchored_guard else "各轴实际轨迹外裕量 J1–J6",
                 _text(margins), "rad；独立于命令与实际的跟踪偏差"],
                ["各轴实际位置包络 J1–J6", "; ".join(f"J{axis + 1}: [{_text(bounds[0])}, {_text(bounds[1])}]" for axis, bounds in enumerate(envelopes)), "rad；相对本轮固定锚点"],
                ["各轴命令跟踪阈值", _text(cfg["COMMAND_MAX_TRACKING_ERROR_RAD"] if hold_diagnostic else cfg[prefix + "_TRACKING_RAD"]), "rad；六轴逐帧检查，异步展示差不作该守卫判定"],
            ])
            if moving_axes:
                rows["admittance_rows"].append([
                    "运动轴命令行程 / 速度 / 加速度",
                    f"{extent:g} rad / {cfg[prefix + '_MAX_QD_RAD_S']:g} rad/s / {cfg[prefix + '_MAX_QDD_RAD_S2']:g} rad/s²",
                    "相对固定锚点；实际反馈守卫使用上方独立阈值"])
        if preset_motion and cfg.get("DIAGNOSTIC_MOTION_PROFILE") == "bidirectional_trapezoid":
            from UR_Utils.admittance_trajectory import profile_parameters
            plan = profile_parameters(cfg)
            rows["admittance_rows"].extend([
                ["双向无接触轨迹", f"0 → +{cfg['DIAGNOSTIC_MOTION_AMPLITUDE_RAD']:g} → 0 → −{cfg['DIAGNOSTIC_MOTION_AMPLITUDE_RAD']:g} → 0 rad", "各段零速起止；外载荷不生成命令"],
                ["节点 / 末尾静止", f"{cfg['DIAGNOSTIC_MOTION_NODE_HOLD_S']:g} / {cfg['DIAGNOSTIC_MOTION_TAIL_HOLD_S']:g} s", "首节点及中间节点分别停留；末尾尾段另计"],
                ["离散有效运行时长", f"{plan['duration_s']:g} s", "含逐帧限幅收敛段；启动保持、偏置与停止确认另计"],
                ["无接触输入判据", "J1 非零 drive 周期数 = 0 且峰值 = 0", "机器人逐模型周期累计；载荷页抽样不能代替完整统计"],
                ["重复与结论边界", "同候选独立运行三轮，每轮重采偏置", "URSim 不能验证真实残差、接触让位或撤力趋稳"],
            ])
        configuration = snapshot["configuration"]
        for key, label, unit in (("tcp_offset", "本轮 TCP 偏置", "m、旋转向量 rad"),
                                 ("payload", "负载质量", "kg"), ("payload_cog", "负载质心", "m"),
                                 ("payload_inertia", "负载惯量", "kg·m²"), ("timestamp", "配置机器人时间", "s")):
            rows["robot_configuration_rows"].append([label, "等待本轮配置" if configuration is None else _text(configuration[key]), unit])
        rows["robot_configuration_rows"].append(["目标 TCP 来源", "本地 UR5e 模型估算（含本轮 TCP）", "模型与机器人校准可能存在差异"])
        if master["gripper"] is not None:
            gripper_names = {"requested_target": "夹爪当前设置", "sent_target": "夹爪最近已发送目标",
                             "target_sent_monotonic": "夹爪发送单调时刻", "feedback": "夹爪反馈",
                             "feedback_received_monotonic": "夹爪反馈接收时刻", "feedback_count": "夹爪反馈计数",
                             "following_thread_active": "夹爪控制线程运行", "last_error": "夹爪通信错误",
                             "settings": "夹爪默认设置及范围", "paused": "夹爪暂停状态"}
            for key, value in master["gripper"].items():
                rows["master_rows"].append([gripper_names[key], _text(value),
                    "电流为原始寄存器值，未换算安培" if key == "feedback" else "夹爪已有线程只读快照"])
            feedback_time = master["gripper"]["feedback_received_monotonic"]
            rows["master_rows"].append(["夹爪反馈年龄", _text(_age(snapshot["now"], feedback_time)), "s；未取得反馈时不显示默认零值"])
        context = snapshot["context"]
        if context is not None:
            rows["master_rows"].extend([
                ["UDP 反馈目的地址", _text(context["feedback_destination"]), "主程序已有反馈路径"],
                ["主从同步状态", "不适用" if fixed else ("同步运动中" if context["udp_sync_pending"] else ("已同步" if context["udp_synced"] else "未同步")), "主程序同步状态"],
                ["UDP 夹爪跟随", "正在跟随" if context["gripper_following"] else "未跟随", "定点柔顺期间主端不驱动夹爪"],
            ])
            rows["diagnostic_rows"].append(["主界面速度请求", _text(context["requested_speed_percent"]), "%；请求值不代表实测速度"])
        names = {"runtime_state": "机器人运行状态", "safety_status_bits": "安全状态位", "cycle_count": "机器人周期计数",
                 "exit_reason": "退出码", "exit_reason_text": "退出原因", "last_error": "后端错误",
                 "state_receive_count": "状态实际接收计数", "state_received_monotonic": "状态主机接收时刻",
                 "reference_send_count": "参考实际发送计数", "heartbeat_send_count": "心跳实际发送计数",
                 "last_reference_sent_monotonic": "参考最近主机发送时刻", "last_heartbeat_sent_monotonic": "心跳最近主机发送时刻",
                 "state_age_s": "状态年龄", "state_timeout_s": "状态过期阈值"}
        for key, label in names.items():
            rows["diagnostic_rows"].append([label, _text(diag[key]), "主机发送成功不代表机器人收到" if "sent" in key or "send_count" in key else "专用状态快照"])
        for key, label, explanation in TIMING_FIELDS:
            rows["diagnostic_rows"].append([label, _text(diag[key]), explanation])
        for key, segment in diag["host_timing"].items():
            label = HOST_TIMING_LABELS[key]
            rows["diagnostic_rows"].extend([
                [label + " · 最近墙钟 / CPU", f"{_text(segment['last_wall_s'])} / {_text(segment['last_cpu_s'])}",
                 "s；同一次分段测量；两者差值包含等待和调度"],
                [label + " · 最大墙钟 / 同次 CPU / 最大 CPU",
                 f"{_text(segment['max_wall_s'])} / {_text(segment['max_wall_cpu_s'])} / {_text(segment['max_cpu_s'])}",
                 "s；最后一项为独立最大值，不与最大墙钟拼成一次样本"],
                [label + " · 最大墙钟起止 / 样本数",
                 f"{_text(segment['max_wall_started_monotonic_s'])} → {_text(segment['max_wall_finished_monotonic_s'])} / {_text(segment['count'])}",
                 "主机单调时刻 s；用于定位最坏段与心跳迟到的先后关系"],
            ])
        rate_names = {"udp_receive_hz": "UDP 实测有效接收速率", "state_receive_hz": "状态实测接收速率",
                      "reference_send_hz": "参考实测发送速率", "heartbeat_send_hz": "心跳实测发送速率", "cycle_count_hz": "机器人周期计数速率"}
        for key, label in rate_names.items():
            rows["diagnostic_rows"].append([label, _text(snapshot["rates"][key]), "Hz；计数差 / 主机时间差，不证明闭环性能"])
        for label, frequency in (("参考配置频率", cfg["TELEOP_REFERENCE_HZ"]),
                                 ("状态订阅配置频率", admittance_telemetry.state_frequency(cfg)),
                                 ("心跳配置频率", cfg["ADMITTANCE_HEARTBEAT_HZ"])):
            rows["diagnostic_rows"].append([label, _text(frequency), "Hz；配置值"])
        rows["diagnostic_rows"].extend([
            ["显示 / 绘图 / 采集目标频率", "20 / 10 / 20 Hz", "独立定时器；非闭环频率"],
            ["q_des / 末端候选位姿", "机器人未上报", "不由不同时间样本反推"],
            ["载荷 / 执行 / 计时页", "各页独立冻结并保留身份", "不把不同模型采样周期拼成同步样本"],
        ])
        continuous_rows = [
            ["停止原因", f"{_text(diag['exit_reason'])} / {_text(diag['exit_reason_text'])}", "来自本轮完整页；停止是否完成由程序退出及六轴静止确认"],
            ["速度缩放 / 目标速度比例", f"{_text(diag['speed_scaling'])} / {_text(diag['target_speed_fraction'])}", "当前 RTDE 值；本阶段只允许两项均为 1"],
            ["主机缩放许可", _text(diag["scaling_ok"]), "由实际缩放反馈写入；变化后进入停止流程"],
            ["遥测布局 / 当前页", f"{_text(diag['telemetry_layout'])} / {_text(diag['page_type'])} / {_text(diag['page_seq'])}", "原始包身份；下列缓存页分别保留自身身份"],
            ["伺服 / 模型设计周期", f"{cfg['SERVO_DT_S']:g} / {cfg['MODEL_DT_S']:g} s", "配置时基；实际间隔另列，不以平均值代替最大值"],
            ["伺服启动间隔：观测 / 退出", f"{cfg['SERVO_DT_S'] * 1000:g} / {cfg['SERVO_START_MAX_INTERVAL_S'] * 1000:g} ms",
             f"两者均保留 {cfg['TIMING_TOLERANCE_S'] * 1e6:g} μs 计时容差；模型预算、片段耗尽独立退出"],
        ]
        page_groups = (("执行页", diag["continuous"]["execution"], (
            ("generated_seq", "模型生成序号", "生成不表示执行采用"),
            ("submitted_seq", "模型提交序号", "完整片段已提交"),
            ("adopted_seq", "执行采用序号", "执行器已接受片段"),
            ("frame", "执行帧号", "有限片段内的实际采用帧"),
            ("based_on_seq", "依据的已接受身份", "片段生成所依据的命令状态"),
            ("start_frame", "片段起始帧", "适用帧范围"),
            ("end_frame", "片段截止帧", "过期片段不补发"),
            ("gap_count", "伺服启动间隔观测超期累计数", "超过设计周期加容差累计；仅超过退出间隔停止，不是 RTDE 丢包数"),
            ("limit_reason", "命令约束原因", "执行页所带约束码"),
            ("servoj_start_interval_s", "最近伺服开始间隔", "s；执行器相邻调用起点差"),
            ("servoj_max_interval_s", "本轮最大伺服开始间隔", "s；执行器累计最大值"),
        )), ("计时页", diag["continuous"]["timing"], (
            ("generated_seq", "生成 / 提交所属生成序号", "计时页自身身份"),
            ("submitted_seq", "提交序号", "计时页自身身份"),
            ("adopted_seq", "采用序号", "计时页自身身份"),
            ("frame", "执行帧号", "计时页自身身份"),
            ("sample_time_s", "模型采样时刻", "s；脚本计时原点"),
            ("generated_time_s", "模型生成时刻", "s；脚本计时原点"),
            ("submitted_time_s", "模型提交时刻", "s；脚本计时原点"),
            ("adopted_time_s", "执行采用时刻", "s；脚本计时原点"),
            ("target_age_s", "当前目标年龄", "s；执行器有限目标有效期"),
            ("target_max_age_s", "本轮最大目标年龄", "s；执行器累计最大值"),
            ("dt_model_actual_s", "最近模型采样间隔", "s；滤波和积分时基独立于伺服"),
            ("dt_model_max_s", "本轮最大模型间隔", "s；模型累计最大值"),
            ("servoj_start_interval_s", "最近伺服开始间隔", "s；计时页独立快照"),
            ("servoj_max_interval_s", "本轮最大伺服开始间隔", "s；计时页独立快照"),
            ("dt_base_s", "控制器基础步长", "s"),
            ("dt_model_design_s", "模型设计周期", "s"),
            ("compute_duration_s", "最近目标准备耗时", "s；模型线程"),
            ("compute_max_duration_s", "最大目标准备耗时", "s；本轮最大值"),
            ("servoj_duration_s", "最近 servoj 调用耗时", "s；执行线程"),
            ("servoj_duration_max_s", "最大 servoj 调用耗时", "s；本轮最大值"),
            ("publish_duration_s", "最近遥测发布耗时", "s；遥测线程"),
            ("publish_max_duration_s", "最大遥测发布耗时", "s；本轮最大值"),
            ("bias_std_nm", "六轴偏置标准差", "N·m；独立计时页"),
            ("bias_sample_count", "偏置样本数", "实际取样数"),
            ("bias_ready", "偏置已完成", "启动基线已冻结"),
        )), ("载荷页", diag["load"], (
            ("sample_time_s", "载荷采样时刻", "s；脚本计时原点"),
            ("generated_seq", "模型生成序号", "载荷页自身身份"),
            ("submitted_seq", "模型提交序号", "载荷页自身身份"),
            ("tau_raw_nm" if diag["telemetry_layout"] == 3 else "input_raw_nm",
             "六轴原始输入", "N·m；来源见载荷页，映射输入不等同于补偿关节力矩"),
            ("bias_nm", "六轴输入偏置", "N·m；法兰来源为固定 wrench 的当前 Jᵀ 投影"),
            ("filtered_nm", "六轴滤波载荷", "N·m；冻结载荷页"),
            ("drive", "六轴死区后驱动力", "N·m；诊断模式不驱动运动"),
            ("dt_model_actual_s", "模型实际采样间隔", "s；载荷页自身采样"),
            ("dt_model_max_s", "最大模型采样间隔", "s；载荷页独立快照"),
        )))
        for title, page, fields in page_groups:
            identity = "尚未收到" if page is None else (
                f"轮次 {page['run_id']} / 页 {page['page_seq']} / 模型采样 {page['model_sample_seq']} / 阶段 {page['phase']}")
            continuous_rows.append([title + "身份", identity, "该页全部字段来自同一冻结对象"])
            if page is None:
                continue
            if title == "载荷页":
                source = "joint_torque" if diag["telemetry_layout"] == 3 else page["load_source"]
                continuous_rows.append(["载荷页 · 输入来源", source,
                                        "joint_torque：补偿关节力矩；flange_wrench：法兰末端载荷经 Jᵀ 映射"])
            continuous_rows.append([title + "快照时刻 / 接收年龄",
                                    f"{page['snapshot_time_s']:.6f} s / {_text(_age(snapshot['now'], page['received_monotonic_s']))} s",
                                    "脚本时间 / 主机时间；不将接收年龄当采样年龄"])
            for key, label, explanation in fields:
                continuous_rows.append([title + " · " + label, _text(page[key]), explanation])
        statistics = diag.get("statistics")
        if statistics is None:
            continuous_rows.append(["J1 逐模型周期统计", "尚未取得", "缺统计或旧日志不得按零 drive 通过"])
        else:
            continuous_rows.extend([
                ["统计身份", f"运行 {_text(statistics.get('run_id'))} / 配置 {_text(statistics.get('config_id'))} / 版本 {_text(statistics.get('statistics_schema_version'))}", "与本轮参数记录对应，页间独立冻结"],
                ["统计采样序号 / 有效周期数", f"{_text(statistics.get('start_sample_seq'))} ～ {_text(statistics.get('end_sample_seq'))} / {_text(statistics.get('valid_cycle_count'))}", "偏置完成后开始，覆盖有效模型周期"],
                ["J1 滤波残差范围", f"{_text(statistics.get('filtered_min_nm'))} ～ {_text(statistics.get('filtered_max_nm'))} N·m", "死区前；来自逐周期累计极值"],
                ["J1 非零 drive 周期数 / 峰值", f"{_text(statistics.get('nonzero_drive_count'))} / {_text(statistics.get('max_abs_drive_nm'))} N·m", "任一非零锁定本轮输入不通过，不自动调整死区"],
                ["J1 首次非零序号 / 时刻", f"{_text(statistics.get('first_nonzero_sample_seq'))} / {_text(statistics.get('first_nonzero_time_s'))} s", "无首次非零时保留未提供，不补零"],
                ["轨迹完成 / 最终统计 / 覆盖完整", f"{_text(statistics.get('trajectory_complete'))} / {_text(statistics.get('final'))} / {_text(statistics.get('complete'))}", "三者独立核对；中间零值不能作为验收结论"],
                ["失效周期 / 统计校验错误", f"{_text(statistics.get('invalid_cycle_count'))} / {_text(diag.get('statistics_validation_errors'))}", "异常数据不按零处理；完整日志报告还检查运动与状态"],
                ["已收到有效终态统计", _text(diag.get("statistics_final_received")), "停止确认不等同于轨迹完成或统计通过"],
            ])
        rows["diagnostic_rows"] = continuous_rows + rows["diagnostic_rows"]
        if diag["realtime"] is not None:
            real = diag["realtime"]
            for key, label in (("tcp_force", "实时 TCP 载荷（独立通道）"), ("speed_scaling", "速度缩放"),
                               ("requested_speed_percent", "主界面速度请求百分比"), ("last_error", "实时通道错误")):
                if key in real:
                    rows["diagnostic_rows"].append([label, _text(real[key]), f"实时机器人时间 {_text(real['timestamp'])}；不替代导纳驱动力"])
        if diag["ft_sensor"] is not None:
            sensor = diag["ft_sensor"]
            rows["diagnostic_rows"].append([sensor["name"], _text(sensor["values"]), f"独立订阅时间 {_text(sensor['timestamp'])}；主机接收时刻 {_text(sensor['received_monotonic'])}"])
        return rows

    def plot_series(self, signal, axis, angle_unit="deg", length_unit="mm"):
        """每次只输出同一物理单位的选轴曲线，跨轮及失效样本保留断点。"""
        definitions = {
            "joint_targets": ("关节目标对照", "joint", ("q_master_adopted", "q_nom", "q_cmd", "actual_q")),
            "joint_errors": ("关节误差", "errors", ("nominal_following", "command_correction", "tracking")),
            "tcp_targets": ("TCP 位姿对照（目标为模型估算）", "tcp", ("q_master_adopted", "q_nom", "q_cmd", "actual_TCP_pose")),
            "delta_q": ("关节导纳偏移", "admittance", ("delta_q",)),
            "delta_x": ("末端导纳偏移", "admittance", ("delta_x",)),
            "drive": ("导纳驱动力", "admittance", ("drive",)),
            "actual_qd": ("实际关节速度", "joint", ("actual_qd",)),
            "master_received": ("主端最新接收关节目标", "joint", ("q_master_received",)),
        }
        title, group, keys = definitions[signal]
        colors = ("#2377c6", "#a97a14", "#cf4a51", "#229774")
        styles = ("dash", "dot", "solid", "solid")
        samples = self.udp_samples if signal == "master_received" else self.robot_samples
        series = []
        unit = ""
        for index, key in enumerate(keys):
            points = []
            label = key
            for sample in samples:
                if self.snapshot is not None and signal == "drive" and sample["space"] != self.snapshot["space"]:
                    continue
                item = sample[group][key]
                factor, unit = self._unit_factor(item["unit"], axis, angle_unit, length_unit)
                label = item["label"]
                value = None if item["value"] is None else item["value"][axis] * factor
                points.append({"time": sample["time"], "value": value, "run_id": sample["run_id"],
                               "segment": sample["segment"], "valid": item["state"] == "fresh" and value is not None})
            if self.snapshot is not None:
                item = self.snapshot[group][key]
                label = item["label"]
                _, unit = self._unit_factor(item["unit"], axis, angle_unit, length_unit)
            series.append({"name": label, "color": colors[index], "style": styles[index], "points": points})
        return {"title": title, "unit": unit, "series": series,
                "now": time.monotonic() if self.snapshot is None else self.snapshot["now"]}
