"""异步 UDP v2：独立收包、周期反馈、同帧回显以及逐包事件；布局见 UDP反馈协议.md。"""

from __future__ import annotations

import zlib
import errno
from copy import deepcopy
from enum import IntEnum
import logging
import math
import select
import socket
import struct
import threading
import time
from typing import Optional, Tuple, List


from UR_Utils.udp_events import monotonic_us, new_session, record

logger = logging.getLogger(__name__)

class TeleopState(IntEnum):
    """从端反馈中的遥操作状态。"""
    OFF = 0
    RTDE_FOLLOW = 1
    IMPEDANCE = 2


class UDPControlMode:
    """UDP 协议里的控制模式常量。"""
    OFF = 0
    JOINT_TRACK = 1
    JOINT_DELTA = 2
    TCP_DELTA = 3
    TCP_VEL = 4

    _NAMES = {
        0: "OFF",
        1: "JOINT_TRACK",
        2: "JOINT_DELTA",
        3: "TCP_DELTA",
        4: "TCP_VEL",
    }

    _CN_NAMES = {
        0: "关闭",
        1: "关节跟踪",
        2: "关节增量",
        3: "末端增量",
        4: "末端速度",
    }

    @classmethod
    def name(cls, mode: int) -> str:
        return cls._NAMES.get(mode, f"UNKNOWN({mode})")

    @classmethod
    def cn_name(cls, mode: int) -> str:
        return cls._CN_NAMES.get(mode, f"未知({mode})")


class UDPCommand:
    """解析后的一帧控制指令。"""

    __slots__ = ("q_arm", "mode", "q_gripper", "qd_arm", "raw_packet", "recv_time", "master_session", "cmd_seq", "t1_us", "t2_us", "kind")

    def __init__(
        self,
        q_arm: List[float],
        mode: int,
        q_gripper: List[float],
        qd_arm: List[float],
        raw_packet: Optional[bytes] = None,
        *, master_session=0, cmd_seq=0, t1_us=0, kind=0,
    ):
        self.master_session = master_session
        self.cmd_seq, self.t1_us = cmd_seq, t1_us
        self.kind, self.t2_us = kind, 0
        self.q_arm = q_arm
        self.qd_arm = qd_arm
        self.mode = mode
        self.q_gripper = q_gripper
        self.raw_packet = raw_packet
        self.recv_time = time.time()

    def __repr__(self) -> str:
        return (
            f"UDPCommand(mode={UDPControlMode.name(self.mode)}, "
            f"q_arm={[f'{v:.4f}' for v in self.q_arm]}, "
            f"q_gripper={[f'{v:.4f}' for v in self.q_gripper]})"
        )


class URUDPClient:
    """
    UDP 控制指令收发客户端。

    后台接收线程解析并存储最新指令；后台发送线程发送最新反馈。
    界面使用 queue_send_to() 提交反馈，不等待网络，也不积压旧帧。

    Parameters
    ----------
    bind_host:
        本机绑定地址，默认 '0.0.0.0' 监听所有网卡。
    bind_port:
        本机绑定端口。
    recv_timeout:
        共用 socket 的收发超时（秒），网络等待在后台线程完成。
    send_interval:
        连续发送的最小间隔（秒），用于限速。设为 0 不限速。

    Usage
    -----
    client = URUDPClient(bind_host='0.0.0.0', bind_port=5005)
    client.start()

    # 读取最新指令
    cmd = client.get_latest()
    if cmd is not None:
        print(cmd.q_arm, cmd.mode, cmd.q_gripper)

    # 将从端实测状态发往主端
    client.send_to(('192.168.1.100', 6005), q_arm, UDPControlMode.OFF, q_gripper, qd_arm, tau_arm, teleop_state, qdd_arm)

    client.stop()
    """

    # 帧结构
    HEADER = b"\x55\xAA\x55\xAA"
    FOOTER = b"\x0D\x0A\x0D\x0A"
    HEADER_SIZE = 4
    FOOTER_SIZE = 4
    CRC_SIZE = 4

    FEEDBACK_FORMAT = "!6dB3d6d6dB6d"
    DATA_FORMAT = "!6dB3d6d"
    DATA_SIZE = struct.calcsize(DATA_FORMAT)  # 121

    DATA_OFFSET = HEADER_SIZE                 # 4
    COMMAND_EXTENSION = struct.Struct(">BB3Q")
    FEEDBACK_EXTENSION = struct.Struct(">BB13Q")
    CRC_OFFSET = DATA_OFFSET + DATA_SIZE + COMMAND_EXTENSION.size  # 151
    FOOTER_OFFSET = CRC_OFFSET + CRC_SIZE     # 155
    PACKET_SIZE = FOOTER_OFFSET + FOOTER_SIZE # 159
    RECEIVE_DEADLINE_S = 0.25
    ERROR_LOG_INTERVAL = 5.0

    def __init__(
        self,
        bind_host: str = "0.0.0.0",
        bind_port: int = 5005,
        recv_timeout: float = 0.1,
        send_interval: float = 0.0,
        *, kind=0, peer=None,
    ):
        if kind not in (0, 1):
            raise ValueError("UDP 模式须为 0 或 1")
        self.kind = kind
        self.peer = peer
        self.slave_session = 0
        self._master_session = 0
        self._retired_master_sessions = set()
        self._last_progress_us = 0
        self._link_state = "waiting"
        self._link_event_count = 0
        self._link_reason = None
        self._publication_restart_reason = None
        self._echo = (0, 0, 0, 0)
        self._fb_seq = 0
        self._highest_cmd_seq = 0
        self._packet_counts = {}
        self._seen = {}
        self._seen_max = {}
        self._applied = (0, 0, 0)
        self.bind_host = bind_host
        self.bind_port = bind_port
        self.recv_timeout = recv_timeout
        self.send_interval = send_interval

        self._sock: Optional[socket.socket] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._writer_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        self._cond = threading.Condition()
        self._latest_command: Optional[UDPCommand] = None
        self._frame_count: int = 0
        self._published_frame_count = 0
        self._last_update_time: Optional[float] = None
        self._last_received_monotonic: Optional[float] = None
        self._sender_address: Optional[Tuple[str, int]] = None
        self._last_error: Optional[BaseException] = None

        self._control_event_seq = 0
        self._control_event_reason = None
        self._send_lock = threading.Lock()
        self._last_send_time: float = 0.0
        self._send_cond = threading.Condition()
        self._pending_send = None
        self._last_send_error: Optional[OSError] = None
        self._send_count = 0
        self._send_error_count = 0
        self._last_send_error_logged = float('-inf')

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def start(self) -> None:
        """
        创建 UDP socket、绑定端口并启动后台收发线程。
        重复调用安全：如果已启动则直接返回。
        """
        if (self.is_alive() and self._writer_thread is not None
                and self._writer_thread.is_alive()):
            return
        self.stop()
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.settimeout(self.recv_timeout)
            sock.bind((self.bind_host, self.bind_port))
        except OSError:
            sock.close()
            raise
        with self._cond:
            self.slave_session = new_session()
            self._link_state = "waiting"
            self._last_progress_us = 0
            self._publication_restart_reason = None
            self._last_error = None
            self._last_send_error = None
        record("slave_session_started", slave_session=self.slave_session, kind=self.kind,
               t_event_us=monotonic_us(), reason="socket_start")
        self._echo = (0, 0, 0, 0)
        self._fb_seq = 0
        self._highest_cmd_seq = 0
        self._packet_counts = {}
        self._seen = {}
        self._seen_max = {}
        self._latest_command = None
        self._applied = (0, 0, 0)
        self._sock = sock

        self._stop_event.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name=f"URUDPReader-{self.bind_host}:{self.bind_port}",
            daemon=True,
        )
        self._writer_thread = threading.Thread(
            target=self._writer_loop,
            name=f"URUDPWriter-{self.bind_host}:{self.bind_port}",
            daemon=True,
        )
        self._reader_thread.start()
        self._writer_thread.start()

    def stop(self) -> None:
        """停止后台线程并关闭 socket。"""
        self._stop_event.set()
        if self._reader_thread is not None:
            self._control_event("UDP 接收已停止")
        with self._send_cond:
            self._pending_send = None
            self._send_cond.notify_all()

        for thread in (self._reader_thread, self._writer_thread):
            if (thread is not None and thread.is_alive()
                    and threading.current_thread() is not thread):
                thread.join(timeout=1.0)
        self._reader_thread = None
        self._writer_thread = None
        sock = self._sock
        if sock is not None:
            sock.close()
            self._sock = None
            with self._cond:
                self._link_event_locked("stopped", "socket_stop", monotonic_us())

    def restart(self) -> None:
        """重新创建本端发布会话；对端可依据新会话独立重置反馈序号。"""
        self.stop()

        with self._cond:
            self._latest_command = None
            self._last_error = None
            self._frame_count = 0
            self._published_frame_count = 0
            self._last_update_time = None
            self._last_received_monotonic = None
            self._sender_address = None
            self._last_send_error = None
            self._send_count = 0
            self._send_error_count = 0
        self._last_send_time = 0.0
        self._last_send_error_logged = float('-inf')

        self.start()

    # ------------------------------------------------------------------
    # 后台接收线程
    # ------------------------------------------------------------------

    def _control_event(self, reason):
        """单调事件身份保留终止原因，后续位置包不能撤销本轮停止。"""
        with self._cond:
            self._control_event_seq += 1
            self._control_event_reason = reason
            self._cond.notify_all()

    def _decode_packet(self, packet):
        """验证协议边界和有限数，不修改报文字段或运动目标。"""
        if len(packet) != self.PACKET_SIZE:
            raise ValueError(f"UDP 长度错误：{len(packet)}，要求 {self.PACKET_SIZE}")
        if packet[:self.HEADER_SIZE] != self.HEADER or packet[self.FOOTER_OFFSET:] != self.FOOTER:
            raise ValueError("UDP 帧头或帧尾错误")
        data = packet[self.DATA_OFFSET:self.CRC_OFFSET]
        expected = struct.unpack("!I", packet[self.CRC_OFFSET:self.FOOTER_OFFSET])[0]
        if expected != zlib.crc32(data) & 0xFFFFFFFF:
            raise ValueError("UDP CRC 校验失败")
        version, kind, session, seq, t1 = self.COMMAND_EXTENSION.unpack(data[self.DATA_SIZE:])
        if version != 2 or kind not in (0, 1) or session == 0 or seq == 0:
            raise ValueError("UDP v2 版本、模式或会话/序号无效")
        payload = data[:self.DATA_SIZE]
        if kind == 1 and payload != bytes(self.DATA_SIZE):
            raise ValueError("网络诊断业务载荷必须全零")
        q_arm, mode, q_gripper, qd_arm = self._parse_data(payload)
        if mode not in UDPControlMode._NAMES or not all(math.isfinite(v) for v in (*q_arm, *q_gripper, *qd_arm)):
            raise ValueError("UDP 模式未知或目标包含非有限数")
        return UDPCommand(q_arm, mode, q_gripper, qd_arm, raw_packet=packet,
                          master_session=session, cmd_seq=seq, t1_us=t1, kind=kind)

    def _reader_loop(self) -> None:
        """读端独立采用非阻塞操作，发送端沿用原 socket 的超时。"""
        try:
            with self._sock.dup() as reader:
                reader.setblocking(False)
                self._receive_batches(reader)
        except OSError as exc:
            if not self._stop_event.is_set():
                self._receive_network_error(exc)

    def _read_datagram(self, sock):
        """在套接字读取返回后立即采集用户态单调时刻。"""
        packet, addr = sock.recvfrom(self.PACKET_SIZE + 1)
        t2_us = monotonic_us()
        return packet, addr, t2_us

    def _receive_batches(self, sock) -> None:
        """有界排空后只发布最新位置，线程停顿后丢弃残留报文至排空。"""
        previous_poll = time.monotonic()
        backlog_since = None
        discard = False
        pending = None
        while not self._stop_event.is_set():
            now = time.monotonic()
            self._check_link_deadline()
            # 空闲等待最多 100 ms；超出通信期限的调度停顿不能刷新旧包年龄。
            if now - previous_poll > self.RECEIVE_DEADLINE_S:
                discard = True
                pending = None
                self._control_event("UDP 接收线程停顿，丢弃残留报文")
                with self._cond:
                    self._last_error = RuntimeError("UDP 接收线程停顿")
            previous_poll = now
            if not select.select([sock], [], [], min(self.recv_timeout, 0.1))[0]:
                pending = None
                backlog_since = None
                discard = False
                continue
            deadline = time.monotonic() + 0.005
            drained = False
            for _ in range(256):
                try:
                    packet, addr, t2_us = self._read_datagram(sock)
                    received = t2_us / 1_000_000
                except BlockingIOError:
                    drained = True
                    break
                except OSError as exc:
                    if exc.errno not in (errno.ENETDOWN, errno.ENETUNREACH, errno.EHOSTUNREACH, errno.ECONNREFUSED):
                        raise
                    self._receive_network_error(exc)
                    pending = None
                    drained = True
                    self._stop_event.wait(self.recv_timeout)
                    break
                try:
                    cmd = self._decode_packet(packet)
                except ValueError as exc:
                    with self._cond:
                        self._packet_counts["rejected"] = self._packet_counts.get("rejected", 0) + 1
                    record("udp_rejected", t2_us=t2_us, reason=str(exc), address=addr, size=len(packet))
                else:
                    cmd.t2_us = t2_us
                    if not self._accept_received(cmd, addr, discard) or self.kind == 1:
                        if time.monotonic() >= deadline:
                            break
                        continue
                    with self._cond:
                        self._frame_count += 1
                        seq = self._frame_count
                    if cmd.mode != UDPControlMode.JOINT_TRACK:
                        self._control_event("主端退出关节跟踪模式：" + UDPControlMode.cn_name(cmd.mode))
                    if not discard:
                        pending = (cmd, addr, received, seq)
                if time.monotonic() >= deadline:
                    break
            if not drained:
                # 正好用完本批预算时也要发布已经读尽的最后一帧。
                drained = not select.select([sock], [], [], 0)[0]
            if time.monotonic() - previous_poll > self.RECEIVE_DEADLINE_S and not discard:
                # 排空过程中发生调度停顿时，本批的读取时刻不能代表目标新鲜度。
                discard = True
                pending = None
                self._control_event("UDP 排空过程超出通信期限")
                with self._cond:
                    self._last_error = RuntimeError("UDP 排空过程超出通信期限")
            if drained:
                if pending is not None and not discard and pending[0].master_session == self._master_session:
                    cmd, addr, received, seq = pending
                    with self._cond:
                        self._latest_command = cmd
                        self._published_frame_count = seq
                        self._last_error = None
                        self._last_update_time = cmd.recv_time
                        self._last_received_monotonic = received
                        self._sender_address = addr
                        self._cond.notify_all()
                        record("udp_publish", master_session=cmd.master_session, cmd_seq=cmd.cmd_seq,
                               source_seq=seq, t2_us=cmd.t2_us, t_publish_us=monotonic_us())
                pending = None
                backlog_since = None
                discard = False
            else:
                if backlog_since is None:
                    backlog_since = time.monotonic()
                elif not discard and time.monotonic() - backlog_since > self.RECEIVE_DEADLINE_S:
                    discard = True
                    pending = None
                    self._control_event("UDP 接收缓冲持续积压")
                    with self._cond:
                        self._last_error = RuntimeError("UDP 接收缓冲持续积压")
                self._stop_event.wait(0.001)

    def _receive_network_error(self, exc):
        """网络接口故障清除可用目标；后续读操作接收新帧，不重放失败报文。"""
        with self._cond:
            self._last_error = exc
            self._latest_command = None
            self._echo = (0, 0, 0, 0)
            if self._link_state != "error":
                self._publication_restart_reason = "receive_error"
                self._link_event_locked("error", str(exc), monotonic_us())
                if self.kind == 0:
                    self._control_event("UDP 接收失败：" + str(exc))
            self._cond.notify_all()

    def _link_event_locked(self, state, reason, t_event_us):
        """在接收状态边界只发布一次事件，界面和日志读取同一状态。"""
        self._link_state = state
        self._link_event_count += 1
        self._link_reason = reason
        record("udp_link", state=state, reason=reason, t_event_us=t_event_us,
               master_session=self._master_session, slave_session=self.slave_session)

    def _check_link_deadline(self):
        """有效新帧超期表示断流；保留序号高水位，重复包不能冒充恢复。"""
        now_us = monotonic_us()
        with self._cond:
            if self._link_state == "online" and now_us - self._last_progress_us > round(self.RECEIVE_DEADLINE_S * 1_000_000):
                self._latest_command = None
                self._echo = (0, 0, 0, 0)
                self._publication_restart_reason = "receive_timeout"
                self._link_event_locked("timeout", "超过 250 ms 没有有效新帧", now_us)
                if self.kind == 0:
                    self._control_event("UDP 主端断流")

    def _accept_received(self, cmd, addr, discard):
        """自动接纳新主端会话，已退役会话只能计数，不能重新接管目标。"""
        key = (addr, cmd.kind, cmd.master_session)
        seen = self._seen.setdefault(key, set())
        duplicate = cmd.cmd_seq in seen
        previous_max = self._seen_max.get(key, 0)
        self._seen_max[key] = max(previous_max, cmd.cmd_seq)
        seen.add(cmd.cmd_seq)
        reason = "new"
        with self._cond:
            if self.peer is not None and addr != self.peer:
                reason = "wrong_peer"
            elif cmd.kind != self.kind:
                reason = "wrong_kind"
            elif cmd.master_session in self._retired_master_sessions:
                reason = "retired_session"
            else:
                if cmd.master_session != self._master_session:
                    previous_session = self._master_session
                    if previous_session:
                        self._retired_master_sessions.add(previous_session)
                    self._master_session = cmd.master_session
                    self._highest_cmd_seq = 0
                    self._latest_command = None
                    self._last_received_monotonic = None
                    self._applied = (0, 0, 0)
                    self._echo = (0, 0, 0, 0)
                    self._link_event_locked("online", "master_session_changed" if previous_session else "master_session_discovered", cmd.t2_us)
                    record("master_session_changed", previous_master_session=previous_session,
                           master_session=cmd.master_session, first_cmd_seq=cmd.cmd_seq, t_event_us=cmd.t2_us)
                    if previous_session and self.kind == 0:
                        self._control_event("主端 UDP 发布会话已重启")
                if duplicate:
                    reason = "duplicate"
                elif cmd.cmd_seq <= self._highest_cmd_seq:
                    reason = "out_of_order"
                else:
                    self._highest_cmd_seq = cmd.cmd_seq
                    self._last_progress_us = cmd.t2_us
                    if not discard:
                        self._last_error = None
                    self._echo = (cmd.master_session, cmd.cmd_seq, cmd.t1_us, cmd.t2_us)
                    if self._link_state != "online":
                        self._link_event_locked("online", "receive_recovered", cmd.t2_us)
                    if discard and self.kind == 0:
                        reason = "backlog_discard"
            self._packet_counts[reason] = self._packet_counts.get(reason, 0) + 1
        record("udp_receive", master_session=cmd.master_session,
               cmd_seq=cmd.cmd_seq, t1_us=cmd.t1_us, t2_us=cmd.t2_us, kind=cmd.kind,
               disposition=reason, duplicate=duplicate, out_of_order=cmd.cmd_seq < previous_max,
               q_rad=cmd.q_arm, qd_rad_s=cmd.qd_arm, mode=cmd.mode, address=addr)
        return reason == "new"

    def note_applied(self, event):
        """跨进程快照可重复读取，只有更晚的采用事件推进反馈字段。"""
        if event is None or self.kind != 0:
            return
        with self._cond:
            if event["applied_master_session"] == self._master_session and event["t_apply_us"] > self._applied[2]:
                self._applied = (event["applied_master_session"], event["applied_cmd_seq"], event["t_apply_us"])

    @staticmethod
    def _parse_data(data: bytes) -> Tuple[List[float], int, List[float], List[float]]:
        """解析 121 字节指令数据段，位置、夹爪和速度共享收包身份。"""
        values = struct.unpack(URUDPClient.DATA_FORMAT, data)
        q_arm = list(values[:6])
        mode = int(values[6])
        q_gripper = list(values[7:10])
        return q_arm, mode, q_gripper, list(values[10:16])

    # ------------------------------------------------------------------
    # 对外读取接口
    # ------------------------------------------------------------------

    def get_latest(self) -> Optional[UDPCommand]:
        """
        获取最新一帧控制指令（非阻塞）。
        无数据时返回 None。
        """
        with self._cond:
            return self._latest_command

    def get_latest_and_count(self) -> Tuple[Optional[UDPCommand], int]:
        """获取最新指令及帧计数。"""
        with self._cond:
            return self._latest_command, self._published_frame_count

    def wait_joint_target(self, previous_seq, control_event_seq, stop_event, deadline):
        """等待并在同一把锁内取走最新关节目标，首次调用直接读取当前帧。"""
        with self._cond:
            updated = self._cond.wait_for(
                lambda: self._published_frame_count != previous_seq
                or self._control_event_seq != control_event_seq
                or stop_event.is_set(),
                timeout=None if deadline is None else max(0.0, deadline - time.monotonic()),
            )
            if stop_event.is_set():
                return None
            if self._control_event_seq != control_event_seq:
                raise ValueError(self._control_event_reason)
            if not updated:
                raise TimeoutError("UDP 目标断流")
            cmd = self._latest_command
            if self._last_error is not None or cmd is None or cmd.mode != UDPControlMode.JOINT_TRACK:
                raise ValueError("UDP 模式变化或接收错误")
            return (self._published_frame_count, self._last_received_monotonic,
                    tuple(cmd.q_arm), tuple(cmd.qd_arm), (cmd.master_session, cmd.cmd_seq))

    def wake_control_waiters(self):
        """让等待新帧的适配器立即检查关闭信号。"""
        with self._cond:
            self._cond.notify_all()

    def get_monitor_snapshot(self) -> dict:
        """在同一锁内读取最新有效指令、来源和时刻；显示层取得独立指令副本。"""
        with self._cond:
            return {
                "command": deepcopy(self._latest_command),
                "frame_count": self._published_frame_count,
                "control_event_seq": self._control_event_seq,
                "control_event_reason": self._control_event_reason,
                "received_time": self._last_update_time,
                "received_monotonic": self._last_received_monotonic,
                "sender_address": self._sender_address,
                "last_error": None if self._last_error is None else str(self._last_error),
                "bind_address": (self.bind_host, self.bind_port),
                "send_last_error": None if self._last_send_error is None else str(self._last_send_error),
                "send_count": self._send_count,
                "send_error_count": self._send_error_count,
                "packet_counts": dict(self._packet_counts),
                "kind": self.kind,
                "slave_session": self.slave_session, "master_session": self._master_session,
                "link_state": self._link_state, "link_event_count": self._link_event_count,
                "link_reason": self._link_reason, "latest_cmd_seq": self._highest_cmd_seq,
                "fb_seq": self._fb_seq,
            }

    def wait_next(
        self,
        timeout: Optional[float] = None,
    ) -> Optional[UDPCommand]:
        """
        等待并返回下一帧指令（阻塞）。

        Parameters
        ----------
        timeout:
            最大等待时间（秒），None 表示无限等待。

        Returns
        -------
        UDPCommand | None — 超时则返回 None。
        """
        with self._cond:
            target = self._published_frame_count

            self._cond.wait_for(
                lambda: self._published_frame_count > target
                or self._last_error is not None,
                timeout=timeout,
            )
            return self._latest_command

    def get_frame_count(self) -> int:
        """获取已成功接收的帧数。"""
        with self._cond:
            return self._published_frame_count

    def get_last_update_time(self) -> Optional[float]:
        """获取最近一次成功收包的时间戳（time.time()）。"""
        with self._cond:
            return self._last_update_time

    def get_last_error(self) -> Optional[BaseException]:
        """获取后台线程的最近一次异常（正常时为 None）。"""
        with self._cond:
            return self._last_error

    def is_alive(self) -> bool:
        """后台接收线程是否在运行。"""
        return self._reader_thread is not None and self._reader_thread.is_alive()

    # ------------------------------------------------------------------
    # 对外发送接口
    # ------------------------------------------------------------------

    @classmethod
    def pack_command(cls, q_arm, mode, q_gripper, qd_arm, *, master_session, cmd_seq, t1_us, kind=0):
        """将人工主端的同帧位置和速度打包为 159 字节 v2 指令。"""
        data = struct.pack(cls.DATA_FORMAT, *q_arm, mode, *q_gripper, *qd_arm)
        if kind == 1:
            data = bytes(cls.DATA_SIZE)
        return cls._wrap_data(data + cls.COMMAND_EXTENSION.pack(2, kind, master_session, cmd_seq, t1_us))

    @classmethod
    def _wrap_data(cls, data):
        crc = struct.pack("!I", zlib.crc32(data) & 0xFFFFFFFF)
        return cls.HEADER + data + crc + cls.FOOTER

    @classmethod
    def pack(cls, q_arm, mode, q_gripper, qd_arm, tau_arm, teleop_state, qdd_arm, *, extension):
        """离线编码完整 v2 反馈；在线发送由发送线程临近 sendto 取 t3。"""
        data = cls.pack_payload(q_arm, mode, q_gripper, qd_arm, tau_arm, teleop_state, qdd_arm)
        if extension[1] == 1:
            data = bytes(struct.calcsize(cls.FEEDBACK_FORMAT))
        return cls._wrap_data(data + cls.FEEDBACK_EXTENSION.pack(*extension))

    @classmethod
    def pack_payload(cls, q_arm, mode, q_gripper, qd_arm, tau_arm, teleop_state, qdd_arm) -> bytes:
        """将实际关节角、夹爪、轴速度、关节力矩、遥操作状态及实际加速度打包为 218 字节原业务载荷。"""
        if (len(q_arm), len(qd_arm), len(tau_arm), len(qdd_arm)) != (6, 6, 6, 6):
            raise ValueError("关节角、关节速度、关节力矩和关节加速度各需六轴数据")
        if len(q_gripper) != 3:
            raise ValueError(f"q_gripper 长度必须为 3，实际 {len(q_gripper)}")
        if not (0 <= mode <= 255):
            raise ValueError(f"mode 必须在 0-255 之间，实际 {mode}")

        data = struct.pack(cls.FEEDBACK_FORMAT, *q_arm, mode, *q_gripper, *qd_arm, *tau_arm, TeleopState(teleop_state), *qdd_arm)
        return data

    def send_to(
        self,
        target: Tuple[str, int],
        q_arm: List[float],
        mode: int,
        q_gripper: List[float],
        qd_arm: List[float],
        tau_arm: List[float],
        teleop_state: TeleopState,
        qdd_arm: List[float],
    ) -> bool:
        """
        同步发送，网络失败返回 False 并记录发送错误；界面应使用 queue_send_to()。

        Parameters
        ----------
        target:
            (ip, port) 目标地址。
        q_arm, mode, q_gripper, qd_arm, tau_arm, teleop_state, qdd_arm:
            控制数据，见 pack()。
        """
        packet = self.pack_payload(q_arm, mode, q_gripper, qd_arm, tau_arm, teleop_state, qdd_arm)
        return self._send_packet(packet, target)

    def _send_packet(self, payload: bytes, target: Tuple[str, int], sample_times=(0, 0, 0)) -> bool:
        with self._send_lock:
            delay = self.send_interval - (time.monotonic() - self._last_send_time)
            if self._stop_event.wait(max(0.0, delay)):
                return False
            sock = self._sock
            if sock is None:
                return False
            with self._cond:
                restart_reason = self._publication_restart_reason
                if restart_reason is not None:
                    self.slave_session = new_session()
                    self._fb_seq = 0
                    self._publication_restart_reason = None
                    record("slave_session_started", slave_session=self.slave_session, kind=self.kind,
                           t_event_us=monotonic_us(), reason=restart_reason)
                echo = self._echo
                applied = self._applied if self.kind == 0 else (0, 0, 0)
                self._fb_seq += 1
                slave_session, fb_seq = self.slave_session, self._fb_seq
            if self.kind == 1:
                payload = bytes(struct.calcsize(self.FEEDBACK_FORMAT))
                sample_times = (0, 0, 0)
            # 快照已冻结；只剩扩展打包和 CRC，不能把入队时刻当作发送时刻。
            t3_us = monotonic_us()
            extension = (2, self.kind, slave_session, fb_seq, *echo, t3_us, *applied, *sample_times)
            packet = self._wrap_data(payload + self.FEEDBACK_EXTENSION.pack(*extension))
            try:
                sock.sendto(packet, target)
            except OSError as exc:
                record("udp_send", extension=extension, success=False, error=str(exc))
                if not self._stop_event.is_set():
                    now = time.monotonic()
                    with self._cond:
                        if self._last_send_error is None:
                            self._publication_restart_reason = "send_error"
                            record("udp_send_error", error=str(exc), t_event_us=monotonic_us())
                        self._last_send_error = exc
                        self._send_error_count += 1
                    if now - self._last_send_error_logged >= self.ERROR_LOG_INTERVAL:
                        logger.warning("UDP 反馈发送失败，等待后续反馈重试 (%s): %s", target, exc)
                        self._last_send_error_logged = now
                return False
            record("udp_send", extension=extension, success=True, payload_hex=payload.hex())
            self._last_send_time = time.monotonic()
            with self._cond:
                if self._last_send_error is not None:
                    record("udp_send_recovered", slave_session=slave_session, fb_seq=fb_seq,
                           t_event_us=monotonic_us())
                self._last_send_error = None
                self._send_count += 1
            return True

    def queue_send_to(
        self, target: Tuple[str, int], q_arm: List[float], mode: int,
        q_gripper: List[float], qd_arm: List[float], tau_arm: List[float], teleop_state: TeleopState,
        qdd_arm: List[float],
        *, max_age: float = 0.5, sample_times=(0, 0, 0),
    ) -> bool:
        """提交最新反馈；等待超过 max_age 秒则丢弃。True 不代表远端已收到。"""
        packet = self.pack_payload(q_arm, mode, q_gripper, qd_arm, tau_arm, teleop_state, qdd_arm)
        with self._send_cond:
            if self._sock is None or self._stop_event.is_set():
                return False
            self._pending_send = (packet, target, time.monotonic() + max_age, sample_times)
            self._send_cond.notify()
        return True

    def discard_pending_feedback(self):
        """机器人断开连接时清除尚在等待发送的反馈。"""
        with self._send_cond:
            self._pending_send = None

    def _writer_loop(self) -> None:
        next_attempt = 0.0
        while not self._stop_event.is_set():
            with self._send_cond:
                self._send_cond.wait_for(
                    lambda: self._pending_send is not None or self._stop_event.is_set())
            if self._stop_event.wait(max(0.0, next_attempt - time.monotonic())):
                break
            # 按原本地节拍取最新反馈，不对失败报文自动重发。
            with self._send_cond:
                if self._stop_event.is_set():
                    break
                pending, self._pending_send = self._pending_send, None
            if pending is None:
                continue
            packet, target, deadline, sample_times = pending
            if time.monotonic() > deadline:
                continue
            self._send_packet(packet, target, sample_times)
            next_attempt = time.monotonic() + self.send_interval
