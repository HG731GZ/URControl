"""
URUdpClient.py — UDP通信客户端，用于接收/发送外部控制信号。

通讯协议（定长数据包，大端字节序，无填充）：
    offset  size  type        含义
    ------  ----  ----        ----
    0        4    uint8[4]   帧头 (0x55 0xAA 0x55 0xAA)
    4       48    6×float64  机械臂广义坐标（单位取决模式）
                             - JOINT_TRACK / JOINT_DELTA: 关节角 q[0..5] (rad)
                             - TCP_DELTA: 末端位姿增量 [dx, dy, dz, drx, dry, drz] (m, rad)
                             - TCP_VEL:  末端速度 [vx, vy, vz, wx, wy, wz] (m/s, rad/s)
    52       1    uint8      控制模式（0=OFF, 1=JOINT_TRACK, 2=JOINT_DELTA, 3=TCP_DELTA, 4=TCP_VEL）
    53      24    3×float64  夹钳广义坐标
    77      48    6×float64  主端六轴速度（J1～J6）
    125      4    uint32     CRC32 (IEEE 802.3)，校验范围 offset 4 ~ 124
    129       4    uint8[4]   帧尾 (0x0D 0x0A 0x0D 0x0A)
    主端指令总长度：133 字节。从端反馈沿用 85 字节格式，夹爪后直接写 CRC32。
"""

from __future__ import annotations

import binascii
from copy import deepcopy
import logging
import math
import select
import socket
import struct
import threading
import time
from typing import Optional, Tuple, List


logger = logging.getLogger(__name__)


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

    __slots__ = ("q_arm", "mode", "q_gripper", "qd_arm", "raw_packet", "recv_time")

    def __init__(
        self,
        q_arm: List[float],
        mode: int,
        q_gripper: List[float],
        qd_arm: List[float],
        raw_packet: Optional[bytes] = None,
    ):
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

    # 发送指令到远程
    client.send_to(('192.168.1.100', 5005), q_arm=[0.0]*6, mode=1, q_gripper=[0.0]*3)

    client.stop()
    """

    # 帧结构
    HEADER = b"\x55\xAA\x55\xAA"
    FOOTER = b"\x0D\x0A\x0D\x0A"
    HEADER_SIZE = 4
    FOOTER_SIZE = 4
    CRC_SIZE = 4

    FEEDBACK_FORMAT = "!6dB3d"
    DATA_FORMAT = "!6dB3d6d"
    DATA_SIZE = struct.calcsize(DATA_FORMAT)  # 121

    DATA_OFFSET = HEADER_SIZE                 # 4
    CRC_OFFSET = DATA_OFFSET + DATA_SIZE      # 125
    FOOTER_OFFSET = CRC_OFFSET + CRC_SIZE     # 129
    PACKET_SIZE = FOOTER_OFFSET + FOOTER_SIZE # 133
    RECEIVE_DEADLINE_S = 0.25
    SEND_RETRY_INTERVAL = 1.0
    ERROR_LOG_INTERVAL = 5.0

    def __init__(
        self,
        bind_host: str = "0.0.0.0",
        bind_port: int = 5005,
        recv_timeout: float = 0.1,
        send_interval: float = 0.0,
    ):
        self.bind_host = bind_host
        self.bind_port = bind_port
        self.recv_timeout = recv_timeout
        self.send_interval = send_interval

        self._sock: Optional[socket.socket] = None
        self._reader_thread: Optional[threading.Thread] = None
        self._writer_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        self._cond = threading.Condition()
        self._receive_trace = None
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

    def restart(self) -> None:
        """重新启动：先 stop 再 start。"""
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
        if expected != binascii.crc32(data) & 0xFFFFFFFF:
            raise ValueError("UDP CRC 校验失败")
        q_arm, mode, q_gripper, qd_arm = self._parse_data(data)
        if mode not in UDPControlMode._NAMES or not all(math.isfinite(v) for v in (*q_arm, *q_gripper, *qd_arm)):
            raise ValueError("UDP 模式未知或目标包含非有限数")
        return UDPCommand(q_arm, mode, q_gripper, qd_arm, raw_packet=packet)

    def _reader_loop(self) -> None:
        """有界排空后只发布最新位置；线程停顿后丢弃残留报文至排空。"""
        previous_poll = time.monotonic()
        backlog_since = None
        discard = False
        pending = None
        while not self._stop_event.is_set():
            try:
                sock = self._sock
                if sock is None:
                    break
                now = time.monotonic()
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
                    packet, addr = sock.recvfrom(self.PACKET_SIZE + 1)
                    received = time.monotonic()
                    try:
                        cmd = self._decode_packet(packet)
                    except ValueError as exc:
                        pending = None
                        self._control_event(str(exc))
                        with self._cond:
                            self._last_error = exc
                            self._cond.notify_all()
                    else:
                        with self._cond:
                            self._frame_count += 1
                            seq = self._frame_count
                            if self._receive_trace is not None:
                                self._receive_trace.record("udp_receive", dict(
                                    source_seq=seq, source_monotonic_s=received, q_source=cmd.q_arm))
                        if cmd.mode != UDPControlMode.JOINT_TRACK:
                            self._control_event("主端退出关节跟踪模式：" + UDPControlMode.cn_name(cmd.mode))
                        if not discard:
                            pending = (cmd, addr, received, seq)
                    if not select.select([sock], [], [], 0)[0]:
                        drained = True
                        break
                    if time.monotonic() >= deadline:
                        break
                if time.monotonic() - previous_poll > self.RECEIVE_DEADLINE_S and not discard:
                    # 排空过程中发生调度停顿时，本批不能以恢复后的读取时刻冒充新鲜输入。
                    discard = True
                    pending = None
                    self._control_event("UDP 排空过程超出通信期限")
                    with self._cond:
                        self._last_error = RuntimeError("UDP 排空过程超出通信期限")
                if drained:
                    if pending is not None and not discard:
                        cmd, addr, received, seq = pending
                        with self._cond:
                            self._latest_command = cmd
                            self._published_frame_count = seq
                            self._last_error = None
                            self._last_update_time = cmd.recv_time
                            self._last_received_monotonic = received
                            self._sender_address = addr
                            self._cond.notify_all()
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
            except OSError as exc:
                if self._stop_event.is_set():
                    break
                self._control_event("UDP 接收失败：" + str(exc))
                with self._cond:
                    self._last_error = exc
                    self._cond.notify_all()
                break

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

    def set_receive_trace(self, writer):
        """仅在遥操作记录期间采集所有有效包，包括尚未发布就被覆盖的包。"""
        with self._cond:
            self._receive_trace = writer

    def wait_control_update(self, frame_count, control_event_seq, stop_event, deadline):
        """等到发布新帧、发生控制事件、适配器关闭或本帧到期。"""
        with self._cond:
            return self._cond.wait_for(
                lambda: self._published_frame_count != frame_count
                or self._control_event_seq != control_event_seq
                or stop_event.is_set(),
                timeout=max(0.0, deadline - time.monotonic()),
            )

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
    def pack_command(cls, q_arm, mode, q_gripper, qd_arm):
        """将人工主端的同帧位置和速度打包为 133 字节指令。"""
        data = struct.pack(cls.DATA_FORMAT, *q_arm, mode, *q_gripper, *qd_arm)
        return cls._wrap_data(data)

    @classmethod
    def _wrap_data(cls, data):
        crc = struct.pack("!I", binascii.crc32(data) & 0xFFFFFFFF)
        return cls.HEADER + data + crc + cls.FOOTER

    @staticmethod
    def pack(q_arm: List[float], mode: int, q_gripper: List[float]) -> bytes:
        """
        将从端反馈打包为完整的 85 字节协议帧（帧头 + 数据 + CRC32 + 帧尾）。

        Parameters
        ----------
        q_arm:
            6 个机械臂广义坐标。
        mode:
            控制模式，取值见 UDPControlMode。
        q_gripper:
            3 个夹钳广义坐标。
        """
        if len(q_arm) != 6:
            raise ValueError(f"q_arm 长度必须为 6，实际 {len(q_arm)}")
        if len(q_gripper) != 3:
            raise ValueError(f"q_gripper 长度必须为 3，实际 {len(q_gripper)}")
        if not (0 <= mode <= 255):
            raise ValueError(f"mode 必须在 0-255 之间，实际 {mode}")

        data = struct.pack(
            URUDPClient.FEEDBACK_FORMAT,
            float(q_arm[0]), float(q_arm[1]), float(q_arm[2]),
            float(q_arm[3]), float(q_arm[4]), float(q_arm[5]),
            int(mode),
            float(q_gripper[0]), float(q_gripper[1]), float(q_gripper[2]),
        )

        return URUDPClient._wrap_data(data)

    def send_to(
        self,
        target: Tuple[str, int],
        q_arm: List[float],
        mode: int,
        q_gripper: List[float],
    ) -> bool:
        """
        同步发送，网络失败返回 False 并记录发送错误；界面应使用 queue_send_to()。

        Parameters
        ----------
        target:
            (ip, port) 目标地址。
        q_arm, mode, q_gripper:
            控制数据，见 pack()。
        """
        packet = self.pack(q_arm, mode, q_gripper)
        return self._send_packet(packet, target)

    def _send_packet(self, packet: bytes, target: Tuple[str, int]) -> bool:
        with self._send_lock:
            delay = self.send_interval - (time.monotonic() - self._last_send_time)
            if self._stop_event.wait(max(0.0, delay)):
                return False
            sock = self._sock
            if sock is None:
                return False
            try:
                sock.sendto(packet, target)
            except OSError as exc:
                if not self._stop_event.is_set():
                    now = time.monotonic()
                    with self._cond:
                        self._last_send_error = exc
                        self._send_error_count += 1
                    if now - self._last_send_error_logged >= self.ERROR_LOG_INTERVAL:
                        logger.warning("UDP 反馈发送失败，等待后续反馈重试 (%s): %s", target, exc)
                        self._last_send_error_logged = now
                return False
            self._last_send_time = time.monotonic()
            with self._cond:
                self._last_send_error = None
                self._send_count += 1
            return True

    def queue_send_to(
        self, target: Tuple[str, int], q_arm: List[float], mode: int,
        q_gripper: List[float], *, max_age: float = 0.5,
    ) -> bool:
        """提交最新反馈；等待超过 max_age 秒则丢弃。True 不代表远端已收到。"""
        packet = self.pack(q_arm, mode, q_gripper)
        with self._send_cond:
            if self._sock is None or self._stop_event.is_set():
                return False
            self._pending_send = (packet, target, time.monotonic() + max_age)
            self._send_cond.notify()
        return True

    def _writer_loop(self) -> None:
        next_attempt = 0.0
        while not self._stop_event.is_set():
            with self._send_cond:
                self._send_cond.wait_for(
                    lambda: self._pending_send is not None or self._stop_event.is_set())
            if self._stop_event.wait(max(0.0, next_attempt - time.monotonic())):
                break
            # 等待退避结束后再取最新帧，期间的旧反馈已被覆盖。
            with self._send_cond:
                if self._stop_event.is_set():
                    break
                pending, self._pending_send = self._pending_send, None
            if pending is None:
                continue
            packet, target, deadline = pending
            if time.monotonic() > deadline:
                continue
            sent = self._send_packet(packet, target)
            next_attempt = time.monotonic() + (self.send_interval if sent else self.SEND_RETRY_INTERVAL)
