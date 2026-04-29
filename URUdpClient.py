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
    77       4    uint32     CRC32 (IEEE 802.3)，校验范围 offset 4 ~ 76
    81       4    uint8[4]   帧尾 (0x0D 0x0A 0x0D 0x0A)
    总长度：85 字节
"""

from __future__ import annotations

import binascii
import socket
import struct
import threading
import time
from typing import Optional, Tuple, List


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

    __slots__ = ("q_arm", "mode", "q_gripper", "raw_packet", "recv_time")

    def __init__(
        self,
        q_arm: List[float],
        mode: int,
        q_gripper: List[float],
        raw_packet: Optional[bytes] = None,
    ):
        self.q_arm = q_arm
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

    内部维护一个后台线程持续接收 UDP 数据包，按协议解析后存储最新一帧。
    对外暴露非阻塞的读取接口和频率可控的发送接口。

    Parameters
    ----------
    bind_host:
        本机绑定地址，默认 '0.0.0.0' 监听所有网卡。
    bind_port:
        本机绑定端口。
    recv_timeout:
        socket 接收超时（秒），影响线程响应 stop 事件的延迟。
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

    DATA_FORMAT = "!6dB3d"
    DATA_SIZE = struct.calcsize(DATA_FORMAT)  # 73

    DATA_OFFSET = HEADER_SIZE                 # 4
    CRC_OFFSET = DATA_OFFSET + DATA_SIZE      # 77
    FOOTER_OFFSET = CRC_OFFSET + CRC_SIZE     # 81
    PACKET_SIZE = FOOTER_OFFSET + FOOTER_SIZE # 85

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
        self._stop_event = threading.Event()

        self._cond = threading.Condition()
        self._latest_command: Optional[UDPCommand] = None
        self._frame_count: int = 0
        self._last_update_time: Optional[float] = None
        self._last_error: Optional[BaseException] = None

        self._send_lock = threading.Lock()
        self._last_send_time: float = 0.0

    # ------------------------------------------------------------------
    # 生命周期
    # ------------------------------------------------------------------

    def start(self) -> None:
        """
        创建 UDP socket、绑定端口并启动后台接收线程。
        重复调用安全：如果已启动则直接返回。
        """
        if self._reader_thread is not None and self._reader_thread.is_alive():
            return

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.settimeout(self.recv_timeout)
        self._sock.bind((self.bind_host, self.bind_port))

        self._stop_event.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name=f"URUDPReader-{self.bind_host}:{self.bind_port}",
            daemon=True,
        )
        self._reader_thread.start()

    def stop(self) -> None:
        """停止后台线程并关闭 socket。"""
        self._stop_event.set()

        sock = self._sock
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
            self._sock = None

        thread = self._reader_thread
        if thread is not None and thread.is_alive():
            if threading.current_thread() is not thread:
                thread.join(timeout=1.0)
        self._reader_thread = None

    def restart(self) -> None:
        """重新启动：先 stop 再 start。"""
        self.stop()

        with self._cond:
            self._latest_command = None
            self._last_error = None
            self._frame_count = 0
            self._last_update_time = None

        self.start()

    # ------------------------------------------------------------------
    # 后台接收线程
    # ------------------------------------------------------------------

    def _reader_loop(self) -> None:
        """后台线程：循环接收 UDP 包，校验帧头/CRC/帧尾后解析。"""
        buf = bytearray(self.PACKET_SIZE)

        while not self._stop_event.is_set():
            try:
                sock = self._sock
                if sock is None:
                    break

                nbytes, addr = sock.recvfrom_into(buf, self.PACKET_SIZE)

                if nbytes != self.PACKET_SIZE:
                    with self._cond:
                        self._last_error = ValueError(
                            f"收到异常长度数据包: {nbytes} bytes, "
                            f"期望 {self.PACKET_SIZE} bytes, 来自 {addr}"
                        )
                        self._cond.notify_all()
                    continue

                if bytes(buf[:self.HEADER_SIZE]) != self.HEADER:
                    with self._cond:
                        self._last_error = ValueError(
                            f"帧头不匹配，来自 {addr}"
                        )
                        self._cond.notify_all()
                    continue

                data = buf[self.DATA_OFFSET:self.CRC_OFFSET]

                expected_crc = struct.unpack("!I", buf[self.CRC_OFFSET:self.FOOTER_OFFSET])[0]
                actual_crc = binascii.crc32(data) & 0xFFFFFFFF
                if expected_crc != actual_crc:
                    with self._cond:
                        self._last_error = ValueError(
                            f"CRC 校验失败: expected=0x{expected_crc:08X}, "
                            f"actual=0x{actual_crc:08X}, 来自 {addr}"
                        )
                        self._cond.notify_all()
                    continue

                if bytes(buf[self.FOOTER_OFFSET:]) != self.FOOTER:
                    with self._cond:
                        self._last_error = ValueError(
                            f"帧尾不匹配，来自 {addr}"
                        )
                        self._cond.notify_all()
                    continue

                q_arm, mode, q_gripper = self._parse_data(data)

                cmd = UDPCommand(
                    q_arm=q_arm,
                    mode=mode,
                    q_gripper=q_gripper,
                    raw_packet=bytes(buf),
                )

                with self._cond:
                    self._latest_command = cmd
                    self._last_error = None
                    self._frame_count += 1
                    self._last_update_time = time.time()
                    self._cond.notify_all()

            except socket.timeout:
                # 仅在超时时检查 stop 事件，正常情况
                continue

            except OSError:
                if not self._stop_event.is_set():
                    with self._cond:
                        self._last_error = RuntimeError("UDP socket 异常关闭")
                        self._cond.notify_all()
                break

            except Exception as e:
                with self._cond:
                    self._last_error = e
                    self._cond.notify_all()

    @staticmethod
    def _parse_data(data: bytes) -> Tuple[List[float], int, List[float]]:
        """解析 73 字节数据段（6 double + 1 uint8 + 3 double）。"""
        values = struct.unpack(URUDPClient.DATA_FORMAT, data)
        q_arm = list(values[:6])
        mode = int(values[6])
        q_gripper = list(values[7:])
        return q_arm, mode, q_gripper

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
            return self._latest_command, self._frame_count

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
            target = self._frame_count

            self._cond.wait_for(
                lambda: self._frame_count > target
                or self._last_error is not None,
                timeout=timeout,
            )
            return self._latest_command

    def get_frame_count(self) -> int:
        """获取已成功接收的帧数。"""
        with self._cond:
            return self._frame_count

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

    @staticmethod
    def pack(q_arm: List[float], mode: int, q_gripper: List[float]) -> bytes:
        """
        将控制指令打包为完整的 85 字节协议帧（帧头 + 数据 + CRC32 + 帧尾）。

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
            URUDPClient.DATA_FORMAT,
            float(q_arm[0]), float(q_arm[1]), float(q_arm[2]),
            float(q_arm[3]), float(q_arm[4]), float(q_arm[5]),
            int(mode),
            float(q_gripper[0]), float(q_gripper[1]), float(q_gripper[2]),
        )

        crc = struct.pack("!I", binascii.crc32(data) & 0xFFFFFFFF)

        return URUDPClient.HEADER + data + crc + URUDPClient.FOOTER

    def send_to(
        self,
        target: Tuple[str, int],
        q_arm: List[float],
        mode: int,
        q_gripper: List[float],
    ) -> None:
        """
        向指定地址发送控制指令。

        Parameters
        ----------
        target:
            (ip, port) 目标地址。
        q_arm, mode, q_gripper:
            控制数据，见 pack()。
        """
        if self._sock is None:
            raise RuntimeError("UDP socket 未创建，请先调用 start()")

        # 发送限速
        if self.send_interval > 0:
            with self._send_lock:
                elapsed = time.time() - self._last_send_time
                if elapsed < self.send_interval:
                    time.sleep(self.send_interval - elapsed)
                self._last_send_time = time.time()

        packet = self.pack(q_arm, mode, q_gripper)
        self._sock.sendto(packet, target)
