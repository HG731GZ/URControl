"""导纳专用 RTDE 传输：固定序列化版本，运行期按单包期限执行有限工作。"""

from __future__ import annotations

from importlib.metadata import version
import select
import socket
import struct
import time

from rtde import serialize
from rtde.rtde import Command

from UR_Utils.URScriptClient import URScriptClient
from UR_Utils.URTcpClient import URTcpError


RTDE_LIBRARY_VERSION = "2.8.4"
PRIMARY_MAX_MESSAGE_BYTES = 65536


def clock_sample() -> tuple[float, float]:
    """同时采样单调墙钟和本线程 CPU；墙钟包含调度、GIL 及系统调用等待。"""
    return time.monotonic(), time.thread_time()


def elapsed_sample(started: tuple[float, float]) -> dict:
    """生成一段实际完成工作的计时，不将配置预算当作可抢占保证。"""
    return clock_interval(started, clock_sample())


def clock_interval(started: tuple[float, float], finished: tuple[float, float]) -> dict:
    """由已采集的时间点生成分段记录，允许发送后再分配诊断对象。"""
    return {
        "wall_s": finished[0] - started[0], "cpu_s": finished[1] - started[1],
        "started_monotonic_s": started[0], "finished_monotonic_s": finished[0],
    }


class AdmittanceRTDE:
    """唯一通信线程使用的有限 RTDE 客户端，复用官方公开配方序列化接口。

    每次运行期读取至多执行一次非阻塞 recv 和一个帧的解包；剩余字节留给下次。
    发送仅补齐本包尚未发送的字节，共用一个绝对期限，失败后调用方终止本轮。
    Python 线程可能被系统调度暂停，因此有限工作量不等于硬实时墙钟上限。
    """

    def __init__(self, hostname: str, *, buffer_limit_bytes: int, port: int = 30004):
        installed = version("UrRtde")
        if installed != RTDE_LIBRARY_VERSION:
            raise RuntimeError(f"导纳 RTDE 适配要求 UrRtde {RTDE_LIBRARY_VERSION}，当前为 {installed}")
        self.hostname = hostname
        self.port = port
        self.buffer_limit_bytes = buffer_limit_bytes
        self._sock = None
        self._buffer = bytearray()
        self._input_config = {}
        self._output_config = None
        self._started = False
        self.last_receive_timing = {}
        self.last_send_timing = {}
        self.buffered_max_bytes = 0
        self.last_socket_read_bytes = 0
        self.last_receive_drained = False

    @property
    def buffered_bytes(self) -> int:
        """返回已读入但尚未消费的字节；不代表内核缓冲区或绝对网络年龄。"""
        return len(self._buffer)

    def connect(self) -> None:
        """上传前建立连接并明确使用协议 2；运行期 socket 始终非阻塞。"""
        self._sock = socket.create_connection((self.hostname, self.port), timeout=1.0)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock.setblocking(False)
        payload = self._exchange(Command.RTDE_REQUEST_PROTOCOL_VERSION, struct.pack(">H", 2))
        if payload != b"\x01":
            raise RuntimeError("控制器拒绝 RTDE 协议 2")

    def disconnect(self) -> None:
        """关闭当前唯一连接，不自动重连。"""
        if self._sock is not None:
            self._sock.close()
            self._sock = None
        self._started = False

    def get_controller_version(self) -> tuple[int, int, int, int]:
        """读取控制器版本，供本轮日志记录。"""
        return struct.unpack(">IIII", self._exchange(Command.RTDE_GET_URCONTROL_VERSION))

    def send_input_setup(self, names: list[str], types: list[str]):
        """使用官方公开 DataConfig 建立当前输入配方。"""
        payload = self._exchange(Command.RTDE_CONTROL_PACKAGE_SETUP_INPUTS, ",".join(names).encode())
        config = serialize.DataConfig.unpack_recipe(payload)
        if config.types != types:
            raise RuntimeError(f"导纳输入配方类型不匹配：{config.types}")
        config.names = names
        self._input_config[config.id] = config
        return serialize.DataObject.create_empty(names, config.id)

    def send_output_setup(self, names: list[str], types: list[str], *, frequency: float) -> bool:
        """建立固定输出配方；每次解包只处理该配方的一帧。"""
        request = struct.pack(">d", frequency) + ",".join(names).encode()
        config = serialize.DataConfig.unpack_recipe(self._exchange(Command.RTDE_CONTROL_PACKAGE_SETUP_OUTPUTS, request))
        if config.types != types:
            raise RuntimeError(f"导纳输出配方类型不匹配：{config.types}")
        config.names = names
        self._output_config = config
        return True

    def send_start(self) -> bool:
        """开始同步；开始之后不再发送配方或协议协商请求。"""
        self._started = self._exchange(Command.RTDE_CONTROL_PACKAGE_START) == b"\x01"
        return self._started

    def prepare_inputs(self, input_data) -> bytes:
        """一次组装完整小型输入包，将组包与实际 socket 发送分开计时。"""
        if not self._started:
            raise RuntimeError("导纳 RTDE 同步尚未开始")
        payload = self._input_config[input_data.recipe_id].pack(input_data)
        return self._frame(Command.RTDE_DATA_PACKAGE, payload)

    @staticmethod
    def _frame(command: int, payload: bytes) -> bytes:
        """按 RTDE 三字节包头构造完整报文。"""
        return struct.pack(">HB", len(payload) + 3, command) + payload

    def send_prepared(self, data: bytes, *, deadline: float) -> None:
        """在同一绝对期限内补齐 TCP 部分写；绝不把已发送前缀重新发一遍。"""
        view = memoryview(data)
        sent_bytes = 0
        wait_total = {"wall_s": 0.0, "cpu_s": 0.0}
        send_total = {"wall_s": 0.0, "cpu_s": 0.0}
        self.last_send_timing = {
            "socket_writable_wait": wait_total, "socket_send": send_total,
            "packet_bytes": len(data), "sent_bytes": 0, "send_calls": 0,
            "deadline_monotonic_s": deadline,
        }
        while sent_bytes < len(data):
            wait_started = clock_sample()
            remaining = deadline - wait_started[0]
            if remaining <= 0:
                raise TimeoutError("导纳 RTDE 单包发送期限已过")
            writable = select.select([], [self._sock], [], remaining)[1]
            wait = elapsed_sample(wait_started)
            self._accumulate(wait_total, wait)
            if not writable or wait["finished_monotonic_s"] >= deadline:
                raise TimeoutError("导纳 RTDE 等待可写超过单包期限")
            send_started = clock_sample()
            count = self._sock.send(view[sent_bytes:])
            sent = elapsed_sample(send_started)
            self._accumulate(send_total, sent)
            self.last_send_timing["send_calls"] += 1
            if count == 0:
                raise ConnectionError("导纳 RTDE 发送返回零字节")
            sent_bytes += count
            self.last_send_timing["sent_bytes"] = sent_bytes
            if sent["finished_monotonic_s"] > deadline:
                raise TimeoutError("导纳 RTDE 发送完成时已超过单包期限")

    @staticmethod
    def _accumulate(total: dict, sample: dict) -> None:
        """聚合同一包的部分写计时，保留实际起止和系统调用总耗时。"""
        if "started_monotonic_s" not in total:
            total["started_monotonic_s"] = sample["started_monotonic_s"]
        total["finished_monotonic_s"] = sample["finished_monotonic_s"]
        total["wall_s"] += sample["wall_s"]
        total["cpu_s"] += sample["cpu_s"]

    def _read_packet_once(self):
        """至多一次非阻塞读取与一次有限帧切片，不在库函数内部排空 socket。"""
        read_started = clock_sample()
        self.last_socket_read_bytes = 0
        self.last_receive_drained = False
        complete = len(self._buffer) >= 3 and len(self._buffer) >= struct.unpack_from(">H", self._buffer)[0]
        readable = [] if complete else select.select([self._sock], [], [], 0)[0]
        if not complete and not readable and not self._buffer:
            self.last_receive_drained = True
        if readable:
            capacity = self.buffer_limit_bytes - len(self._buffer)
            if capacity <= 0:
                raise RuntimeError("导纳 RTDE 未完成帧已占满限定缓冲")
            data = self._sock.recv(min(4096, capacity))
            if not data:
                raise ConnectionError("导纳 RTDE 连接已被控制器关闭")
            self._buffer.extend(data)
            self.last_socket_read_bytes = len(data)
            self.buffered_max_bytes = max(self.buffered_max_bytes, len(self._buffer))
        self.last_receive_timing = {"rtde_read": elapsed_sample(read_started)}
        if len(self._buffer) < 3:
            return None
        size, command = struct.unpack_from(">HB", self._buffer)
        if not 3 <= size <= self.buffer_limit_bytes:
            raise RuntimeError(f"导纳 RTDE 帧长度不符合缓冲契约：{size}")
        if len(self._buffer) < size:
            return None
        payload = bytes(self._buffer[3:size])
        del self._buffer[:size]
        return command, payload

    @staticmethod
    def _text_message(payload: bytes) -> None:
        """控制器错误立即上报；普通协议通知不影响状态新鲜度。"""
        message = serialize.Message.unpack(payload)
        if message.level in (serialize.Message.EXCEPTION_MESSAGE, serialize.Message.ERROR_MESSAGE):
            raise RuntimeError(f"RTDE 控制器消息：{message.source}: {message.message}")

    def receive_once(self):
        """读取并解码至多一个 RTDE 包，空返回不代表已经排空内核缓冲。"""
        packet = self._read_packet_once()
        unpack_started = clock_sample()
        state = None
        if packet is not None:
            command, payload = packet
            if command == Command.RTDE_DATA_PACKAGE:
                state = self._output_config.unpack(payload)
            elif command == Command.RTDE_TEXT_MESSAGE:
                self._text_message(payload)
            else:
                raise RuntimeError(f"导纳运行期收到非预期 RTDE 命令：{command}")
        self.last_receive_timing["rtde_unpack"] = elapsed_sample(unpack_started)
        return state

    def receive(self):
        """仅供上传前基线读取；等待总期限为一秒。"""
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            state = self.receive_once()
            if state is not None:
                return state
            select.select([self._sock], [], [], max(0.0, deadline - time.monotonic()))
        return None

    def _exchange(self, command: int, payload: bytes = b"") -> bytes:
        """上传前控制消息在同一秒级期限内完成，运行期不调用该协商路径。"""
        deadline = time.monotonic() + 1.0
        self.send_prepared(self._frame(command, payload), deadline=deadline)
        while time.monotonic() < deadline:
            packet = self._read_packet_once()
            if packet is not None:
                received_command, received_payload = packet
                if received_command == command:
                    return received_payload
                if received_command == Command.RTDE_TEXT_MESSAGE:
                    self._text_message(received_payload)
                else:
                    raise RuntimeError(f"RTDE 协商回复不匹配：{received_command}，期望 {command}")
            if packet is None:
                select.select([self._sock], [], [], max(0.0, deadline - time.monotonic()))
        raise TimeoutError(f"RTDE 协商超时，命令 {command}")


class AdmittanceScriptClient(URScriptClient):
    """上传后专供唯一导纳线程读取 Primary；一次处理一个有限消息。"""

    def send_script(self, script: str) -> None:
        """上传完整程序后切换非阻塞读取，不在运行期发送其他运动命令。"""
        super().send_script(script)
        self._ensure_connected().setblocking(False)

    def poll_script_error(self, *, deadline: float, max_read_bytes: int, buffer_limit_bytes: int):
        """每次最多一次 recv 与一个消息解析，预算不依赖可阻塞 socket。"""
        if time.monotonic() >= deadline:
            return None
        sock = self._ensure_connected()
        buffer = self._script_response_buffer
        message_limit = min(PRIMARY_MAX_MESSAGE_BYTES, buffer_limit_bytes)
        complete = len(buffer) >= 5 and len(buffer) >= struct.unpack_from(">I", buffer)[0]
        if not complete and select.select([sock], [], [], 0)[0]:
            capacity = message_limit - len(buffer)
            if capacity <= 0:
                raise URTcpError("Primary 未完成消息已占满限定缓冲")
            data = sock.recv(min(self.recv_buffer_size, max_read_bytes, capacity))
            if not data:
                raise URTcpError("Primary 连接已被控制器关闭")
            buffer.extend(data)
        if len(buffer) < 5 or time.monotonic() >= deadline:
            return None
        size, message_type = struct.unpack_from(">IB", buffer)
        if not 5 <= size <= message_limit:
            raise URTcpError(f"Primary 消息长度超出导纳读取契约：{size}")
        if len(buffer) < size:
            return None
        error = None
        if message_type == 20 and size >= 23 and buffer[14] == 10:
            line, column = struct.unpack_from(">II", buffer, 15)
            message = bytes(buffer[23:size]).rstrip(b"\x00").decode("utf-8", errors="replace")
            error = f"URScript 错误（第 {line} 行，第 {column} 列）：{message}"
        del buffer[:size]
        return error
