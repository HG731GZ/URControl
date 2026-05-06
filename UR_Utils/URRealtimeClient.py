import socket
import struct
import threading
import time
from typing import Optional, Tuple
from UR_Utils.URTcpClient import URTcpClient, URTcpTimeoutError
from UR_Utils.URRealtimeUtils import URRealtimeParser, URRealtimeState, URRealtimeParseError


class URRealtimeClient(URTcpClient):
    """
    UR Realtime Client。

    继承 URTcpClient，并在连接成功后启动后台线程持续读取机械臂状态。

    常用端口：
    - 30003: Realtime Interface，可读写，常用于实时数据与控制
    - 30013: Realtime Read-Only Interface，只读状态，更适合单纯采集数据
    """

    DEFAULT_PORT = 30003
    READ_ONLY_PORT = 30013

    def __init__(
            self,
            host: str,
            port: int = READ_ONLY_PORT,
            timeout: float = 2.0,
            strict_parser: bool = False,
            auto_connect: bool = False,
            max_packet_size: int = 4096,
    ):
        """
        Parameters
        ----------
        host:
            UR 机械臂 IP 地址。

        port:
            Realtime 端口。
            只读状态建议使用 30013。
            如果需要通过 Realtime 发送控制，可用 30003。

        timeout:
            socket 接收超时时间。

        strict_parser:
            是否严格解析字段。
            False 时，报文中不存在的字段会被置为 None。

        auto_connect:
            是否初始化后自动连接。

        max_packet_size:
            允许的最大报文长度，用于防止错误帧头导致异常大长度。
        """

        # 注意：这里不要把 auto_connect 传给父类。
        # 因为父类 __init__ 中如果调用 self.connect()，
        # 会触发本类重写后的 connect()，此时线程相关成员还没初始化。
        super().__init__(
            host=host,
            port=port,
            timeout=timeout,
            encoding="utf-8",
            recv_buffer_size=4096,
            terminator=b"\n",
            auto_connect=False,
        )

        self.parser = URRealtimeParser(strict=strict_parser)
        self.max_packet_size = max_packet_size

        self._reader_thread: Optional[threading.Thread] = None
        self._stop_reader_event = threading.Event()

        self._state_cond = threading.Condition()
        self._latest_state: Optional[URRealtimeState] = None
        self._latest_packet: Optional[bytes] = None
        self._last_error: Optional[BaseException] = None
        self._frame_count: int = 0
        self._last_update_time: Optional[float] = None

        if auto_connect:
            self.connect()

    # ============================================================
    # 连接与线程生命周期
    # ============================================================

    def connect(self) -> None:
        """
        建立 TCP 连接，并启动后台读取线程。
        """
        super().connect()
        self.start_reader_thread()

    def close(self) -> None:
        """
        停止后台线程并关闭 TCP 连接。
        """
        self._stop_reader_event.set()

        # 先关闭 socket，用于打断正在阻塞的 recv。
        super().close()

        self._join_reader_thread()

    def reconnect(self) -> None:
        """
        重新连接，并重新启动后台读取线程。
        """
        self.close()

        with self._state_cond:
            self._latest_state = None
            self._latest_packet = None
            self._last_error = None
            self._frame_count = 0
            self._last_update_time = None

        self._stop_reader_event.clear()
        self.connect()

    def start_reader_thread(self) -> None:
        """
        启动后台读取线程。

        一般不需要外部手动调用，因为 connect() 会自动调用。
        """
        if self._reader_thread is not None and self._reader_thread.is_alive():
            return

        self._stop_reader_event.clear()

        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name=f"URRealtimeReader-{self.host}:{self.port}",
            daemon=True,
        )
        self._reader_thread.start()

    def stop_reader_thread(self) -> None:
        """
        停止后台读取线程，但不主动重连。

        实际使用中通常直接调用 close()。
        """
        self.close()

    def _join_reader_thread(self, timeout: float = 1.0) -> None:
        thread = self._reader_thread

        if thread is None:
            return

        if threading.current_thread() is thread:
            return

        if thread.is_alive():
            thread.join(timeout=timeout)

    # ============================================================
    # 后台读取逻辑
    # ============================================================

    def _reader_loop(self) -> None:
        """
        后台线程主循环。

        持续读取完整报文，并解析为 URRealtimeState。
        """
        while not self._stop_reader_event.is_set():
            try:
                packet = self._read_packet()
                state = self.parser.parse(packet)

                with self._state_cond:
                    self._latest_packet = packet
                    self._latest_state = state
                    self._last_error = None
                    self._frame_count += 1
                    self._last_update_time = time.time()
                    self._state_cond.notify_all()

            except socket.timeout as e:
                # timeout 不一定代表连接断开，可能只是短时间没收到数据。
                with self._state_cond:
                    self._last_error = e
                    self._state_cond.notify_all()
                continue

            except OSError as e:
                # close() 时也会触发 OSError，此时不作为异常报错。
                if not self._stop_reader_event.is_set():
                    with self._state_cond:
                        self._last_error = e
                        self._state_cond.notify_all()
                break

            except BaseException as e:
                # 解析失败、连接断开等都会进入这里。
                with self._state_cond:
                    self._last_error = e
                    self._state_cond.notify_all()
                break

    def _read_packet(self) -> bytes:
        """
        读取一帧完整 Realtime 报文。

        Realtime 报文格式：
            int32 message_size, big-endian
            double data[...], big-endian
        """
        header = self._recv_exact(4)
        message_size = struct.unpack("!i", header)[0]

        if message_size < 4:
            raise URRealtimeParseError(
                f"非法 message_size: {message_size}"
            )

        if message_size > self.max_packet_size:
            raise URRealtimeParseError(
                f"message_size 异常: {message_size}, "
                f"超过 max_packet_size={self.max_packet_size}。"
                f"可能是帧头未对齐、端口错误或大小端解析错误。"
            )

        payload = self._recv_exact(message_size - 4)
        return header + payload

    def _recv_exact(self, n: int) -> bytes:
        """
        精确接收 n 个字节。

        TCP 是流式协议，一次 recv 不保证得到完整一帧，
        因此必须循环读取。
        """
        chunks = []
        remaining = n

        while remaining > 0:
            if self._stop_reader_event.is_set():
                raise OSError("Realtime reader 已停止")

            sock = self._sock
            if sock is None:
                raise OSError("Realtime socket 未连接")

            chunk = sock.recv(remaining)

            if not chunk:
                raise ConnectionError("UR Realtime 连接已断开")

            chunks.append(chunk)
            remaining -= len(chunk)

        return b"".join(chunks)

    # ============================================================
    # 对外数据接口
    # ============================================================

    def get_latest_state(self) -> Optional[URRealtimeState]:
        """
        获取最近一帧解析后的状态。

        非阻塞。
        如果还没有收到数据，则返回 None。
        """
        with self._state_cond:
            return self._latest_state

    def get_latest_packet(self) -> Optional[bytes]:
        """
        获取最近一帧原始二进制报文。

        一般调试时使用。
        """
        with self._state_cond:
            return self._latest_packet

    def wait_first_state(
            self,
            timeout: Optional[float] = None,
    ) -> Optional[URRealtimeState]:
        """
        等待第一帧数据。

        Parameters
        ----------
        timeout:
            最大等待时间，单位 s。
            None 表示一直等待。

        Returns
        -------
        URRealtimeState | None
            超时则返回 None。
        """
        with self._state_cond:
            if self._latest_state is not None:
                return self._latest_state

            self._state_cond.wait_for(
                lambda: self._latest_state is not None or self._last_error is not None,
                timeout=timeout,
            )

            return self._latest_state

    def wait_next_state(
            self,
            last_frame_count: Optional[int] = None,
            timeout: Optional[float] = None,
    ) -> Tuple[Optional[URRealtimeState], int]:
        """
        等待下一帧数据。

        Parameters
        ----------
        last_frame_count:
            上一次读取时的帧计数。
            如果为 None，则等待当前时刻之后的新一帧。

        timeout:
            最大等待时间，单位 s。
            None 表示一直等待。

        Returns
        -------
        state, frame_count:
            state 为最新状态；
            frame_count 为当前帧计数。
            如果超时且没有新数据，state 可能为 None。
        """
        with self._state_cond:
            if last_frame_count is None:
                target_count = self._frame_count
            else:
                target_count = last_frame_count

            self._state_cond.wait_for(
                lambda: self._frame_count > target_count or self._last_error is not None,
                timeout=timeout,
            )

            return self._latest_state, self._frame_count

    def get_frame_count(self) -> int:
        """
        获取已经成功解析的帧数量。
        """
        with self._state_cond:
            return self._frame_count

    def get_last_update_time(self) -> Optional[float]:
        """
        获取最近一次成功收到数据的本机时间戳。

        单位为 time.time() 的秒。
        """
        with self._state_cond:
            return self._last_update_time

    def get_last_error(self) -> Optional[BaseException]:
        """
        获取后台线程最近一次异常。

        如果数据正常更新，一般为 None。
        """
        with self._state_cond:
            return self._last_error

    def is_reader_alive(self) -> bool:
        """
        判断后台读取线程是否仍在运行。
        """
        return (
                self._reader_thread is not None
                and self._reader_thread.is_alive()
        )
