import socket
import threading
from typing import Optional, Union


class URTcpError(Exception):
    """UR TCP 通信基础异常"""
    pass


class URTcpTimeoutError(URTcpError):
    """UR TCP 接收超时异常"""
    pass


class URTcpDisconnectedError(URTcpError):
    """UR TCP 连接断开异常"""
    pass


class URTcpClient:
    """
    UR 机械臂 TCP Client 基类。

    设计目标：
    1. 只负责底层 TCP 通信；
    2. 支持文本命令和二进制数据；
    3. 支持继承扩展，例如 Dashboard Server、URScript 发送端等；
    4. 提供 connect / close / reconnect / send / recv / request 等通用方法。
    """

    def __init__(
        self,
        host: str,
        port: int,
        timeout: float = 3.0,
        encoding: str = "utf-8",
        recv_buffer_size: int = 4096,
        terminator: bytes = b"\n",
        auto_connect: bool = False,
    ):
        """
        Parameters
        ----------
        host : str
            UR 机械臂 IP 地址，例如 "192.168.1.10"
        port : int
            TCP 端口，例如 Dashboard Server 通常为 29999
        timeout : float
            socket 超时时间，单位 s
        encoding : str
            文本编码方式
        recv_buffer_size : int
            单次接收缓存大小
        terminator : bytes
            文本协议的结束符，Dashboard Server 通常为 b"\\n"
        auto_connect : bool
            是否在初始化时自动连接
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.encoding = encoding
        self.recv_buffer_size = recv_buffer_size
        self.terminator = terminator

        self._sock: Optional[socket.socket] = None
        self._lock = threading.RLock()

        if auto_connect:
            self.connect()

    # =========================
    # 连接管理
    # =========================

    @property
    def is_connected(self) -> bool:
        return self._sock is not None

    def connect(self) -> None:
        """
        建立 TCP 连接。
        """
        with self._lock:
            if self._sock is not None:
                return

            try:
                sock = socket.create_connection(
                    (self.host, self.port),
                    timeout=self.timeout,
                )
                sock.settimeout(self.timeout)
                self._sock = sock

            except socket.timeout as e:
                raise URTcpTimeoutError(
                    f"连接 UR 机械臂超时: {self.host}:{self.port}"
                ) from e

            except OSError as e:
                raise URTcpError(
                    f"无法连接 UR 机械臂: {self.host}:{self.port}, error={e}"
                ) from e

    def close(self) -> None:
        """
        关闭 TCP 连接。
        """
        with self._lock:
            if self._sock is not None:
                try:
                    self._sock.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass

                try:
                    self._sock.close()
                finally:
                    self._sock = None

    def reconnect(self) -> None:
        """
        重新连接。
        """
        with self._lock:
            self.close()
            self.connect()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def __del__(self):
        self.close()

    # =========================
    # 底层发送与接收
    # =========================

    def _ensure_connected(self) -> socket.socket:
        if self._sock is None:
            raise URTcpDisconnectedError(
                f"尚未连接 UR 机械臂: {self.host}:{self.port}"
            )
        return self._sock

    def send_bytes(self, data: bytes) -> None:
        """
        发送原始 bytes 数据。
        """
        with self._lock:
            sock = self._ensure_connected()

            try:
                sock.sendall(data)

            except socket.timeout as e:
                raise URTcpTimeoutError("发送数据超时") from e

            except OSError as e:
                self.close()
                raise URTcpDisconnectedError(
                    f"发送数据失败，连接可能已断开: {e}"
                ) from e

    def send_text(
        self,
        text: str,
        append_terminator: bool = True,
    ) -> None:
        """
        发送文本命令。

        Dashboard Server 通常要求命令以换行符结束。
        """
        data = text.encode(self.encoding)

        if append_terminator and not data.endswith(self.terminator):
            data += self.terminator

        self.send_bytes(data)

    def recv_once(self) -> bytes:
        """
        接收一次数据。

        注意：TCP 是流式协议，一次 recv 不一定是一条完整消息。
        """
        with self._lock:
            sock = self._ensure_connected()

            try:
                data = sock.recv(self.recv_buffer_size)

                if not data:
                    self.close()
                    raise URTcpDisconnectedError("连接已被对端关闭")

                return data

            except socket.timeout as e:
                raise URTcpTimeoutError("接收数据超时") from e

            except OSError as e:
                self.close()
                raise URTcpDisconnectedError(
                    f"接收数据失败，连接可能已断开: {e}"
                ) from e

    def recv_until(
        self,
        terminator: Optional[bytes] = None,
        max_bytes: int = 1024 * 1024,
    ) -> bytes:
        """
        持续接收，直到遇到结束符。

        适合 Dashboard Server 这类一问一答的文本协议。
        """
        if terminator is None:
            terminator = self.terminator

        chunks = []
        total_size = 0

        while True:
            chunk = self.recv_once()
            chunks.append(chunk)
            total_size += len(chunk)

            if terminator in chunk:
                break

            if total_size > max_bytes:
                raise URTcpError(
                    f"接收数据超过最大限制: {max_bytes} bytes"
                )

        return b"".join(chunks)

    def recv_text(
        self,
        terminator: Optional[bytes] = None,
        strip: bool = True,
    ) -> str:
        """
        接收文本数据。
        """
        data = self.recv_until(terminator=terminator)
        text = data.decode(self.encoding, errors="replace")

        if strip:
            text = text.strip()

        return text

    # =========================
    # 请求-响应封装
    # =========================

    def request_bytes(
        self,
        data: bytes,
        terminator: Optional[bytes] = None,
    ) -> bytes:
        """
        发送 bytes 请求，并等待响应。
        """
        with self._lock:
            self.send_bytes(data)
            return self.recv_until(terminator=terminator)

    def request_text(
        self,
        command: str,
        append_terminator: bool = True,
        strip: bool = True,
    ) -> str:
        """
        发送文本命令，并等待文本响应。

        适合 Dashboard Server。
        """
        with self._lock:
            self.send_text(command, append_terminator=append_terminator)
            return self.recv_text(strip=strip)