from UR_Utils.URTcpClient import URTcpClient


class URDashboardClient(URTcpClient):
    """
    UR Dashboard 服务客户端。

    Dashboard Server 常用端口：29999

    典型功能：
    - 加载程序
    - 播放程序
    - 停止程序
    - 上电
    - 释放刹车
    - 查询机器人状态
    """

    DEFAULT_PORT = 29999

    def __init__(
        self,
        host: str,
        port: int = DEFAULT_PORT,
        timeout: float = 3.0,
        auto_connect: bool = False,
    ):
        super().__init__(
            host=host,
            port=port,
            timeout=timeout,
            encoding="utf-8",
            terminator=b"\n",
            auto_connect=auto_connect,
        )

    def connect(self) -> str:
        """
        Dashboard Server 建立连接后，通常会先返回一行欢迎信息。
        """
        super().connect()
        return self.recv_text()

    def command(self, cmd: str) -> str:
        """
        发送 Dashboard 命令。
        """
        if self.is_connected:
            return self.request_text(cmd)
        return "Socket is not connected"

    def power_on(self) -> str:
        return self.command("power on")

    def power_off(self) -> str:
        return self.command("power off")

    def brake_release(self) -> str:
        return self.command("brake release")

    def unlock_protective_stop(self) -> str:
        return self.command("unlock protective stop")

    def close_popup(self) -> str:
        return self.command("close popup")

    def load_program(self, program_name: str) -> str:
        """
        加载 Polyscope 程序，例如：
        load_program("test.urp")
        """
        return self.command(f"load {program_name}")

    def play(self) -> str:
        return self.command("play")

    def pause(self) -> str:
        return self.command("pause")

    def stop(self) -> str:
        return self.command("stop")

    def running(self) -> str:
        return self.command("running")

    def robot_mode(self) -> str:
        return self.command("robotmode")

    def program_state(self) -> str:
        return self.command("programState")

    def safety_status(self) -> str:
        return self.command("safetystatus")

    def shutdown(self) -> str:
        return self.command("shutdown")

    def quit(self) -> str:
        """
        Dashboard Server 的 quit 命令会关闭服务端当前连接。
        """
        try:
            return self.command("quit")
        finally:
            self.close()
