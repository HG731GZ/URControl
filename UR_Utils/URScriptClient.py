import math
from typing import Literal, Optional, Sequence

import numpy as np

from UR_Utils.URTcpClient import URTcpClient
from UR_Utils.ur_pose_math import pose_trans, twist_tool_to_base

class URScriptClient(URTcpClient):
    """
    简单 URScript 发送 Client。

    常用端口：
    - 30001 Primary Interface
    - 30002 Secondary Interface
    - 30003 Realtime Interface

    简单发送脚本时，常用 30002。
    """

    DEFAULT_PORT = 30002

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

    def send_script(self, script: str) -> None:
        """
        发送一段 URScript。
        """
        if not script.endswith("\n"):
            script += "\n"
        if self.is_connected:
            self.send_text(script, append_terminator=False)


    def movej(
        self,
        q,
        a: float = 1.2,
        v: float = 0.25,
        t: float = 0.0,
        r: float = 0.0,
    ) -> None:
        """
        发送 movej 指令。

        q: 6 个关节角，单位 rad
        """
        if len(q) != 6:
            raise ValueError("movej 的 q 必须包含 6 个关节角")

        q_str = ", ".join(f"{x:.6f}" for x in q)
        script = f"movej([{q_str}], a={a}, v={v}, t={t}, r={r})"
        self.send_script(script)

    def movel(
        self,
        pose: Sequence[float],
        a: float = 1.2,
        v: float = 0.25,
        t: float = 0.0,
        r: float = 0.0,
        frame: Literal["tool", "base_add", "base_abs"] = "base_add",
        actual_pose: Optional[Sequence[float]] = None,
    ) -> None:
        """
        发送 movel 指令。

        frame="base_add"：
            target_pose = actual_pose + pose

        frame="tool"：
            target_pose = poseTrans(actual_pose, pose)

        frame="base_abs"：
            直接把 pose 解释为 base 坐标系下的绝对 TCP 目标位姿。

        由于 URScriptClient 无法自行读取机械臂状态，因此必须由外部传入 actual_pose
        作为当前 TCP pose 基准；当 frame="base_abs" 时不需要 actual_pose。
        """
        self._check_vector(pose, 6, "pose")

        if frame == "base_abs":
            target_pose = np.asarray(pose, dtype=float)
        else:
            if frame not in ("tool", "base_add", "base_abs"):
                raise ValueError("frame must be 'tool', 'base_add' or 'base_abs'")

            self._check_vector(actual_pose, 6, "actual_pose")

            actual_pose_arr = np.asarray(actual_pose, dtype=float)
            delta_pose_arr = np.asarray(pose, dtype=float)

            if frame == "tool":
                target_pose = pose_trans(actual_pose_arr, delta_pose_arr)
            else:
                target_pose = actual_pose_arr + delta_pose_arr

        pose_str = ", ".join(f"{x:.6f}" for x in target_pose)
        script = f"movel(p[{pose_str}], a={a}, v={v}, t={t}, r={r})"
        self.send_script(script)

    def speedj(
            self,
            qd: Sequence[float],
            a: float = 1.2,
            t: float = 0.1,
    ) -> None:
        """
        关节速度控制。

        对应 URScript:
            speedj(qd, a, t)

        Parameters
        ----------
        qd:
            6 个关节速度，单位 rad/s。
            例如 [0.1, 0, 0, 0, 0, 0]

        a:
            关节加速度，单位 rad/s^2。

        t:
            指令持续时间，单位 s。

        注意：
            如果你要连续速度控制，建议以固定周期重复发送 speedj。
            停止时调用 stopj()。
        """
        self._check_vector(qd, 6, "qd")

        qd_str = self._format_list(qd)
        script = f"speedj([{qd_str}], a={a}, t={t})"
        self.send_script(script)

    # ============================================================
    # 末端速度控制
    # ============================================================

    def speedl(
            self,
            xd: Sequence[float],
            a: float = 0.5,
            t: float = 0.1,
            frame: Literal["tool", "base_add"] = "base_add",
            actual_pose: Optional[Sequence[float]] = None,
    ) -> None:
        """
        末端速度控制。

        对应 URScript:
            speedl(xd, a, t)

        Parameters
        ----------
        xd:
            TCP 空间速度，格式为：
            [vx, vy, vz, wx, wy, wz]

            vx, vy, vz:
                末端线速度，单位 m/s。

            wx, wy, wz:
                末端角速度，单位 rad/s。

        a:
            TCP 加速度，单位 m/s^2。

        t:
            指令持续时间，单位 s。

        frame:
            "base_add" 时，xd 直接解释为 base 坐标系下的 TCP 速度。
            "tool" 时，xd 解释为当前 TCP 坐标系下的速度，并基于 actual_pose
            转换到 base 坐标系。

        注意：
            如果你要连续末端速度控制，建议以固定周期重复发送 speedl。
            停止时调用 stopl()。
        """
        if frame not in ("tool", "base_add"):
            raise ValueError("frame must be 'tool' or 'base_add'")

        self._check_vector(xd, 6, "xd")
        if frame == "tool":
            self._check_vector(actual_pose, 6, "actual_pose")

        xd_arr = np.asarray(xd, dtype=float)
        if frame == "tool":
            xd_arr = twist_tool_to_base(np.asarray(actual_pose, dtype=float), xd_arr)

        xd_str = self._format_list(xd_arr)
        script = f"speedl([{xd_str}], a={a}, t={t})"
        self.send_script(script)

    # ============================================================
    # 停止函数
    # ============================================================

    def stopj(self, a: float = 2.0) -> None:
        """
        停止关节速度运动。

        a:
            停止加速度，单位 rad/s^2。
        """
        self.send_script(f"stopj(a={a})")

    def stopl(self, a: float = 1.2) -> None:
        """
        停止末端速度运动。

        a:
            停止加速度，单位 m/s^2。
        """
        self.send_script(f"stopl(a={a})")

    # ============================================================
    # 内部工具函数
    # ============================================================

    @staticmethod
    def _check_vector(vec: Optional[Sequence[float]], length: int, name: str) -> None:
        if vec is None:
            raise ValueError(f"{name} 不能为空")

        if len(vec) != length:
            raise ValueError(f"{name} 必须包含 {length} 个元素")

        for i, value in enumerate(vec):
            if not isinstance(value, (int, float)):
                raise TypeError(f"{name}[{i}] 不是数字: {value}")

            if not math.isfinite(value):
                raise ValueError(f"{name}[{i}] 不是有限值: {value}")

    @staticmethod
    def _format_list(vec: Sequence[float]) -> str:
        return ", ".join(f"{float(x):.6f}" for x in vec)
