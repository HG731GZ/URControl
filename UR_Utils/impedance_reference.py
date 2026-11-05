"""源身份、提交身份和通信心跳分别推进，时间均为本机单调时钟。"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class JointTargetFrame:
    run_id: int
    source: str
    source_seq: int
    q_rad: tuple
    source_monotonic_s: float
    valid: bool = True
    end_of_source: bool = False
    trajectory_row_position: float = 0.0
    qd_rad_s: tuple | None = None

    def __post_init__(self):
        object.__setattr__(self, "q_rad", tuple(float(v) for v in self.q_rad))
        if (
            self.source not in ("trajectory", "step", "teleop")
            or not isinstance(self.source_seq, int)
            or not 1 <= self.source_seq < 2**31
            or not 1 <= self.run_id < 2**31
        ):
            raise ValueError("非法目标源或源序号")
        if len(self.q_rad) != 6 or not all(math.isfinite(v) for v in self.q_rad):
            raise ValueError("目标必须是六轴有限 rad")
        if self.source == "teleop":
            if self.qd_rad_s is None or len(self.qd_rad_s) != 6 or not all(map(math.isfinite, self.qd_rad_s)):
                raise ValueError("遥操作目标须含同帧六轴有限速度（rad/s）")
            object.__setattr__(self, "qd_rad_s", tuple(self.qd_rad_s))
        if (
            not math.isfinite(self.trajectory_row_position)
            or self.trajectory_row_position < 0
        ):
            raise ValueError("轨迹行进度必须为非负有限数")
        if not math.isfinite(self.source_monotonic_s):
            raise ValueError("源时刻必须有限")

    def check(self, run_id, now, config):
        if self.run_id != run_id or not self.valid:
            raise ValueError("目标运行身份或有效标记错误")
        age = now - self.source_monotonic_s
        if not 0 <= age <= config["SOURCE_TIMEOUT_S"]:
            raise ValueError("目标源已过期或时刻来自未来")
        if any(
            abs(q) > lim
            for q, lim in zip(self.q_rad, config["JOINT_POSITION_LIMIT_RAD"])
        ):
            raise ValueError("目标超出绝对关节范围")
        return age

