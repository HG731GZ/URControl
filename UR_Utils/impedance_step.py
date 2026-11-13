"""单次关节位置阶跃：先保持，再直接切换位置参考，目标速度始终为零。"""

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class JointStep:
    anchor: tuple
    axis: int
    offset_rad: float
    delay_s: float = 1.0
    hold_s: float = 5.0
    sample_hz: float = 100.0
    coordinate: str = "absolute"

    def __post_init__(self):
        if self.axis not in range(6) or len(self.anchor) != 6:
            raise ValueError("阶跃必须指定六轴起点及有效目标轴")
        if not all(math.isfinite(x) for x in (*self.anchor, self.offset_rad, self.delay_s, self.hold_s)):
            raise ValueError("阶跃参数必须是有限数")
        if self.offset_rad == 0:
            raise ValueError("阶跃偏置不能为零")
        if not 0.5 <= self.delay_s <= 10 or not 1 <= self.hold_s <= 30:
            raise ValueError("阶跃前保持须为 0.5～10 秒，响应观察须为 1～30 秒")
        if self.sample_hz != 100.0 or self.coordinate != "absolute":
            raise ValueError("阶跃固定使用 100 Hz、绝对关节角")

    @property
    def target(self):
        q = list(self.anchor)
        q[self.axis] += self.offset_rad
        return tuple(q)

    @property
    def rows(self):
        return tuple(
            self.anchor if i / self.sample_hz < self.delay_s else self.target
            for i in range(round(self.duration() * self.sample_hz) + 1)
        )

    def duration(self, play_hz=None):
        return self.delay_s + self.hold_s

    def at(self, elapsed, play_hz):
        return (
            self.anchor if elapsed < self.delay_s else self.target,
            min(elapsed, self.duration()) * self.sample_hz,
            elapsed >= self.duration(),
        )

    def validate_playback(self, config, play_hz):
        if play_hz != self.sample_hz:
            raise ValueError("阶跃不允许通过播放倍率改变保持时长")
        step_limit = 0.1
        if abs(self.offset_rad) > step_limit:
            raise ValueError(f"J{self.axis+1} 阶跃幅度须不超过 {step_limit:g} rad")

    def bind(self, anchor, config, play_hz):
        self.validate_playback(config, play_hz)
        endpoints = np.array([self.anchor, self.target])
        if np.any(np.abs(endpoints) > config["JOINT_POSITION_LIMIT_RAD"]):
            raise ValueError("阶跃目标超出关节绝对范围")
        if np.any(np.abs(endpoints - anchor) > config["ENVELOPE_RAD"]):
            raise ValueError("阶跃目标超出本轮运动包络")
        if max(abs(a - b) for a, b in zip(self.anchor, anchor)) > config["START_TOLERANCE_RAD"]:
            raise ValueError("当前位置已改变，请重新生成阶跃")
        return self

    def save(self, path):
        Path(path).write_text(json.dumps(
            {"record_type": "joint_impedance_step", "schema_version": 1, **asdict(self)},
            ensure_ascii=False, indent=2,
        ), encoding="utf-8")
