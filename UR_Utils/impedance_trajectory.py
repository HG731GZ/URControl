"""严格 CSV v1，冻结轨迹及绝对单调时间插值。"""

from dataclasses import dataclass
import csv
import io
import math
import numpy as np


@dataclass(frozen=True)
class JointTrajectory:
    rows: tuple
    sample_hz: float = 100.0
    coordinate: str = "offset"

    def __post_init__(self):
        rows = tuple(tuple(float(v) for v in row) for row in self.rows)
        object.__setattr__(self, "rows", rows)
        if (
            len(rows) < 2
            or any(len(row) != 6 for row in rows)
            or not np.isfinite(rows).all()
        ):
            raise ValueError("轨迹至少两行，固定六列且所有值有限")
        if (
            self.coordinate not in ("offset", "absolute")
            or not math.isfinite(self.sample_hz)
            or self.sample_hz <= 0
        ):
            raise ValueError("坐标或名义采样率不合法")
        if self.coordinate == "offset" and any(rows[0]):
            raise ValueError("相对轨迹第一行必须全零")

    def duration(self, play_hz=None):
        hz = self.sample_hz if play_hz is None else play_hz
        if not math.isfinite(hz) or hz <= 0:
            raise ValueError("播放采样率必须为正有限数")
        return (len(self.rows) - 1) / hz

    def at(self, elapsed, play_hz):
        self.duration(play_hz)
        pos = min(max(0.0, elapsed * play_hz), len(self.rows) - 1)
        i = min(int(pos), len(self.rows) - 2)
        q = np.array(self.rows[i]) + (pos - i) * (
            np.array(self.rows[i + 1]) - self.rows[i]
        )
        return tuple(q), pos, pos >= len(self.rows) - 1

    def bind(self, anchor, config, play_hz):
        self.validate_speed(config, play_hz)
        rows = np.array(self.rows) + (
            np.array(anchor) if self.coordinate == "offset" else 0
        )
        if np.any(np.abs(rows) > config["JOINT_POSITION_LIMIT_RAD"]):
            raise ValueError("绑定后的整条轨迹超出绝对范围")
        if np.any(np.abs(rows - anchor) > config["ENVELOPE_RAD"]):
            raise ValueError("整条轨迹超出本轮运动包络")
        if np.max(np.abs(rows[0] - anchor)) > config["START_TOLERANCE_RAD"]:
            raise ValueError("轨迹起点未同步，请先显式移动到起点")
        return JointTrajectory(tuple(map(tuple, rows)), self.sample_hz, "absolute")

    def validate_speed(self, config, play_hz):
        self.duration(play_hz)
        rows = np.asarray(self.rows)
        velocity = np.diff(rows, axis=0) * play_hz
        acceleration = np.diff(velocity, axis=0) * play_hz
        if np.any(np.abs(velocity) > np.array(config["REF_MAX_QD_RAD_S"]) * 0.9):
            raise ValueError("播放速度超出参考速度预算")
        if acceleration.size and np.any(
            np.abs(acceleration) > np.array(config["REF_MAX_QDD_RAD_S2"]) * 0.9
        ):
            raise ValueError("播放速度超出参考加速度预算")
        # 100 Hz 主机发布，最多允许每次发布跨越的角度为源跳变门槛。
        if np.any(
            np.max(np.abs(velocity), axis=0) / 100 > config["MAX_SOURCE_STEP_RAD"]
        ):
            raise ValueError("轨迹超出 100 Hz 目标发布带宽")

    def csv_text(self):
        output = io.StringIO()
        output.write(
            f"# schema_version=1\n# unit=rad\n# coordinate={self.coordinate}\n# sample_hz={self.sample_hz}\n"
        )
        writer = csv.writer(output)
        writer.writerow([f"q{i}" for i in range(1, 7)])
        writer.writerows(self.rows)
        return output.getvalue()

    def save(self, path):
        from pathlib import Path

        Path(path).write_text(self.csv_text())

    @classmethod
    def load(cls, path):
        from pathlib import Path

        metadata, lines = {}, []
        for line in Path(path).read_text().splitlines():
            if line.startswith("#"):
                key, value = line[1:].strip().split("=", 1)
                if key in metadata:
                    raise ValueError("重复元数据")
                metadata[key] = value
            elif line.strip():
                lines.append(line)
        if (
            set(metadata) != {"schema_version", "unit", "coordinate", "sample_hz"}
            or metadata["schema_version"] != "1"
            or metadata["unit"] != "rad"
        ):
            raise ValueError("只支持 schema_version=1、unit=rad 的标准六轴 CSV")
        reader = csv.reader(lines)
        if next(reader) != [f"q{i}" for i in range(1, 7)]:
            raise ValueError("列必须为 q1～q6")
        return cls(
            tuple(tuple(float(v) for v in row) for row in reader),
            float(metadata["sample_hz"]),
            metadata["coordinate"],
        )


def generate(
    kind="single", axis=0, amplitude=0.05, segment_s=2.0, hold_s=0.5, sample_hz=100.0
):
    if kind not in ("hold", "single", "staggered", "sine") or axis not in range(6):
        raise ValueError("生成器类型或轴号错误")
    if (
        not all(math.isfinite(v) for v in (amplitude, segment_s, hold_s, sample_hz))
        or segment_s <= 0
        or hold_s < 0
        or sample_hz <= 0
    ):
        raise ValueError("生成参数不合法")
    rows = [np.zeros(6)]
    if kind == "sine":
        t = np.linspace(0, 4 * segment_s, round(4 * segment_s * sample_hz) + 1)
        for v in (
            amplitude
            * np.sin(2 * np.pi * t / segment_s)
            * np.sin(np.pi * t / t[-1]) ** 2
        ):
            q = np.zeros(6)
            q[axis] = v
            rows.append(q)
    elif kind != "hold":
        for j in range(6) if kind == "staggered" else [axis]:
            for target in (amplitude, 0.0, -amplitude, 0.0):
                start = rows[-1].copy()
                goal = start.copy()
                goal[j] = target
                for u in np.linspace(0, 1, max(2, round(segment_s * sample_hz) + 1))[
                    1:
                ]:
                    rows.append(
                        start + (goal - start) * (10 * u**3 - 15 * u**4 + 6 * u**5)
                    )
                rows.extend([goal.copy() for _ in range(round(hold_s * sample_hz))])
    rows.extend([np.zeros(6) for _ in range(max(1, round(hold_s * sample_hz)))])
    return JointTrajectory(tuple(map(tuple, rows)), sample_hz)


def generate_axis_tuning(anchor, axis, offset_rad, max_speed_rad_s, config):
    """从当前角度依次到正偏置、负偏置和起点，三段均为五次平滑曲线。"""
    if axis not in range(6) or len(anchor) != 6:
        raise ValueError("目标轴或当前位置无效")
    if not all(math.isfinite(v) for v in (*anchor, offset_rad, max_speed_rad_s)):
        raise ValueError("轨迹参数必须为有限数")
    if offset_rad == 0 or max_speed_rad_s <= 0:
        raise ValueError("角度偏置不能为零，最大速度必须为正")
    hz = 100.0
    speed = min(max_speed_rad_s, config["REF_MAX_QD_RAD_S"][axis] * 0.85)
    acceleration = config["REF_MAX_QDD_RAD_S2"][axis] * 0.85
    start = np.asarray(anchor, dtype=float)
    rows = [start.copy()]
    for offset in (offset_rad, -offset_rad, 0.0):
        goal = np.asarray(anchor, dtype=float).copy()
        goal[axis] += offset
        distance = abs(goal[axis] - start[axis])
        # 五次插值的峰值速度系数为 1.875，峰值加速度系数为 10/√3。
        duration = max(
            1.875 * distance / speed,
            math.sqrt((10 / math.sqrt(3)) * distance / acceleration),
            0.1,
        )
        count = math.ceil(duration * hz)
        if len(rows) + count > 60001:
            raise ValueError("轨迹超过 600 秒，请提高最大速度或减小角度偏置")
        for index in range(1, count + 1):
            u = index / count
            blend = 10 * u**3 - 15 * u**4 + 6 * u**5
            rows.append(start + (goal - start) * blend)
        start = goal
    trajectory = JointTrajectory(tuple(map(tuple, rows)), hz, "absolute")
    return trajectory.bind(anchor, config, hz)
