import struct
from dataclasses import dataclass
from typing import Dict, List, Optional, Any


class URRealtimeParseError(Exception):
    pass


@dataclass
class URRealtimeState:
    """
    UR Realtime Interface 解析结果。

    message_size:
        原始报文长度，单位 byte。
    raw_values:
        message_size 后面的 double 序列。
        第 1 个 double 对应官方表格中的 Gnuplot col. 1。
    fields:
        按字段名解析出来的结果。
    """
    message_size: int
    raw_values: List[float]
    fields: Dict[str, Any]

    def get(self, name: str, default=None):
        return self.fields.get(name, default)

    @property
    def time(self) -> Optional[float]:
        return self.get("time")

    @property
    def q_actual(self) -> Optional[List[float]]:
        return self.get("q_actual")

    @property
    def qd_actual(self) -> Optional[List[float]]:
        return self.get("qd_actual")

    @property
    def tcp_pose(self) -> Optional[List[float]]:
        """
        [x, y, z, rx, ry, rz]
        位置单位 m，姿态为旋转向量 rad。
        """
        return self.get("tool_vector_actual")

    @property
    def tcp_speed(self) -> Optional[List[float]]:
        return self.get("tcp_speed_actual")

    @property
    def tcp_force(self) -> Optional[List[float]]:
        return self.get("tcp_force")

    @property
    def robot_mode(self) -> Optional[int]:
        value = self.get("robot_mode")
        return None if value is None else int(round(value))

    @property
    def safety_mode(self) -> Optional[int]:
        value = self.get("safety_mode")
        return None if value is None else int(round(value))

    @property
    def safety_status(self) -> Optional[int]:
        value = self.get("safety_status")
        return None if value is None else int(round(value))

    @property
    def program_state(self) -> Optional[int]:
        value = self.get("program_state")
        return None if value is None else int(round(value))

    def digital_inputs_as_int(self) -> Optional[int]:
        value = self.get("digital_input_bits")
        return None if value is None else int(round(value))

    def digital_outputs_as_int(self) -> Optional[int]:
        value = self.get("digital_outputs")
        return None if value is None else int(round(value))

    def get_digital_input(self, index: int) -> Optional[bool]:
        """
        index: 0 ~ 63
        """
        bits = self.digital_inputs_as_int()
        if bits is None:
            return None
        return bool((bits >> index) & 1)

    def get_digital_output(self, index: int) -> Optional[bool]:
        """
        index: 0 ~ 63
        """
        bits = self.digital_outputs_as_int()
        if bits is None:
            return None
        return bool((bits >> index) & 1)


class URRealtimeParser:
    """
    UR Realtime Client Interface 数据解析器。

    适用于官方 RealTime 5.9 / 5.10 表格中列出的 e-Series 数据格式。
    报文结构：
        int32 message_size
        double data[...]
    """

    # 官方表格中的 Gnuplot col. 是从 1 开始编号的 double 序列索引。
    # 这里的 start_col 也采用 1-based，便于和官方文档对照。
    FIELD_SPECS = [
        ("time", 1, 1),

        ("q_target", 2, 6),
        ("qd_target", 8, 6),
        ("qdd_target", 14, 6),
        ("i_target", 20, 6),
        ("m_target", 26, 6),

        ("q_actual", 32, 6),
        ("qd_actual", 38, 6),
        ("i_actual", 44, 6),
        ("i_control", 50, 6),

        ("tool_vector_actual", 56, 6),
        ("tcp_speed_actual", 62, 6),
        ("tcp_force", 68, 6),
        ("tool_vector_target", 74, 6),
        ("tcp_speed_target", 80, 6),

        ("digital_input_bits", 86, 1),
        ("motor_temperatures", 87, 6),
        ("controller_timer", 93, 1),
        ("test_value", 94, 1),
        ("robot_mode", 95, 1),
        ("joint_modes", 96, 6),
        ("safety_mode", 102, 1),

        # 注意：103 ~ 108 在官方 5.9/5.10 表格中没有给出有效字段。
        ("tool_accelerometer", 109, 3),

        # 注意：112 ~ 117 在官方表格中没有给出有效字段。
        ("speed_scaling", 118, 1),
        ("linear_momentum_norm", 119, 1),

        # 注意：120 ~ 121 在官方表格中没有给出有效字段。
        ("v_main", 122, 1),
        ("v_robot", 123, 1),
        ("i_robot", 124, 1),
        ("v_actual", 125, 6),
        ("digital_outputs", 131, 1),
        ("program_state", 132, 1),
        ("elbow_position", 133, 3),
        ("elbow_velocity", 136, 3),
        ("safety_status", 139, 1),

        # 注意：140 ~ 142 在官方表格中没有给出有效字段。
        ("payload_mass", 143, 1),
        ("payload_cog", 144, 3),
        ("payload_inertia", 147, 6),
    ]

    MAX_EXPECTED_SIZE = 1220

    def __init__(self, strict: bool = False):
        """
        Parameters
        ----------
        strict:
            True  : 字段不完整时抛出异常；
            False : 字段不完整时该字段置为 None。
        """
        self.strict = strict

    def parse(self, packet: bytes) -> URRealtimeState:
        """
        解析一帧完整 Realtime 报文。

        Parameters
        ----------
        packet:
            必须是一整帧数据，包含前 4 字节 message_size。

        Returns
        -------
        URRealtimeState
        """
        if len(packet) < 4:
            raise URRealtimeParseError("数据长度不足，无法读取 message_size")

        message_size = struct.unpack_from("!i", packet, 0)[0]

        if message_size <= 0:
            raise URRealtimeParseError(f"非法 message_size: {message_size}")

        if len(packet) < message_size:
            raise URRealtimeParseError(
                f"数据不完整: len(packet)={len(packet)}, message_size={message_size}"
            )

        # 只解析本帧声明长度内的数据，忽略后面可能粘包的数据。
        frame = packet[:message_size]
        payload = frame[4:]

        if len(payload) % 8 != 0:
            raise URRealtimeParseError(
                f"payload 长度不是 8 的整数倍: {len(payload)}"
            )

        double_count = len(payload) // 8

        if double_count > 0:
            raw_values = list(struct.unpack_from(f"!{double_count}d", payload, 0))
        else:
            raw_values = []

        fields = self._parse_fields(raw_values)

        return URRealtimeState(
            message_size=message_size,
            raw_values=raw_values,
            fields=fields,
        )

    def _parse_fields(self, values: List[float]) -> Dict[str, Any]:
        fields: Dict[str, Any] = {}

        for name, start_col, count in self.FIELD_SPECS:
            start = start_col - 1
            end = start + count

            if end <= len(values):
                data = values[start:end]

                if count == 1:
                    fields[name] = data[0]
                else:
                    fields[name] = data
            else:
                if self.strict:
                    raise URRealtimeParseError(
                        f"字段 {name} 不完整: 需要 double[{start}:{end}], "
                        f"但当前只有 {len(values)} 个 double"
                    )
                fields[name] = None

        return fields

    @staticmethod
    def parse_from_values(message_size: int, values: List[float]) -> URRealtimeState:
        """
        如果你已经自己把 double 序列解析出来了，可以用这个方法转成字段字典。
        """
        parser = URRealtimeParser(strict=False)
        fields = parser._parse_fields(values)
        return URRealtimeState(
            message_size=message_size,
            raw_values=values,
            fields=fields,
        )