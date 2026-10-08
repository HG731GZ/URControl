from __future__ import annotations

import numpy as np


class URRTDETorqueClient:
    """
    持续订阅实际电流换算的关节力矩，保留重力、摩擦等分量。

    使用官方 RTDE 客户端依赖 rtde，只订阅时间戳和六轴力矩。
    需要控制器版本 5.23.0 或 10.11.0 及以上，最高采集频率为 500 Hz。
    在同一采集线程中持续调用 read()，停止采集时关闭连接。
    """

    def __init__(self, host: str, frequency: float = 500.0):
        from rtde.rtde import RTDE

        self._receiver = RTDE(host)
        self._receiver.connect()
        self._receiver.send_output_setup(
            ["timestamp", "actual_current_as_torque"],
            ["DOUBLE", "VECTOR6D"],
            frequency=frequency,
        )
        self._receiver.send_start()

    def read(self) -> tuple[float, np.ndarray]:
        """
        等待接收数据，返回控制器时间戳（秒）和六轴力矩（Nm）。

        顺序为基座、肩部、肘部、腕部一、腕部二、腕部三。
        处理变慢时允许跳过旧帧；可用时间戳判断采样间隔。
        """
        state = self._receiver.receive()
        return state.timestamp, np.asarray(state.actual_current_as_torque, dtype=float)

    def close(self) -> None:
        """关闭只读订阅连接。"""
        self._receiver.disconnect()

    def __enter__(self) -> URRTDETorqueClient:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()

