from __future__ import annotations

import threading

import numpy as np
from rtde.rtde import RTDE, RTDEException


class URRTDETorqueClient:
    """
    持续订阅实际电流换算的关节力矩，保留重力、摩擦等分量。

    使用官方 RTDE 客户端依赖 rtde，只订阅时间戳和六轴力矩。
    需要控制器版本 5.23.0 或 10.11.0 及以上，最高采集频率为 500 Hz。
    可同步调用 read()，或启动后台读取后通过 get_latest() 获取缓存。
    """

    def __init__(self, host: str, frequency: float = 500.0):
        self._stop_event = threading.Event()
        self._reader_thread = None
        self._lock = threading.Lock()
        self._latest = None
        self._last_error = None
        self._receiver = RTDE(host)
        try:
            self._receiver.connect()
            self._receiver.send_output_setup(
                ["timestamp", "actual_current_as_torque"],
                ["DOUBLE", "VECTOR6D"],
                frequency=frequency,
            )
            self._receiver.send_start()
        except (OSError, RTDEException, ValueError):
            self._receiver.disconnect()
            raise

    def read(self) -> tuple[float, np.ndarray]:
        """
        等待接收数据，返回控制器时间戳（秒）和六轴力矩（Nm）。

        顺序为基座、肩部、肘部、腕部一、腕部二、腕部三。
        处理变慢时允许跳过旧帧；可用时间戳判断采样间隔。
        """
        state = self._receiver.receive()
        if state is None:
            raise RuntimeError('关节力矩订阅接收超时')
        return state.timestamp, np.asarray(state.actual_current_as_torque, dtype=float)

    def start_reader_thread(self) -> None:
        """启动缓存读取线程；同一连接只启动一次。"""
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name='URTorqueReader', daemon=True)
        self._reader_thread.start()

    def _reader_loop(self) -> None:
        try:
            while not self._stop_event.is_set():
                sample = self.read()
                with self._lock:
                    self._latest = sample
        except (OSError, RTDEException, RuntimeError) as exc:
            with self._lock:
                self._latest = None
                self._last_error = exc

    def get_latest(self):
        """非阻塞返回最新控制器时间戳和六轴力矩。"""
        with self._lock:
            return self._latest

    def get_last_error(self):
        """返回后台读取错误，交给界面显示。"""
        with self._lock:
            return self._last_error

    def close(self) -> None:
        """等待当前读取完成或超时，再关闭只读订阅连接。"""
        self._stop_event.set()
        if self._reader_thread is not None:
            self._reader_thread.join()
            self._reader_thread = None
        self._receiver.disconnect()

    def __enter__(self) -> URRTDETorqueClient:
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.close()
