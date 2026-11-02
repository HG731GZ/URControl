"""主窗口补偿关节力矩的后台读取，只消费当前控制通道提供的测量。"""

import threading
import time

from rtde.rtde import RTDEException


class URCompensatedTorqueReader:
    """按 2 ms 目标周期读取补偿力矩；界面与遥测共用缓存，不等待控制接口。"""

    def __init__(self, controller):
        self._controller = controller
        self._stop_event = threading.Event()
        self._reader_thread = None
        self._latest = None
        self._last_error = None

    def start_reader_thread(self):
        """本连接只启动一次读取线程。"""
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="URCompensatedTorqueReader", daemon=True)
        self._reader_thread.start()

    def _reader_loop(self):
        """不可用时清空缓存；读取错误交由主窗口显示，不重启控制脚本。"""
        try:
            while not self._stop_event.is_set():
                # 按 500 Hz 目标节拍调度，读取及等锁耗时计入本周期。
                next_read = time.monotonic() + .002
                sample = self._controller.get_compensated_torque_snapshot()
                received = time.monotonic()
                self._latest = (received, sample)
                self._stop_event.wait(max(0., next_read - received))
        except (OSError, RTDEException, RuntimeError, ValueError) as error:
            self._latest = None
            self._last_error = error

    def get_latest(self):
        """等待新样本或控制通道切换超过 500 ms 时留空，不显示陈旧力矩。"""
        snapshot = self.get_snapshot()
        if snapshot["status"] != "available":
            return None
        sample = snapshot["sample"]
        return sample["robot_timestamp_s"], sample["values_nm"]

    def get_snapshot(self):
        """返回带来源的当前测量，缺测和读取错误明确区分。"""
        if self._last_error is not None:
            return {"status": "error", "sample": None, "reason": str(self._last_error)}
        latest = self._latest
        if latest is None:
            return {"status": "unavailable", "sample": None, "reason": "尚未取得补偿力矩"}
        if time.monotonic() - latest[0] > .5:
            return {"status": "unavailable", "sample": None, "reason": "补偿力矩读取已超过 500 ms 未更新"}
        if latest[1] is None:
            return {"status": "unavailable", "sample": None, "reason": "当前控制程序或导纳载荷页未提供有效补偿力矩"}
        return {"status": "available", "sample": latest[1], "reason": None}

    def get_last_error(self):
        """返回后台读取错误，沿用主窗口的错误提示。"""
        return self._last_error

    def close(self):
        """先结束读取，再由拥有者关闭控制器连接。"""
        self._stop_event.set()
        if self._reader_thread is not None:
            self._reader_thread.join()
            self._reader_thread = None
        self._latest = None
