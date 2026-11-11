"""主界面的只读实时状态进程，避免 RTDE 原生初始化占用 GIL 时阻塞收包。"""

import multiprocessing
import threading

from UR_Utils.URRealtimeClient import URRealtimeClient
from UR_Utils.control_process_ipc import LatestSnapshot


def _receive_states(host, stop_event, first_state, snapshots):
    """子进程持续接收；只保留最新帧，界面阻塞时不积压旧状态。"""
    client = URRealtimeClient(host)
    try:
        client.connect()
        frame_count = 0
        first_frame = True
        while not stop_event.is_set():
            client.wait_next_state(frame_count, timeout=0.1)
            snapshot = client.get_snapshot()
            if snapshot["error"] is not None:
                raise snapshot["error"]
            if snapshot["frame_count"] == frame_count:
                continue
            frame_count = snapshot["frame_count"]
            if snapshots.publish(snapshot) and first_frame:
                first_state.set()
                first_frame = False
    except Exception as exc:
        # 错误跨进程明确送达；不自动重连，也不把旧帧当作有效状态。
        snapshots.publish({
            "state": None, "received_time": None, "received_monotonic": None,
            "frame_count": 0, "error": f"{type(exc).__name__}: {exc}",
        }, blocking=True)
    finally:
        client.close()
        first_state.set()


class URRealtimeMonitor:
    """每次连接创建一个只读进程，首帧到达后才向主界面确认成功。"""

    def __init__(self, host):
        self.host = host
        self._context = multiprocessing.get_context("spawn")
        self._stop_event = self._context.Event()
        self._first_state = self._context.Event()
        self._snapshots = LatestSnapshot(self._context, capacity=65536)
        self._process = None
        self._read_lock = threading.Lock()
        self._sequence = 0
        self._latest = {
            "state": None, "received_time": None, "received_monotonic": None,
            "frame_count": 0, "error": None,
        }

    def connect(self):
        """等待进程启动及首帧，启动失败或超时由连接入口显示并清理。"""
        process = self._context.Process(
            target=_receive_states,
            args=(self.host, self._stop_event, self._first_state, self._snapshots),
            name="URRealtimeMonitor", daemon=True,
        )
        process.start()
        self._process = process
        self._first_state.wait(timeout=10.0)
        snapshot = self.get_snapshot()
        if snapshot["error"] is not None:
            raise RuntimeError(f'实时状态连接失败：{snapshot["error"]}')
        if snapshot["state"] is None:
            raise TimeoutError("实时状态接收进程启动后未在 10 秒内提供首帧")

    def get_snapshot(self):
        """返回最新完整快照；新鲜度使用子进程的接收时刻，不使用读取时刻。"""
        # 界面与反馈定时线程共同推进游标，避免较早读到的帧覆盖新帧。
        with self._read_lock:
            exitcode = None if self._process is None else self._process.exitcode
            self._sequence, snapshot = self._snapshots.read(self._sequence, blocking=True)
            if snapshot is not None:
                self._latest = snapshot
            if (exitcode is not None
                    and not self._stop_event.is_set() and self._latest["error"] is None):
                self._latest = {
                    "state": None, "received_time": None, "received_monotonic": None,
                    "frame_count": 0,
                    "error": f"实时状态接收进程已退出，退出码 {exitcode}",
                }
            return self._latest

    def close(self):
        """先结束只读接收并回收进程；机器人运动停止仍由主窗口负责。"""
        if self._process is not None:
            self._stop_event.set()
            self._process.join()
            self._process.close()
            self._process = None
