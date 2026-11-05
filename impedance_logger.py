"""有界异步 JSONL，缺口和写盘异常不会伪装成完整记录。"""

import json
from pathlib import Path
from queue import Queue, Full
import threading
import time


class RecordWriter:
    """控制进程和 UDP 接收进程各用独立的有界队列和文件。"""

    def __init__(self, path):
        self.path = Path(path)
        self.queue = Queue(16384)
        self.record_lock = threading.Lock()
        self.dropped = 0
        self.error = None
        self.pending_gap = 0
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()

    def record(self, kind, payload):
        with self.record_lock:
            self._enqueue(kind, payload)

    def _enqueue(self, kind, payload):
        try:
            if self.pending_gap:
                self.queue.put_nowait(
                    {
                        "kind": "log_gap",
                        "host_monotonic_s": time.monotonic(),
                        "payload": {"dropped_count": self.pending_gap},
                    }
                )
                self.pending_gap = 0
            self.queue.put_nowait(
                {"kind": kind, "host_monotonic_s": time.monotonic(), "payload": payload}
            )
        except Full:
            self.dropped += 1
            self.pending_gap += 1

    def _write(self):
        try:
            with self.path.open("x") as f:
                while True:
                    row = self.queue.get()
                    if row is None:
                        break
                    f.write(json.dumps(row, ensure_ascii=False) + "\n")
        except Exception as exc:
            self.error = str(exc)

    def finish(self):
        try:
            self.queue.put(None, timeout=2)
        except Full:
            self.error = "日志队列无法完成收尾"
        self.thread.join(3)
        return {
            "log_dropped": self.dropped,
            "log_error": self.error,
            "log_complete": not self.thread.is_alive()
            and not self.error
            and self.dropped == 0,
        }


class ImpedanceLogger(RecordWriter):
    def __init__(self, directory, metadata):
        self.directory = Path(directory)
        self.metadata = metadata
        self.update_context({})
        super().__init__(self.directory / "records.jsonl")

    def update_context(self, context):
        self.metadata.update(context)
        (self.directory / "metadata.json").write_text(
            json.dumps(self.metadata, ensure_ascii=False, indent=2)
        )

    def finish(self, status):
        result = {**status, **super().finish()}
        (self.directory / "summary.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2)
        )
