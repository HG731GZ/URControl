"""有界异步 JSONL，缺口和写盘异常不会伪装成完整记录。"""

import json
from pathlib import Path
from queue import Queue, Full
import threading
import time


class ImpedanceLogger:
    def __init__(self, directory, metadata):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.queue = Queue(16384)
        self.dropped = 0
        self.error = None
        self.pending_gap = 0
        self.metadata = metadata
        self.update_context({})
        self.thread = threading.Thread(target=self._write, daemon=True)
        self.thread.start()

    def update_context(self, context):
        self.metadata.update(context)
        (self.directory / "metadata.json").write_text(
            json.dumps(self.metadata, ensure_ascii=False, indent=2)
        )

    def record(self, kind, payload):
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
            with (self.directory / "records.jsonl").open("w") as f:
                while True:
                    row = self.queue.get()
                    if row is None:
                        break
                    f.write(json.dumps(row, ensure_ascii=False) + "\n")
        except Exception as exc:
            self.error = str(exc)

    def finish(self, status):
        try:
            self.queue.put(None, timeout=2)
        except Full:
            self.error = "日志队列无法完成收尾"
        self.thread.join(3)
        result = {
            **status,
            "log_dropped": self.dropped,
            "log_error": self.error,
            "log_complete": not self.thread.is_alive()
            and not self.error
            and self.dropped == 0,
        }
        (self.directory / "summary.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2)
        )
