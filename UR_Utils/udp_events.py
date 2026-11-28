"""本机整数时钟与采用快照；UDP 后台记录仅由网络测试脚本显式开启。"""

import atexit
import json
import os
import secrets
from pathlib import Path
import threading
import time

from impedance_logger import RecordWriter


def monotonic_us():
    """保留整数微秒，不经浮点秒转换。"""
    return time.monotonic_ns() // 1000


def new_session():
    """每次实际启动发布自动生成非零 uint64 会话，无需人工配置。"""
    return secrets.randbelow(2**64 - 1) + 1


_writer = None
_lock = threading.Lock()


def open_events(directory):
    """由网络测试入口显式传入目录，不通过环境变量向控制进程传播。"""
    global _writer
    with _lock:
        if _writer is None:
            path = Path(directory) / f"events_{os.getpid()}_{time.monotonic_ns()}.jsonl"
            _writer = RecordWriter(path)


def record(event_kind, /, **payload):
    """事件类型只按位置传入，载荷可含协议 kind 字段；这里只提交后台记录。"""
    with _lock:
        if _writer is not None:
            _writer.record(event_kind, payload)


def log_status():
    """供网络测试脚本检查写盘失败和记录缺口。"""
    with _lock:
        return {} if _writer is None else {"dropped": _writer.dropped, "error": _writer.error}


def close_events():
    """生产线程退出后排空并保存完整性摘要。"""
    global _writer
    with _lock:
        writer, _writer = _writer, None
    if writer is not None:
        result = writer.finish()
        writer.path.with_suffix(".summary.json").write_text(
            json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")


atexit.register(close_events)


def applied_event(identity, q, boundary):
    """只在下层接口提交成功后调用；返回值同时供反馈快照使用。"""
    event = dict(applied_master_session=identity[0], applied_cmd_seq=identity[1],
                 t_apply_us=monotonic_us(), q_applied_rad=list(q), boundary=boundary)
    return event
