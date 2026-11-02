"""非阻塞最新完整快照槽。"""
import pickle

class LatestSnapshot:
    """单格冻结快照，双方锁内只复制 bytes；生产者不等待显示消费者。"""

    def __init__(self, context, capacity: int):
        self.data = context.RawArray("B", capacity)
        self.size = context.RawValue("I", 0)
        self.sequence = context.RawValue("Q", 0)
        self.lock = context.Lock()

    def publish(self, value: dict) -> bool:
        """锁忙时保留上一快照，下一次发布直接覆盖为最新状态。"""
        encoded = pickle.dumps(value, protocol=5)
        if len(encoded) > len(self.data):
            raise ValueError("控制 IPC 快照超过固定槽位容量")
        if not self.lock.acquire(False):
            return False
        try:
            memoryview(self.data).cast("B")[:len(encoded)] = encoded
            self.size.value = len(encoded)
            self.sequence.value += 1
        finally:
            self.lock.release()
        return True

    def read(self, last_sequence: int):
        """只返回最新完整快照，解码在共享锁之外进行。"""
        if not self.lock.acquire(False):
            return last_sequence, None
        try:
            sequence = self.sequence.value
            encoded = bytes(memoryview(self.data).cast("B")[:self.size.value]) if sequence != last_sequence else None
        finally:
            self.lock.release()
        return sequence, None if encoded is None else pickle.loads(encoded)


