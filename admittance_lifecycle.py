"""导纳启停的阻塞等待任务；所有界面与采集状态由主线程维护。"""

from copy import deepcopy
import threading

from PyQt5.QtCore import QThread, pyqtSignal


class AdmittanceLifecycleWorker(QThread):
    """串行执行一次启动或停止，完成信号仅传回结果与只读快照。"""

    completed = pyqtSignal(dict)

    def __init__(self, controller, operation, *, start_kwargs=None,
                 stop_event=None, log_sink=None, parent=None):
        super().__init__(parent)
        if operation not in ("start", "stop"):
            raise ValueError("导纳生命周期任务必须为 start 或 stop")
        self.controller = controller
        self.operation = operation
        self.start_kwargs = deepcopy({} if start_kwargs is None else start_kwargs)
        self.stop_event = threading.Event() if stop_event is None else stop_event
        self.log_sink = log_sink

    def run(self):
        """任务异常在本操作边界明确报告，不重试、不恢复其他控制程序。"""
        result = {
            "operation": self.operation, "success": False, "cancelled": False,
            "error": None, "status": None, "parameters_accepted": False,
        }
        attempted_start = False
        try:
            if self.operation == "start":
                if self.stop_event.is_set():
                    result["cancelled"] = True
                else:
                    attempted_start = True
                    result["success"] = self.controller.start_admittance(
                        **self.start_kwargs, stop_event=self.stop_event, log_sink=self.log_sink,
                    )
                    result["cancelled"] = self.stop_event.is_set()
                    if result["cancelled"]:
                        result["success"] = False
            else:
                self.stop_event.set()
                self.controller.request_admittance_stop()
                self.controller.stop(stop_script=True)
                result["success"] = True
        except Exception as exc:
            result["error"] = str(exc)
            if self.operation == "start":
                result["cancelled"] = self.stop_event.is_set()
        finally:
            if attempted_start:
                result["parameters_accepted"] = self.controller.last_admittance_start_accepted
            if self.operation == "stop" or result["parameters_accepted"]:
                result["status"] = self.controller.get_status()["admittance"]
                if (self.operation == "start" and result["error"] is None
                        and result["status"]["last_error"] is not None):
                    # 后端故障也会设置共享停止事件；保留故障，不能误报为用户取消。
                    result["error"] = result["status"]["last_error"]
                    result["success"] = False
                    result["cancelled"] = False
            self.completed.emit(result)


class AdmittanceLogFinishWorker(QThread):
    """在机械臂停止确认后等待日志落盘，不阻塞界面或下一轮通信。"""

    def __init__(self, logger, summary, parent=None):
        super().__init__(parent)
        self.logger = logger
        self.summary = deepcopy(summary)
        self.result = None

    def run(self):
        self.result = self.logger.finish(self.summary)
