"""轨迹源按绝对时间推进；UDP 源读取后台接收身份，不依赖 Qt。"""

import threading
import time
from UR_Utils.impedance_reference import JointTargetFrame


class TrajectorySource:
    def __init__(self, trajectory, play_hz, run_id, max_lag):
        self.trajectory, self.play_hz, self.run_id = trajectory, play_hz, run_id
        self.start_time = None
        self.previous = None
        self.sequence = 0
        self.max_lag = max_lag

    def sample(self, now, running):
        if running and self.start_time is None:
            self.start_time = now
        if self.previous is not None and now - self.previous > self.max_lag:
            raise RuntimeError("轨迹发布调度超期")
        self.previous = now
        elapsed = 0 if self.start_time is None else now - self.start_time
        q, pos, eof = self.trajectory.at(elapsed, self.play_hz)
        self.sequence += 1
        return JointTargetFrame(
            self.run_id,
            "trajectory",
            self.sequence,
            q,
            now,
            end_of_source=eof,
            trajectory_row_position=pos,
        )


class UDPImpedanceSource:
    def __init__(self, client, backend):
        self.client, self.backend = client, backend
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run, daemon=True, name="ImpedanceUDPSource"
        )

    def start(self):
        self.thread.start()

    def close(self):
        self.stop_event.set()
        self.thread.join(1)

    def _run(self):
        try:
            self._receive()
        except Exception as exc:
            self.backend.request_stop("UDP 目标异常：" + str(exc))

    def _receive(self):
        previous = -1
        while not self.stop_event.wait(0.005):
            snapshot = self.client.get_monitor_snapshot()
            command = snapshot["command"]
            if snapshot["last_error"] or command is None or command.mode != 1:
                self.backend.request_stop("UDP 模式变化或接收错误")
                return
            if (
                snapshot["received_monotonic"] is None
                or time.monotonic() - snapshot["received_monotonic"]
                > self.backend.parameters["SOURCE_TIMEOUT_S"]
            ):
                self.backend.request_stop("UDP 目标断流")
                return
            if snapshot["frame_count"] != previous:
                previous = snapshot["frame_count"]
                frame = JointTargetFrame(
                    self.backend.run_id,
                    "teleop",
                    previous,
                    tuple(command.q_arm),
                    snapshot["received_monotonic"],
                )
                self.backend.submit_reference(frame)
