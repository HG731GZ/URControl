"""轨迹源按绝对时间推进；UDP 源读取后台接收身份，不依赖 Qt。"""

import threading
import time
import json
from impedance_logger import RecordWriter
from UR_Utils.impedance_reference import JointTargetFrame


class TrajectorySource:
    def __init__(self, trajectory, play_hz, run_id, max_lag, source="trajectory"):
        self.trajectory, self.play_hz, self.run_id = trajectory, play_hz, run_id
        self.source = source
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
            self.source,
            self.sequence,
            q,
            now,
            end_of_source=eof,
            trajectory_row_position=pos,
        )


class UDPImpedanceSource:
    def __init__(self, client, backend, control_event_seq):
        self.client, self.backend = client, backend
        self.control_event_seq = control_event_seq
        self.stop_event = threading.Event()
        self.thread = threading.Thread(
            target=self._run, daemon=True, name="ImpedanceUDPSource"
        )

    def start(self):
        self.trace = RecordWriter(self.backend.directory / "source_timing.jsonl")
        self.client.set_receive_trace(self.trace)
        self.thread.start()

    def close(self):
        self.stop_event.set()
        self.thread.join(1)

    def _run(self):
        try:
            self._receive()
        except Exception as exc:
            self.backend.request_stop("UDP 目标异常：" + str(exc))
        finally:
            self.client.set_receive_trace(None)
            result = self.trace.finish()
            (self.backend.directory / "source_timing_summary.json").write_text(
                json.dumps(result, ensure_ascii=False, indent=2))

    def _receive(self):
        previous = -1
        while not self.stop_event.wait(0.005):
            if self.trace.error:
                raise RuntimeError(self.trace.error)
            snapshot = self.client.get_monitor_snapshot()
            if snapshot["control_event_seq"] != self.control_event_seq:
                self.backend.request_stop(snapshot["control_event_reason"])
                return
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
            # 每次有效收包都推进身份；角度相同也算新目标，反复读取旧缓存不算。
            if snapshot["frame_count"] != previous:
                frame = JointTargetFrame(
                    self.backend.run_id,
                    "teleop",
                    snapshot["frame_count"],
                    tuple(command.q_arm),
                    snapshot["received_monotonic"],
                    qd_rad_s=command.qd_arm,
                )
                started = time.monotonic()
                if self.backend.submit_reference(frame):
                    self.trace.record("adapter_submit", dict(
                        source_seq=frame.source_seq, submit_start_monotonic_s=started,
                        submit_done_monotonic_s=time.monotonic(), q_source=frame.q_rad,
                        source_monotonic_s=frame.source_monotonic_s))
                    previous = snapshot["frame_count"]
