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
        self.client.wake_control_waiters()
        self.thread.join(1)

    def _run(self):
        try:
            self._receive()
        except (ValueError, TimeoutError) as exc:
            self.backend.request_stop(str(exc))
        except Exception as exc:
            self.backend.request_stop("UDP 目标异常：" + str(exc))
        finally:
            self.client.set_receive_trace(None)
            result = self.trace.finish()
            (self.backend.directory / "source_timing_summary.json").write_text(
                json.dumps(result, ensure_ascii=False, indent=2))

    def _receive(self):
        timeout = self.backend.parameters["SOURCE_TIMEOUT_S"]
        previous_seq = None
        deadline = None
        while not self.stop_event.is_set():
            if self.trace.error:
                raise RuntimeError(self.trace.error)
            target = self.client.wait_joint_target(previous_seq, self.control_event_seq, self.stop_event, deadline)
            if target is None:
                return
            previous_seq, received, q, qd = target
            if time.monotonic() >= received + timeout or (deadline is not None and received > deadline):
                self.backend.request_stop("UDP 目标断流")
                return
            deadline = received + timeout
            frame = JointTargetFrame(self.backend.run_id, "teleop", previous_seq, q, received, qd_rad_s=qd)
            started = time.monotonic()
            if self.backend.submit_reference(frame):
                self.trace.record("adapter_submit", dict(
                    source_seq=frame.source_seq, submit_start_monotonic_s=started,
                    submit_done_monotonic_s=time.monotonic(), q_source=frame.q_rad,
                    source_monotonic_s=received))
