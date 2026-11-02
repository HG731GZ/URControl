"""独立停止监督：不依赖已经失败的工作 RTDE，确认新反馈、退出和连续静止。"""

import time
from UR_Utils.rtde_script_transport import ScriptRTDE, DashboardStop


def confirm_stopped(
    ip, *, deadline, speed_limit=0.01, window=0.1, record=lambda *a: None
):
    conn = ScriptRTDE(ip, buffer_limit_bytes=65536)
    dashboard = None
    try:
        record("dashboard_stop_requested", {"deadline_monotonic_s": deadline})
        dashboard = DashboardStop(ip, deadline=deadline)
        conn.connect()
        conn.send_output_setup(
            ["timestamp", "runtime_state", "actual_q", "actual_qd"],
            ["DOUBLE", "UINT32", "VECTOR6D", "VECTOR6D"],
            frequency=100,
        )
        if not conn.send_start():
            raise RuntimeError("停止监督 RTDE 同步失败")
        accepted = False
        last = None
        stationary = None
        while time.monotonic() < deadline:
            if dashboard is not None:
                response = dashboard.poll()
                if response is not None:
                    record("dashboard_stop_reply", {"response": response})
                    accepted = response == "Stopped"
                    dashboard.close()
                    dashboard = None
            state = conn.receive_once()
            if state is not None and (last is None or state.timestamp > last):
                fresh = last is not None and state.timestamp - last < 0.1
                last = state.timestamp
                if (
                    accepted
                    and fresh
                    and state.runtime_state == 1
                    and max(map(abs, state.actual_qd)) <= speed_limit
                ):
                    if stationary is None:
                        stationary = time.monotonic()
                    if time.monotonic() - stationary >= window:
                        sample = {
                            key: getattr(state, key)
                            for key in (
                                "timestamp",
                                "runtime_state",
                                "actual_q",
                                "actual_qd",
                            )
                        }
                        record("stop_confirmed", sample)
                        return sample
                else:
                    stationary = None
            time.sleep(0.001)
        raise TimeoutError("停止未确认：程序退出和连续新鲜静止反馈不足")
    finally:
        conn.disconnect()
        if dashboard is not None:
            dashboard.close()
