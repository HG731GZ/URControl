"""显式运行的本机 URSim 集成检查；包含小幅模拟运动，不用于真机。"""

import json
from pathlib import Path
import socket
import sys
import time

import numpy as np
import rtde_receive

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from UR_Utils import admittance_config
from UR_Utils.URAdmittanceController import URAdmittanceController
from UR_Utils.URRTDEController import URRTDEController


HOST = "127.0.0.1"


def simulator_state():
    """只测试本机已就绪且没有运行程序的模拟器，不自动上电或解除停止。"""
    with socket.create_connection((HOST, 29999), timeout=2) as sock:
        stream = sock.makefile("rb")
        stream.readline()
        state = {}
        for command in ("PolyscopeVersion", "robotmode", "programState", "safetystatus"):
            sock.sendall((command + "\n").encode("ascii"))
            state[command] = stream.readline().decode().strip()
    assert state["robotmode"] == "Robotmode: RUNNING", state
    assert state["programState"].startswith("STOPPED"), state
    assert state["safetystatus"] == "Safetystatus: NORMAL", state
    return state


def exercise_backend(reader, source, space, *, scenario="run"):
    # URSim 在客户端 close 后异步释放输入配方；各独立场景之间留出释放时间。
    time.sleep(0.2)
    q_start = np.asarray(reader.getActualQ())
    cfg = admittance_config.snapshot()
    if scenario == "run":
        # URSim 没有真实接触；用小目标载荷让导纳偏移和末端 IK 分支实际执行。
        if space == "joint":
            cfg["JOINT_TARGET_TORQUE_NM"][0] = -2.0
        else:
            cfg["CART_TARGET_WRENCH"][0] = -4.0
    kwargs = {"q_master": q_start, "source_seq": 1} if source == "teleop" else {}
    backend = URAdmittanceController(HOST, source, space, config=cfg, **kwargs)
    if scenario == "compile_error":
        backend._script = "def compile_failure():\n  local value = abs(-1)\nend\n"
    elif scenario == "runtime_error":
        backend._script = ("def runtime_failure():\n  local values = [0]\n"
                           "  local value = values[1]\nend\n")
    started = time.monotonic()
    running_at = None
    phases = set()
    seq = 1
    backend.start()
    try:
        while time.monotonic() - started < 7:
            status = backend.get_status()
            phases.add(status["phase"])
            if not status["active"]:
                break
            if status["running"] and running_at is None:
                running_at = time.monotonic()
            if source == "teleop" and not status["stopping"]:
                if scenario != "stale_master" or running_at is None:
                    seq += 1
                    target = q_start.copy()
                    if running_at is not None:
                        target[0] += 0.006
                    backend.submit_reference(target, source_seq=seq)
            if scenario == "cancel_bias" and status["phase"] == 1:
                backend.request_stop()
            if scenario == "run" and running_at is not None and time.monotonic() - running_at > 0.6:
                backend.request_stop()
            time.sleep(0.01)
    finally:
        backend.stop()
    status = backend.get_status()
    assert status["stopped_confirmed"] and status["runtime_state"] == 1, status
    assert backend._connection is None and backend._script_client is None
    if scenario in ("compile_error", "runtime_error"):
        assert "URScript 错误" in (status["last_error"] or ""), status
        assert time.monotonic() - started < cfg["ADMITTANCE_START_TIMEOUT_S"], status
        if scenario == "compile_error":
            assert "compile_error_name_not_found:abs:" in status["last_error"], status
    elif scenario == "stale_master":
        assert running_at is not None and status["exit_reason"] == 3, status
    else:
        assert status["last_error"] is None and status["exit_reason"] == 1, status
        if scenario == "run":
            assert running_at is not None, status
            assert np.max(np.abs(np.asarray(status["q_cmd"]) - q_start)) > 1e-5, status
        else:
            assert running_at is None, status
    result = {"case": f"{source}/{space}/{scenario}", "phases": sorted(phases),
              "start_s": None if running_at is None else round(running_at - started, 3),
              "exit_reason": status["exit_reason"], "error": status["last_error"],
              "stopped_confirmed": status["stopped_confirmed"]}
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return result


def exercise_handoff():
    controller = URRTDEController(HOST)
    try:
        # 与 UI 相同的普通控制 → 导纳 → 停止 → 普通控制恢复流程。
        assert controller.start()
        controller.track_joint(controller.get_actual_q())
        time.sleep(0.1)
        assert controller.start_admittance("free", "joint")
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            state = controller.get_status()["admittance"]
            if state["running"]:
                break
            assert state["last_error"] is None, state
            time.sleep(0.01)
        assert state["running"], state
        controller.stop(stop_script=True)
        assert controller.get_status()["admittance"]["stopped_confirmed"]
        assert controller.start()
        controller.track_joint(controller.get_actual_q())
        time.sleep(0.1)
        assert controller.is_running(), controller.get_status()
        assert controller.get_status()["last_error"] is None
    finally:
        controller.shutdown()
    print(json.dumps({"case": "ordinary/admittance/ordinary", "passed": True}), flush=True)


def main():
    print(json.dumps(simulator_state(), ensure_ascii=False), flush=True)
    reader = rtde_receive.RTDEReceiveInterface(HOST, 100)
    try:
        for source in ("free", "teleop"):
            for space in ("joint", "cartesian"):
                exercise_backend(reader, source, space)
        exercise_backend(reader, "free", "joint", scenario="cancel_bias")
        exercise_backend(reader, "teleop", "joint", scenario="stale_master")
        exercise_backend(reader, "free", "joint", scenario="compile_error")
        exercise_backend(reader, "free", "joint", scenario="runtime_error")
    finally:
        reader.disconnect()
    exercise_handoff()
    print(json.dumps({"final": simulator_state(), "passed": True}, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
