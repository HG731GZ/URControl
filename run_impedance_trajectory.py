"""使用与 GUI 相同的 spawn 后端单次播放，默认仅连接本机 URSim。"""

import argparse
import json
import time
from UR_Utils.impedance_process import ImpedanceProcess
from UR_Utils.impedance_trajectory import JointTrajectory
from UR_Utils.impedance_config import read_preset, read_saved_parameters, snapshot


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("trajectory")
    p.add_argument("--robot-ip", default="127.0.0.1")
    p.add_argument("--play-hz", type=float)
    p.add_argument("--config")
    p.add_argument("--log-directory")
    a = p.parse_args()
    cfg = snapshot()
    if a.config:
        preset = json.loads(open(a.config).read())
        if preset.get("record_type") == "joint_impedance_preset":
            cfg = read_preset(preset)
        else:
            cfg = read_saved_parameters(preset.get("parameters", preset))
    backend = ImpedanceProcess(
        a.robot_ip,
        config=cfg,
        trajectory=JointTrajectory.load(a.trajectory),
        play_hz=a.play_hz,
        log_directory=a.log_directory,
    )
    backend.start()
    previous = None
    try:
        while backend.active:
            status = backend.get_status()
            if status["phase"] != previous:
                previous = status["phase"]
                print(json.dumps(status, ensure_ascii=False), flush=True)
            if previous == "STOP_UNCONFIRMED":
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        backend.request_stop()
    finally:
        backend.stop()
    status = backend.get_status()
    print(json.dumps(status, ensure_ascii=False, indent=2))
    if status["last_error"] or not status.get("completed_trajectory"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
