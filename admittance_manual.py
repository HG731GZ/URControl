"""导纳单轮人工联调入口：只读检查或显式运行 J1 无接触候选。"""

from __future__ import annotations

import argparse
import json
from math import isfinite
from pathlib import Path
import signal
import socket
import time

from admittance_logger import AdmittanceLogger
from analyze_admittance_log import analyze_j1_no_contact
from UR_Utils import admittance_config
from UR_Utils.admittance_process import AdmittanceProcess
from UR_Utils.admittance_telemetry import ROBOT_FIELDS
from UR_Utils.admittance_transport import AdmittanceRTDE


def read_robot_state(host):
    """只读取得控制器身份和连续 300 ms 原始反馈，不查询或更改远程模式。"""
    dashboard = {}
    with socket.create_connection((host, 29999), timeout=3) as connection:
        connection.recv(4096)
        for command in ("running", "robotmode", "safetystatus", "PolyscopeVersion", "get loaded program"):
            connection.sendall((command + "\n").encode())
            dashboard[command] = connection.recv(4096).decode().strip()
    receiver = AdmittanceRTDE(host, buffer_limit_bytes=4096)
    try:
        receiver.connect()
        version = receiver.get_controller_version()
        names, types = [x[0] for x in ROBOT_FIELDS], [x[1] for x in ROBOT_FIELDS]
        receiver.send_output_setup(names, types, frequency=100)
        if not receiver.send_start():
            raise RuntimeError("只读状态同步启动失败")
        samples = []
        deadline = time.monotonic() + 3
        while len(samples) < 30 and time.monotonic() < deadline:
            state = receiver.receive()
            if state is not None:
                samples.append({name: getattr(state, name) for name in names})
            time.sleep(.001)
        if len(samples) != 30:
            raise RuntimeError("只读状态没有取得足够的新鲜样本")
        return {"robot_address": host, "dashboard": dashboard,
                "controller_version": version, "samples": samples}
    finally:
        receiver.disconnect()


def require_stopped(observation, speed_limit):
    """仅以时间持续推进、程序退出及六轴连续低速确认停止。"""
    samples = observation["samples"]
    if any(not all(isfinite(value) for value in (s["timestamp"], *s["actual_qd"])) for s in samples):
        raise RuntimeError("停止确认包含非有限时间或关节速度")
    if any(b["timestamp"] <= a["timestamp"] for a, b in zip(samples, samples[1:])):
        raise RuntimeError("停止确认的机器人时间未持续推进")
    if any(s["runtime_state"] != 1 or max(map(abs, s["actual_qd"])) > speed_limit for s in samples):
        raise RuntimeError("尚未确认程序退出和六轴静止，不能释放本轮控制权")


def require_empty_flange_ready(observation, config):
    """检查空法兰、静止、正常安全状态及完整 J1 运动包络。"""
    require_stopped(observation, admittance_config.stop_speed_limit(config))
    for state in observation["samples"]:
        if state["robot_mode"] != 7 or state["safety_mode"] != 1 or state["target_speed_fraction"] != 1:
            raise RuntimeError("本轮要求机器人 RUNNING、NORMAL、速度滑条 100%")
        if state["payload"] != 0 or any(state["tcp_offset"]):
            raise RuntimeError("本轮空法兰候选要求零负载和零 TCP 偏移")
    _, margins = admittance_config.actual_guard_limits(config)
    for axis, (angle, limit, margin) in enumerate(zip(
            observation["samples"][-1]["actual_q"], config["JOINT_POSITION_LIMIT_RAD"], margins)):
        if abs(angle) + margin > limit:
            raise RuntimeError(f"J{axis + 1} 本轮实际运动包络触及绝对关节边界")


def run_stage_a(args):
    """每次调用只运行一轮正式进程，保留完整日志，再生成只读验收报告。"""
    config = admittance_config.j1_no_contact_config()
    before = read_robot_state(args.robot_ip)
    require_empty_flange_ready(before, config)
    logger = AdmittanceLogger(enabled=True)
    logger.start({
        "run_id": f"stage_a_{args.environment}_{time.time_ns()}",
        "reference_source": "fixed", "space": "joint", "prepared_parameters": config,
        "robot_address": args.robot_ip, "debug_logging_enabled_at_start": True,
        "manual_validation": True, "manual_label": args.label, "manual_precheck": before,
        "validation_environment": args.environment, "validation_profile": "AdmittanceProcess",
        "validation_scope": "正式通信进程、正常自动 GC、500 Hz 状态和实时磁盘日志；未加载完整主界面及其他设备",
        "validation_scenario": "J1_empty_flange_no_contact", "no_contact_confirmed": True,
        "empty_flange_confirmed": True,
    })
    controller = AdmittanceProcess(args.robot_ip, "fixed", "joint", config=config, log_sink=logger)

    def request_stop(signum, frame):
        """终端中断直接提交正式停止请求，等待新鲜静止证据后再退出。"""
        controller.request_stop()
        print("已请求停止，等待程序退出和六轴静止", flush=True)

    old_handlers = {number: signal.signal(number, request_stop) for number in (signal.SIGINT, signal.SIGTERM)}
    path = Path(logger.get_status()["path"])
    print(json.dumps({"本轮日志": str(path), "运行秒数": config["DIAGNOSTIC_DURATION_S"],
                      "J1参考位置": before["samples"][-1]["actual_q"][0]}, ensure_ascii=False), flush=True)
    def confirm_external_stop():
        """通信进程失效后独立确认静止，再向原拥有者交付停止证据。"""
        observation = read_robot_state(args.robot_ip)
        require_stopped(observation, admittance_config.stop_speed_limit(config))
        logger.record("manual_stop_observations", observation)
        controller.confirm_external_stop(observation["samples"][-1])

    try:
        controller.start()
        deadline = time.monotonic() + config["ADMITTANCE_START_TIMEOUT_S"] + config["DIAGNOSTIC_DURATION_S"]
        next_display = 0.
        timed_stop = False
        while controller.active:
            now = time.monotonic()
            if now >= deadline and not timed_stop:
                controller.request_stop()
                timed_stop = True
            if controller.needs_external_stop_confirmation():
                confirm_external_stop()
                break
            if now >= next_display:
                state = controller.get_status()
                statistics = state["statistics"]
                print(json.dumps({"阶段": state["phase_name"], "退出": state["exit_reason"],
                                  "错误": state["last_error"],
                                  "非零drive周期": None if statistics is None else statistics["nonzero_drive_count"]},
                                 ensure_ascii=False), flush=True)
                next_display = now + 5
            time.sleep(.02)
    finally:
        try:
            if controller.active:
                try:
                    controller.stop()
                except RuntimeError:
                    if not controller.needs_external_stop_confirmation():
                        raise
                    confirm_external_stop()
            controller.close()
            status = controller.get_status()
            after = read_robot_state(args.robot_ip)
            require_stopped(after, admittance_config.stop_speed_limit(config))
            logger.record("manual_postcheck", after)
            logger.finish({"status": status})
            (path / "manual_final_status.json").write_text(json.dumps(status, ensure_ascii=False, indent=2), encoding="utf-8")
        finally:
            for number, handler in old_handlers.items():
                signal.signal(number, handler)
    report = analyze_j1_no_contact(path)
    (path / "stage_a_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"日志目录": str(path), "结论": report["结论"], "证据问题": report["证据问题"],
                      "不通过原因": report["不通过原因"], "最终静止确认": status["stopped_confirmed"],
                      "最终安全状态": after["samples"][-1]["safety_mode"]}, ensure_ascii=False), flush=True)
    return 0 if report["软件与记录检查通过"] else 1


def main():
    """只读检查为独立子命令；运动必须显式声明本轮现场条件，不自动连续复跑。"""
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)
    inspect = subparsers.add_parser("inspect", help="只读状态，不启动运动")
    inspect.add_argument("--robot-ip", required=True)
    inspect.add_argument("--output", type=Path, required=True)
    stage = subparsers.add_parser("stage-a", help="运行一轮 J1 无接触双向往返")
    stage.add_argument("--robot-ip", required=True)
    stage.add_argument("--environment", choices=("ursim", "real_robot"), required=True)
    stage.add_argument("--label", required=True)
    stage.add_argument("--empty-flange", action="store_true", required=True, help="现场已确认空法兰")
    stage.add_argument("--no-contact", action="store_true", required=True, help="现场已确认整轮无人接触")
    args = parser.parse_args()
    if args.command == "inspect":
        observation = read_robot_state(args.robot_ip)
        args.output.write_text(json.dumps(observation, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps({"状态文件": str(args.output), "控制器": observation["dashboard"],
                          "末帧": observation["samples"][-1]}, ensure_ascii=False, indent=2))
        return 0
    return run_stage_a(args)


if __name__ == "__main__":
    raise SystemExit(main())
