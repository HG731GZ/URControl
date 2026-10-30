"""导纳单轮人工联调入口：只读检查、逐轴无接触轨迹或选定关节的固定/遥操作联调。"""

from __future__ import annotations

import argparse
import json
from math import isfinite
from pathlib import Path
import signal
import select
import socket
import sys
import time

from admittance_logger import AdmittanceLogger
from analyze_admittance_log import analyze, analyze_j1_no_contact
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
    """检查空法兰、静止、正常安全状态及六轴实际运动包络。"""
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


def run_manual(args):
    """每次调用只运行一轮正式进程，保留完整日志，再生成只读验收报告。"""
    stage_a = args.command == "stage-a"
    source = args.reference_source if args.command == "joint" else "fixed"
    if stage_a:
        config = admittance_config.j1_no_contact_config()
    elif args.command == "j3-motion":
        config = admittance_config.j3_no_contact_config()
    elif args.command == "joint-motion":
        config = admittance_config.joint_no_contact_config(args.axis)
    else:
        config = admittance_config.joint_contact_config(tuple(args.axes), reference_source=source)
        config["DIAGNOSTIC_DURATION_S"] = args.duration
        config = admittance_config.validate_run_config(source, "joint", config=config)
    before = read_robot_state(args.robot_ip)
    require_empty_flange_ready(before, config)
    anchor = list(before["samples"][-1]["actual_q"])
    reference = list(anchor)
    source_seq = 0
    reference_enabled = True
    console_open = True
    logger = AdmittanceLogger(enabled=True)
    logger.start({
        "run_id": f"{args.command}_{args.environment}_{time.time_ns()}",
        "reference_source": source, "space": "joint", "prepared_parameters": config,
        "robot_address": args.robot_ip, "debug_logging_enabled_at_start": True,
        "manual_validation": True, "manual_label": args.label, "manual_precheck": before,
        "validation_environment": args.environment, "validation_profile": "AdmittanceProcess",
        "validation_scope": "正式通信进程、正常自动 GC、500 Hz 状态和实时磁盘日志；未加载完整主界面及其他设备",
        "validation_scenario": ("J1_empty_flange_no_contact" if stage_a else
                                "J3_cosine_no_contact" if args.command == "j3-motion" else
                                f"J{args.axis}_bidirectional_no_contact" if args.command == "joint-motion" else
                                "joint_empty_flange_no_contact"),
        "manual_reference_definition": "终端显式设定相对启动姿态的目标，100 Hz 发布；不是现场主端" if source == "teleop" else None,
        "no_contact_confirmed": True,
        "empty_flange_confirmed": True,
    })
    controller = AdmittanceProcess(args.robot_ip, source, "joint", config=config, log_sink=logger,
                                   q_master=reference if source == "teleop" else None,
                                   source_seq=source_seq if source == "teleop" else None)

    def request_stop(signum, frame):
        """终端中断直接提交正式停止请求，等待新鲜静止证据后再退出。"""
        controller.request_stop()
        print("已请求停止，等待程序退出和六轴静止", flush=True)

    old_handlers = {number: signal.signal(number, request_stop) for number in (signal.SIGINT, signal.SIGTERM)}
    path = Path(logger.get_status()["path"])
    print(json.dumps({"本轮日志": str(path), "运行秒数": config["DIAGNOSTIC_DURATION_S"],
                      "启动关节姿态": anchor}, ensure_ascii=False), flush=True)
    if source == "teleop":
        print("终端命令：target 后接六轴相对启动角度(rad)；pause 停止新参考以观察断流；stop 请求停止。", flush=True)
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
        next_reference = 0.
        timed_stop = False
        while controller.active:
            now = time.monotonic()
            if not stage_a and console_open and select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                console_open = bool(line)
                words = line.split()
                if words == ["stop"]:
                    controller.request_stop()
                elif words == ["pause"] and source == "teleop":
                    reference_enabled = False
                    logger.record("manual_reference_paused", {"source_seq": source_seq})
                    print("已停止发布新参考，保留通信心跳以观察机器人主端超时。", flush=True)
                elif words and words[0] == "target" and source == "teleop":
                    try:
                        offsets = list(map(float, words[1:]))
                        limits = config["JOINT_COMMAND_MAX_OFFSET_RAD"]
                        if len(offsets) != 6 or any(not isfinite(x) or abs(x) > limits[i]
                                                    or not config["JOINT_AXES"][i] and x != 0
                                                    for i, x in enumerate(offsets)):
                            raise ValueError("需要六个有限偏移，启用轴不得超过命令边界，禁用轴必须为零")
                        reference = [q + offset for q, offset in zip(anchor, offsets)]
                        logger.record("manual_reference_target", {"offset_rad": offsets, "q_master": reference})
                        print("已设定最新目标：" + str(offsets), flush=True)
                    except ValueError as error:
                        print(f"目标未接受：{error}", flush=True)
                elif words:
                    print("可用命令：target 六轴偏移、pause、stop", flush=True)
            if (source == "teleop" and reference_enabled and now >= next_reference
                    and not controller.get_status()["stopping"]):
                source_seq += 1
                controller.submit_reference(reference, source_seq=source_seq)
                next_reference = now + .01
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
            time.sleep(.002 if source == "teleop" else .02)
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
    report = analyze_j1_no_contact(path) if stage_a else analyze(path)
    report_name = "stage_a_report.json" if stage_a else "descriptive_report.json"
    (path / report_name).write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    if args.command == "joint-motion":
        print(json.dumps({"观测轴": args.axis, "死区_Nm": config["JOINT_DEADBAND_NM"][args.axis - 1],
                          "最终逐周期统计": report["选定轴最终逐周期统计"]}, ensure_ascii=False), flush=True)
    print(json.dumps({"日志目录": str(path), "报告": report_name,
                      "结论": report["结论"] if stage_a else "本轮描述性记录，不替代真实接触验收",
                      "退出原因": status["exit_reason"], "错误": status["last_error"],
                      "最终静止确认": status["stopped_confirmed"],
                      "最终安全状态": after["samples"][-1]["safety_mode"]}, ensure_ascii=False), flush=True)
    successful = report["软件与记录检查通过"] if stage_a else status["stopped_confirmed"] and not status["last_error"]
    return 0 if successful else 1


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
    j3 = subparsers.add_parser("j3-motion", help="单轮 J3 正向余弦往返及 10 秒尾段；外力不驱动命令")
    j3.add_argument("--robot-ip", required=True)
    j3.add_argument("--environment", choices=("ursim", "real_robot"), required=True)
    j3.add_argument("--label", required=True)
    j3.add_argument("--empty-flange", action="store_true", required=True)
    j3.add_argument("--no-contact", action="store_true", required=True)
    motion = subparsers.add_parser("joint-motion", help="指定单轴完成正负往返、节点保持及 10 秒尾段；外力不驱动命令")
    motion.add_argument("--robot-ip", required=True)
    motion.add_argument("--environment", choices=("ursim", "real_robot"), required=True)
    motion.add_argument("--label", required=True)
    motion.add_argument("--empty-flange", action="store_true", required=True)
    motion.add_argument("--no-contact", action="store_true", required=True)
    motion.add_argument("--axis", type=int, choices=range(1, 7), required=True)
    joint = subparsers.add_parser("joint", help="单轮关节无接触联调；遥操作目标由终端显式给定")
    joint.add_argument("--robot-ip", required=True)
    joint.add_argument("--environment", choices=("ursim", "real_robot"), required=True)
    joint.add_argument("--label", required=True)
    joint.add_argument("--empty-flange", action="store_true", required=True)
    joint.add_argument("--no-contact", action="store_true", required=True)
    joint.add_argument("--axes", type=int, nargs="+", choices=range(1, 7), default=[3],
                       help="单轴、1 3，或固定参考六轴 1 2 3 4 5 6")
    joint.add_argument("--reference-source", choices=("fixed", "teleop"), default="fixed")
    joint.add_argument("--duration", type=float, default=15., help="运行秒数，最长 60 秒")
    args = parser.parse_args()
    if args.command == "inspect":
        observation = read_robot_state(args.robot_ip)
        args.output.write_text(json.dumps(observation, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps({"状态文件": str(args.output), "控制器": observation["dashboard"],
                          "末帧": observation["samples"][-1]}, ensure_ascii=False, indent=2))
        return 0
    return run_manual(args)


if __name__ == "__main__":
    raise SystemExit(main())
