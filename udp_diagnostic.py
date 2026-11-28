"""无驱动 UDP 联调：默认纯网络模式，也可观察正常业务帧；固定 200 Hz 反馈。"""

import argparse
import json
import os
import time
from pathlib import Path
import signal
import threading

import udp_settings
from HighPreciseTimer import run_periodic
from UR_Utils.URUdpClient import URUDPClient
from UR_Utils.udp_events import close_events, log_status, open_events


# PyCharm 直接运行本文件即可；日常修改下面两项，不需要填写运行参数。
# network：纯网络诊断，主端也须选择 kind=1；normal：观察正常业务通信，不连接驱动。
TEST_MODE = 'network'
TEST_DURATION_S = 120
# 网卡、端口和反馈频率统一在 udp_settings.py 配置。


def begin_run(kind):
    """创建本地记录目录，路径仅用于日志归档，不参与线协议或命令准入。"""
    if kind not in (0, 1):
        raise ValueError("UDP 模式须为 0 或 1")
    # 固定写入项目目录，避免 PyCharm 工作目录不同导致日志散落到其他位置。
    directory = Path(__file__).resolve().parent / "logs/udp" / f"{time.strftime('%Y%m%d_%H%M%S')}_{time.time_ns()}_{os.getpid()}"
    directory.mkdir(parents=True)
    (directory / "metadata.json").write_text(json.dumps(dict(
        version=2, kind=kind,
        local=[udp_settings.LOCAL_IP, udp_settings.LOCAL_PORT], remote=[udp_settings.REMOTE_IP, udp_settings.REMOTE_PORT], feedback_hz=udp_settings.FEEDBACK_HZ,
        clock="time.monotonic_ns() // 1000，PC 接口/缓存边界，不是设备采样时钟",
    ), ensure_ascii=False, indent=2), encoding="utf-8")
    open_events(directory)
    return directory

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('network', 'normal'), default=TEST_MODE,
                        help='network 发全零诊断帧；normal 发未跟随、无测量的正常反馈，不连接驱动')
    parser.add_argument('--duration', type=float, default=TEST_DURATION_S,
                        help='运行秒数；不传参数时使用文件顶部 TEST_DURATION_S')
    args = parser.parse_args()
    if args.mode not in ('network', 'normal'):
        parser.error('TEST_MODE 必须为 network 或 normal')
    if args.duration <= 0:
        parser.error('运行时长必须大于零')
    kind = 1 if args.mode == 'network' else 0
    directory = begin_run(kind)
    target = (udp_settings.REMOTE_IP, udp_settings.REMOTE_PORT)
    client = URUDPClient(udp_settings.LOCAL_IP, udp_settings.LOCAL_PORT, kind=kind,
                         peer=target)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    last_notice = None
    empty = 0.0 if kind == 1 else float('nan')

    def feedback():
        nonlocal last_notice
        client.queue_send_to(target, [empty]*6, 0, [empty]*3, [empty]*6, [empty]*6, 0, [empty]*6)
        state = client.get_monitor_snapshot()
        notice = (state['link_event_count'], state['slave_session'], state['send_last_error'])
        if notice != last_notice:
            last_notice = notice
            print(f"UDP {state['link_state']}：master_session={state['master_session']}，"
                  f"slave_session={state['slave_session']}，cmd_seq={state['latest_cmd_seq']}，"
                  f"事件={state['link_reason']}，发送错误={state['send_last_error']}", flush=True)

    timer = threading.Timer(args.duration, stop.set)
    try:
        client.start()
        timer.start()
        print(f'无驱动 UDP 联调：{args.mode}，kind={kind}，主端须使用相同 kind', flush=True)
        print(f'运行 {args.duration:g} 秒后自动结束并保存日志', flush=True)
        print(f'{client.bind_host}:{client.bind_port} → {target}，'
              f'{udp_settings.FEEDBACK_HZ} Hz，336 B；Ctrl+C 退出', flush=True)
        print(f'记录目录：{directory}', flush=True)
        run_periodic(udp_settings.FEEDBACK_HZ, stop, feedback)
    finally:
        timer.cancel()
        client.stop()
        print(f'已停止：{client.get_monitor_snapshot()}；日志：{log_status()}', flush=True)
        close_events()


if __name__ == '__main__':
    main()
