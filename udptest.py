"""
udptest.py — URUDPClient 使用示例 / 基础功能测试。

启动后在本机 127.0.0.1:5005 上收发 UDP 控制指令，
演示读取、发送、限速、等待新帧等接口。
"""

import time
import threading
import numpy as np

from UR_Utils.URUdpClient import URUDPClient, UDPControlMode

BIND_HOST = "127.0.0.1"
BIND_PORT = 5005


def print_sep(title: str) -> None:
    print(f"\n{'=' * 50}")
    print(f"  {title}")
    print(f"{'=' * 50}")


def sample_q_arm(t: float = 0.0):
    """生成示例关节角（模拟正弦摆动）。"""
    return [
        np.sin(t) * 0.5,
        -np.pi / 2 + np.sin(t + 1.0) * 0.1,
        -np.pi / 2 + np.sin(t + 2.0) * 0.1,
        0.0,
        np.pi / 2 + np.sin(t + 3.0) * 0.1,
        np.sin(t * 1.5) * 0.3,
    ]


def test_basic_send_recv():
    """1. 基本收发：启动客户端，发送一帧，读取回来。"""
    print_sep("1. 基本收发")

    client = URUDPClient(bind_host=BIND_HOST, bind_port=BIND_PORT)
    client.start()

    q_arm = sample_q_arm(0.0)
    mode = UDPControlMode.JOINT_TRACK
    q_gripper = [0.0, 0.0, 0.0]

    client.send_to((BIND_HOST, BIND_PORT), q_arm=q_arm, mode=mode, q_gripper=q_gripper)
    time.sleep(0.05)

    cmd = client.get_latest()
    if cmd is not None:
        print(f"收到指令: {cmd}")
        print(f"  帧计数: {client.get_frame_count()}")
    else:
        print("未收到数据")

    client.stop()
    return True


def test_pack_standalone():
    """2. 独立打包/解包：不经过 socket，直接验证协议打包-解包往返。"""
    print_sep("2. 独立打包/解包")

    q_arm = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    mode = UDPControlMode.TCP_DELTA
    q_gripper = [0.01, 0.02, 0.03]

    packet = URUDPClient.pack(q_arm, mode, q_gripper)

    data = packet[URUDPClient.DATA_OFFSET:URUDPClient.CRC_OFFSET]
    q2, m2, g2 = URUDPClient._parse_data(data)

    ok = q_arm == q2 and mode == m2 and q_gripper == g2
    print(f"原始 q_arm:    {[f'{v:.1f}' for v in q_arm]}")
    print(f"解析 q_arm:    {[f'{v:.1f}' for v in q2]}")
    print(f"原始 mode:     {mode} ({UDPControlMode.name(mode)})")
    print(f"解析 mode:     {m2} ({UDPControlMode.name(m2)})")
    print(f"原始 q_gripper:{[f'{v:.2f}' for v in q_gripper]}")
    print(f"解析 q_gripper:{[f'{v:.2f}' for v in g2]}")
    print(f"帧长度:        {len(packet)} bytes")
    print(f"  帧头: {packet[:4].hex(' ').upper()}")
    print(f"  数据: {len(data)} bytes")
    print(f"  CRC:  {packet[77:81].hex(' ').upper()}")
    print(f"  帧尾: {packet[81:85].hex(' ').upper()}")
    print(f"往返一致:      {'OK' if ok else 'FAIL'}")
    return ok


def test_send_rate_limit():
    """3. 发送限速：send_interval=0.05 时，连发 5 包验证速率控制。"""
    print_sep("3. 发送限速 (send_interval=0.05s)")

    client = URUDPClient(
        bind_host=BIND_HOST,
        bind_port=BIND_PORT + 1,
        send_interval=0.05,
    )
    client.start()

    t0 = time.perf_counter()
    for i in range(5):
        client.send_to(
            (BIND_HOST, BIND_PORT + 1),
            q_arm=sample_q_arm(i * 0.5),
            mode=UDPControlMode.JOINT_DELTA,
            q_gripper=[float(i) * 0.01] * 3,
        )
    elapsed = time.perf_counter() - t0
    expected_min = 0.05 * 4  # 第 1 次无延迟，后 4 次各等 0.05s

    print(f"5 次发送耗时: {elapsed:.3f}s (预期 >= {expected_min:.2f}s)")
    print(f"限速生效:      {'OK' if elapsed >= expected_min * 0.9 else '疑似未限速'}")
    client.stop()


def test_wait_next():
    """4. 阻塞等待新帧：在另一个线程中发送，主线程 wait_next 等待。"""
    print_sep("4. 阻塞等待新帧")

    client = URUDPClient(bind_host=BIND_HOST, bind_port=BIND_PORT + 2)
    client.start()

    def delayed_send(delay: float):
        time.sleep(delay)
        client.send_to(
            (BIND_HOST, BIND_PORT + 2),
            q_arm=[0.0] * 6,
            mode=UDPControlMode.OFF,
            q_gripper=[0.0] * 3,
        )

    sender = threading.Thread(target=delayed_send, args=(0.3,))
    sender.start()

    t0 = time.perf_counter()
    cmd = client.wait_next(timeout=2.0)
    elapsed = time.perf_counter() - t0

    if cmd is not None:
        print(f"等待到新帧，模式={UDPControlMode.name(cmd.mode)}，耗时={elapsed:.3f}s")
    else:
        print("超时未收到数据")

    sender.join()
    client.stop()


def test_continuous_stream():
    """5. 连续收发：模拟外部控制器以固定频率发送，客户端持续接收。"""
    print_sep("5. 连续收发 (模拟 50Hz 外部控制器)")

    client = URUDPClient(
        bind_host=BIND_HOST,
        bind_port=BIND_PORT + 3,
        recv_timeout=0.01,
    )
    client.start()

    # 外部发送线程：50Hz 发送 2 秒
    stop_tx = threading.Event()
    tx_count = [0]

    def tx_loop():
        t = 0.0
        dt = 0.02  # 50Hz
        while not stop_tx.is_set():
            client.send_to(
                (BIND_HOST, BIND_PORT + 3),
                q_arm=sample_q_arm(t),
                mode=UDPControlMode.JOINT_TRACK,
                q_gripper=[np.sin(t * 2.0) * 0.01] * 3,
            )
            tx_count[0] += 1
            t += dt
            time.sleep(dt)

    tx_thread = threading.Thread(target=tx_loop, daemon=True)
    tx_thread.start()
    time.sleep(2.0)
    stop_tx.set()
    tx_thread.join(timeout=0.5)

    rx_count = client.get_frame_count()
    loss = tx_count[0] - rx_count
    loss_rate = loss / tx_count[0] * 100 if tx_count[0] > 0 else 0

    print(f"发送帧数: {tx_count[0]}")
    print(f"接收帧数: {rx_count}")
    print(f"丢包:     {loss} ({loss_rate:.1f}%)")

    client.stop()


def test_error_handling():
    """6. 异常处理：发送长度错误的数据、无效参数等。"""
    print_sep("6. 异常处理")

    # 无效 q_arm 长度
    try:
        URUDPClient.pack([0.0] * 5, 0, [0.0] * 3)
        print("q_arm 长度检查: FAIL（应抛异常）")
    except ValueError as e:
        print(f"q_arm 长度检查: OK -> {e}")

    # 无效 q_gripper 长度
    try:
        URUDPClient.pack([0.0] * 6, 0, [0.0] * 2)
        print("q_gripper 长度检查: FAIL（应抛异常）")
    except ValueError as e:
        print(f"q_gripper 长度检查: OK -> {e}")

    # 无效 mode 值
    try:
        URUDPClient.pack([0.0] * 6, 256, [0.0] * 3)
        print("mode 范围检查: FAIL（应抛异常）")
    except ValueError as e:
        print(f"mode 范围检查: OK -> {e}")

    # 未 start 就 send
    client = URUDPClient(bind_host=BIND_HOST, bind_port=BIND_PORT + 4)
    try:
        client.send_to(
            (BIND_HOST, BIND_PORT + 4),
            q_arm=[0.0] * 6,
            mode=0,
            q_gripper=[0.0] * 3,
        )
        print("未 start 发送: FAIL（应抛异常）")
    except RuntimeError as e:
        print(f"未 start 发送: OK -> {e}")


def test_mode_names():
    """7. 控制模式名称映射。"""
    print_sep("7. 控制模式")

    for m in range(5):
        print(f"  mode={m}: {UDPControlMode.name(m)}")
    print(f"  mode=99 (未定义): {UDPControlMode.name(99)}")


def main():
    print("URUDPClient 使用示例 / 功能测试")
    print(f"绑定地址: {BIND_HOST}:{BIND_PORT}")

    tests = [
        test_pack_standalone,
        test_basic_send_recv,
        test_send_rate_limit,
        test_wait_next,
        test_continuous_stream,
        test_error_handling,
        test_mode_names,
    ]

    passed = 0
    for test in tests:
        try:
            test()
            passed += 1
        except Exception as e:
            print(f"\n  !! {test.__name__} 异常: {e}")

    print_sep(f"完成 ({passed}/{len(tests)} 项通过)")


if __name__ == "__main__":
    main()
