"""在 PyCharm 中直接运行，打印并动态绘制指定关节的补偿力矩和两个诊断比例。

使用项目的 urcontrol 解释器，依赖 rtde_control、官方 rtde 客户端和 matplotlib。
运行前机械臂应已上电、松抱闸并处于远程模式，且没有其他控制程序占用。
连接时会上传并运行 ur_rtde 控制脚本，仅查询力矩，不发送运动指令。
两个比例字段要求 PolyScope 5.26.3 / 10.15.0 或以上版本支持。
补偿力矩与比例来自两次独立读取，不代表严格同一控制周期的测量。
三幅子图独立自动缩放纵轴，共享滚动时间窗口；横轴为首帧起的 RTDE 时间。
关闭曲线窗口或使用 Ctrl+C 结束，退出时停止本脚本的控制程序并断开连接。
"""

from collections import deque
from contextlib import ExitStack
import threading

import matplotlib
import rtde_control
from rtde.rtde import RTDE


ROBOT_IP = "192.168.3.15"  # 实机地址。
JOINT_NUMBER = 3  # 1～6：基座、肩部、肘部、腕部一、腕部二、腕部三。
PRINT_FREQUENCY_HZ = 100.0  # 比例订阅及力矩查询的目标频率，范围为 1～500 Hz。
PRINT_TO_CONSOLE = True  # 是否同时逐帧打印；只看曲线时可关闭。
TIME_WINDOW_SECONDS = 20.0  # 滚动时间窗口，至少 10 秒。
PLOT_REFRESH_INTERVAL_MS = 100  # 图形刷新间隔，与采样频率分开设置。
MIN_Y_SPANS = (0.01, 1e-6, 1e-6)  # 三幅子图的最小纵轴跨度：N·m、比例、比例。
Y_PADDING_FRACTION = 0.15  # 数据上下各留出的边距，占当前跨度的比例。


def collect_samples(samples, lock, stop_event):
    """在后台串行读取机器人，缓存最近一个时间窗口内的数据。"""
    joint_index = JOINT_NUMBER - 1
    start_timestamp = None
    print(f"连接 {ROBOT_IP}，读取 J{JOINT_NUMBER}，目标 {PRINT_FREQUENCY_HZ:g} Hz……", flush=True)
    with ExitStack() as resources:
        receiver = RTDE(ROBOT_IP)
        resources.callback(receiver.disconnect)
        receiver.connect()
        if not receiver.send_output_setup(
            ["timestamp", "joint_position_deviation_ratios", "collision_detection_ratios"],
            ["DOUBLE", "VECTOR6D", "VECTOR6D"],
            frequency=PRINT_FREQUENCY_HZ,
        ):
            raise RuntimeError("RTDE 比例字段订阅失败，请检查控制器是否支持这两个字段")

        if stop_event.is_set():
            return
        control = rtde_control.RTDEControlInterface(ROBOT_IP)
        resources.callback(control.disconnect)
        resources.callback(control.stopScript)
        if not receiver.send_start():
            raise RuntimeError("RTDE 比例订阅启动失败")

        print("连接成功，关闭曲线窗口或按 Ctrl+C 结束。", flush=True)
        while not stop_event.is_set():
            state = receiver.receive()
            if state is None:
                raise RuntimeError("RTDE 比例数据接收超时或连接已断开")
            if not control.isProgramRunning():
                raise RuntimeError("机器人端控制脚本已停止，无法读取补偿力矩")
            # 直接读取 UR 补偿力矩，不额外减偏置、不滤波、不设置死区。
            torques = control.getJointTorques()
            if start_timestamp is None:
                start_timestamp = state.timestamp
            elapsed = state.timestamp - start_timestamp
            values = (torques[joint_index], state.joint_position_deviation_ratios[joint_index],
                      state.collision_detection_ratios[joint_index])
            with lock:
                samples.append((elapsed, *values))
                while samples[0][0] < elapsed - TIME_WINDOW_SECONDS:
                    samples.popleft()
            if PRINT_TO_CONSOLE:
                print(
                    f"时间={elapsed:.3f} s | J{JOINT_NUMBER} | "
                    f"补偿力矩={values[0]:+.8g} N·m | "
                    f"joint_position_deviation_ratios={values[1]:.8g} | "
                    f"collision_detection_ratios={values[2]:.8g}",
                    flush=True,
                )


class DiagnosticPlot:
    """用三个独立纵轴显示原始数据，按可见窗口的极值自动缩放。"""

    def __init__(self):
        import matplotlib.pyplot as plt

        self.figure, self.axes = plt.subplots(3, 1, sharex=True, figsize=(11, 8), layout="constrained")
        self.figure.canvas.manager.set_window_title(f"UR J{JOINT_NUMBER} diagnostics")
        self.figure.suptitle(f"UR J{JOINT_NUMBER} | {TIME_WINDOW_SECONDS:g} s window | Waiting for data")
        self.names = ("Compensated joint torque", "joint_position_deviation_ratios", "collision_detection_ratios")
        self.lines = []
        for axis, name, unit, color, minimum in zip(
            self.axes, self.names, ("N m", "Ratio", "Ratio"),
            ("#1479b8", "#d58219", "#269269"), MIN_Y_SPANS,
        ):
            self.lines.append(axis.plot([], [], color=color, linewidth=1.3)[0])
            axis.set_title(name, loc="left", fontsize=10)
            axis.set_ylabel(unit)
            axis.set_ylim(-minimum / 2, minimum / 2)
            axis.grid(True, alpha=0.25)
            axis.ticklabel_format(axis="y", style="sci", scilimits=(-3, 4), useOffset=False)
        self.axes[-1].set_xlabel("Elapsed RTDE time (s)")
        self.axes[-1].set_xlim(0, TIME_WINDOW_SECONDS)

    def update(self, samples):
        """只用窗口内的值确定上下限；常值和全零曲线保留可读的最小跨度。"""
        if not samples:
            return
        columns = list(zip(*samples))
        right = max(TIME_WINDOW_SECONDS, columns[0][-1])
        self.axes[-1].set_xlim(right - TIME_WINDOW_SECONDS, right)
        for axis, line, name, values, minimum in zip(
            self.axes, self.lines, self.names, columns[1:], MIN_Y_SPANS,
        ):
            line.set_data(columns[0], values)
            low, high = min(values), max(values)
            center = (low + high) / 2
            span = max(high - low, minimum)
            half_span = span * (0.5 + Y_PADDING_FRACTION)
            axis.set_ylim(center - half_span, center + half_span)
            axis.set_title(f"{name}    latest: {values[-1]:+.8g}", loc="left", fontsize=10)
        self.figure.suptitle(f"UR J{JOINT_NUMBER} | {TIME_WINDOW_SECONDS:g} s window | Auto Y scale")


def main():
    """主线程负责图形事件，后台负责采样；关闭窗口后等待连接释放。"""
    if type(JOINT_NUMBER) is not int or JOINT_NUMBER not in range(1, 7):
        raise ValueError("JOINT_NUMBER 必须是 1～6 的整数")
    if not 1.0 <= PRINT_FREQUENCY_HZ <= 500.0:
        raise ValueError("PRINT_FREQUENCY_HZ 必须在 1～500 Hz 范围内")
    if TIME_WINDOW_SECONDS < 10:
        raise ValueError("TIME_WINDOW_SECONDS 至少为 10 秒")
    if PLOT_REFRESH_INTERVAL_MS <= 0 or Y_PADDING_FRACTION < 0:
        raise ValueError("图形刷新间隔必须为正数，纵轴边距不能为负数")
    if len(MIN_Y_SPANS) != 3 or any(span <= 0 for span in MIN_Y_SPANS):
        raise ValueError("MIN_Y_SPANS 必须包含三个正数")

    # 使用独立图形窗口，避免 PyCharm 内嵌绘图窗口接管动态刷新。
    matplotlib.use("QtAgg")
    import matplotlib.pyplot as plt

    plot = DiagnosticPlot()
    samples = deque()
    lock = threading.Lock()
    stop_event = threading.Event()
    errors = []

    def acquire():
        """将采集或清理错误交给主线程报告，不自动重连。"""
        try:
            collect_samples(samples, lock, stop_event)
        except Exception as exc:
            errors.append(exc)

    def refresh():
        """定时取缓存快照，图形操作仅在主线程执行。"""
        if errors:
            plt.close(plot.figure)
            return
        with lock:
            snapshot = list(samples)
        plot.update(snapshot)
        plot.figure.canvas.draw_idle()

    worker = threading.Thread(target=acquire, name="URJointDiagnostics")
    timer = plot.figure.canvas.new_timer(interval=PLOT_REFRESH_INTERVAL_MS)
    timer.add_callback(refresh)
    plot.figure.canvas.mpl_connect("close_event", lambda event: stop_event.set())
    worker.start()
    timer.start()
    try:
        plt.show(block=True)
    finally:
        timer.stop()
        stop_event.set()
        worker.join()
        plt.close(plot.figure)
    if errors:
        raise errors[0]


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n已结束读取并断开连接。", flush=True)
