import time


def run_periodic(frequency_hz, stop_event, callback):
    """用单调纳秒时钟按绝对期限调度，末段忙等，超期跳过错过的周期。"""
    period_ns = round(1_000_000_000 / frequency_hz)
    deadline = time.perf_counter_ns()
    while not stop_event.is_set():
        callback()
        elapsed = time.perf_counter_ns() - deadline
        deadline += (elapsed // period_ns + 1) * period_ns
        sleep_ns = deadline - time.perf_counter_ns() - 200_000
        if sleep_ns > 0 and stop_event.wait(sleep_ns / 1_000_000_000):
            return
        while time.perf_counter_ns() < deadline:
            pass


def precise_delay_us(us: int, spin_threshold_us: int = 200):
    target = time.perf_counter_ns() + us * 1000

    sleep_time_us = us - spin_threshold_us
    if sleep_time_us > 0:
        time.sleep(sleep_time_us / 1_000_000)

    while time.perf_counter_ns() < target:
        pass


def precise_delay_ms(ms: int):
    precise_delay_us(ms * 1000)
