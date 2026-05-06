import time

def precise_delay_us(us: int, spin_threshold_us: int = 200):
    target = time.perf_counter_ns() + us * 1000

    sleep_time_us = us - spin_threshold_us
    if sleep_time_us > 0:
        time.sleep(sleep_time_us / 1_000_000)

    while time.perf_counter_ns() < target:
        pass


def precise_delay_ms(ms: int):
    precise_delay_us(ms * 1000)
