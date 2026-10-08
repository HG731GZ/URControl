"""测量持续关节力矩订阅的新样本频率、到达间隔和控制器时间戳缺口。"""

import argparse
import json
import time
from pathlib import Path

import numpy as np

from UR_Utils.URRTDETorqueClient import URRTDETorqueClient


def measure(read_sample, duration: float, frequency: float) -> dict:
    """采集期间只读数据并记录时间，结束后统一统计，避免逐帧打印影响结果。"""
    arrivals = []
    timestamps = []
    started = time.perf_counter()
    deadline = started + duration
    while time.perf_counter() < deadline:
        timestamp, torques = read_sample()
        arrivals.append(time.perf_counter())
        timestamps.append(timestamp)

    intervals_ms = np.diff(arrivals) * 1000
    controller_intervals = np.diff(timestamps)
    sample_steps = np.rint(controller_intervals * frequency).astype(int)
    duplicates = int(np.count_nonzero(sample_steps == 0))
    host_span = arrivals[-1] - arrivals[0]
    controller_span = timestamps[-1] - timestamps[0]
    return {
        "requested_frequency_hz": frequency,
        "samples": len(arrivals),
        "elapsed_seconds": arrivals[-1] - started,
        "fresh_samples_per_second": (len(arrivals) - 1 - duplicates) / host_span,
        "controller_samples_per_second": (len(arrivals) - 1 - duplicates) / controller_span,
        "controller_span_seconds": controller_span,
        "controller_to_host_clock_ratio": controller_span / host_span,
        "missing_sample_periods": int(np.maximum(sample_steps - 1, 0).sum()),
        "duplicate_timestamps": duplicates,
        "arrival_interval_ms": {
            "mean": float(np.mean(intervals_ms)),
            "p50": float(np.percentile(intervals_ms, 50)),
            "p95": float(np.percentile(intervals_ms, 95)),
            "p99": float(np.percentile(intervals_ms, 99)),
            "max": float(np.max(intervals_ms)),
        },
        "arrival_gaps_over_4ms": int(np.count_nonzero(intervals_ms > 4)),
        "arrival_gaps_over_10ms": int(np.count_nonzero(intervals_ms > 10)),
        "controller_interval_max_ms": float(controller_intervals.max() * 1000),
        "last_torques_nm": torques.tolist(),
    }


def main():
    parser = argparse.ArgumentParser(description="测量 RTDE 关节力矩的持续采集频率")
    parser.add_argument("--host", default="127.0.0.1", help="模拟器或机械臂地址")
    parser.add_argument("--frequency", type=float, default=500, help="订阅频率，单位 Hz")
    parser.add_argument("--duration", type=float, default=30, help="测量时长，单位秒")
    parser.add_argument("--output", type=Path, help="统计结果的 JSON 保存路径")
    args = parser.parse_args()

    with URRTDETorqueClient(args.host, args.frequency) as client:
        # 先接收半秒数据，排除连接和订阅初始化的开销。
        for _ in range(int(args.frequency / 2)):
            client.read()
        result = measure(client.read, args.duration, args.frequency)

    result["host"] = args.host
    text = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    print(text, end="")
    if args.output:
        args.output.write_text(text)


if __name__ == "__main__":
    main()
