"""离线生成标准 CSV，不连接机器人。"""

import argparse
from UR_Utils.impedance_trajectory import generate


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("output")
    p.add_argument(
        "--kind", choices=["hold", "single", "staggered", "sine"], default="single"
    )
    p.add_argument("--axis", type=int, default=1)
    p.add_argument("--amplitude", type=float, default=0.05)
    p.add_argument("--segment", type=float, default=2)
    p.add_argument("--hold", type=float, default=0.5)
    p.add_argument("--sample-hz", type=float, default=100)
    a = p.parse_args()
    trajectory = generate(
        a.kind, a.axis - 1, a.amplitude, a.segment, a.hold, a.sample_hz
    )
    trajectory.save(a.output)
    print(
        f"{len(trajectory.rows)} rows, {trajectory.duration():.3f} s, {trajectory.sha256}"
    )


if __name__ == "__main__":
    main()
