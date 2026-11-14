"""Single-process timing of OA-Stack (run THIS on the Raspberry Pi 5, alongside the other
algorithms, to check the compute budget). No simulator needed.

    python bench/timing_stack.py            # 2000 cycles, prints mean / p50 / p95 / p99 / max (ms)

It replays a fixed, realistic workload: a 500-beam scan of a corridor every 2nd cycle
(10 Hz at a 20 Hz loop), UP/DOWN ToF frames every 3rd cycle (~7 Hz each, worst case both),
a duct and a low box in view, the drone moving at 0.6 m/s.
"""
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "tests"))

from oa_stack import StackAvoider, StackConfig  # noqa: E402
from test_oa_stack import ANG, frames, scan_of_boxes  # noqa: E402


def main(n=2000):
    cfg = StackConfig()
    boxes = [((-5.0, 0.75, -1.0), (8.0, 0.85, 1.4)), ((-5.0, -0.85, -1.0), (8.0, -0.75, 1.4)),   # corridor
             ((1.0, -0.75, 0.13), (1.3, 0.75, 0.19)),                                            # duct
             ((2.5, -0.3, -1.0), (2.9, 0.2, 0.0))]                                               # low box
    fr, sc = frames(cfg, boxes, 1.0), scan_of_boxes(boxes)
    av = StackAvoider(cfg)
    ms = []
    for k in range(n):
        t = 0.05 * k
        t0 = time.perf_counter()
        av.update(t, (0.6 * t % 1.0, 0.0, 0.0), (0.6, 0.05), scan=(ANG, sc) if k % 2 == 0 else None,
                  tof_frames=fr if k % 3 == 0 else (), height=1.0)
        ms.append(1000 * (time.perf_counter() - t0))
    ms = np.array(ms[50:])
    print(f"cycles {len(ms)}  mean {ms.mean():.2f} ms  p50 {np.percentile(ms, 50):.2f}  "
          f"p95 {np.percentile(ms, 95):.2f}  p99 {np.percentile(ms, 99):.2f}  max {ms.max():.2f}")
    print(f"budget check (<= 10 ms mean, 20 Hz loop): {'PASS' if ms.mean() <= 10 else 'FAIL'}; "
          f"CPU share of one core at 20 Hz: {100 * ms.mean() * 20 / 1000:.1f}%")


if __name__ == "__main__":
    main()
