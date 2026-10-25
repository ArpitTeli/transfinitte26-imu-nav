"""Per-sample update time of the causal navigator, against the live budget.

The phone streams at 400 Hz, so each sample must be processed in under 2.5 ms (the shoe preset is what runs live).
The other modes are timed on the same data for comparison.
Run: python scripts/bench_realtime.py [--fs 400]
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imunav.pipeline import PRESETS, Config, make_navigator  # noqa: E402
from imunav.synth import make_foot_walk  # noqa: E402


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--fs", type=float, default=400.0)
    a = ap.parse_args()
    budget_ms = 1000.0 / a.fs
    seq = make_foot_walk(fs=a.fs, seed=0)
    dts = np.diff(seq.t, prepend=seq.t[0] - 1.0 / a.fs)
    print(f"{'config':16s} {'mean ms':>9s} {'p99 ms':>9s} {'max ms':>9s}  budget {budget_ms:.2f} ms ({a.fs:.0f} Hz)")
    ok = True
    for name, cfg in [("shoe (live)", PRESETS["shoe"])] + [(m, Config(mode=m)) for m in ("madgwick", "mahony", "ekf")]:
        nav, _ = make_navigator(seq, cfg)
        times = np.empty(len(seq.t))
        for k in range(len(seq.t)):
            t0 = time.perf_counter()
            nav.step(seq.gyro[k], seq.acc[k], dts[k])
            times[k] = (time.perf_counter() - t0) * 1e3
        print(f"{name:16s} {times.mean():9.3f} {np.percentile(times, 99):9.3f} {times.max():9.3f}")
        ok &= np.percentile(times, 99) < budget_ms
    print("PASS: p99 under budget" if ok else "FAIL: p99 over budget")


if __name__ == "__main__":
    main()
