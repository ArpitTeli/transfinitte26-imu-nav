"""Allan deviation of a long static IMU recording.

Run: python scripts/run_allan.py imu.csv --fs 200 --layout configs/layout.json
Writes results/allan_gyro.png and results/allan_acc.png and prints N, B, K per axis.
"""
import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imunav.calib import allan_deviation, allan_params  # noqa: E402
from imunav.io import EUROC_IMU, load_imu_csv  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--fs", type=float, default=200.0)
    ap.add_argument("--layout", help="JSON with imu_layout, gyro_scale, acc_scale (default: EuRoC)")
    ap.add_argument("--out", default="results")
    a = ap.parse_args()
    kw = json.load(open(a.layout)) if a.layout else {}
    _, gyro, acc = load_imu_csv(a.csv, kw.get("imu_layout", EUROC_IMU), kw.get("gyro_scale", 1.0), kw.get("acc_scale", 1.0))
    Path(a.out).mkdir(exist_ok=True)
    for name, x, unit in (("gyro", gyro, "rad/s"), ("acc", acc, "m/s²")):
        fig, ax = plt.subplots(figsize=(6, 4))
        for k, ax_name in enumerate("xyz"):
            taus, adev = allan_deviation(x[:, k] - x[:, k].mean(), a.fs)
            p = allan_params(taus, adev)
            print(f"{name} {ax_name}: N={p['N']:.3e} {unit}/√Hz  B={p['B']:.3e} {unit} (tau={p['tau_B']:.0f} s)  K={p['K']:.3e}")
            ax.loglog(taus, adev, label=ax_name)
        ax.set(xlabel="tau [s]", ylabel=f"Allan deviation [{unit}]", title=f"{name} Allan deviation")
        ax.grid(alpha=0.3, which="both")
        ax.legend()
        fig.tight_layout()
        fig.savefig(Path(a.out) / f"allan_{name}.png", dpi=130)


if __name__ == "__main__":
    main()
