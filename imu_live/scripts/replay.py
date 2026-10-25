"""Replay a recording from the live app through the same tracker, for tuning and scoring.

Run:  python scripts/replay.py results/live/20261024_101500.csv --end 50 0
      python scripts/replay.py rec.csv --loop                  (walked back to the start)
      python scripts/replay.py rec.csv --preset shoe --set zupt_gyro_thr=0.5
--end X Y: measured end point of the marked path (m, start at 0,0, first leg along +x after
           --align). Prints final error and drift as % of distance.
--align:   rotate the track so its first 3 m point along +x (the phone does not know north).
"""
import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imunav.live.recording import load_recording  # noqa: E402
from imunav.live.tracker import LiveTracker  # noqa: E402
from imunav.pipeline import PRESETS  # noqa: E402


def heading_align(p, dist=3.0):
    """Rotate so the direction from the start to the first point `dist` away is +x."""
    d = np.linalg.norm(p[:, :2] - p[0, :2], axis=1)
    i = int(np.argmax(d >= dist)) if np.any(d >= dist) else len(p) - 1
    ang = np.arctan2(p[i, 1] - p[0, 1], p[i, 0] - p[0, 0])
    c, s = np.cos(-ang), np.sin(-ang)
    q = p - p[0]
    return np.column_stack([c * q[:, 0] - s * q[:, 1], s * q[:, 0] + c * q[:, 1], q[:, 2]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--preset", default="shoe", choices=sorted(PRESETS))
    ap.add_argument("--set", nargs="*", default=[], help="override config fields, e.g. zupt_win=0.1")
    ap.add_argument("--end", nargs=2, type=float, metavar=("X", "Y"))
    ap.add_argument("--loop", action="store_true", help="path ends at the start")
    ap.add_argument("--align", action="store_true", help="first 3 m of the track define +x")
    ap.add_argument("--out")
    a = ap.parse_args()

    cfg = PRESETS[a.preset]
    for kv in a.set:
        k, v = kv.split("=")
        cfg = cfg.with_(**{k: type(getattr(cfg, k))(v)})
    df, info = load_recording(a.csv)
    print(f"{info['format']}: {info['duration']:.0f} s, {info['n_gaps']} data gaps totalling {info['gap_time']:.1f} s "
          f"(longest {info['longest_gap']:.1f} s)")
    tr = LiveTracker(cfg, display_hz=1e9)  # keep every sample
    for row in df.itertuples(index=False):
        tr.add(row.t, np.array([row.gx, row.gy, row.gz]), np.array([row.ax, row.ay, row.az]), row.gap)
    s = tr.snapshot()
    p = s["p"]
    if s["state"] != "tracking" or len(p) < 2:
        raise SystemExit("never left calibration: the recording must start with the shoe still")
    if a.align:
        p = heading_align(p)
    else:
        p = p - p[0]
    end = p[-1, :2]
    print(f"steps {s['steps']}  distance {s['distance']:.2f} m  end ({end[0]:.2f}, {end[1]:.2f}) m  "
          f"rejected {s['rejected']}")
    target = np.zeros(2) if a.loop else (np.array(a.end) if a.end else None)
    if target is not None:
        err = float(np.linalg.norm(end - target))
        print(f"final error {err:.2f} m = {100 * err / s['distance']:.2f} % of distance travelled"
              + ("  (return-to-start error)" if a.loop else ""))

    out = Path(a.out) if a.out else Path(a.csv).with_suffix(".png")
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(p[:, 0], p[:, 1], lw=1.5, label="estimate")
    ax.plot(*p[0, :2], "ko", label="start")
    if target is not None:
        ax.plot(*target, "r*", ms=14, label="true end")
    ax.set(xlabel="x [m]", ylabel="y [m]", aspect="equal", title=Path(a.csv).name)
    ax.grid(alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    print("plot:", out)


if __name__ == "__main__":
    main()
