"""Physics checks for a shoe recording with no ground truth, stride by stride.

A working foot-mounted INS must show, on level walking:
  * one detected stance per stride (stance = foot flat, detector says still),
  * speed close to 0 at the start of every stance (before the ZUPT pulls it to 0),
  * stride length about 1.0-1.6 m, peak swing speed below about 6 m/s,
  * foot height that stays near 0.
Run:  python scripts/diagnose.py recordings/20261024_150205.csv [--set k=v ...] [--plot out.png --at 40]
"""
import argparse
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imunav.calib import static_calibration  # noqa: E402
from imunav.live.recording import load_recording  # noqa: E402
from imunav.pipeline import PRESETS, navigator_from_static  # noqa: E402


def find_start(t, gyro, acc, gap, win):
    """first window of `win` seconds that looks static and has no data gap (the tracker does the same, in halves)"""
    i = 0
    while t[-1] - t[i] > win:
        j = np.searchsorted(t, t[i] + win)
        if not gap[i + 1:j].any() and static_calibration(t[i:j], gyro[i:j], acc[i:j], win)["looks_static"]:
            return i, j
        i = np.searchsorted(t, t[i] + win / 2)
    raise SystemExit("no still window found")


def run(df, cfg):
    t = df.t.to_numpy(); gyro = df[["gx", "gy", "gz"]].to_numpy(); acc = df[["ax", "ay", "az"]].to_numpy()
    gap = df.gap.to_numpy()
    i0, i1 = find_start(t, gyro, acc, gap, cfg.init_window)
    nav, cal = navigator_from_static(t[i0:i1], gyro[i0:i1], acc[i0:i1], cfg.with_(fs=1 / np.median(np.diff(t))))
    n = len(t) - i1
    P, V, S, D = np.zeros((n, 3)), np.zeros((n, 3)), np.zeros(n, bool), np.zeros((n, 3))
    for k in range(n):
        j = i1 + k
        if gap[j] > 0:
            nav.gap(gap[j])                      # nothing is integrated across a hole
        else:
            nav.step(gyro[j], acc[j], t[j] - t[j - 1])
        P[k], V[k], S[k] = nav.p, nav.v, nav.still
        D[k] = [np.linalg.norm(gyro[j] - nav.ins.bg), np.linalg.norm(acc[j]), 0]
    return dict(t=t[i1:], p=P, v=V, still=S, gm=D[:, 0], am=D[:, 1], nav=nav, cal=cal, t_start=t[i1])


def strides(r):
    """stance onsets, and per stride: length, peak speed, speed at the next stance onset"""
    s = r["still"].astype(int); on = np.flatnonzero(np.diff(s) == 1) + 1
    sp = np.linalg.norm(r["v"], axis=1)
    rows = []
    for a, b in zip(on[:-1], on[1:]):
        rows.append(dict(t=r["t"][a], dur=r["t"][b] - r["t"][a], length=float(np.linalg.norm(r["p"][b, :2] - r["p"][a, :2])),
                         peak_v=float(sp[a:b].max()), v_at_stance=float(sp[b - 1]), dz=float(r["p"][b, 2] - r["p"][a, 2])))
    return pd.DataFrame(rows, columns=["t", "dur", "length", "peak_v", "v_at_stance", "dz"])


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("csv"); ap.add_argument("--preset", default="shoe")
    ap.add_argument("--set", nargs="*", default=[]); ap.add_argument("--plot"); ap.add_argument("--at", type=float, default=None)
    a = ap.parse_args()
    cfg = PRESETS[a.preset]
    for kv in a.set:
        k, v = kv.split("="); cfg = cfg.with_(**{k: type(getattr(cfg, k))(v)})
    df, info = load_recording(a.csv)
    print(f"{info['format']}: {info['n_gaps']} data gaps, {info['gap_time']:.1f} s missing (longest {info['longest_gap']:.1f} s)")
    r = run(df, cfg); st = strides(r)
    walking = st[(st.dur > 0.6) & (st.dur < 2.5)]            # normal strides; long gaps = standing / shuffling
    e = r["nav"].ins
    print(f"{Path(a.csv).name}: {r['t'][-1] - r['t'][0]:.0f} s after calibration, {len(st)} stance-to-stance intervals, "
          f"{len(walking)} look like strides (0.6-2.5 s)")
    if len(walking):
        q = lambda c: f"median {walking[c].median():6.2f}  p90 {walking[c].quantile(.9):7.2f}  max {walking[c].max():8.2f}"
        print(f"  stride length m      {q('length')}")
        print(f"  peak swing speed m/s {q('peak_v')}")
        print(f"  speed at next stance {q('v_at_stance')}   (should be ~0)")
    print(f"  total distance {np.sum(np.linalg.norm(np.diff(r['p'][:, :2], axis=0), axis=1)):.0f} m, final height {r['p'][-1, 2]:+.1f} m, "
          f"max |height| {np.abs(r['p'][:, 2]).max():.1f} m")
    print(f"  still fraction {r['still'].mean():.2f}; updates {e.n_updates}; rejected {e.n_rejected}; forced {e.n_forced}")
    long_gaps = st[st.dur >= 2.5]
    if len(long_gaps):
        print(f"  {len(long_gaps)} intervals >= 2.5 s with no stance; longest {long_gaps.dur.max():.1f} s at t={long_gaps.loc[long_gaps.dur.idxmax(), 't'] - r['t_start']:.1f} s")
    if a.plot:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        t = r["t"] - r["t_start"]; t0 = a.at if a.at is not None else 10.0; m = (t >= t0) & (t <= t0 + 10)
        fig, ax = plt.subplots(4, 1, figsize=(12, 9), sharex=True)
        ax[0].plot(t[m], r["gm"][m], lw=.8); ax[0].axhline(cfg.zupt_gyro_thr, color="r", ls="--", lw=.8); ax[0].set_ylabel("|gyro - bias| rad/s"); ax[0].set_yscale("log")
        ax[1].plot(t[m], r["am"][m], lw=.8); ax[1].set_ylabel("|accel| m/s²")
        ax[2].plot(t[m], np.linalg.norm(r["v"][m], axis=1), lw=.8); ax[2].set_ylabel("|v| m/s")
        ax[3].plot(t[m], r["p"][m, 2], lw=.8); ax[3].set_ylabel("height m"); ax[3].set_xlabel("time after calibration (s)")
        for x in ax: x.fill_between(t[m], 0, 1, where=r["still"][m], transform=x.get_xaxis_transform(), color="g", alpha=.15, lw=0); x.grid(alpha=.3)
        ax[0].set_title(f"{Path(a.csv).name}: green = detector says still")
        fig.tight_layout(); fig.savefig(a.plot, dpi=110); print("plot:", a.plot)


if __name__ == "__main__":
    main()
