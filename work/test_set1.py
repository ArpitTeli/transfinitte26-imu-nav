"""Held-out evaluation on set 1 (NCLT 2012-01-15) with FROZEN settings (frozen_settings.json, tuned on 01-08 + 02-04 only).
No parameter is chosen from this session. Writes test_set1.json + test_set1_*.png."""
import json, numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from concurrent.futures import ProcessPoolExecutor
from multi import run, stats, fit
FROZEN = json.load(open("frozen_settings.json"))["args"]
CFG = [("IMU only", "enc_v=0 enc_w=0 mag=0 zupt=0"), ("+ wheel speed", "enc_v=1 enc_w=0 mag=0 zupt=0"),
       ("+ ZUPT", "enc_v=1 enc_w=0 mag=0 zupt=1"), ("+ magnetometer (full)", "enc_v=1 enc_w=0 mag=1 zupt=1"),
       ("full + wheel yaw-rate (ref)", "enc_v=1 enc_w=1 mag=1 zupt=1")]
DEFAULT = ("untuned defaults (full)", "enc_v=1 enc_w=1 mag=1 zupt=1")      # original algorithm defaults, no tuning at all
def job(a):
    pre, name, args = a; r = run(pre, args); return pre, name, r
def heading_err(r, k=50):
    d1 = r["Ea"][k:] - r["Ea"][:-k]; d2 = r["G"][k:] - r["G"][:-k]; m = np.hypot(*d2.T) > 1.0
    dh = np.degrees((np.arctan2(d1[:, 1], d1[:, 0]) - np.arctan2(d2[:, 1], d2[:, 0]) + np.pi) % (2 * np.pi) - np.pi)
    return r["t"][:-k][m], dh[m]
def metrics(r):
    t, e = r["t"], r["e"]; s = stats(r)
    E, G = r["Ea"], r["G"]; M, tr = fit(E[::20], G[::20]); ate = np.hypot(*((E @ M.T + tr) - G).T)
    at = {f"{m}min": float(e[min(np.searchsorted(t, 60 * m), len(e) - 1)]) for m in (5, 10, 30, 60, 90) if 60 * m <= t[-1]}
    th, dh = heading_err(r)
    return dict(rmse=s["rmse"], mean=float(e.mean()), median=float(np.median(e)), p95=float(np.percentile(e, 95)), max=s["max"],
                t_max_min=float(t[np.argmax(e)] / 60), final=s["final"], pct=s["pct"], ate_bestfit=float(np.sqrt(np.mean(ate ** 2))),
                at=at, head_med_abs=float(np.median(np.abs(dh))), head_p95_abs=float(np.percentile(np.abs(dh), 95)),
                dist_km=float(r["dist"][-1] / 1000), dur_min=float(t[-1] / 60), log=r["log"])
if __name__ == "__main__":
    sess = ["s0115", "s0108", "s0204"]                  # set 1 + the two training sessions for context
    jobs = [(p, n, f"{FROZEN} {c}") for p in sess for n, c in CFG] + [(p, DEFAULT[0], DEFAULT[1]) for p in sess]
    with ProcessPoolExecutor(12) as pool: res = list(pool.map(job, jobs))
    R = {}
    for p, n, r in res: R.setdefault(p, {})[n] = r
    out = {p: {n: metrics(r) for n, r in d.items()} for p, d in R.items()}
    out["_frozen_args"] = FROZEN
    out["_calib"] = {p: open(f"{p}_params.txt").read().strip() for p in sess}
    json.dump(out, open("test_set1.json", "w"), indent=1)
    S = R["s0115"]; names = [n for n, _ in CFG] + [DEFAULT[0]]
    cols = {"IMU only": "#9aa0a6", "+ wheel speed": "#e8a33d", "+ ZUPT": "#3b8ea5", "+ magnetometer (full)": "#d1495b",
            "full + wheel yaw-rate (ref)": "#7b5ea7", "untuned defaults (full)": "#5c946e"}
    G = S[names[0]]["G"]
    # 1 trajectory
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.plot(G[:, 0], G[:, 1], color="black", lw=4, alpha=0.25, label="ground truth")
    for n in ["+ wheel speed", "untuned defaults (full)", "+ magnetometer (full)"]:
        ax.plot(S[n]["Ea"][:, 0], S[n]["Ea"][:, 1], lw=1.0, color=cols[n], label=n)
    lo, hi = G.min(0), G.max(0); pad = 0.12 * (hi - lo).max()
    ax.set_xlim(lo[0] - pad, hi[0] + pad); ax.set_ylim(lo[1] - pad, hi[1] + pad); ax.set_aspect("equal")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.legend(fontsize=9, loc="best"); ax.grid(alpha=0.3)
    ax.set_title("Set 1 (2012-01-15): estimated vs ground-truth path")
    fig.tight_layout(); fig.savefig("test_set1_traj.png", dpi=110); plt.close(fig)
    # 2 error vs time
    fig, ax = plt.subplots(figsize=(11, 5))
    for n in names: ax.plot(S[n]["t"] / 60, S[n]["e"], lw=1.0, color=cols[n], label=n)
    ax.set_yscale("log"); ax.set_xlabel("time [min]"); ax.set_ylabel("position error [m]"); ax.grid(alpha=0.3, which="both")
    ax.legend(fontsize=8, ncol=2); ax.set_title("Set 1: position error vs time (log scale)")
    fig.tight_layout(); fig.savefig("test_set1_err.png", dpi=110); plt.close(fig)
    # 3 heading error of full filter
    th, dh = heading_err(S["+ magnetometer (full)"])
    fig, ax = plt.subplots(figsize=(11, 3.6))
    ax.plot(th / 60, dh, lw=0.5, color=cols["+ magnetometer (full)"], alpha=0.6)
    ax.axhline(0, color="black", lw=0.8); ax.set_ylim(-30, 30); ax.set_xlabel("time [min]"); ax.set_ylabel("heading error [deg]")
    ax.grid(alpha=0.3); ax.set_title("Set 1: heading error of the full filter (direction of travel vs ground truth)")
    fig.tight_layout(); fig.savefig("test_set1_heading.png", dpi=110); plt.close(fig)
    # 4 CDF
    fig, ax = plt.subplots(figsize=(7, 4.5))
    for n in names:
        e = np.sort(S[n]["e"]); ax.plot(e, np.linspace(0, 1, len(e)), color=cols[n], lw=1.4, label=n)
    ax.set_xscale("log"); ax.set_xlabel("position error [m]"); ax.set_ylabel("fraction of time below"); ax.grid(alpha=0.3, which="both")
    ax.legend(fontsize=8); ax.set_title("Set 1: error distribution (CDF)")
    fig.tight_layout(); fig.savefig("test_set1_cdf.png", dpi=110); plt.close(fig)
    for p in sess:
        print(f"\n{p}")
        for n in names:
            m = out[p][n]
            print(f"  {n:28s} RMSE {m['rmse']:7.1f}  median {m['median']:7.1f}  p95 {m['p95']:7.1f}  max {m['max']:7.1f} @{m['t_max_min']:5.1f}min  final {m['final']:7.1f} ({m['pct']:.2f}%)  ATEfit {m['ate_bestfit']:6.1f}  head|med| {m['head_med_abs']:.1f}° p95 {m['head_p95_abs']:.1f}°")
        print("  gating (full):", out[p]["+ magnetometer (full)"]["log"])
