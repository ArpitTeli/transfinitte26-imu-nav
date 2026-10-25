"""Ablation table + plot per NCLT session with given tuned settings.
usage: python final_report.py tuned.json s0108 s0204 ..."""
import sys, json, numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from concurrent.futures import ProcessPoolExecutor
from multi import run, stats
CFG = [("IMU only", "enc_v=0 enc_w=0 mag=0 zupt=0"), ("+ wheel speed", "enc_v=1 enc_w=0 mag=0 zupt=0"),
       ("+ ZUPT", "enc_v=1 enc_w=0 mag=0 zupt=1"), ("+ magnetometer (full)", "enc_v=1 enc_w=0 mag=1 zupt=1")]
def job(a):
    pre, args = a; r = run(pre, args); return pre, args, r, stats(r)
if __name__ == "__main__":
    if len(sys.argv) < 3: sys.exit(__doc__)          # no arguments: show the usage, not a traceback
    tuned = json.load(open(sys.argv[1]))["args"]; sess = sys.argv[2:]
    jobs = [(pre, f"{tuned} {c}") for pre in sess for _, c in CFG]
    with ProcessPoolExecutor(8) as p: res = list(p.map(job, jobs))
    for i, pre in enumerate(sess):
        rows = res[i * len(CFG):(i + 1) * len(CFG)]
        G = rows[0][2]["G"]; dist = rows[0][2]["dist"][-1]
        print(f"\n{pre}: {dist/1000:.2f} km, {rows[0][2]['t'][-1]/60:.0f} min")
        print(f"  {'config':24s} {'RMSE m':>7s} {'first40':>8s} {'after40':>8s} {'final m':>8s} {'%dist':>6s}")
        fig, ax = plt.subplots(1, 2, figsize=(15, 7))
        ax[0].plot(G[:, 0], G[:, 1], "k", lw=3, alpha=0.35, label="ground truth")
        for (name, _), (_, _, r, s) in zip(CFG, rows):
            print(f"  {name:24s} {s['rmse']:7.1f} {s['tune']:8.1f} {s['held']:8.1f} {s['final']:8.1f} {s['pct']:6.2f}")
            ax[0].plot(r["Ea"][:, 0], r["Ea"][:, 1], lw=0.8, label=name)
            ax[1].plot(r["t"] / 60, r["e"], lw=0.8, label=name)
        print("  gating (full):", rows[-1][2]["log"])
        lo, hi = G.min(0), G.max(0); pad = 0.15 * (hi - lo).max()      # zoom to GT (IMU-only leaves the map)
        ax[0].set_xlim(lo[0] - pad, hi[0] + pad); ax[0].set_ylim(lo[1] - pad, hi[1] + pad)
        ax[0].set_aspect("equal"); ax[0].legend(fontsize=7); ax[0].grid(alpha=0.3)
        ax[0].set_title(f"NCLT {pre}: trajectory (aligned on first 60 s of motion only)")
        ax[1].set_yscale("log"); ax[1].set_xlabel("t [min]"); ax[1].set_ylabel("position error [m]")
        ax[1].axvline(40, color="grey", ls="--", lw=1); ax[1].text(40.5, ax[1].get_ylim()[0] * 1.5, "tuning | held-out", fontsize=8, color="grey")
        ax[1].set_title("Position error vs time"); ax[1].legend(fontsize=7); ax[1].grid(alpha=0.3, which="both")
        fig.tight_layout(); fig.savefig(f"result_{pre}.png", dpi=110); plt.close(fig)
    print("\nsaved", ", ".join(f"result_{p}.png" for p in sess))
