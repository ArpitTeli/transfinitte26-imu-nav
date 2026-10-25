"""Run ./replay.exe on one run and plot estimate vs ground truth for every ablation config.
usage: python plot.py [seed] [out.png]     (needs run_<seed>.csv and gt_<seed>.csv from sim.py)
Also works for converted real data: any run_<name>.csv + gt_<name>.csv (gt = t,x,y,yaw, same frame).
"""
import sys
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from evaluate import CONFIGS, run

seed = sys.argv[1] if len(sys.argv) > 1 else "0"
out = sys.argv[2] if len(sys.argv) > 2 else f"plot_{seed}.png"

results = []
for name, flags in CONFIGS:
    est, gt, e, dist, log = run(seed, flags)
    results.append((name, est, gt, e))
gt = results[0][2]
est_full = results[-1][1]
t = est_full[:, 0]

fig, ax = plt.subplots(2, 2, figsize=(14, 10))

a = ax[0, 0]
a.plot(gt[:, 1], gt[:, 2], "k", lw=3, alpha=0.4, label="ground truth")
for name, est, _, _ in results:
    a.plot(est[:, 1], est[:, 2], lw=1, label=name)
a.set_aspect("equal"); a.set_xlabel("x [m]"); a.set_ylabel("y [m]")
a.set_title("Trajectory"); a.legend(fontsize=7); a.grid(alpha=0.3)

a = ax[0, 1]
for name, est, _, e in results:
    a.plot(est[:, 0], e, lw=1, label=name)
a.set_xlabel("t [s]"); a.set_ylabel("position error [m]")
a.set_title("Position error vs time"); a.legend(fontsize=7); a.grid(alpha=0.3)

a = ax[1, 0]
a.plot(t, np.unwrap(gt[:, 3]) if gt.shape[1] > 3 else 0, "k", lw=3, alpha=0.4, label="ground truth")
a.plot(t, np.unwrap(est_full[:, 3]), lw=1, label="full filter")
a.set_xlabel("t [s]"); a.set_ylabel("yaw [rad]"); a.set_title("Yaw (full config)")
a.legend(fontsize=7); a.grid(alpha=0.3)

a = ax[1, 1]
a.plot(t, est_full[:, 5], label="gyro-z bias [rad/s]")
a.plot(t, est_full[:, 6], label="accel-fwd bias [m/s^2]")
a.fill_between(t, a.get_ylim()[0], a.get_ylim()[1], where=est_full[:, 7] > 0, color="grey", alpha=0.15, label="ZUPT active")
a.set_xlabel("t [s]"); a.set_title("Estimated biases (full config)")
a.legend(fontsize=7); a.grid(alpha=0.3)

fig.tight_layout()
fig.savefig(out, dpi=110)
print("saved", out)
for name, _, _, e in results:
    print(f"{name:30s} final {e[-1]:.2f} m   max {e.max():.2f} m   rmse {np.sqrt(np.mean(e**2)):.2f} m")
