"""Run replay.exe on the converted NCLT run (all ablation configs), compare to ground truth, plot.
usage: python nclt_eval.py [prefix=nclt] [extra replay args...]
Alignment: est frame starts at (0,0,yaw 0); GT is in a world frame (and mirrored: its heading is clockwise-positive).
We fit a rigid transform (rotation, optional mirror, translation) on the first 20-80 s of motion ONLY, then apply it to
the whole trajectory, so drift later in the run is not hidden. 'ATE(best-fit)' re-fits over the whole run for reference.
"""
import sys, subprocess, numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
pre = sys.argv[1] if len(sys.argv) > 1 else "nclt"
extra = " ".join(sys.argv[2:])
params = open(f"{pre}_params.txt").read().strip()
CONFIGS = [("IMU only (gyro+accel)", "enc_v=0 enc_w=0 mag=0 zupt=0"),
           ("+ wheel speed", "enc_v=1 enc_w=0 mag=0 zupt=0"),
           ("+ ZUPT", "enc_v=1 enc_w=0 mag=0 zupt=1"),
           ("+ magnetometer (full)", "enc_v=1 enc_w=0 mag=1 zupt=1"),
           ("full + wheel yaw-rate (ref)", "enc_v=1 enc_w=1 mag=1 zupt=1")]
gt = np.loadtxt(f"{pre}_gt.csv", delimiter=",", skiprows=1)

def fit(P, Q):
    """best rigid map P->Q, tries both a rotation and a mirrored rotation; returns (M, t, mirror)."""
    best = None
    for mir in (1, -1):
        Pm = P * np.array([1, mir]); mp, mq = Pm.mean(0), Q.mean(0)
        U, _, Vt = np.linalg.svd((Pm - mp).T @ (Q - mq))
        d = np.sign(np.linalg.det(Vt.T @ U.T)); R = Vt.T @ np.diag([1, d]) @ U.T
        M = R @ np.diag([1, mir]); t = mq - R @ mp
        e = np.sqrt(np.mean(np.sum((P @ M.T + t - Q) ** 2, 1)))
        if best is None or e < best[0]: best = (e, M, t, mir)
    return best[1], best[2], best[3]

res = []
for name, flags in CONFIGS:
    cmd = f"./replay.exe {pre}_run.csv est_{pre}.csv {params} {flags} {extra}"
    log = subprocess.run(cmd.split(), capture_output=True, text=True).stderr.strip()
    est = np.loadtxt(f"est_{pre}.csv", delimiter=",", skiprows=1)
    t = est[:, 0]; ok = (t >= gt[0, 0]) & (t <= gt[-1, 0])
    t, E = t[ok], est[ok, 1:3]
    G = np.column_stack([np.interp(t, gt[:, 0], gt[:, 1]), np.interp(t, gt[:, 0], gt[:, 2])])
    w = (t >= 20) & (t <= 80)
    M, tr, mir = fit(E[w], G[w])
    Ea = E @ M.T + tr
    e = np.hypot(*(Ea - G).T)
    dist = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(G, axis=0).T))])
    M2, t2, _ = fit(E[::20], G[::20]); e2 = np.hypot(*((E @ M2.T + t2) - G).T)
    res.append((name, t, Ea, e, dist, np.sqrt(np.mean(e2 ** 2)), mir, log))

Gfull = G
print(f"run: {params}   mirror needed: {res[0][6] == -1}   GT distance {res[0][4][-1]/1000:.2f} km over {res[0][1][-1]/60:.0f} min")
print(f"{'config':28s} {'RMSE m':>8s} {'tune<40min':>11s} {'held-out':>9s} {'final m':>8s} {'max m':>8s} {'final %dist':>12s} {'ATE(best-fit) m':>16s}")
for name, t, Ea, e, dist, ate, mir, log in res:
    r = lambda m: np.sqrt(np.mean(e[m]**2))
    print(f"{name:28s} {np.sqrt(np.mean(e**2)):8.1f} {r(t<=2400):11.1f} {r(t>2400):9.1f} {e[-1]:8.1f} {e.max():8.1f} {100*e[-1]/dist[-1]:12.2f} {ate:16.1f}")
print("gating (full):", res[-1][7])

fig, ax = plt.subplots(1, 2, figsize=(15, 7))
ax[0].plot(Gfull[:, 0], Gfull[:, 1], "k", lw=3, alpha=0.35, label="ground truth")
for name, t, Ea, e, dist, ate, mir, log in res: ax[0].plot(Ea[:, 0], Ea[:, 1], lw=0.8, label=name)
ax[0].set_aspect("equal"); ax[0].set_title("NCLT 2012-02-04: trajectory (aligned on first 20-80 s)"); ax[0].legend(fontsize=7); ax[0].grid(alpha=0.3)
for name, t, Ea, e, dist, ate, mir, log in res: ax[1].plot(t / 60, e, lw=0.8, label=name)
ax[1].set_xlabel("t [min]"); ax[1].set_ylabel("position error [m]"); ax[1].set_yscale("log"); ax[1].set_title("Position error vs time"); ax[1].legend(fontsize=7); ax[1].grid(alpha=0.3, which="both")
fig.tight_layout(); fig.savefig(f"{pre}_result.png", dpi=110); print("saved", f"{pre}_result.png")
