"""Score the filter against the hackathon tiers.
Tier 1: max position error over the first 30 s of motion, starting from the true pose when motion begins.
Tier 2: error after 50 m of travel as % of distance (target <= 3 %, i.e. 1.5 m), each 50 m stretch started from the true pose
        (relative error, like a fresh run on a marked 50 m path). Sim: the 60 m closed loop, incl. return-to-start error.
usage: python targets.py settings.json        (settings.json "args" are applied to the NCLT runs; the sim uses defaults)"""
import sys, os, json, numpy as np
from concurrent.futures import ProcessPoolExecutor
from multi import run_replay
CFG = [("IMU only", "enc_v=0 enc_w=0 mag=0 zupt=0"), ("+ wheel speed", "enc_v=1 enc_w=0 mag=0 zupt=0"),
       ("+ ZUPT", "enc_v=1 enc_w=0 mag=0 zupt=1"), ("+ magnetometer (full)", "enc_v=1 enc_w=0 mag=1 zupt=1")]
SESS = ["s0108", "s0204", "s0115", "s0122"]
SEG, STEP = 50.0, 25.0

def replay(csv_in, params):
    out = f"tg_{os.getpid()}.csv"
    run_replay(["./replay.exe", csv_in, out] + params)
    e = np.loadtxt(out, delimiter=",", skiprows=1); os.remove(out); return e

def from_pose(E, yaw, t0i, Gp, gh, mirror):
    """map estimate positions after index t0i into the GT frame using only the true pose at t0i"""
    d = E[t0i:] - E[t0i]
    if mirror:   # est heading psi corresponds to GT heading -psi (+const): reflection
        th = gh + yaw[t0i]; M = np.array([[np.cos(th), np.sin(th)], [np.sin(th), -np.cos(th)]])
    else:
        th = gh - yaw[t0i]; M = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    return Gp + d @ M.T

def nclt_job(a):
    pre, name, extra = a
    params = open(f"{pre}_params.txt").read().split() + extra.split()
    e = replay(f"{pre}_run.csv", params)
    gt = np.loadtxt(f"{pre}_gt.csv", delimiter=",", skiprows=1); tm = float(open(f"{pre}_meta.txt").read())
    t = e[:, 0]; ok = (t >= gt[0, 0]) & (t <= gt[-1, 0]); e, t = e[ok], t[ok]
    E, yaw = e[:, 1:3], np.unwrap(e[:, 3])
    G = np.column_stack([np.interp(t, gt[:, 0], gt[:, 1]), np.interp(t, gt[:, 0], gt[:, 2])])
    H = np.interp(t, gt[:, 0], gt[:, 3])
    dist = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(G, axis=0).T))])
    # tier 1
    i0 = np.searchsorted(t, tm); i1 = np.searchsorted(t, tm + 30)
    A = from_pose(E, yaw, i0, G[i0], H[i0], True)
    t1 = float(np.hypot(*(A[:i1 - i0] - G[i0:i1]).T).max()); d30 = float(dist[i1] - dist[i0])
    # tier 2: 50 m stretches every 25 m of travel
    ends, maxs = [], []
    s = dist[i0]
    while s + SEG <= dist[-1]:
        a = np.searchsorted(dist, s); b = np.searchsorted(dist, s + SEG)
        A = from_pose(E[:b + 1], yaw[:b + 1], a, G[a], H[a], True)
        err = np.hypot(*(A - G[a:b + 1]).T); ends.append(err[-1]); maxs.append(err.max()); s += STEP
    return pre, name, t1, d30, np.array(ends), np.array(maxs)

def sim_job(a):
    seed, name, flags = a
    e = replay(f"run_{seed}.csv", "still=600 moff=0.05,-0.03,0.02".split() + flags.split())
    gt = np.loadtxt(f"gt_{seed}.csv", delimiter=",", skiprows=1)
    n = min(len(e), len(gt) - 1); E, G, t = e[:n, 1:3], gt[1:n + 1, 1:3], e[:n, 0]
    err = np.hypot(*(E - G).T); dist = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(G, axis=0).T))])
    mv = t[np.argmax(np.hypot(*np.diff(G, axis=0, prepend=G[:1]).T) > 1e-4)]
    m30 = (t >= mv) & (t <= mv + 30)
    return seed, name, float(err[m30].max()), float(err[-1]), float(100 * err[-1] / dist[-1]), float(err.max()), float(np.hypot(*(E[-1] - E[0]))), float(dist[-1])

if __name__ == "__main__":
    tuned = json.load(open(sys.argv[1]))["args"]
    with ProcessPoolExecutor(12) as pool:
        R = list(pool.map(nclt_job, [(p, n, f"{tuned} {c}") for p in SESS for n, c in CFG]))
        S = list(pool.map(sim_job, [(s, n, c) for s in range(6) for n, c in CFG]))
    print(f"settings: {sys.argv[1]}")
    print("\n=== TIER 1: max error over first 30 s of motion (target < 1 m) ===")
    print(f"{'config':24s} " + " ".join(f"{p:>9s}" for p in SESS) + f" {'sim (6 seeds, worst)':>21s}")
    for n, _ in CFG:
        row = [next(x[2] for x in R if x[0] == p and x[1] == n) for p in SESS]
        sw = max(x[2] for x in S if x[1] == n)
        print(f"{n:24s} " + " ".join(f"{v:9.2f}" for v in row) + f" {sw:21.2f}")
    print("  distance covered in those 30 s (m): " + " ".join(f"{p} {next(x[3] for x in R if x[0] == p):.0f}" for p in SESS) + "   sim ~14")
    print("\n=== TIER 2, NCLT: error after 50 m, each stretch started from the true pose (target <= 3 % = 1.5 m) ===")
    print(f"{'config':24s} {'stretches':>9s} {'median %':>9s} {'p90 %':>7s} {'worst %':>8s} {'meet 3 %':>9s} {'median max-along m':>19s}")
    for n, _ in CFG:
        ends = np.concatenate([x[4] for x in R if x[1] == n]); maxs = np.concatenate([x[5] for x in R if x[1] == n])
        pc = 100 * ends / SEG
        print(f"{n:24s} {len(pc):9d} {np.median(pc):9.2f} {np.percentile(pc, 90):7.2f} {pc.max():8.1f} {100 * np.mean(pc <= 3):8.1f}% {np.median(maxs):19.2f}")
    print("\n=== TIER 2, sim: 60 m closed loop (6 seeds, mean / worst) ===")
    print(f"{'config':24s} {'final m':>14s} {'final %':>14s} {'max m':>14s} {'return-to-start m':>18s}")
    for n, _ in CFG:
        r = [x for x in S if x[1] == n]
        f = lambda i: f"{np.mean([x[i] for x in r]):6.2f} / {max(x[i] for x in r):5.2f}"
        print(f"{n:24s} {f(3):>14s} {f(4):>14s} {f(5):>14s} {f(6):>18s}")
