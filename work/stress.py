"""Old filter vs new filter over many seeds and disturbance strengths (full config, final/rmse position error)."""
import numpy as np, os
from multi import run_replay
import sim
FULL = "enc_v=1 enc_w=1 mag=1 zupt=1"
def err(exe, seed, extra=""):
    out = f"est_{seed}.csv"
    run_replay(f"{exe} run_{seed}.csv {out} still=600 moff=0.05,-0.03,0.02 {FULL} {extra}".split())
    est = np.loadtxt(out, delimiter=",", skiprows=1); gt = np.loadtxt(f"gt_{seed}.csv", delimiter=",", skiprows=1)
    n = min(len(est), len(gt)-1); e = np.hypot(est[:n,1]-gt[1:n+1,1], est[:n,2]-gt[1:n+1,2])
    os.remove(out); return e[-1], np.sqrt(np.mean(e**2))
N = 20
import sys
if __name__ != "__main__": sys.exit  # noqa
print(f"{'disturb':>8s} {'bias_rw':>8s} | {'old final':>10s} {'new final':>10s} | {'old rmse':>9s} {'new rmse':>9s} | new better / worse than old")
sd = 9000
for bias_rw in (2e-5, 1e-4):
  for d in (0.0, 0.04, 0.08, 0.15, 0.25):
    res = []
    for k in range(N):
        sd += 1; sim.make(sd, ".", disturb=d, bias_rw=bias_rw)
        res.append(err("./replay_old.exe", sd) + err("./replay.exe", sd))
        os.remove(f"run_{sd}.csv"); os.remove(f"gt_{sd}.csv")
    r = np.array(res); m = r.mean(0)
    better = int((r[:,2] < r[:,0]-0.02).sum()); worse = int((r[:,2] > r[:,0]+0.02).sum())
    print(f"{d:8.2f} {bias_rw:8.0e} | {m[0]:10.2f} {m[2]:10.2f} | {m[1]:9.2f} {m[3]:9.2f} | {better:2d} / {worse:2d}   (worst new final {r[:,2].max():.2f})")
