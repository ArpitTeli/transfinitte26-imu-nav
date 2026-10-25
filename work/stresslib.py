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
