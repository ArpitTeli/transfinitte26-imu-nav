"""Run ./replay on runs, compare to ground truth, print ablation table.
usage: python3 evaluate.py [seeds...]    (needs run_<seed>.csv and gt_<seed>.csv)
For real datasets: convert to the replay CSV format and write gt as t,x,y,yaw in the SAME frame
(or add the alignment step noted in the README).
"""
import os, sys, numpy as np
from multi import run_replay

CONFIGS = [
    ("IMU only (gyro+accel)",      "enc_v=0 enc_w=0 mag=0 zupt=0"),
    ("+ wheel speed",              "enc_v=1 enc_w=0 mag=0 zupt=0"),
    ("+ wheel yaw-rate vs gyro",   "enc_v=1 enc_w=1 mag=0 zupt=0"),
    ("+ ZUPT",                     "enc_v=1 enc_w=1 mag=0 zupt=1"),
    ("+ magnetometer (full)",      "enc_v=1 enc_w=1 mag=1 zupt=1"),
]

def run(seed, flags, extra=""):
    out = f"est_{seed}_{os.getpid()}.csv"                 # per process: parallel callers must not share a file
    cmd = f"./replay.exe run_{seed}.csv {out} still=600 moff=0.05,-0.03,0.02 {flags} {extra}"
    log = run_replay(cmd.split())
    est = np.loadtxt(out, delimiter=",", skiprows=1); os.remove(out)
    gt = np.loadtxt(f"gt_{seed}.csv", delimiter=",", skiprows=1)
    n = min(len(est), len(gt) - 1)
    e = np.hypot(est[:n,1] - gt[1:n+1,1], est[:n,2] - gt[1:n+1,2])
    dist = np.concatenate([[0], np.cumsum(np.hypot(np.diff(gt[:n+1,1]), np.diff(gt[:n+1,2])))])[1:]
    return est[:n], gt[1:n+1], e, dist, log

def summary(seed, flags, extra=""):
    est, gt, e, dist, log = run(seed, flags, extra)
    total = dist[-1]
    ret = np.hypot(est[-1,1] - est[0,1], est[-1,2] - est[0,2])        # start == end for the loop
    return dict(final=e[-1], pct=100*e[-1]/total, max=e.max(), rmse=np.sqrt(np.mean(e**2)), ret=ret, dist=total, log=log)

if __name__ == "__main__":
    seeds = [int(s) for s in sys.argv[1:]] or [0, 1, 2]
    print(f"seeds {seeds}")
    print(f"{'config':32s} {'final err m':>12s} {'% of dist':>10s} {'max err m':>10s} {'return-to-start m':>18s}")
    for name, flags in CONFIGS:
        r = [summary(s, flags) for s in seeds]
        m = lambda k: np.mean([x[k] for x in r])
        print(f"{name:32s} {m('final'):12.2f} {m('pct'):10.2f} {m('max'):10.2f} {m('ret'):18.2f}")
    print("path length m:", round(np.mean([summary(s, CONFIGS[-1][1])['dist'] for s in seeds]), 1))
    print("gating log (last seed, full config):", summary(seeds[-1], CONFIGS[-1][1])["log"])
