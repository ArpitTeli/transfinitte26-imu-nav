"""Validate the batch ZUPT gyro-bias update on the TRAINING sessions and the sim only (set 1 / s0115 is not touched).
Variants are replay.exe arguments; zb_mode=0 is the old per-sample update."""
import json, os, numpy as np
from concurrent.futures import ProcessPoolExecutor
import sim
from multi import run, stats, run_replay
from validate_consider import jump
VAR = {"per-sample (old)": "zb_mode=0",
       "per-sample + lag0.5": "zb_mode=2 zb_lag=0.5", "per-sample + lag0.25": "zb_mode=2 zb_lag=0.25",
       "batch min0.5 lag0.25": "zb_mode=1 zb_min=0.5 zb_floor=3e-4 zb_lag=0.25",
       "batch min1 floor3e-4": "zb_mode=1 zb_min=1 zb_floor=3e-4", "batch min2 floor3e-4": "zb_mode=1 zb_min=2 zb_floor=3e-4",
       "batch min5 floor1e-3": "zb_mode=1 zb_min=5 zb_floor=1e-3"}
FROZEN = json.load(open("frozen_settings.json"))["args"]
FORCE_ZUPT = "zupt_s=2 zupt_w=0.015"
CFG = {"wheel+zupt": "enc_v=1 enc_w=0 mag=0 zupt=1", "full": "enc_v=1 enc_w=0 mag=1 zupt=1"}
SIMCFG = {"+ ZUPT": "enc_v=1 enc_w=1 mag=0 zupt=1", "full": "enc_v=1 enc_w=1 mag=1 zupt=1",
          "wheel+zupt (no enc_w, no mag)": "enc_v=1 enc_w=0 mag=0 zupt=1"}
SIM_SETS = {"sim default drift (seeds 0-5)": list(range(6)), "sim 5x gyro-bias drift (seeds 300-311)": list(range(300, 312))}

def nclt_job(a):
    pre, cfg, forced, var = a
    args = f"{FROZEN} {CFG[cfg]} {FORCE_ZUPT if forced else ''} {VAR[var]}"
    s = stats(run(pre, args))
    j, nz = jump(f"{pre}_run.csv", open(f"{pre}_params.txt").read().split(), args, "./replay.exe")
    return pre, cfg, forced, var, s, nz

def sim_job(a):
    seed, cfg, var = a
    out = f"zb_{os.getpid()}.csv"
    run_replay(["./replay.exe", f"run_{seed}.csv", out, "still=600", "moff=0.05,-0.03,0.02"] + SIMCFG[cfg].split() + VAR[var].split())
    est = np.loadtxt(out, delimiter=",", skiprows=1); os.remove(out)
    gt = np.loadtxt(f"gt_{seed}.csv", delimiter=",", skiprows=1)
    n = min(len(est), len(gt) - 1); e = np.hypot(est[:n, 1] - gt[1:n + 1, 1], est[:n, 2] - gt[1:n + 1, 2])
    return seed, cfg, var, float(e[-1]), float(e.max()), float(np.sqrt(np.mean(e ** 2)))

if __name__ == "__main__":
    for s in SIM_SETS["sim 5x gyro-bias drift (seeds 300-311)"]:
        if not os.path.exists(f"run_{s}.csv"): sim.make(s, ".", bias_rw=1e-4)
    jobs = [(p, c, f, v) for p in ("s0108", "s0204") for c in CFG for f in (False, True) for v in VAR]
    sjobs = [(s, c, v) for ss in SIM_SETS.values() for s in ss for c in SIMCFG for v in VAR]
    with ProcessPoolExecutor(14) as pool:
        R = list(pool.map(nclt_job, jobs)); SR = list(pool.map(sim_job, sjobs))
    json.dump(dict(nclt=[dict(session=p, cfg=c, forced=f, var=v, zupt_n=nz, **s) for p, c, f, v, s, nz in R],
                   sim=[dict(seed=s, cfg=c, var=v, final=fi, max=mx, rmse=r) for s, c, v, fi, mx, r in SR]),
              open("validate_zb.json", "w"), indent=1)
    for forced in (False, True):
        print(f"\n=== NCLT training, {'ZUPT FORCED (' + FORCE_ZUPT + ')' if forced else 'frozen settings'}: RMSE / max / final (m) ===")
        print(f"{'variant':24s} " + " ".join(f"{p + ' ' + c:>26s}" for p in ("s0108", "s0204") for c in CFG))
        for v in VAR:
            cells = []
            for p in ("s0108", "s0204"):
                for c in CFG:
                    d = next(x for p2, c2, f2, v2, x, nz in R if (p2, c2, f2, v2) == (p, c, forced, v))
                    cells.append(f"{d['rmse']:7.1f} /{d['max']:7.1f} /{d['final']:7.1f}")
            print(f"{v:24s} " + " ".join(f"{x:>26s}" for x in cells))
    for name, seeds in SIM_SETS.items():
        print(f"\n=== {name}: mean RMSE / max / final (m) ===")
        print(f"{'variant':24s} " + " ".join(f"{c:>32s}" for c in SIMCFG))
        for v in VAR:
            cells = []
            for c in SIMCFG:
                r = [x for x in SR if x[0] in seeds and x[1] == c and x[2] == v]
                cells.append(f"{np.mean([x[5] for x in r]):7.3f} /{np.mean([x[4] for x in r]):7.3f} /{np.mean([x[3] for x in r]):7.3f}")
            print(f"{v:24s} " + " ".join(f"{x:>32s}" for x in cells))
    # summary: RMSE ratio vs the old per-sample update, per case; cases where ZUPT never fires are identical and left out
    old = "per-sample (old)"; gm = lambda a: float(np.exp(np.mean(np.log(a))))
    print("\n=== RMSE ratio vs old (geometric mean; <1 = better) ===")
    print(f"{'variant':24s} {'real forced (4)':>16s} {'sim default (3)':>16s} {'sim 5x drift (3)':>17s} {'all (10)':>9s} {'worst case':>11s}")
    for v in VAR:
        real = [x["rmse"] / next(y["rmse"] for p2, c2, f2, v2, y, nz in R if (p2, c2, f2, v2) == (p, c, True, old))
                for p, c, f, vv, x, nz in R if f and vv == v]
        grp = []
        for seeds in SIM_SETS.values():
            g = []
            for c in SIMCFG:
                a = np.mean([x[5] for x in SR if x[0] in seeds and x[1] == c and x[2] == v])
                o = np.mean([x[5] for x in SR if x[0] in seeds and x[1] == c and x[2] == old])
                g.append(a / o)
            grp.append(g)
        allr = real + grp[0] + grp[1]
        print(f"{v:24s} {gm(real):16.2f} {gm(grp[0]):16.2f} {gm(grp[1]):17.2f} {gm(allr):9.2f} {max(allr):11.2f}")
