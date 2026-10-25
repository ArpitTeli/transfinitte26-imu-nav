"""Validate the position-consider fix (pos_consider) on the TRAINING sessions and the sim only. Set 1 (s0115) is not touched.
Compares replay_pre.exe (before the fix) with replay.exe (after)."""
import json, os, numpy as np
from concurrent.futures import ProcessPoolExecutor
from multi import run, stats, fit, run_replay
EXES = {"before": "./replay_pre.exe", "zupt": "./replay.exe pos_consider=8", "zupt+v": "./replay.exe pos_consider=9",
        "zupt+v+w": "./replay.exe pos_consider=11", "all": "./replay.exe pos_consider=15"}
FROZEN = json.load(open("frozen_settings.json"))["args"]
FORCE_ZUPT = "zupt_s=2 zupt_w=0.015"            # looser detector so ZUPT actually fires on the training sessions
CFG = {"wheel": "enc_v=1 enc_w=0 mag=0 zupt=0", "wheel+zupt": "enc_v=1 enc_w=0 mag=0 zupt=1",
       "full": "enc_v=1 enc_w=0 mag=1 zupt=1"}

def jump(csv_in, params, args, exe):
    """largest single-step position change beyond what the estimated speed explains (m), plus ZUPT sample count"""
    out = f"j_{os.getpid()}.csv"
    run_replay([exe, csv_in, out] + params + args.split())
    e = np.loadtxt(out, delimiter=",", skiprows=1); os.remove(out)
    dt = np.diff(e[:, 0]); step = np.hypot(np.diff(e[:, 1]), np.diff(e[:, 2]))
    excess = step - np.abs(e[1:, 4]) * dt
    return float(excess.max()), int((e[:, 7] > 0).sum())

def nclt_job(a):
    pre, cfg, extra, tag = a
    args = f"{FROZEN} {CFG[cfg]} {extra}"
    exe, *xa = EXES[tag].split(); args += " " + " ".join(xa)
    s = stats(run(pre, args, exe=exe))
    j, nz = jump(f"{pre}_run.csv", open(f"{pre}_params.txt").read().split(), args, exe)
    return pre, cfg, extra, tag, s, j, nz

SIM = [("IMU only", "enc_v=0 enc_w=0 mag=0 zupt=0"), ("+ wheel speed", "enc_v=1 enc_w=0 mag=0 zupt=0"),
       ("+ wheel yaw-rate", "enc_v=1 enc_w=1 mag=0 zupt=0"), ("+ ZUPT", "enc_v=1 enc_w=1 mag=0 zupt=1"),
       ("+ magnetometer (full)", "enc_v=1 enc_w=1 mag=1 zupt=1")]
def sim_job(a):
    seed, name, flags, tag = a
    out = f"s_{os.getpid()}.csv"; base = "still=600 moff=0.05,-0.03,0.02"
    exe, *xa = EXES[tag].split(); flags += " " + " ".join(xa)
    run_replay([exe, f"run_{seed}.csv", out] + base.split() + flags.split())
    est = np.loadtxt(out, delimiter=",", skiprows=1); os.remove(out)
    gt = np.loadtxt(f"gt_{seed}.csv", delimiter=",", skiprows=1)
    n = min(len(est), len(gt) - 1); e = np.hypot(est[:n, 1] - gt[1:n + 1, 1], est[:n, 2] - gt[1:n + 1, 2])
    j, _ = jump(f"run_{seed}.csv", base.split(), flags, exe)
    return seed, name, tag, float(e[-1]), float(e.max()), float(np.sqrt(np.mean(e ** 2))), j

if __name__ == "__main__":
    sess = ["s0108", "s0204"]
    jobs = [(p, c, x, t) for p in sess for c in CFG for x in ("", FORCE_ZUPT) for t in EXES]
    sjobs = [(s, n, f, t) for s in range(6) for n, f in SIM for t in EXES]
    with ProcessPoolExecutor(14) as pool:
        R = list(pool.map(nclt_job, jobs)); SR = list(pool.map(sim_job, sjobs))
    out = {"nclt": [dict(session=p, cfg=c, forced_zupt=bool(x), build=t, **s, jump=j, zupt_samples=nz) for p, c, x, t, s, j, nz in R],
           "sim": [dict(seed=s, cfg=n, build=t, final=f, max=m, rmse=r, jump=j) for s, n, t, f, m, r, j in SR]}
    json.dump(out, open("validate_consider.json", "w"), indent=1)
    for forced in (False, True):
        print(f"\n=== NCLT training sessions, {'ZUPT FORCED to fire (' + FORCE_ZUPT + ')' if forced else 'frozen settings'} ===")
        print(f"{'session':7s} {'config':11s} {'build':9s} {'ZUPT n':>7s} {'max jump m':>11s} {'RMSE m':>8s} {'max m':>8s} {'final m':>8s}")
        for d in out["nclt"]:
            if d["forced_zupt"] != forced: continue
            print(f"{d['session']:7s} {d['cfg']:11s} {d['build']:9s} {d['zupt_samples']:7d} {d['jump']:11.2f} {d['rmse']:8.1f} {d['max']:8.1f} {d['final']:8.1f}")
    print("\n=== Sim, seeds 0-5 (mean) ===")
    print(f"{'config':24s} {'build':9s} {'final m':>8s} {'max m':>7s} {'RMSE m':>7s} {'max jump m':>11s}")
    for n, _ in SIM:
        for t in EXES:
            r = [d for d in out["sim"] if d["cfg"] == n and d["build"] == t]
            m = lambda k: np.mean([d[k] for d in r])
            print(f"{n:24s} {t:9s} {m('final'):8.3f} {m('max'):7.3f} {m('rmse'):7.3f} {max(d['jump'] for d in r):11.3f}")
