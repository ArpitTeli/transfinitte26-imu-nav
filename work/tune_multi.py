"""Parallel coordinate descent. Objective = mean over TUNE sessions of RMSE on t <= 2400 s (first 40 min).
usage: python tune_multi.py out.json sessA[,sessB] [report sessions...]
Everything after 40 min, and any session not in the tune list, is held-out: shown, never used to choose."""
import sys, json, numpy as np
from concurrent.futures import ProcessPoolExecutor
from multi import run, stats
FIXED = "enc_v=1 mag=1 zupt=1 zupt_v=0.03 zupt_s=8.4 zupt_w=0.005"
SPACE = {"vscale": [0.93, 0.94, 0.95, 0.96, 0.97, 0.98, 1.0], "r_mag": [1.5, 3.0, 5.0, 7.0, 10.0, 15.0, 25.0],
         "mag_tol": [0.15, 0.2, 0.3, 0.4, 0.5], "dip_tol": [0.02, 0.03, 0.05, 0.07, 0.1, 0.15, 0.0], "step_tol": [0.3, 0.45, 0.6, 0.9, 0.0],
         "gate": [6, 8, 10, 15, 25, 50], "sig_bg": [3e-6, 1e-5, 3e-5, 1e-4, 3e-4], "r_enc_v": [0.03, 0.06, 0.1, 0.2],
         "recover": [0.5, 1, 2, 3, 5], "beta": [0.001, 0.002, 0.005, 0.01, 0.02], "enc_w": [0, 1], "r_enc_w": [0.05, 0.2, 0.5, 1.0],
         "mag_ref_s": [120, 200, 250, 300, 360, 450]}
START = {"vscale": 0.95, "r_mag": 0.6, "mag_tol": 0.3, "dip_tol": 0.1, "step_tol": 0.6, "gate": 6, "sig_bg": 1e-4,
         "r_enc_v": 0.03, "recover": 3, "beta": 0.02, "enc_w": 0, "r_enc_w": 0.05, "mag_ref_s": 300}
args = lambda o: FIXED + " " + " ".join(f"{k}={v}" for k, v in o.items())
def job(a):
    pre, o = a; return pre, stats(run(pre, args(o)))
def evaluate(pool, opts, sess):
    jobs = [(p, o) for o in opts for p in sess]
    res = list(pool.map(job, jobs)); out = []
    for i in range(len(opts)):
        out.append({p: s for p, s in res[i * len(sess):(i + 1) * len(sess)]})
    return out
if __name__ == "__main__":
    if len(sys.argv) < 3: sys.exit(__doc__)          # no arguments: show the usage, not a traceback
    outf, tune = sys.argv[1], sys.argv[2].split(",")
    report = [a for a in sys.argv[3:] if not a.startswith("start=")] or tune
    for a in sys.argv[3:]:
        if a.startswith("start="): START.update(json.load(open(a[6:]))["opt"])
    obj = lambda r: float(np.mean([r[p]["tune"] for p in tune]))
    with ProcessPoolExecutor(14) as pool:
        opt = dict(START); cur = evaluate(pool, [opt], tune)[0]; best = obj(cur)
        print(f"start: objective {best:.2f}", flush=True)
        for p in range(3):
            changed = False
            for k, vals in SPACE.items():
                cand = [dict(opt, **{k: v}) for v in vals if v != opt[k]]
                for o, r in zip(cand, evaluate(pool, cand, tune)):
                    if obj(r) < best - 0.3: best, opt, changed = obj(r), o, True
                print(f"pass{p} {k:8s} -> {opt[k]!s:7s} objective {best:.2f}", flush=True)
            if not changed: break
        final = evaluate(pool, [opt], sorted(set(report) | set(tune)))[0]
    print("FINAL", json.dumps(opt))
    for p, s in final.items():
        tag = "tuned" if p in tune else "NOT tuned on"
        print(f"  {p} ({tag}): first40 {s['tune']:.1f}  after40 {s['held']:.1f}  whole {s['rmse']:.1f}  final {s['final']:.1f} m ({s['pct']:.2f}%)")
    json.dump(dict(opt=opt, args=args(opt), result=final), open(outf, "w"), indent=1)
