"""Joint coordinate-descent tuning on ALL given sessions (whole runs; no within-session hold-out).
Objective: geometric mean over sessions of whole-run RMSE, so each session's relative improvement counts equally.
usage: python tune_all.py out.json s0108,s0204,s0115,s0122 [start=settings.json]"""
import sys, json, numpy as np
from concurrent.futures import ProcessPoolExecutor
from multi import run, stats
FIXED = "enc_v=1 mag=1 zupt=1 zupt_v=0.03"
SPACE = {"vscale": [0.94, 0.95, 0.96, 0.97, 0.98],
         "r_mag": [3.0, 5.0, 7.0, 10.0, 15.0], "mag_ref_s": [150, 200, 250, 300, 360],
         "mag_tol": [0.1, 0.15, 0.2, 0.3], "dip_tol": [0.03, 0.05, 0.07, 0.1, 0.0], "step_tol": [0.3, 0.45, 0.6, 0.9, 0.0],
         "gate": [5, 6, 8, 10, 15], "recover": [0.5, 1, 2, 3], "r_enc_v": [0.03, 0.06, 0.1],
         "sig_bg": [1e-5, 3e-5, 1e-4], "beta": [0.005, 0.01, 0.02],
         "enc_w": [0, 1], "r_enc_w": [0.05, 0.2, 0.5, 1.0],
         "zupt_w": [0.005, 0.008, 0.015], "zupt_s": [2, 4, 8.4], "zb_min": [1, 2, 5]}
args = lambda o: FIXED + " " + " ".join(f"{k}={v}" for k, v in o.items())
def job(a):
    pre, o = a; return pre, stats(run(pre, args(o)))
def evaluate(pool, opts, sess):
    res = list(pool.map(job, [(p, o) for o in opts for p in sess]))
    return [{p: s for p, s in res[i * len(sess):(i + 1) * len(sess)]} for i in range(len(opts))]
obj = lambda r: float(np.exp(np.mean(np.log([s["rmse"] for s in r.values()]))))
if __name__ == "__main__":
    outf, sess = sys.argv[1], sys.argv[2].split(",")
    start = json.load(open(sys.argv[3][6:]))["opt"] if len(sys.argv) > 3 else {}
    opt = {k: start.get(k, v[len(v) // 2]) for k, v in SPACE.items()}
    opt.update({"zupt_w": start.get("zupt_w", 0.005), "zupt_s": start.get("zupt_s", 8.4), "zb_min": start.get("zb_min", 1)})
    with ProcessPoolExecutor(14) as pool:
        cur = evaluate(pool, [opt], sess)[0]; best = obj(cur)
        print(f"start: objective {best:.2f}  " + " ".join(f"{p} {cur[p]['rmse']:.1f}" for p in sess), flush=True)
        for it in range(4):
            changed = False
            for k, vals in SPACE.items():
                cand = [dict(opt, **{k: v}) for v in vals if v != opt[k]]
                for o, r in zip(cand, evaluate(pool, cand, sess)):
                    if obj(r) < best * 0.995: best, opt, cur, changed = obj(r), o, r, True
                print(f"pass{it} {k:9s} -> {opt[k]!s:7s} objective {best:.2f}  " + " ".join(f"{p} {cur[p]['rmse']:.1f}" for p in sess), flush=True)
            if not changed: break
    print("FINAL", json.dumps(opt))
    for p, s in cur.items():
        print(f"  {p}: RMSE {s['rmse']:.1f}  max {s['max']:.1f}  final {s['final']:.1f} m ({s['pct']:.2f}%)")
    json.dump(dict(opt=opt, args=args(opt), result=cur), open(outf, "w"), indent=1)
