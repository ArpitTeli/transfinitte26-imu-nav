"""Ablation on several NCLT sessions with one settings file.
usage: python eval4.py settings.json [sessions...]   (default: s0108 s0204 s0115 s0122)"""
import sys, json, numpy as np
from concurrent.futures import ProcessPoolExecutor
from multi import run, stats
CFG = [("IMU only", "enc_v=0 enc_w=0 mag=0 zupt=0"), ("+ wheel speed", "enc_v=1 enc_w=0 mag=0 zupt=0"),
       ("+ ZUPT", "enc_v=1 enc_w=0 mag=0 zupt=1"), ("+ magnetometer (full)", "enc_v=1 enc_w=0 mag=1 zupt=1")]
def job(a):
    pre, name, args = a; r = run(pre, args); s = stats(r); s["log"] = r["log"]; s["km"] = r["dist"][-1] / 1000
    return pre, name, s
if __name__ == "__main__":
    if len(sys.argv) < 2: sys.exit(__doc__)          # no arguments: show the usage, not a traceback
    base = json.load(open(sys.argv[1]))["args"]
    sess = sys.argv[2:] or ["s0108", "s0204", "s0115", "s0122"]
    with ProcessPoolExecutor(14) as p:
        res = list(p.map(job, [(s, n, f"{base} {c}") for s in sess for n, c in CFG]))
    out = {}
    for s in sess:
        rows = [(n, x) for p2, n, x in res if p2 == s]
        print(f"\n{s}  ({rows[0][1]['km']:.2f} km)")
        print(f"  {'config':24s} {'RMSE':>8s} {'max':>8s} {'final':>8s} {'%dist':>6s}")
        for n, x in rows:
            print(f"  {n:24s} {x['rmse']:8.1f} {x['max']:8.1f} {x['final']:8.1f} {x['pct']:6.2f}")
            out.setdefault(s, {})[n] = x
        print("  gating (full):", rows[-1][1]["log"])
    full = [out[s]["+ magnetometer (full)"]["rmse"] for s in sess]
    print(f"\nfull filter RMSE per session: {np.round(full, 1)}  mean {np.mean(full):.1f}  geometric mean {np.exp(np.mean(np.log(full))):.1f}")
    json.dump(out, open(sys.argv[1].replace(".json", "_eval4.json"), "w"), indent=1)
