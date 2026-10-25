"""Sweep one parameter on several sessions in parallel.
usage: python sweep.py base.json key v1,v2,... [sessions=s0108,s0204] [extra replay args...]"""
import sys, json
from concurrent.futures import ProcessPoolExecutor
from multi import run, stats
def job(a):
    pre, base, k, v = a; return pre, v, stats(run(pre, f"{base} {k}={v}"))
if __name__ == "__main__":
    base = json.load(open(sys.argv[1]))["args"].replace("zupt_n=400", "zupt_s=8.4")
    k, vals = sys.argv[2], sys.argv[3].split(",")
    sess = ["s0108", "s0204"]; extra = []
    for a in sys.argv[4:]:
        if a.startswith("sessions="): sess = a[9:].split(",")
        else: extra.append(a)
    base += " " + " ".join(extra)
    with ProcessPoolExecutor(14) as p:
        res = list(p.map(job, [(pre, base, k, v) for v in vals for pre in sess]))
    for v in vals:
        rr = {pre: s for pre, vv, s in res if vv == v}
        mean = sum(s["tune"] for s in rr.values()) / len(rr)
        print(f"{k}={v:>7s} | " + " | ".join(f"{pre} first40 {s['tune']:5.1f} after40 {s['held']:5.1f} final {s['final']:5.1f}" for pre, s in rr.items()) + f" | mean first40 {mean:5.1f}")
