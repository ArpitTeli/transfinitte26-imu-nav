"""Coordinate descent on the first 2400 s only. Held-out = everything after (reported, never used to choose)."""
import numpy as np
from nclt_tune import run
TMAX = 2400
def score(opt):
    ex = "enc_v=1 enc_w=0 mag=1 zupt=1 zupt_v=0.03 zupt_n=200 " + " ".join(f"{k}={v}" for k, v in opt.items())
    t, e = run(ex); m = t <= TMAX
    return np.sqrt(np.mean(e[m]**2)), np.sqrt(np.mean(e[~m]**2)), e[-1]
space = {"vscale": [1.0, 0.97, 0.95, 0.93, 0.9], "r_mag": [0.1, 0.2, 0.3, 0.6, 1.0], "mag_tol": [0.15, 0.3, 0.5, 1.0],
         "dip_tol": [0.03, 0.06, 0.1, 0.2, 0.0], "step_tol": [0.07, 0.15, 0.3, 0.6, 0.0], "gate": [3, 4, 6]}
opt = {"vscale": 0.93, "r_mag": 0.3, "mag_tol": 0.3, "dip_tol": 0.1, "step_tol": 0.3, "gate": 3}
best = score(opt); print("start", opt, "tune rmse %.1f | held-out rmse %.1f final %.1f" % best, flush=True)
for p in range(2):
    for k, vals in space.items():
        for v in vals:
            if v == opt[k]: continue
            o = dict(opt); o[k] = v; s = score(o)
            if s[0] < best[0] - 0.5: opt, best = o, s; print(f"pass{p} {k}={v}: tune rmse {s[0]:.1f}  (held-out rmse {s[1]:.1f} final {s[2]:.1f})", flush=True)
print("FINAL", opt, "tune rmse %.1f | held-out rmse %.1f final %.1f" % best)
