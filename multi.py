"""Multi-session evaluation for NCLT runs converted by nclt_convert.py (prefix_run.csv, _gt.csv, _params.txt, _meta.txt).
Alignment: rigid (+mirror) fit on the first 60 s after the robot starts moving; applied to the whole run, so drift is not hidden.
"""
import subprocess, numpy as np, os
_cache = {}


def run_replay(args):
    """run replay.exe; raise with its own message if it fails (e.g. an unknown option), so a stale or missing
    output file is never read as a result. Returns replay's stderr (the accepted/rejected update counts)."""
    p = subprocess.run([str(a) for a in args], capture_output=True, text=True)
    if p.returncode != 0:
        raise RuntimeError(f"replay failed ({p.returncode}): {p.stderr.strip()}")
    return p.stderr.strip()


def _read(path):
    with open(path) as f:
        return f.read()


def session(pre):
    if pre not in _cache:
        _cache[pre] = (_read(f"{pre}_params.txt").split(),
                       np.loadtxt(f"{pre}_gt.csv", delimiter=",", skiprows=1),
                       float(_read(f"{pre}_meta.txt")))
    return _cache[pre]
def fit(P, Q):
    best = None
    for mir in (1, -1):
        Pm = P * np.array([1, mir]); mp, mq = Pm.mean(0), Q.mean(0)
        U, _, Vt = np.linalg.svd((Pm - mp).T @ (Q - mq)); d = np.sign(np.linalg.det(Vt.T @ U.T))
        R = Vt.T @ np.diag([1, d]) @ U.T; M = R @ np.diag([1, mir]); t = mq - R @ mp
        e = np.sqrt(np.mean(np.sum((P @ M.T + t - Q) ** 2, 1)))
        if best is None or e < best[0]: best = (e, M, t)
    return best[1], best[2]
def run(pre, extra, exe="./replay.exe"):
    params, gt, tm = session(pre)
    out = f"est_{pre}_{os.getpid()}.csv"
    log = run_replay([exe, f"{pre}_run.csv", out] + params + extra.split())
    est = np.loadtxt(out, delimiter=",", skiprows=1); os.remove(out)
    t = est[:, 0]; ok = (t >= gt[0, 0]) & (t <= gt[-1, 0]); t, E = t[ok], est[ok, 1:3]
    G = np.column_stack([np.interp(t, gt[:, 0], gt[:, 1]), np.interp(t, gt[:, 0], gt[:, 2])])
    w = (t >= tm) & (t <= tm + 60); M, tr = fit(E[w], G[w]); Ea = E @ M.T + tr
    e = np.hypot(*(Ea - G).T)
    dist = np.concatenate([[0], np.cumsum(np.hypot(*np.diff(G, axis=0).T))])
    return dict(t=t, e=e, Ea=Ea, G=G, dist=dist, log=log)
def stats(r, tsplit=2400):
    t, e = r["t"], r["e"]; m = t <= tsplit
    q = lambda x: float(np.sqrt(np.mean(x ** 2))) if len(x) else float("nan")
    return dict(rmse=q(e), tune=q(e[m]), held=q(e[~m]), final=float(e[-1]), pct=100 * e[-1] / r["dist"][-1], max=float(e.max()))
