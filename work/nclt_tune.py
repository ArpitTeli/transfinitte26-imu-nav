import subprocess, numpy as np, sys
params = open("nclt_params.txt").read().split()
gt = np.loadtxt("nclt_gt.csv", delimiter=",", skiprows=1)
def fit(P, Q):
    best = None
    for mir in (1, -1):
        Pm = P*np.array([1, mir]); mp, mq = Pm.mean(0), Q.mean(0)
        U, _, Vt = np.linalg.svd((Pm-mp).T@(Q-mq)); d = np.sign(np.linalg.det(Vt.T@U.T)); R = Vt.T@np.diag([1, d])@U.T
        M = R@np.diag([1, mir]); t = mq-R@mp; e = np.sqrt(np.mean(np.sum((P@M.T+t-Q)**2, 1)))
        if best is None or e < best[0]: best = (e, M, t)
    return best[1], best[2]
def run(extra, name="e_tune.csv"):
    subprocess.run(["./replay.exe", "nclt_run.csv", name] + params + extra.split(), capture_output=True)
    est = np.loadtxt(name, delimiter=",", skiprows=1); t = est[:, 0]; ok = (t >= gt[0,0]) & (t <= gt[-1,0])
    t, E = t[ok], est[ok, 1:3]
    G = np.column_stack([np.interp(t, gt[:,0], gt[:,1]), np.interp(t, gt[:,0], gt[:,2])])
    w = (t >= 20) & (t <= 80); M, tr = fit(E[w], G[w]); e = np.hypot(*((E@M.T+tr)-G).T)
    return t, e
def score(extra, tmax=900):
    t, e = run(extra); m = t <= tmax
    return np.sqrt(np.mean(e[m]**2)), e[m][-1], np.sqrt(np.mean(e**2)), e[-1]
if __name__ == "__main__":
    base = "enc_v=1 enc_w=1 mag=1 zupt=1 zupt_v=0.03"
    for label, ex in [("base", base), ("vscale .93", base+" vscale=0.93"), ("vscale .93 r_enc_w .2", base+" vscale=0.93 r_enc_w=0.2"),
                      ("vscale .93 enc_w off", "enc_v=1 enc_w=0 mag=1 zupt=1 zupt_v=0.03 vscale=0.93"), ("... mag off", "enc_v=1 enc_w=1 mag=0 zupt=1 zupt_v=0.03 vscale=0.93")]:
        s = score(ex); print(f"{label:26s} tune(<15min): rmse {s[0]:7.1f} final {s[1]:7.1f} | whole run: rmse {s[2]:7.1f} final {s[3]:7.1f}")
