"""Processing engine for Fusion Player: NCLT sensor archive + ground truth -> filter runs -> 10 Hz playback timeline.

Inputs per dataset (exactly what the NCLT site gives you):
  * <date>_sen.tar.gz   sensor archive; only ms25.csv (IMU + magnetometer) and wheels.csv are read
  * groundtruth_<date>.csv
The same C filter as the firmware (filter/replay.exe) runs once per condition. Results are cached in cache/.
"""
import os, io, sys, json, hashlib, tarfile, subprocess, tempfile
from concurrent.futures import ThreadPoolExecutor
import numpy as np, pandas as pd

FROZEN = getattr(sys, "frozen", False)                       # running as the installed Fusion Player.exe
HERE = getattr(sys, "_MEIPASS", os.path.dirname(os.path.abspath(__file__)))   # filter/ and settings/ live here
REPLAY = os.path.join(HERE, "filter", "replay.exe")
# the installed app may sit where it cannot write, so its cache goes to the user's local app data
CACHE = (os.path.join(os.environ.get("LOCALAPPDATA", os.path.expanduser("~")), "Fusion Player", "cache") if FROZEN
         else os.path.join(HERE, "cache"))
FS = 10.0                      # playback timeline rate (Hz)
G = 9.81
ENGINE_VERSION = 2             # bump when processing changes, so cached results are recomputed


def _log(cb, msg):
    if cb: cb(msg)


# ---------------------------------------------------------------- reading
def read_sensors(tar_path, cb=None):
    _log(cb, "Reading sensor archive")
    try:
        tf = tarfile.open(tar_path, "r:*")
    except tarfile.TarError:
        raise ValueError(f"{os.path.basename(tar_path)} is not a tar archive. For the sensor archive, pick the NCLT "
                         "<date>_sen.tar.gz file.")
    want = {"ms25.csv": "IMU (ms25.csv)", "wheels.csv": "wheel speeds (wheels.csv)"}
    got = {}
    with tf:
        for m in tf:                      # stream: stop once both files are read (the archive holds ~300 MB more)
            name = os.path.basename(m.name)
            if m.isfile() and name in want and name not in got:
                _log(cb, f"Reading {want[name]}")
                got[name] = pd.read_csv(io.BytesIO(tf.extractfile(m).read()), header=None).to_numpy(float)
                if len(got) == len(want):
                    break
    missing = [n for n in want if n not in got]
    if missing:
        raise ValueError(f"{os.path.basename(tar_path)} has no {', '.join(missing)}. Pick the NCLT sen.tar.gz archive.")
    return got["ms25.csv"], got["wheels.csv"]


def read_groundtruth(path, cb=None):
    _log(cb, "Reading ground truth")
    gt = pd.read_csv(path, header=None, skipinitialspace=True).to_numpy(float)
    gt = gt[np.isfinite(gt).all(1)]
    if gt.shape[1] < 7 or len(gt) < 100:
        raise ValueError(f"{os.path.basename(path)} does not look like an NCLT ground-truth file (utime,x,y,z,roll,pitch,yaw).")
    if np.ptp(gt[:, 0]) < 1e6:
        raise ValueError(f"The timestamps in {os.path.basename(path)} are damaged (all nearly equal). This happens when the "
                         "file is opened and saved in Excel. Use the original download.")
    return gt


# ---------------------------------------------------------------- conversion + self-calibration (sensor data only)
def convert(ms, wh, gt, cb=None):
    _log(cb, "Converting to filter format and self-calibrating")
    t0 = ms[0, 0]
    t = (ms[:, 0] - t0) / 1e6
    flip = np.array([1, -1, -1])                      # MS25: z down -> body x fwd, y left, z up
    m, a, g = ms[:, 1:4] * flip, ms[:, 4:7] * flip, ms[:, 7:10] * flip
    tw = (wh[:, 0] - t0) / 1e6
    vL, vR = np.interp(t, tw, wh[:, 1]), np.interp(t, tw, wh[:, 2])
    k = max(1, int(1.0 / np.median(np.diff(t))))
    vs = np.convolve(np.abs(vL) + np.abs(vR), np.ones(k) / k, mode="same")
    t_move = float(t[np.argmax(vs > 0.2)])
    still = t < max(2.0, t_move - 2.0)
    wz = g[:, 2] - g[still, 2].mean(); dv = vR - vL; mv = np.abs(vL) + np.abs(vR) > 0.6
    den = np.sum(dv[mv] * wz[mv])
    track = float(np.sum(dv[mv] ** 2) / den) if mv.sum() > 100 and den != 0 else float("nan")   # (vR-vL)/b = gyro yaw rate
    if not 0.1 < track < 3.0:             # robot never drove, or a wild fit: fall back to the Segway's measured width
        _log(cb, f"Track width fit failed ({track:.3f} m); using 0.59 m")
        track = 0.59
    A = np.column_stack([2 * m, np.ones(len(m))])
    moff = np.linalg.lstsq(A, (m ** 2).sum(1), rcond=None)[0][:3]            # hard-iron sphere fit
    run = np.column_stack([t, a, g, m, vL, vR])
    G_ = np.column_stack([(gt[:, 0] - t0) / 1e6, gt[:, 1], gt[:, 2], np.unwrap(gt[:, 6])])
    calib = dict(still_rows=int(still.sum()), still_s=float(t[still][-1]), t_move=t_move, track=track, moff=moff.tolist())
    params = [f"still={calib['still_rows']}", f"track={track:.4f}", f"moff={moff[0]:.5f},{moff[1]:.5f},{moff[2]:.5f}"]
    return run, G_, calib, params


# ---------------------------------------------------------------- filter runs
def run_filter(run_csv, params, args):
    fd, out = tempfile.mkstemp(suffix=".csv", dir=CACHE); os.close(fd)
    try:
        p = subprocess.run([REPLAY, run_csv, out] + params + args.split(), capture_output=True, text=True,
                           creationflags=getattr(subprocess, "CREATE_NO_WINDOW", 0))
        if p.returncode != 0: raise RuntimeError(f"filter failed: {p.stderr.strip()}")
        est = pd.read_csv(out).to_numpy(float)          # t,x,y,yaw,v,bg,ba,stationary
    finally:
        os.remove(out)
    return est, p.stderr.strip()


def fit_rigid(P, Q):
    """best rotation (+optional mirror) and translation P->Q; returns M (2x2), t, mirror"""
    best = None
    for mir in (1, -1):
        Pm = P * np.array([1, mir]); mp, mq = Pm.mean(0), Q.mean(0)
        U, _, Vt = np.linalg.svd((Pm - mp).T @ (Q - mq)); d = np.sign(np.linalg.det(Vt.T @ U.T))
        R = Vt.T @ np.diag([1, d]) @ U.T; M = R @ np.diag([1, mir]); tr = mq - R @ mp
        e = np.sqrt(np.mean(np.sum((P @ M.T + tr - Q) ** 2, 1)))
        if best is None or e < best[0]: best = (e, M, tr, mir)
    return best[1], best[2], best[3]


def start_pose_map(E0, psi0, Gp0, h0, mirror):
    """map est -> GT using only the true pose at one instant"""
    if mirror < 0:
        th = h0 + psi0; M = np.array([[np.cos(th), np.sin(th)], [np.sin(th), -np.cos(th)]])
    else:
        th = h0 - psi0; M = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    return M, Gp0 - M @ E0


# ---------------------------------------------------------------- the whole pipeline
def settings_list():
    """settings profiles, the one marked "default": true first"""
    d = os.path.join(HERE, "settings")
    files = [os.path.join(d, f) for f in sorted(os.listdir(d)) if f.endswith(".json")]

    def is_default(f):
        with open(f) as fh:
            return json.load(fh).get("default", False)
    return sorted(files, key=lambda f: not is_default(f))


def cache_key(tar_path, gt_path, settings, align):
    h = hashlib.sha1()
    for p in (tar_path, gt_path):
        st = os.stat(p); h.update(f"{os.path.abspath(p)}|{st.st_size}|{int(st.st_mtime)}".encode())
    h.update(json.dumps({k: settings[k] for k in ("args", "conditions")}, sort_keys=True).encode()); h.update(align.encode())
    h.update(str(os.path.getmtime(REPLAY)).encode()); h.update(f"engine{ENGINE_VERSION}".encode())
    return h.hexdigest()[:16]


def process(tar_path, gt_path, settings_path, align="fit60", cb=None, use_cache=True):
    """align: 'fit60' = rigid fit on the first 60 s of motion; 'start' = true pose when motion starts"""
    os.makedirs(CACHE, exist_ok=True)
    with open(settings_path) as f:
        settings = json.load(f)
    key = cache_key(tar_path, gt_path, settings, align)
    cf = os.path.join(CACHE, key + ".npz")
    if use_cache and os.path.exists(cf):
        _log(cb, "Loading from cache")
        return load(cf)

    ms, wh = read_sensors(tar_path, cb)
    gtraw = read_groundtruth(gt_path, cb)
    run, GT, calib, params = convert(ms, wh, gtraw, cb)
    run_csv = os.path.join(CACHE, key + "_run.csv")
    _log(cb, "Writing filter input")
    np.savetxt(run_csv, run, delimiter=",", fmt="%.6f", header="t,ax,ay,az,gx,gy,gz,mx,my,mz,vL,vR", comments="")
    conds = settings["conditions"]
    _log(cb, f"Running the filter in {len(conds)} conditions")
    try:
        with ThreadPoolExecutor(len(conds)) as ex:
            outs = list(ex.map(lambda c: run_filter(run_csv, params, settings["args"] + " " + c["flags"]), conds))
    finally:
        os.remove(run_csv)                # also when a run fails (it is tens of MB)

    _log(cb, "Aligning to ground truth and building the timeline")
    t_lo = max(GT[0, 0], run[0, 0]); t_hi = min(GT[-1, 0], run[-1, 0])
    T = np.arange(np.ceil(t_lo * FS) / FS, t_hi, 1 / FS)
    gx, gy, gh = (np.interp(T, GT[:, 0], GT[:, i]) for i in (1, 2, 3))
    # GT speed from positions at native rate, 0.5 s smoothing
    dtg = np.diff(GT[:, 0]); sp = np.hypot(np.diff(GT[:, 1]), np.diff(GT[:, 2])) / np.where(dtg > 0, dtg, np.nan)
    sp = np.nan_to_num(sp); kk = max(1, int(0.5 / np.median(dtg)))
    sp = np.convolve(sp, np.ones(kk) / kk, mode="same"); gv = np.interp(T, GT[1:, 0], sp)
    dist = np.concatenate([[0], np.cumsum(np.hypot(np.diff(gx), np.diff(gy)))])
    t_move = calib["t_move"]
    out = dict(t=T, gt_x=gx, gt_y=gy, gt_h=gh, gt_v=gv, dist=dist)
    # raw sensors for the strip charts, bin-averaged onto the timeline
    idx = np.clip(np.searchsorted(T, run[:, 0]), 0, len(T) - 1)
    def binmean(x):
        s = np.bincount(idx, x, len(T)); n = np.bincount(idx, None, len(T)); return s / np.maximum(n, 1)
    out["s_gz"] = np.degrees(binmean(run[:, 6])); out["s_ax"] = binmean(run[:, 1])
    out["s_vl"] = binmean(run[:, 10]); out["s_vr"] = binmean(run[:, 11])
    stats, logs = [], []
    for ci, (c, (est, log)) in enumerate(zip(conds, outs)):
        te = est[:, 0]
        ex_, ey_ = np.interp(T, te, est[:, 1]), np.interp(T, te, est[:, 2])
        epsi = np.interp(T, te, np.unwrap(est[:, 3])); ev = np.interp(T, te, est[:, 4])
        E = np.column_stack([ex_, ey_])
        w = (T >= t_move) & (T <= t_move + 60)
        if w.sum() < 20:                  # ground truth does not cover the first minute of motion: use its first minute
            w = T <= T[0] + 60
        M, tr, mir = fit_rigid(E[w], np.column_stack([gx, gy])[w])
        if align == "start":
            i0 = np.searchsorted(T, t_move); M, tr = start_pose_map(E[i0], epsi[i0], np.array([gx[i0], gy[i0]]), gh[i0], mir)
        A = E @ M.T + tr
        hv = np.column_stack([np.cos(epsi), np.sin(epsi)]) @ M.T
        eh = np.unwrap(np.arctan2(hv[:, 1], hv[:, 0]))
        eh += 2 * np.pi * np.round((gh[0] - eh[0]) / (2 * np.pi))       # same branch as GT for plotting
        err = np.hypot(A[:, 0] - gx, A[:, 1] - gy)
        herr = np.degrees((eh - gh + np.pi) % (2 * np.pi) - np.pi)
        out[f"c{ci}_x"], out[f"c{ci}_y"], out[f"c{ci}_h"], out[f"c{ci}_v"] = A[:, 0], A[:, 1], eh, ev
        out[f"c{ci}_err"], out[f"c{ci}_herr"], out[f"c{ci}_zupt"] = err, herr, np.interp(T, te, est[:, 7])
        stats.append(dict(name=c["name"], rmse=float(np.sqrt(np.mean(err ** 2))), max=float(err.max()),
                          final=float(err[-1]), pct=float(100 * err[-1] / max(dist[-1], 1e-9))))
        logs.append(log)
    meta = dict(name=os.path.basename(gt_path).replace("groundtruth_", "").replace(".csv", ""),
                tar=os.path.abspath(tar_path), gt=os.path.abspath(gt_path), settings=settings, align=align,
                calib=calib, stats=stats, logs=logs, conditions=[c["name"] for c in conds],
                duration=float(T[-1] - T[0]), distance=float(dist[-1]))
    np.savez_compressed(cf, meta=json.dumps(meta), **out)
    return load(cf)


def load(cf):
    with np.load(cf, allow_pickle=False) as z:          # closed afterwards: an open .npz stays locked on Windows
        d = {k: z[k] for k in z.files if k != "meta"}
        d["meta"] = json.loads(str(z["meta"]))
    return d


if __name__ == "__main__":       # command-line check: python engine.py sen.tar.gz groundtruth.csv [settings.json]
    import sys
    s = sys.argv[3] if len(sys.argv) > 3 else settings_list()[0]
    d = process(sys.argv[1], sys.argv[2], s, cb=print, use_cache=False)
    m = d["meta"]
    print(f"{m['name']}: {m['distance']/1000:.2f} km, {m['duration']/60:.1f} min, {len(d['t'])} timeline samples")
    for st in m["stats"]: print(f"  {st['name']:24s} RMSE {st['rmse']:8.1f}  max {st['max']:8.1f}  final {st['final']:8.1f} m ({st['pct']:.2f} %)")
