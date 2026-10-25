"""Recorded phone walks for Fusion Player: an IMU Stream (phone) or IMU Live (laptop) recording -> the foot filter ->
a playback timeline shaped like engine.process's NCLT timeline, but with no ground truth.

The filter is imu_live's imunav: the same code and shoe preset as live mode. Results are cached in cache/.
"""
import os, sys, json, hashlib
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
IMU_LIVE = os.path.join(HERE, "..", "imu_live")
if IMU_LIVE not in sys.path:
    sys.path.insert(0, IMU_LIVE)
from imunav.calib import static_calibration  # noqa: E402
from imunav.live.recording import load_recording  # noqa: E402
from imunav.live.session import ALIGN_M  # noqa: E402
from imunav.pipeline import PRESETS, navigator_from_static  # noqa: E402
from imunav.quat import yaw_of  # noqa: E402
import engine  # noqa: E402

FS = 20.0                 # playback timeline rate (Hz): a step lasts about 1 s
VERSION = 1               # bump when processing changes, so cached results are recomputed
SPEED_WIN = 2.0           # walking speed = foot speed averaged over this many seconds


def _log(cb, msg):
    if cb: cb(msg)


def cache_key(path, cfg):
    st = os.stat(path); h = hashlib.sha1()
    h.update(f"{os.path.abspath(path)}|{st.st_size}|{int(st.st_mtime)}|walk{VERSION}".encode())
    h.update(repr(sorted(vars(cfg).items())).encode())
    return "walk_" + h.hexdigest()[:16]


def find_still(t, G, A, gap, win):
    """first window of `win` seconds that looks static and has no data gap (where the live app would calibrate)"""
    i = 0
    while t[-1] - t[i] > win:
        j = int(np.searchsorted(t, t[i] + win))
        if not gap[i + 1:j].any() and static_calibration(t[i:j], G[i:j], A[i:j], win)["looks_static"]:
            return i, j
        i = int(np.searchsorted(t, t[i] + win / 4))
    raise ValueError("The recording never holds still for 2 s, so the filter cannot calibrate. A walk must start "
                     "with the shoe still.")


def process(path, cb=None, use_cache=True):
    cfg = PRESETS["shoe"]
    os.makedirs(engine.CACHE, exist_ok=True)
    cf = os.path.join(engine.CACHE, cache_key(path, cfg) + ".npz")
    if use_cache and os.path.exists(cf):
        _log(cb, "Loading from cache")
        return engine.load(cf)

    _log(cb, "Reading the recording")
    df, info = load_recording(path)
    t = df.t.to_numpy(); gap = df.gap.to_numpy()
    G = df[["gx", "gy", "gz"]].to_numpy(); A = df[["ax", "ay", "az"]].to_numpy()
    if len(t) < 1000:
        raise ValueError(f"{os.path.basename(path)} holds only {len(t)} samples.")
    i0, i1 = find_still(t, G, A, gap, cfg.init_window)
    cfg = cfg.with_(fs=1.0 / float(np.median(np.diff(t[i0:i1]))))
    nav, cal = navigator_from_static(t[i0:i1], G[i0:i1], A[i0:i1], cfg)

    n = len(t) - i1
    P = np.zeros((n, 2)); V = np.zeros(n); Y = np.zeros(n); S = np.zeros(n, bool)
    for m, j in enumerate(range(i1, len(t))):
        if gap[j] > 0:
            nav.gap(gap[j])
        else:
            nav.step(G[j], A[j], t[j] - t[j - 1])
        P[m], V[m], Y[m], S[m] = nav.p[:2], np.hypot(nav.v[0], nav.v[1]), yaw_of(nav.q), nav.still
        if m % 20000 == 0:
            _log(cb, f"Running the foot filter: {100 * m / n:.0f} %")

    _log(cb, "Building the timeline")
    ts = t[i1:]
    # shown like live mode: the start at the origin, turned so the first ALIGN_M metres point up the screen
    q = P - P[0]
    d0 = np.linalg.norm(q, axis=1)
    rot = 0.0
    if (d0 >= ALIGN_M).any():
        x3, y3 = q[int(np.argmax(d0 >= ALIGN_M))]
        rot = np.pi / 2 - np.arctan2(y3, x3)
    c, s = np.cos(rot), np.sin(rot)
    q = np.column_stack([c * q[:, 0] - s * q[:, 1], s * q[:, 0] + c * q[:, 1]])
    step_len = np.r_[0.0, np.hypot(*np.diff(q, axis=0).T)]
    dist = np.cumsum(step_len)
    k = max(1, int(SPEED_WIN * cfg.fs))
    walk_v = np.convolve(V, np.ones(k) / k, mode="same")
    onsets = np.flatnonzero(np.diff(S.astype(int)) == 1) + 1
    steps = np.zeros(n); steps[onsets] = 1; steps = np.cumsum(steps)

    T = np.arange(ts[0], ts[-1], 1 / FS)
    out = dict(t=T, c0_x=np.interp(T, ts, q[:, 0]), c0_y=np.interp(T, ts, q[:, 1]),
               c0_h=np.interp(T, ts, np.unwrap(Y) + rot), c0_v=np.interp(T, ts, walk_v),
               dist=np.interp(T, ts, dist), steps=np.interp(T, ts, steps))
    # strip charts: per timeline bin, the peak of the gyro and accel magnitudes (keeps the heel strikes) and
    # the share of the bin with the foot flat
    idx = np.clip(np.searchsorted(T, ts), 0, len(T) - 1)
    gm = np.degrees(np.linalg.norm(G[i1:], axis=1)); am = np.linalg.norm(A[i1:], axis=1)
    gpk = np.zeros(len(T)); apk = np.zeros(len(T)); np.maximum.at(gpk, idx, gm); np.maximum.at(apk, idx, am)
    flat = np.bincount(idx, S.astype(float), len(T)) / np.maximum(np.bincount(idx, None, len(T)), 1)
    out.update(s_g=gpk, s_a=apk, s_flat=flat, c0_flat=flat)

    stats = [dict(name="Foot filter", distance=float(dist[-1]), steps=int(steps[-1]),
                  speed=float(dist[-1] / max(ts[-1] - ts[0], 1e-9)))]
    meta = dict(kind="walk", name=os.path.splitext(os.path.basename(path))[0], file=os.path.abspath(path),
                format=info["format"], conditions=["Foot filter"], stats=stats,
                duration=float(T[-1] - T[0]), distance=float(dist[-1]),
                settings=dict(name="Shoe preset (tuned on the court walks)",
                              args=" ".join(f"{f}={getattr(cfg, f)}" for f in
                                            ("zupt_gyro_thr", "zupt_acc_var_thr", "zupt_win", "sigma_zupt",
                                             "sigma_g", "sigma_a", "sigma_zaru", "use_gravity", "zupt_period"))),
                calib=dict(still_from=float(t[i0] - t[0]), still_to=float(t[i1] - t[0]),
                           gyro_bias=np.asarray(cal["gyro_bias"]).tolist(), gravity=float(cal["gravity"]),
                           rate=float(cfg.fs)),
                gaps=dict(n=info["n_gaps"], seconds=info["gap_time"], longest=info["longest_gap"]),
                rejected=dict(nav.ins.n_rejected))
    np.savez_compressed(cf, meta=json.dumps(meta), **out)
    return engine.load(cf)


if __name__ == "__main__":      # command-line check: python foot.py recording.csv
    d = process(sys.argv[1], cb=print, use_cache=False)
    m = d["meta"]
    print(f"{m['name']}: {m['duration'] / 60:.1f} min, {m['distance']:.0f} m walked, {m['stats'][0]['steps']} steps, "
          f"{len(d['t'])} timeline samples, gaps {m['gaps']}")
