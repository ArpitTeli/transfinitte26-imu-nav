"""IMU calibration without external equipment: accelerometer and gyroscope bias, scale and misalignment.
Method: Tedaldi, Pretto, Menegatti, "A robust and easy to implement method for IMU calibration without external
equipments", ICRA 2014.

Recording (see CALIBRATION.md):  hold still >= 30 s, then put the IMU in 30-40 different orientations by hand,
holding each still for ~3 s. Rotations between poses should cover all three axes.

Models (sensor frame, calibrated = C (raw - b)):
  accel  a = Ta Ka (a_raw - ba)   Ta = [[1,-a_yz, a_zy],[0,1,-a_zx],[0,0,1]]   Ka = diag(s)        9 params
  gyro   w = Tg Kg (w_raw - bg)   Tg = [[1,-g_yz, g_zy],[g_xz,1,-g_zx],[-g_xy,g_yx,1]]  Kg = diag  9 params + bias
The accelerometer's orthogonalised axes define the calibrated frame (x = accel x; y in the accel x-y plane).

usage: python imu_calib.py log.csv [--counts] [--t-init 30] [--g 9.81] [--out calib]
  log.csv columns: t (s), ax, ay, az, gx, gy, gz  (m/s^2 and rad/s; with --counts: raw MPU6050 counts, +-2 g / +-250 dps)
  writes calib.json and calib.h (constants for the firmware)
"""
import sys, json, argparse, numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

G = 9.81

def accel_matrix(p):
    a_yz, a_zy, a_zx, sx, sy, sz = p[:6]
    return np.array([[1, -a_yz, a_zy], [0, 1, -a_zx], [0, 0, 1]]) @ np.diag([sx, sy, sz])

def gyro_matrix(p):
    g_yz, g_zy, g_xz, g_zx, g_xy, g_yx, sx, sy, sz = p[:9]
    return np.array([[1, -g_yz, g_zy], [g_xz, 1, -g_zx], [-g_xy, g_yx, 1]]) @ np.diag([sx, sy, sz])

def moving_var(x, n):
    """variance over a centred window of n samples, per column"""
    c = np.cumsum(np.vstack([np.zeros((1, x.shape[1])), x]), 0); c2 = np.cumsum(np.vstack([np.zeros((1, x.shape[1])), x ** 2]), 0)
    h = n // 2; i = np.arange(len(x)); lo = np.clip(i - h, 0, len(x)); hi = np.clip(i + h + 1, 0, len(x)); m = (hi - lo)[:, None]
    mean = (c[hi] - c[lo]) / m
    return (c2[hi] - c2[lo]) / m - mean ** 2

def static_intervals(t, a, w, bg, t_init, k, win_s=1.0, min_s=1.0, w_thr=None):
    """contiguous runs where accel variance < k * (its value during the initial still period) and the gyro is quiet"""
    dt = np.median(np.diff(t)); n = max(3, int(win_s / dt) | 1)
    v = np.linalg.norm(moving_var(a, n), axis=1)
    init = t < t[0] + t_init
    thr = k * np.median(v[init])
    if w_thr is None: w_thr = max(0.02, 5 * np.linalg.norm(w[init] - bg, axis=1).std() + np.linalg.norm(w[init] - bg, axis=1).mean())
    st = (v < thr) & (np.linalg.norm(w - bg, axis=1) < w_thr)
    edges = np.flatnonzero(np.diff(np.r_[0, st.astype(int), 0]))
    runs = [(s, e) for s, e in zip(edges[::2], edges[1::2]) if t[e - 1] - t[s] >= min_s]
    return runs

def fit_accel(means, g=G):
    def res(p):
        C = accel_matrix(p); return np.linalg.norm((means - p[6:9]) @ C.T, axis=1) - g
    p0 = np.r_[0, 0, 0, 1, 1, 1, 0, 0, 0]
    sol = least_squares(res, p0, method="lm", x_scale=np.r_[1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 0.1, 0.1, 0.1])
    return sol.x, res(p0), sol.fun

def rotation_between(w, dt):
    """total body rotation from integrating body rates w (N,3) with steps dt (N,): R = R0 R1 ... R(N-1)"""
    r = Rotation.from_rotvec(w * dt[:, None])
    while len(r) > 1:
        if len(r) % 2: r = Rotation.concatenate([r, Rotation.identity()])
        r = r[0::2] * r[1::2]
    return r[0]

def gyro_residual(pg, bg, segs, Ca, ba):
    Cg = gyro_matrix(pg); out = []
    for (u0, u1, wseg, dts) in segs:
        R = rotation_between((wseg - bg) @ Cg.T, dts)
        out.append(u1 - R.inv().apply(u0))
    return np.concatenate(out)

def calibrate(t, a, w, t_init=30.0, g=G, verbose=True):
    init = t < t[0] + t_init
    bg = w[init].mean(0)
    best = None
    for k in (2, 3, 4, 6, 8, 10, 15, 20):                       # pick the stillness threshold with the best accel fit
        runs = static_intervals(t, a, w, bg, t_init, k)
        runs = [r for r in runs if not (t[r[0]] < t[0] + t_init)]   # poses after the initial still period
        if len(runs) < 9: continue
        means = np.array([a[s:e].mean(0) for s, e in runs])
        pa, r0, r1 = fit_accel(means, g)
        score = np.sqrt(np.mean(r1 ** 2))
        if best is None or score < best[0]: best = (score, k, runs, pa, r0, r1)
    if best is None: raise SystemExit("fewer than 9 still poses found: hold each pose still for >= 2 s")
    _, k, runs, pa, r0, r1 = best
    Ca, ba = accel_matrix(pa), pa[6:9]
    # gravity direction in each pose, and the gyro samples between consecutive poses
    u = [(a[s:e].mean(0) - ba) @ Ca.T for s, e in runs]; u = [x / np.linalg.norm(x) for x in u]
    dt = np.diff(t, prepend=t[0] - np.median(np.diff(t)))
    segs = []
    for i in range(len(runs) - 1):
        # integrate from the MIDDLE of pose i to the middle of pose i+1: the slow start and end of a turn can pass the
        # stillness test, and cutting at the pose edges would drop that rotation (biasing the gyro scale high)
        s, e = (runs[i][0] + runs[i][1]) // 2, (runs[i + 1][0] + runs[i + 1][1]) // 2 + 1
        wm = 0.5 * (w[s:e - 1] + w[s + 1:e])                  # midpoint rates
        segs.append((u[i], u[i + 1], wm, dt[s + 1:e]))
    pg0 = np.r_[np.zeros(6), 1, 1, 1]
    sol = least_squares(lambda p: gyro_residual(p, bg, segs, Ca, ba), pg0, method="lm", x_scale=1e-2)
    pg = sol.x; Cg = gyro_matrix(pg)
    # residual = chord between two unit vectors: angle = 2 asin(|d| / 2)
    ang = lambda res: np.degrees(2 * np.arcsin(np.clip(np.linalg.norm(res.reshape(-1, 3), axis=1) / 2, 0, 1)))
    g_before, g_after = ang(gyro_residual(pg0, bg, segs, Ca, ba)), ang(sol.fun)
    # coverage check: poses should point gravity in many directions
    U = np.array(u); ev = np.linalg.eigvalsh(U.T @ U / len(U))
    rep = dict(n_poses=len(runs), threshold_k=k, accel_g_err_before=float(np.sqrt(np.mean(r0 ** 2))),
               accel_g_err_after=float(np.sqrt(np.mean(r1 ** 2))), gyro_err_deg_before=float(np.sqrt(np.mean(g_before ** 2))),
               gyro_err_deg_after=float(np.sqrt(np.mean(g_after ** 2))), coverage_min_eig=float(ev.min()))
    if verbose:
        print(f"still poses used: {len(runs)} (threshold k={k}); orientation coverage {ev.min():.2f} (want > 0.1)")
        print(f"accel |a|-g RMS over poses: {rep['accel_g_err_before']:.4f} -> {rep['accel_g_err_after']:.4f} m/s^2")
        print(f"gyro pose-to-pose gravity error RMS: {rep['gyro_err_deg_before']:.3f} -> {rep['gyro_err_deg_after']:.3f} deg")
        if ev.min() < 0.1: print("WARNING: poses do not cover enough directions; add poses with each axis up and down")
    return dict(Ca=Ca, ba=ba, Cg=Cg, bg=bg, pa=pa, pg=pg, report=rep)

def describe(c):
    pa, pg = c["pa"], c["pg"]
    d = dict(accel_bias=c["ba"].tolist(), accel_scale=pa[3:6].tolist(), accel_misalign_deg=np.degrees(pa[:3]).tolist(),
             gyro_bias=c["bg"].tolist(), gyro_scale=pg[6:9].tolist(), gyro_misalign_deg=np.degrees(pg[:6]).tolist(),
             Ca=c["Ca"].tolist(), Cg=c["Cg"].tolist(), report=c["report"])
    return d

def write_header(c, path):
    f = lambda M: ", ".join("{" + ", ".join(f"{x:.7f}f" for x in row) + "}" for row in M)
    v = lambda x: ", ".join(f"{y:.7f}f" for y in x)
    open(path, "w").write(
        "// IMU calibration generated by imu_calib.py. calibrated = C * (raw_SI - b), raw_SI = counts * nominal LSB.\n"
        "// The gyro bias here is the bias during calibration; the firmware still re-measures it at every boot.\n"
        "#pragma once\n#define IMU_CALIBRATED 1\n"
        f"static const float CAL_ACC_C[3][3] = {{{f(c['Ca'])}}};\nstatic const float CAL_ACC_B[3] = {{{v(c['ba'])}}};\n"
        f"static const float CAL_GYR_C[3][3] = {{{f(c['Cg'])}}};\nstatic const float CAL_GYR_B[3] = {{{v(c['bg'])}}};\n")

def apply(c, a, w):
    return (a - c["ba"]) @ c["Ca"].T, (w - c["bg"]) @ c["Cg"].T

if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("log"); ap.add_argument("--counts", action="store_true")
    ap.add_argument("--t-init", type=float, default=30.0); ap.add_argument("--g", type=float, default=G); ap.add_argument("--out", default="calib")
    A = ap.parse_args()
    import pandas as pd
    df = pd.read_csv(A.log)
    cols = [c.strip().lower() for c in df.columns]
    if all(c in cols for c in ("t", "ax", "ay", "az", "gx", "gy", "gz")):
        # by name: IMU Live recordings are t,gx,gy,gz,ax,ay,az (gyro first), log_capture.py writes accel first
        df.columns = cols
        t, a, w = df.t.to_numpy(float), df[["ax", "ay", "az"]].to_numpy(float), df[["gx", "gy", "gz"]].to_numpy(float)
    else:
        print("no t/ax../gx.. header: assuming columns t, ax, ay, az, gx, gy, gz")
        d = df.to_numpy(float)
        t, a, w = d[:, 0], d[:, 1:4], d[:, 4:7]
    if A.counts: a, w = a / 16384.0 * G, w / 131.0 * np.pi / 180
    c = calibrate(t, a, w, A.t_init, A.g)
    json.dump(describe(c), open(A.out + ".json", "w"), indent=1); write_header(c, A.out + ".h")
    D = describe(c)
    print("accel bias m/s^2", np.round(D["accel_bias"], 4), " scale", np.round(D["accel_scale"], 4), " misalign deg", np.round(D["accel_misalign_deg"], 3))
    print("gyro  bias rad/s", np.round(D["gyro_bias"], 5), " scale", np.round(D["gyro_scale"], 4), " misalign deg", np.round(D["gyro_misalign_deg"], 3))
    print("wrote", A.out + ".json", A.out + ".h")
