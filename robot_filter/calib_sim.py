"""Synthetic IMU calibration sessions with known errors, to validate imu_calib.py.
usage: python calib_sim.py [trials] [poses]"""
import sys, numpy as np
from scipy.spatial.transform import Rotation
import imu_calib as ic

def true_params(rng):
    pa = np.r_[np.radians(rng.normal(0, 1.0, 3)), 1 + rng.normal(0, 0.02, 3), rng.normal(0, 0.3, 3)]   # MPU6050-scale errors
    pg = np.r_[np.radians(rng.normal(0, 1.0, 6)), 1 + rng.normal(0, 0.02, 3)]
    bg = rng.normal(0, 0.05, 3)
    return pa, pg, bg

def session(rng, pa, pg, bg, n_poses=36, t_init=30.0, fs=200.0, still=(2.0, 4.0), move=(1.5, 3.0),
            acc_noise=0.04, gyr_noise=0.003, tremor=True, counts=True):
    dt = 1 / fs; g_w = np.array([0, 0, ic.G])
    R = Rotation.random(random_state=rng.integers(1 << 31))
    ts, A, W = [], [], []
    def hold(R, T):
        n = int(T * fs)
        for _ in range(n):
            a = R.inv().apply(g_w); w = np.zeros(3)
            if tremor: a = a + rng.normal(0, 0.02, 3); w = w + rng.normal(0, 0.002, 3)    # hand is never perfectly still
            A.append(a); W.append(w)
    hold(R, t_init)
    for _ in range(n_poses):
        R1 = Rotation.random(random_state=rng.integers(1 << 31))
        phi = (R.inv() * R1).as_rotvec(); n = int(rng.uniform(*move) * fs); T = n / fs   # T must match the samples
        lin = rng.normal(0, 1.0, 3)                                                          # hand moves while turning
        for i in range(n):
            s = (i + 0.5) / n; sv = 10 * s**3 - 15 * s**4 + 6 * s**5; sd = (30 * s**2 - 60 * s**3 + 30 * s**4) / T
            Rt = R * Rotation.from_rotvec(phi * sv)
            A.append(Rt.inv().apply(g_w) + lin * np.sin(np.pi * s)); W.append(phi * sd)
        R = R * Rotation.from_rotvec(phi)
        hold(R, rng.uniform(*still))
    A, W = np.array(A), np.array(W); t = np.arange(len(A)) * dt
    Ca, Cg = ic.accel_matrix(pa), ic.gyro_matrix(pg)
    bgt = bg + np.cumsum(rng.normal(0, 2e-5 * np.sqrt(dt), (len(t), 3)), 0)                 # slow gyro-bias drift
    a_raw = A @ np.linalg.inv(Ca).T + pa[6:9] + rng.normal(0, acc_noise, A.shape)
    w_raw = W @ np.linalg.inv(Cg).T + bgt + rng.normal(0, gyr_noise, W.shape)
    if counts:   # MPU6050 quantisation (+-2 g, +-250 dps)
        a_raw = np.round(a_raw / ic.G * 16384) / 16384 * ic.G; w_raw = np.round(w_raw * 180 / np.pi * 131) / 131 * np.pi / 180
    return t, a_raw, w_raw

def errors(c, pa, pg):
    """parameter errors, and what they mean: worst accel error on a g-sized reading, heading error per 360 deg turn"""
    Ca_t, Cg_t, ba_t = ic.accel_matrix(pa), ic.gyro_matrix(pg), pa[6:9]
    # calibrated output vs truth for a reading of true vector v: Ca_est (Ca_t^-1 v + ba_t - ba_est) - v
    Ea = c["Ca"] @ np.linalg.inv(Ca_t) - np.eye(3)
    acc_worst = np.linalg.norm(Ea, 2) * ic.G + np.linalg.norm(c["Ca"] @ (ba_t - c["ba"]))
    Eg = c["Cg"] @ np.linalg.inv(Cg_t) - np.eye(3)
    head_360 = np.degrees(np.linalg.norm(Eg, 2) * 2 * np.pi)
    return dict(acc_scale_pct=100 * np.abs(c["pa"][3:6] - pa[3:6]).max(), acc_mis_deg=np.degrees(np.abs(c["pa"][:3] - pa[:3]).max()),
                acc_bias=np.abs(c["ba"] - ba_t).max(), gyr_scale_pct=100 * np.abs(c["pg"][6:9] - pg[6:9]).max(),
                gyr_mis_deg=np.degrees(np.abs(c["pg"][:6] - pg[:6]).max()), acc_worst=acc_worst, head_360=head_360)

def uncal_errors(pa, pg):
    """same two summary numbers with no calibration (datasheet nominal: C = I, accel bias 0)"""
    Ea = np.linalg.inv(ic.accel_matrix(pa)) - np.eye(3)
    Eg = np.linalg.inv(ic.gyro_matrix(pg)) - np.eye(3)
    return np.linalg.norm(Ea, 2) * ic.G + np.linalg.norm(pa[6:9]), np.degrees(np.linalg.norm(Eg, 2) * 2 * np.pi)

if __name__ == "__main__":
    trials = int(sys.argv[1]) if len(sys.argv) > 1 else 20; poses = int(sys.argv[2]) if len(sys.argv) > 2 else 36
    rows, un = [], []
    for k in range(trials):
        rng = np.random.default_rng(1000 + k)
        pa, pg, bg = true_params(rng)
        t, a, w = session(rng, pa, pg, bg, n_poses=poses)
        c = ic.calibrate(t, a, w, verbose=False)
        e = errors(c, pa, pg); e["poses"] = c["report"]["n_poses"]; rows.append(e); un.append(uncal_errors(pa, pg))
    keys = ["poses", "acc_scale_pct", "acc_mis_deg", "acc_bias", "gyr_scale_pct", "gyr_mis_deg", "acc_worst", "head_360"]
    print(f"{trials} synthetic sessions, {poses} poses each (MPU6050-scale errors, hand tremor, quantisation)")
    print(f"{'':12s} " + " ".join(f"{k:>13s}" for k in keys))
    for name, fn in (("median", np.median), ("worst", np.max)):
        print(f"{name:12s} " + " ".join(f"{fn([r[k] for r in rows]):13.4f}" for k in keys))
    un = np.array(un)
    print(f"\nuncalibrated (datasheet nominal), median / worst: accel error on a 1 g reading {np.median(un[:,0]):.3f} / {un[:,0].max():.3f} m/s^2,"
          f" heading error per 360 deg turn {np.median(un[:,1]):.2f} / {un[:,1].max():.2f} deg")
    print(f"calibrated,                     median / worst: accel error on a 1 g reading {np.median([r['acc_worst'] for r in rows]):.4f} / {max(r['acc_worst'] for r in rows):.4f} m/s^2,"
          f" heading error per 360 deg turn {np.median([r['head_360'] for r in rows]):.3f} / {max(r['head_360'] for r in rows):.3f} deg")
