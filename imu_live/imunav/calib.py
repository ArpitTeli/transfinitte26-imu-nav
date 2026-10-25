"""Sensor calibration: static bias, six-position accel, gyro scale, Allan variance."""
import numpy as np

G_STD = 9.80665
# "still" = every axis spreads less than this over the calibration window. A phone lying on a table spreads
# ~0.001 rad/s; one strapped tight to the shoe of someone standing still stays well under the gyro limit.
GYRO_STILL_STD = 0.02     # rad/s
ACC_STILL_STD = 0.3       # m/s^2


def static_calibration(t, gyro, acc, window=2.0):
    """Estimate gyro bias and local gravity from the stationary start of a recording.

    Accelerometer bias is not observable from a single pose; the measured
    gravity magnitude is returned so navigation uses the local value instead.
    """
    m = (t - t[0]) < window
    g_mean, a_mean = gyro[m].mean(0), acc[m].mean(0)
    # moving shows up as spread, not as a large mean (a big gyro bias is still static)
    ok = bool(gyro[m].std(0).max() < GYRO_STILL_STD and acc[m].std(0).max() < ACC_STILL_STD)
    return {
        "gyro_bias": g_mean, "acc_mean": a_mean, "gravity": float(np.linalg.norm(a_mean)),
        "gyro_std": gyro[m].std(0), "acc_std": acc[m].std(0), "looks_static": ok, "n": int(m.sum()),
    }


def six_position_fit(means, g=G_STD):
    """Fit acc_raw = S @ a_true + b from six static means.

    means: (6,3) raw means for +x up, -x up, +y up, -y up, +z up, -z up.
    Returns (S, b). Apply with apply_acc_cal.
    """
    truth = g * np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]], float)
    A = np.hstack([truth, np.ones((6, 1))])
    X, *_ = np.linalg.lstsq(A, np.asarray(means, float), rcond=None)
    return X[:3].T, X[3]


def apply_acc_cal(acc, S, b):
    return (np.linalg.inv(S) @ (acc - b).T).T


def gyro_scale_fit(integrals, angles):
    """Fit gyro_raw = M @ w_true from three rotations about x, y, z.

    integrals: (3,3), row k = integral of bias-removed raw gyro over the
    rotation about axis k (rad). angles: the three known angles (rad).
    Returns M; apply with apply_gyro_cal.
    """
    D = np.asarray(integrals, float)
    return (D / np.asarray(angles, float)[:, None]).T


def apply_gyro_cal(gyro, M, bias):
    return (np.linalg.inv(M) @ (gyro - bias).T).T


def integrate_rotation(t, gyro, bias):
    return np.trapezoid(gyro - bias, t, axis=0)


def allan_deviation(x, fs, n_taus=60):
    """Overlapping Allan deviation of a rate signal x sampled at fs."""
    N = len(x)
    theta = np.concatenate([[0.0], np.cumsum(x) / fs])
    ms = np.unique(np.round(np.logspace(0, np.log10(N // 2 - 1), n_taus)).astype(int))
    taus, adev = [], []
    for m in ms:
        d = theta[2 * m:] - 2 * theta[m:-m] + theta[:-2 * m]
        tau = m / fs
        taus.append(tau)
        adev.append(np.sqrt(0.5 * np.mean(d ** 2)) / tau)
    return np.array(taus), np.array(adev)


def allan_params(taus, adev):
    """Noise parameters from an Allan deviation curve.

    N: white-noise density, read on the -1/2 slope as adev * sqrt(tau) (value at tau = 1 s).
    B: bias instability, minimum / 0.664.
    K: random walk, read on the +1/2 slope as adev * sqrt(3 / tau) (value at tau = 3 s).
    A parameter is nan when its slope region is not present in the curve.
    """
    lt, la = np.log(taus), np.log(adev)
    slope = np.gradient(la, lt)
    i = int(np.argmin(adev))
    out = {"B": float(adev[i] / 0.664), "tau_B": float(taus[i]), "N": float("nan"), "K": float("nan")}
    white = (np.abs(slope + 0.5) < 0.15) & (np.arange(len(taus)) < i)
    if white.any():
        out["N"] = float(np.median(adev[white] * np.sqrt(taus[white])))
    rw = (np.abs(slope - 0.5) < 0.15) & (np.arange(len(taus)) > i)
    if rw.any():
        out["K"] = float(np.median(adev[rw] * np.sqrt(3.0 / taus[rw])))
    return out
