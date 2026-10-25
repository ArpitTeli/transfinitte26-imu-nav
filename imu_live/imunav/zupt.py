"""Stationary (zero-velocity) detection."""
import numpy as np


class ZuptDetector:
    """Causal sliding-window detector.

    Flags 'still' when, over the last `win` seconds:
      * mean bias-removed gyro magnitude < gyro_thr (rad/s),
      * variance of the accel magnitude < acc_var_thr (m^2/s^4), and
      * (if an attitude is supplied) the mean of acc - g * R^T e_z, i.e. the
        specific force not explained by gravity, is below res_thr (m/s^2).
    The last test catches slow horizontal accelerations that leave |acc|
    almost unchanged. It depends on the filter's attitude, so it must not veto
    forever: if the first two tests pass but the third keeps failing for
    `override` seconds in a row, the attitude is what is wrong (nobody
    accelerates smoothly for that long without rotating), so the sample counts as
    still and the filter can correct its tilt.
    One sample in, one bool out.
    """

    def __init__(self, fs=200.0, win=0.25, gyro_thr=0.06, acc_var_thr=0.05, res_thr=0.25, g=9.80665,
                 override=1.5):
        self.n = max(2, int(round(win * fs)))
        self.override_n = max(1, int(round(override * fs))) if np.isfinite(override) else np.inf  # inf = off
        self.veto_run = 0
        self.gyro_thr, self.acc_var_thr, self.res_thr, self.g = gyro_thr, acc_var_thr, res_thr, g
        self.gm = np.zeros(self.n)
        self.am = np.zeros(self.n)
        self.res = np.zeros((self.n, 3))
        self.i = 0
        self.filled = 0

    def update(self, gyro_unbiased, acc, up_body=None):
        self.gm[self.i] = np.sqrt(gyro_unbiased[0] ** 2 + gyro_unbiased[1] ** 2 + gyro_unbiased[2] ** 2)
        self.am[self.i] = np.sqrt(acc[0] ** 2 + acc[1] ** 2 + acc[2] ** 2)
        if up_body is not None:
            self.res[self.i] = acc - self.g * up_body
        self.i = (self.i + 1) % self.n
        self.filled = min(self.filled + 1, self.n)
        if self.filled < self.n:
            return False
        ok = self.gm.mean() < self.gyro_thr and self.am.var() < self.acc_var_thr
        if ok and up_body is not None and self.res_thr is not None:
            if np.linalg.norm(self.res.mean(0)) < self.res_thr:
                self.veto_run = 0
            else:
                self.veto_run += 1
                ok = self.veto_run >= self.override_n
        else:
            self.veto_run = 0
        return bool(ok)


def gt_stationary(gt_v, thr=0.05):
    """Ground-truth 'still' mask from ground-truth speed (m/s)."""
    return np.linalg.norm(gt_v, axis=1) < thr


def detector_stats(still, gt_still):
    """False-positive rate (flagged still while moving) and miss rate (real stop not flagged)."""
    moving, stopped = ~gt_still, gt_still
    return {
        "false_positive_rate": float(still[moving].mean()) if moving.any() else float("nan"),
        "miss_rate": float((~still[stopped]).mean()) if stopped.any() else float("nan"),
        "flagged_fraction": float(still.mean()),
        "gt_still_fraction": float(gt_still.mean()),
    }
