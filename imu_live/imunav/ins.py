"""Strapdown integration and the 15-state error-state EKF.

State (nominal): position p, velocity v (world), attitude q (body->world),
gyro bias bg, accel bias ba. Error state order: [dp, dv, dtheta, dbg, dba],
with the attitude error expressed in the body frame (q_true = q * exp(dtheta)).
"""
import math

import numpy as np

from .quat import qmul, qnormalize, qexp, quat_to_R, skew

P_, V_, TH_, BG_, BA_ = slice(0, 3), slice(3, 6), slice(6, 9), slice(9, 12), slice(12, 15)


def chi2_ppf(p, k):
    """Chi-square quantile (no scipy, so the packaged app stays small). Bisection on the CDF."""
    a = k / 2.0

    def cdf(x):  # regularized lower incomplete gamma P(k/2, x/2), series form
        z = x / 2.0
        if z <= 0:
            return 0.0
        term = s = 1.0 / a
        n = 0
        while term > s * 1e-15 and n < 5000:
            n += 1
            term *= z / (a + n)
            s += term
        return s * math.exp(-z + a * math.log(z) - math.lgamma(a))

    lo, hi = 0.0, 1.0
    while cdf(hi) < p:
        hi *= 2.0
    for _ in range(200):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if cdf(mid) < p else (lo, mid)
    return 0.5 * (lo + hi)


class Strapdown:
    """Dead reckoning with an external attitude: rotate, remove gravity, integrate twice."""

    def __init__(self, p0, v0, g=9.80665):
        self.p, self.v = np.array(p0, float), np.array(v0, float)
        self.gvec = np.array([0.0, 0.0, -g])

    def step(self, q, acc, dt, still=False):
        if still:
            self.v[:] = 0.0
            return
        a = quat_to_R(q) @ acc + self.gvec
        self.p += self.v * dt + 0.5 * a * dt * dt
        self.v += a * dt


class InsEKF:
    def __init__(self, q0, p0, v0, bg0, ba0=None, g=9.80665,
                 sigma_g=1.7e-4, sigma_a=2.0e-3, sigma_bg=1.9e-5, sigma_ba=3.0e-3,
                 p_pos=1e-4, p_vel=1e-4, p_att=1e-3, p_bg=1e-5, p_ba=1e-2,
                 q_scale=1.0, gate_prob=0.9999):
        self.q = np.array(q0, float)
        self.p, self.v = np.array(p0, float), np.array(v0, float)
        self.bg = np.array(bg0, float)
        self.ba = np.zeros(3) if ba0 is None else np.array(ba0, float)
        self.g = g
        self.gvec = np.array([0.0, 0.0, -g])
        self.P = np.diag([p_pos] * 3 + [p_vel] * 3 + [p_att] * 3 + [p_bg] * 3 + [p_ba] * 3).astype(float)
        self.P0 = self.P.copy()   # start-up covariance, reused to restart after a data gap
        s = q_scale
        self.Qd = np.diag([0.0] * 3 + [s * sigma_a ** 2] * 3 + [s * sigma_g ** 2] * 3 +
                          [s * sigma_bg ** 2] * 3 + [s * sigma_ba ** 2] * 3)
        self.gate_prob = gate_prob
        self._chi = {}
        self.n_updates = {}
        self.n_rejected = {}
        self.n_forced = {}

    def predict(self, gyro, acc, dt, turn_sigma=0.0):
        """turn_sigma: fractional uncertainty of the gyro's turn rate about the vertical (a gain error). It only
        grows the heading uncertainty, so a heading reference (the compass) can take the turn error out again."""
        w = gyro - self.bg
        f = acc - self.ba
        q_mid = qnormalize(qmul(self.q, qexp(w * dt / 2)))
        Rm = quat_to_R(q_mid)
        a = Rm @ f + self.gvec
        self.p += self.v * dt + 0.5 * a * dt * dt
        self.v += a * dt
        self.q = qnormalize(qmul(self.q, qexp(w * dt)))
        F = np.zeros((15, 15))
        F[P_, V_] = np.eye(3)
        F[V_, TH_] = -Rm @ skew(f)
        F[V_, BA_] = -Rm
        F[TH_, TH_] = -skew(w)
        F[TH_, BG_] = -np.eye(3)
        Phi = np.eye(15) + F * dt + 0.5 * (F @ F) * dt * dt
        self.P = Phi @ self.P @ Phi.T + self.Qd * dt
        if turn_sigma > 0:
            up = Rm[2, :]                       # world vertical in the body frame
            s = turn_sigma * float(up @ w) * dt
            self.P[TH_, TH_] += s * s * np.outer(up, up)

    def update(self, y, H, R, tag):
        """Generic gated measurement update. y = z - h(x_nominal)."""
        S = H @ self.P @ H.T + R
        Sinv = np.linalg.inv(S)
        d2 = float(y @ Sinv @ y)
        dof = len(y)
        if dof not in self._chi:
            self._chi[dof] = chi2_ppf(self.gate_prob, dof)
        self.n_updates[tag] = self.n_updates.get(tag, 0) + 1
        if d2 > self._chi[dof]:
            self.n_rejected[tag] = self.n_rejected.get(tag, 0) + 1
            return False
        K = self.P @ H.T @ Sinv
        dx = K @ y
        self.p += dx[P_]
        self.v += dx[V_]
        self.q = qnormalize(qmul(self.q, qexp(dx[TH_])))
        self.bg += dx[BG_]
        self.ba += dx[BA_]
        IKH = np.eye(15) - K @ H
        self.P = IKH @ self.P @ IKH.T + K @ R @ K.T
        self.P = 0.5 * (self.P + self.P.T)
        return True

    def reopen(self, blocks, var, tag):
        """Declare state blocks unknown: drop their cross-covariances and set a large variance.

        Used when the filter has diverged and the gate keeps rejecting measurements it
        should accept (otherwise it can never recover).
        """
        for b, v in zip(blocks, var):
            self.P[b, :] = 0.0
            self.P[:, b] = 0.0
            self.P[b, b] = np.eye(3) * v
        self.n_forced[tag] = self.n_forced.get(tag, 0) + 1

    def zupt(self, sigma=0.02):
        H = np.zeros((3, 15))
        H[:, V_] = np.eye(3)
        return self.update(-self.v, H, np.eye(3) * sigma ** 2, "zupt")

    def zaru(self, gyro, sigma=2e-3):
        """Zero angular rate: when still, raw gyro reads the bias."""
        H = np.zeros((3, 15))
        H[:, BG_] = np.eye(3)
        return self.update(gyro - self.bg, H, np.eye(3) * sigma ** 2, "zaru")

    def heading(self, y, H_th, sigma):
        """Compass heading: innovation y (rad) with Jacobian H_th on the attitude error (compass.mag_heading_h)."""
        H = np.zeros((1, 15))
        H[0, TH_] = H_th
        return self.update(np.array([y]), H, np.array([[sigma ** 2]]), "mag")

    def gravity(self, acc, sigma=0.05):
        """When still, specific force = ba + g * R^T e_z: observes tilt and accel bias."""
        vb = quat_to_R(self.q)[2, :]
        H = np.zeros((3, 15))
        H[:, TH_] = self.g * skew(vb)
        H[:, BA_] = np.eye(3)
        return self.update(acc - (self.ba + self.g * vb), H, np.eye(3) * sigma ** 2, "gravity")
