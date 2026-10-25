"""Orientation filters (gyro + accelerometer, no magnetometer).

Each filter exposes update(gyro, acc, dt, use_acc=True) -> q and the
attributes `q` (body->world, [w,x,y,z]) and `bg` (gyro bias estimate, rad/s).
`use_acc=False` propagates on the gyro only; the navigation pipeline passes
use_acc=still so that horizontal acceleration is never mistaken for tilt.
Yaw is not observable without a heading aid and drifts with residual gyro bias.
"""
import numpy as np

from .quat import qmul, qnormalize, qexp, quat_to_R, skew


class _Base:
    def __init__(self, q0, g=9.80665, acc_gate=1.0):
        self.q = np.array(q0, float)
        self.g = g
        self.gate = acc_gate
        self.bg = np.zeros(3)

    def _acc_ok(self, acc, use_acc):
        return use_acc and (self.gate is None or abs(np.linalg.norm(acc) - self.g) < self.gate)


class Madgwick(_Base):
    """Madgwick gradient-descent filter, IMU-only form. beta trades gyro vs accel trust."""

    def __init__(self, q0, beta=0.05, **kw):
        super().__init__(q0, **kw)
        self.beta = beta

    def update(self, gyro, acc, dt, use_acc=True):
        q = self.q
        w, x, y, z = q
        qdot = 0.5 * qmul(q, np.array([0.0, *gyro]))
        n = np.linalg.norm(acc)
        if n > 0 and self._acc_ok(acc, use_acc):
            a = acc / n
            v = np.array([2 * (x * z - w * y), 2 * (y * z + w * x), w * w - x * x - y * y + z * z])
            J = np.array([[-2 * y, 2 * z, -2 * w, 2 * x],
                          [2 * x, 2 * w, 2 * z, 2 * y],
                          [2 * w, -2 * x, -2 * y, 2 * z]])
            grad = J.T @ (v - a)
            gn = np.linalg.norm(grad)
            if gn > 0:
                qdot = qdot - self.beta * grad / gn
        self.q = qnormalize(q + qdot * dt)
        return self.q


class Mahony(_Base):
    """Mahony complementary filter with PI correction (kp, ki)."""

    def __init__(self, q0, kp=1.0, ki=0.02, **kw):
        super().__init__(q0, **kw)
        self.kp, self.ki = kp, ki
        self.ei = np.zeros(3)

    def update(self, gyro, acc, dt, use_acc=True):
        q = self.q
        w, x, y, z = q
        corr = np.zeros(3)
        n = np.linalg.norm(acc)
        if n > 0 and self._acc_ok(acc, use_acc):
            v = np.array([2 * (x * z - w * y), 2 * (y * z + w * x), w * w - x * x - y * y + z * z])
            e = np.cross(acc / n, v)
            self.ei += self.ki * e * dt
            corr = self.kp * e
        self.bg = -self.ei
        gc = gyro + corr + self.ei
        self.q = qnormalize(q + 0.5 * qmul(q, np.array([0.0, *gc])) * dt)
        return self.q


class AttitudeEKF(_Base):
    """Multiplicative EKF: error state [d_theta(3), d_bias(3)], accel as unit-gravity measurement."""

    def __init__(self, q0, sigma_g=1.7e-4, sigma_bg=1.9e-5, sigma_a=0.03,
                 p_att=1e-2, p_bias=1e-6, bg0=None, **kw):
        super().__init__(q0, **kw)
        self.sg, self.sbg, self.sa = sigma_g, sigma_bg, sigma_a
        self.P = np.diag([p_att] * 3 + [p_bias] * 3)
        if bg0 is not None:
            self.bg = np.array(bg0, float)

    def update(self, gyro, acc, dt, use_acc=True):
        w = gyro - self.bg
        self.q = qnormalize(qmul(self.q, qexp(w * dt)))
        F = np.eye(6)
        F[:3, :3] = quat_to_R(qexp(-w * dt))
        F[:3, 3:] = -np.eye(3) * dt
        Q = np.diag([self.sg ** 2 * dt] * 3 + [self.sbg ** 2 * dt] * 3)
        self.P = F @ self.P @ F.T + Q
        n = np.linalg.norm(acc)
        if n > 0 and self._acc_ok(acc, use_acc):
            v = quat_to_R(self.q)[2, :]
            H = np.zeros((3, 6))
            H[:, :3] = skew(v)
            Rm = np.eye(3) * (self.sa / self.g) ** 2
            S = H @ self.P @ H.T + Rm
            K = self.P @ H.T @ np.linalg.inv(S)
            dx = K @ (acc / n - v)
            self.q = qnormalize(qmul(self.q, qexp(dx[:3])))
            self.bg = self.bg + dx[3:]
            IKH = np.eye(6) - K @ H
            self.P = IKH @ self.P @ IKH.T + K @ Rm @ K.T
        return self.q


FILTERS = {"madgwick": Madgwick, "mahony": Mahony, "ekf": AttitudeEKF}
