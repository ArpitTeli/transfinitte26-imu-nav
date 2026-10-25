"""Quaternion and rotation helpers.

Convention: quaternions are [w, x, y, z], unit norm, and q rotates a vector
from the body (sensor) frame into the world frame: v_w = R(q) @ v_b.
World frame is z-up; gravity is [0, 0, -g].
"""
import numpy as np


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def qnormalize(q):
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([1.0, 0.0, 0.0, 0.0])


def qexp(rotvec):
    """Rotation vector (rad) -> unit quaternion."""
    th = np.linalg.norm(rotvec)
    if th < 1e-9:
        return qnormalize(np.array([1.0, *(0.5 * np.asarray(rotvec))]))
    s = np.sin(th / 2) / th
    return np.array([np.cos(th / 2), rotvec[0] * s, rotvec[1] * s, rotvec[2] * s])


def qlog(q):
    """Unit quaternion -> rotation vector (rad)."""
    q = qnormalize(q)
    if q[0] < 0:
        q = -q
    v = q[1:]
    n = np.linalg.norm(v)
    if n < 1e-12:
        return 2 * v
    return 2 * np.arctan2(n, q[0]) * v / n


def skew(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


def quat_to_R(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1.0)
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = 2 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = 2 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    return qnormalize(np.array(q))


def yaw_of(q):
    """Heading of the body x-axis in the world frame (rad)."""
    R = quat_to_R(q)
    return float(np.arctan2(R[1, 0], R[0, 0]))


def quat_to_euler(q):
    """Roll, pitch, yaw (ZYX, rad)."""
    R = quat_to_R(q)
    pitch = -np.arcsin(np.clip(R[2, 0], -1, 1))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.array([roll, pitch, yaw])


def qyaw(psi):
    return np.array([np.cos(psi / 2), 0.0, 0.0, np.sin(psi / 2)])


def q_from_gravity(acc, yaw=0.0):
    """Initial attitude from a static accelerometer reading.

    The accelerometer measures +g along world-up, so acc/|acc| is world-z in
    the body frame. Roll/pitch come from that; `yaw` sets the heading of the
    body x-axis in the world frame.
    """
    u = np.asarray(acc, float)
    u = u / np.linalg.norm(u)
    ref = np.array([1.0, 0.0, 0.0]) if abs(u[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    r0 = ref - np.dot(ref, u) * u  # body x (or y if x is near vertical) projected onto the level plane
    r0 /= np.linalg.norm(r0)
    r1 = np.cross(u, r0)
    q_tilt = R_to_quat(np.vstack([r0, r1, u]))
    return qnormalize(qmul(qyaw(yaw), q_tilt))


def slerp(q0, q1, f):
    d = float(np.dot(q0, q1))
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        return qnormalize(q0 + f * (q1 - q0))
    th = np.arccos(d)
    return (np.sin((1 - f) * th) * q0 + np.sin(f * th) * q1) / np.sin(th)
