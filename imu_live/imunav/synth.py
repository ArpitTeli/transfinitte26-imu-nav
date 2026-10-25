"""Synthetic IMU generators with known ground truth, bias, noise and scale error.

make_walk:      body-mounted rectangular walk with long stops at the corners.
make_foot_walk: shoe-mounted IMU: every stride has a still stance phase and a
                fast swing (forward travel, foot lift, toe-off/heel-strike pitch).
Noise defaults follow the EuRoC ADIS16448 values.
"""
import numpy as np

from .io import Sequence
from .quat import R_to_quat

G = 9.80665
NOISE = dict(gyro_bias=(0.002, -0.0015, 0.001), acc_bias=(0.03, -0.02, 0.04),
             gyro_scale=(1.0, 1.0, 1.0), acc_scale=(1.0, 1.0, 1.0),
             gyro_nd=1.7e-4, acc_nd=2.0e-3, gyro_rw=1.9e-5, acc_rw=3.0e-3)


def _smoothstep(x):
    x = np.clip(x, 0, 1)
    return x * x * (3 - 2 * x)


def _minjerk(x):
    x = np.clip(x, 0, 1)
    return x ** 3 * (10 - 15 * x + 6 * x * x)


def _euler_R(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    R = np.zeros((len(roll), 3, 3))
    R[:, 0, 0], R[:, 0, 1], R[:, 0, 2] = cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr
    R[:, 1, 0], R[:, 1, 1], R[:, 1, 2] = sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr
    R[:, 2, 0], R[:, 2, 1], R[:, 2, 2] = -sp, cp * sr, cp * cr
    return R


def _imu_from_motion(t, vel, R, fs, seed, name, gyro_bias, acc_bias, gyro_scale, acc_scale,
                     gyro_nd, acc_nd, gyro_rw, acc_rw):
    """Ideal + corrupted IMU readings for a body with world velocity `vel` and attitude `R`."""
    rng = np.random.default_rng(seed)
    dt = 1.0 / fs
    N = len(t)
    pos = np.vstack([np.zeros(3), np.cumsum(0.5 * (vel[1:] + vel[:-1]) * dt, axis=0)])
    acc_w = np.gradient(vel, dt, axis=0)
    dR = np.gradient(R, dt, axis=0)
    W = np.einsum("nji,njk->nik", R, dR)  # R^T dR (skew-symmetric)
    w_true = np.column_stack([W[:, 2, 1], W[:, 0, 2], W[:, 1, 0]])
    f_true = np.einsum("nji,nj->ni", R, acc_w + np.array([0, 0, G]))

    s = np.sqrt(fs)
    bg = np.array(gyro_bias) + np.cumsum(rng.normal(0, gyro_rw / s, (N, 3)), axis=0)
    ba = np.array(acc_bias) + np.cumsum(rng.normal(0, acc_rw / s, (N, 3)), axis=0)
    gyro = w_true * np.array(gyro_scale) + bg + rng.normal(0, gyro_nd * s, (N, 3))
    acc = f_true * np.array(acc_scale) + ba + rng.normal(0, acc_nd * s, (N, 3))
    gt_q = np.array([R_to_quat(Rk) for Rk in R])
    return Sequence(name, t, gyro, acc, pos, gt_q, vel,
                    {"true_gyro_bias": np.array(gyro_bias), "true_acc_bias": np.array(acc_bias)})


def make_walk(fs=200.0, leg_len=12.0, legs=4, v_peak=1.6, stop_s=3.0, turn_s=1.2, lead_s=4.0, tail_s=4.0,
              gait_hz=1.8, bounce=0.02, wobble=0.05, seed=0, name="synthetic", **noise):
    """Rectangular walk with stops and in-place turns at the corners, ending at the start."""
    dt = 1.0 / fs
    T_leg = 2 * leg_len / v_peak
    speed, yaw, env = [], [], []
    psi = 0.0

    def seg(n, s, y, e):
        speed.append(np.full(n, s) if np.isscalar(s) else s)
        yaw.append(np.full(n, y) if np.isscalar(y) else y)
        env.append(np.full(n, e) if np.isscalar(e) else e)

    seg(int(lead_s * fs), 0.0, psi, 0.0)
    for _ in range(legs):
        n = int(T_leg * fs)
        tau = np.arange(n) / n
        seg(n, v_peak * np.sin(np.pi * tau) ** 2, psi, np.sin(np.pi * tau) ** 2)
        n_stop = int(stop_s * fs)
        n_turn = int(turn_s * fs)
        pre = (n_stop - n_turn) // 2
        seg(pre, 0.0, psi, 0.0)
        seg(n_turn, 0.0, psi + (np.pi / 2) * _smoothstep(np.arange(n_turn) / n_turn), 0.0)
        psi += np.pi / 2
        seg(n_stop - pre - n_turn, 0.0, psi, 0.0)
    seg(int(tail_s * fs), 0.0, psi, 0.0)
    speed, yaw, env = map(np.concatenate, (speed, yaw, env))
    t = np.arange(len(speed)) * dt

    z = bounce * np.sin(2 * np.pi * gait_hz * t) * env
    vel = np.column_stack([speed * np.cos(yaw), speed * np.sin(yaw), np.gradient(z, dt)])
    roll = wobble * np.sin(2 * np.pi * 0.9 * t) * env
    pitch = wobble * np.cos(2 * np.pi * gait_hz * t) * env
    return _imu_from_motion(t, vel, _euler_R(roll, pitch, yaw), fs, seed, name, **{**NOISE, **noise})


def make_foot_walk(fs=200.0, legs=4, strides_per_leg=7, stride_len=1.4, stride_T=1.1, stance_frac=0.45,
                   lift=0.12, pitch_amp=0.5, turn_strides=2, lead_s=4.0, tail_s=3.0, seed=0,
                   name="synthetic_foot", **noise):
    """Shoe-mounted IMU walking a rectangle (turning 90 deg over `turn_strides` strides per corner)."""
    dt = 1.0 / fs
    n_st = int(stride_T * stance_frac * fs)
    n_sw = int(stride_T * fs) - n_st
    turns = []
    for leg in range(legs):
        turns += [0.0] * strides_per_leg
        if leg < legs - 1:
            turns += [(np.pi / 2) / turn_strides] * turn_strides
    vx, vy, vz, yaw, pitch, roll = [], [], [], [], [], []
    psi = 0.0

    def still(n):
        for a in (vx, vy, vz, pitch, roll):
            a.append(np.zeros(n))
        yaw.append(np.full(n, psi))

    still(int(lead_s * fs))
    tau = np.arange(n_sw) / n_sw
    T_sw = n_sw * dt
    for dpsi in turns:
        still(n_st)
        y = psi + dpsi * _minjerk(tau)
        ds = stride_len * 30 * tau ** 2 * (1 - tau) ** 2 / T_sw  # d/dt of min-jerk travel
        vx.append(ds * np.cos(y))
        vy.append(ds * np.sin(y))
        vz.append(lift * np.pi * np.sin(2 * np.pi * tau) / T_sw)  # z = lift * sin^2(pi tau)
        pitch.append(-pitch_amp * np.sin(2 * np.pi * tau) * np.sin(np.pi * tau))
        roll.append(0.05 * np.sin(2 * np.pi * tau))
        yaw.append(y)
        psi += dpsi
    still(n_st + int(tail_s * fs))
    vx, vy, vz, yaw, pitch, roll = map(np.concatenate, (vx, vy, vz, yaw, pitch, roll))
    t = np.arange(len(vx)) * dt
    vel = np.column_stack([vx, vy, vz])
    return _imu_from_motion(t, vel, _euler_R(roll, pitch, yaw), fs, seed, name, **{**NOISE, **noise})
