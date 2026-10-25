"""Data loading and sanity checks.

Units after loading: gyro rad/s, accel m/s^2, time in seconds from the first
sample, position m, quaternions [w, x, y, z] body->world.
"""
import warnings
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

from .quat import slerp, qnormalize


@dataclass
class Sequence:
    name: str
    t: np.ndarray
    gyro: np.ndarray
    acc: np.ndarray
    gt_p: np.ndarray = None
    gt_q: np.ndarray = None
    gt_v: np.ndarray = None
    meta: dict = field(default_factory=dict)

    @property
    def fs(self):
        return 1.0 / float(np.median(np.diff(self.t)))


# Column layouts: integers are positions, strings are header names.
EUROC_IMU = {"t": 0, "gyro": [1, 2, 3], "acc": [4, 5, 6], "time_scale": 1e-9}
EUROC_GT = {"t": 0, "p": [1, 2, 3], "q": [4, 5, 6, 7], "v": [8, 9, 10], "time_scale": 1e-9}


def _clean_time(t, *arrays):
    """Sort by time and drop repeated timestamps (common in real loggers)."""
    order = np.argsort(t, kind="stable")
    t = t[order]
    keep = np.concatenate([[True], np.diff(t) > 0])
    return (t[keep],) + tuple(None if a is None else a[order][keep] for a in arrays) + (int((~keep).sum()),)


def _cols(df, spec):
    return df[[df.columns[c] if isinstance(c, int) else c for c in spec]].to_numpy(float)


def load_imu_csv(path, layout=EUROC_IMU, gyro_scale=1.0, acc_scale=1.0):
    """Read an IMU CSV. Use gyro_scale=pi/180 for deg/s, acc_scale=9.80665 for g."""
    df = pd.read_csv(path)
    tcol = df.columns[layout["t"]] if isinstance(layout["t"], int) else layout["t"]
    t = df[tcol].to_numpy(float) * layout.get("time_scale", 1.0)
    t, gyro, acc, n_dup = _clean_time(t, _cols(df, layout["gyro"]) * gyro_scale, _cols(df, layout["acc"]) * acc_scale)
    if n_dup:
        warnings.warn(f"{path}: dropped {n_dup} repeated timestamps")
    return t, gyro, acc


def load_gt_csv(path, layout=EUROC_GT):
    df = pd.read_csv(path)
    tcol = df.columns[layout["t"]] if isinstance(layout["t"], int) else layout["t"]
    t = df[tcol].to_numpy(float) * layout.get("time_scale", 1.0)
    p = _cols(df, layout["p"])
    q = _cols(df, layout["q"]) if "q" in layout else None
    v = _cols(df, layout["v"]) if "v" in layout else None
    t, p, q, v, _ = _clean_time(t, p, q, v)
    return t, p, q, v


def check_timestamps(t, fs_expected=None, gap_factor=3.0):
    """Return a report dict; raise if timestamps are not strictly increasing."""
    dt = np.diff(t)
    if np.any(dt <= 0):
        raise ValueError(f"timestamps not strictly increasing ({int(np.sum(dt <= 0))} violations)")
    med = float(np.median(dt))
    gaps = np.where(dt > gap_factor * med)[0]
    rep = {
        "n": len(t), "fs": 1.0 / med, "median_dt": med, "max_dt": float(dt.max()),
        "n_gaps": int(len(gaps)), "gap_idx": gaps, "duration": float(t[-1] - t[0]),
    }
    if fs_expected is not None:
        rep["fs_ok"] = abs(rep["fs"] - fs_expected) / fs_expected < 0.05
    return rep


def interp_gt(t, gt_t, p, q=None, v=None):
    """Interpolate ground truth onto IMU timestamps (linear p/v, slerp q)."""
    lin = lambda X: np.column_stack([np.interp(t, gt_t, X[:, k]) for k in range(X.shape[1])])
    Q = None
    if q is not None:
        idx = np.clip(np.searchsorted(gt_t, t) - 1, 0, len(gt_t) - 2)
        f = np.clip((t - gt_t[idx]) / (gt_t[idx + 1] - gt_t[idx]), 0, 1)
        Q = np.array([slerp(qnormalize(q[i]), qnormalize(q[i + 1]), fi) for i, fi in zip(idx, f)])
    return lin(p), Q, None if v is None else lin(v)


def resample_uniform(seq, fs=None, max_fill=0.05):
    """Put a sequence with timestamp jitter on a uniform grid (linear interpolation).
    Holes longer than `max_fill` s are NOT filled: a straight line across lost data is motion that never
    happened. Grid points inside a hole are dropped, so the time step jumps there and run_sequence treats
    it as a data gap (Navigator.gap)."""
    fs = fs or seq.fs
    t = np.arange(seq.t[0], seq.t[-1], 1.0 / fs)
    idx = np.clip(np.searchsorted(seq.t, t) - 1, 0, len(seq.t) - 2)
    t = t[(seq.t[idx + 1] - seq.t[idx]) <= max_fill]
    lin = lambda X: None if X is None else np.column_stack([np.interp(t, seq.t, X[:, k]) for k in range(X.shape[1])])
    out = Sequence(seq.name, t - t[0], lin(seq.gyro), lin(seq.acc), lin(seq.gt_p), None, lin(seq.gt_v), dict(seq.meta))
    if seq.gt_q is not None:
        idx = np.clip(np.searchsorted(seq.t, t) - 1, 0, len(seq.t) - 2)
        f = np.clip((t - seq.t[idx]) / (seq.t[idx + 1] - seq.t[idx]), 0, 1)
        out.gt_q = np.array([slerp(seq.gt_q[i], seq.gt_q[i + 1], fi) for i, fi in zip(idx, f)])
    return out


def load_custom(imu_csv, gt_csv=None, imu_layout=EUROC_IMU, gt_layout=None,
                gyro_scale=1.0, acc_scale=1.0, name=None, fs_expected=None):
    """Load any IMU/GT CSV pair by describing its columns (e.g. Embien's traces)."""
    t, gyro, acc = load_imu_csv(imu_csv, imu_layout, gyro_scale, acc_scale)
    rep = check_timestamps(t, fs_expected)
    gt = None
    if gt_csv is not None:
        gt_t, gp, gq, gv = load_gt_csv(gt_csv, gt_layout)
        keep = (t >= gt_t[0]) & (t <= gt_t[-1])  # never extrapolate ground truth
        t, gyro, acc = t[keep], gyro[keep], acc[keep]
        gt = interp_gt(t, gt_t, gp, gq, gv)
    seq = Sequence(name or Path(imu_csv).stem, t - t[0], gyro, acc, meta={"timestamp_report": rep})
    if gt is not None:
        seq.gt_p, seq.gt_q, seq.gt_v = gt
    if rep["n_gaps"]:
        seq = resample_uniform(seq, fs_expected)
    return seq


def load_euroc(seq_dir, fs_expected=200.0):
    """Load a EuRoC MAV sequence directory (the one containing mav0/).

    IMU is cropped to the span covered by the ground truth.
    """
    root = Path(seq_dir) / "mav0"
    t, gyro, acc = load_imu_csv(root / "imu0" / "data.csv")
    gt_t, gp, gq, gv = load_gt_csv(root / "state_groundtruth_estimate0" / "data.csv")
    rep = check_timestamps(t, fs_expected)
    keep = (t >= gt_t[0]) & (t <= gt_t[-1])
    t, gyro, acc = t[keep], gyro[keep], acc[keep]
    P, Q, V = interp_gt(t, gt_t, gp, gq, gv)
    seq = Sequence(Path(seq_dir).name, t - t[0], gyro, acc, P, Q, V, {"timestamp_report": rep})
    if rep["n_gaps"]:
        seq = resample_uniform(seq, fs_expected)
    return seq
