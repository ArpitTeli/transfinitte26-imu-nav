"""Evaluation metrics (position and orientation)."""
import numpy as np

from .quat import qconj, qlog, qmul, quat_to_euler


def path_length(p):
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


def motion_start(t, p_gt, min_dist=0.05):
    """Time (s from start) at which the ground truth has moved min_dist metres."""
    dist = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p_gt, axis=0), axis=1))])
    i = int(np.searchsorted(dist, min_dist))
    return float(t[min(i, len(t) - 1)] - t[0])


def position_metrics(t, p_est, p_gt, horizon=30.0):
    """Position metrics. The horizon error is measured `horizon` seconds after motion starts."""
    err = np.linalg.norm(p_est - p_gt, axis=1)
    dist = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p_gt, axis=0), axis=1))])
    L = float(dist[-1])
    t0 = motion_start(t, p_gt)
    tr = t - t[0] - t0
    return {
        "motion_start_s": t0,
        "max_error_m": float(err.max()),
        "final_error_m": float(err[-1]),
        "rmse_m": float(np.sqrt(np.mean(err ** 2))),
        f"error_at_{horizon:g}s_m": float(np.interp(horizon, tr, err)) if tr[-1] >= horizon else float("nan"),
        "return_to_start_est_m": float(np.linalg.norm(p_est[-1] - p_est[0])),
        "return_to_start_gt_m": float(np.linalg.norm(p_gt[-1] - p_gt[0])),
        "path_length_m": L,
        "drift_pct": float(100 * err[-1] / L) if L > 0 else float("nan"),
    }


def error_vs_time(p_est, p_gt):
    return np.linalg.norm(p_est - p_gt, axis=1)


def error_vs_distance(p_est, p_gt):
    err = error_vs_time(p_est, p_gt)
    dist = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p_gt, axis=0), axis=1))])
    return dist, err


def orientation_errors(q_est, q_gt):
    """Total angle error and per-axis roll/pitch/yaw error, all in degrees."""
    ang = np.array([np.linalg.norm(qlog(qmul(qconj(a), b))) for a, b in zip(q_est, q_gt)])
    e = np.array([quat_to_euler(a) - quat_to_euler(b) for a, b in zip(q_est, q_gt)])
    e = (e + np.pi) % (2 * np.pi) - np.pi
    return np.degrees(ang), np.degrees(e)


def orientation_summary(q_est, q_gt):
    ang, e = orientation_errors(q_est, q_gt)
    return {
        "att_rmse_deg": float(np.sqrt(np.mean(ang ** 2))),
        "roll_rmse_deg": float(np.sqrt(np.mean(e[:, 0] ** 2))),
        "pitch_rmse_deg": float(np.sqrt(np.mean(e[:, 1] ** 2))),
        "yaw_rmse_deg": float(np.sqrt(np.mean(e[:, 2] ** 2))),
        "yaw_final_deg": float(e[-1, 2]),
    }
