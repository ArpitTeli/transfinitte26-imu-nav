"""Result plots. Every function saves a PNG and returns its path."""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .metrics import error_vs_distance, error_vs_time, orientation_errors  # noqa: E402


def _save(fig, out):
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def trajectory_2d(results, p_gt, out):
    fig, ax = plt.subplots(figsize=(6, 6))
    if p_gt is not None:
        ax.plot(p_gt[:, 0], p_gt[:, 1], "k-", lw=2, label="ground truth")
    for r in results:
        ax.plot(r.p[:, 0], r.p[:, 1], lw=1.2, label=r.name)
    ax.set(xlabel="x [m]", ylabel="y [m]", title="Top-down trajectory", aspect="equal")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    return _save(fig, out)


def trajectory_3d(results, p_gt, out):
    fig = plt.figure(figsize=(6, 6))
    ax = fig.add_subplot(projection="3d")
    if p_gt is not None:
        ax.plot(*p_gt.T, "k-", lw=2, label="ground truth")
    for r in results:
        ax.plot(*r.p.T, lw=1, label=r.name)
    ax.set(xlabel="x [m]", ylabel="y [m]", zlabel="z [m]", title="3D trajectory")
    ax.legend(fontsize=8)
    return _save(fig, out)


def error_time(results, p_gt, out):
    fig, ax = plt.subplots(figsize=(7, 4))
    for r in results:
        ax.plot(r.t - r.t[0], error_vs_time(r.p, p_gt), label=r.name)
    ax.set(xlabel="time [s]", ylabel="position error [m]", title="Error vs time")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    return _save(fig, out)


def error_distance(results, p_gt, out):
    fig, ax = plt.subplots(figsize=(7, 4))
    for r in results:
        d, e = error_vs_distance(r.p, p_gt)
        ax.plot(d, e, label=r.name)
    ax.set(xlabel="distance travelled [m]", ylabel="position error [m]", title="Error vs distance")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    return _save(fig, out)


def orientation_error_plot(results, q_gt, out):
    fig, axs = plt.subplots(3, 1, figsize=(7, 7), sharex=True)
    for r in results:
        _, e = orientation_errors(r.q, q_gt)
        for k in range(3):
            axs[k].plot(r.t - r.t[0], e[:, k], label=r.name)
    for ax, nm in zip(axs, ["roll", "pitch", "yaw"]):
        ax.set_ylabel(f"{nm} error [deg]")
        ax.grid(alpha=0.3)
    axs[0].legend(fontsize=8)
    axs[-1].set_xlabel("time [s]")
    return _save(fig, out)


def zupt_plot(t, gyro, acc, still, gt_still, out):
    fig, axs = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
    axs[0].plot(t, np.linalg.norm(gyro, axis=1), lw=0.6)
    axs[0].set_ylabel("|gyro| [rad/s]")
    axs[1].plot(t, np.linalg.norm(acc, axis=1), lw=0.6)
    axs[1].set_ylabel("|acc| [m/s²]")
    for ax in axs:
        ax.fill_between(t, 0, 1, where=still, transform=ax.get_xaxis_transform(), color="tab:green",
                        alpha=0.25, label="detected still")
        if gt_still is not None:
            ax.fill_between(t, 0, 0.08, where=gt_still, transform=ax.get_xaxis_transform(), color="k",
                            alpha=0.5, label="ground-truth still")
    axs[0].legend(fontsize=8, loc="upper right")
    axs[1].set_xlabel("time [s]")
    return _save(fig, out)
