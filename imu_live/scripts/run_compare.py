"""Run every filter/ZUPT combination on one sequence; write tables and plots.

Synthetic:        python scripts/run_compare.py --synthetic
Synthetic mean:   python scripts/run_compare.py --synthetic --seeds 10 11 12 13 14
EuRoC:            python scripts/run_compare.py --euroc data/euroc/MH_01_easy
Custom CSV:       python scripts/run_compare.py --imu imu.csv --gt gt.csv --layout configs/layout.json
"""
import argparse
import json
import sys
from pathlib import Path

import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imunav import plots  # noqa: E402
from imunav.io import load_custom, load_euroc  # noqa: E402
from imunav.metrics import orientation_summary, position_metrics  # noqa: E402
from imunav.pipeline import Config, run_sequence  # noqa: E402
from imunav.synth import make_walk  # noqa: E402
from imunav.zupt import detector_stats, gt_stationary  # noqa: E402

# Settings below were tuned on synthetic seeds 1-3 (scripts/tune.py); held-out seeds are 10+.
CONFIGS = [
    ("madgwick, no ZUPT", Config(mode="madgwick", beta=0.002, zupt_res_thr=0.1, zupt=False)),
    ("madgwick + ZUPT", Config(mode="madgwick", beta=0.002, zupt_res_thr=0.1)),
    ("mahony + ZUPT", Config(mode="mahony", kp=0.2, zupt_res_thr=0.1)),
    ("att-EKF + ZUPT", Config(mode="ekf", acc_always=True, att_p_bias=1e-8)),
    ("INS-EKF, no ZUPT", Config(mode="ins", zupt=False)),
    ("INS-EKF + ZUPT", Config(mode="ins")),
]


def evaluate(seq, with_orientation=True):
    rows, results = [], []
    for label, cfg in CONFIGS:
        r = run_sequence(seq, cfg, label)
        results.append(r)
        row = {"filter": label, **position_metrics(r.t, r.p, seq.gt_p)}
        if with_orientation and seq.gt_q is not None:
            row.update(orientation_summary(r.q, seq.gt_q))
        row["zupt_rejected"] = r.n_rejected.get("zupt", 0)
        rows.append(row)
    return pd.DataFrame(rows), results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--synthetic", action="store_true")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--seeds", type=int, nargs="*", help="synthetic: average over these seeds, no plots")
    ap.add_argument("--euroc")
    ap.add_argument("--imu")
    ap.add_argument("--gt")
    ap.add_argument("--layout", help="JSON with imu_layout, gt_layout, gyro_scale, acc_scale")
    ap.add_argument("--out", default="results")
    a = ap.parse_args()
    pd.set_option("display.width", 200)

    if a.synthetic and a.seeds:
        allr = pd.concat([evaluate(make_walk(seed=s), False)[0].assign(seed=s) for s in a.seeds])
        Path(a.out).mkdir(parents=True, exist_ok=True)
        allr.to_csv(Path(a.out) / "synthetic_heldout_all.csv", index=False)
        mean = allr.groupby("filter", sort=False).mean(numeric_only=True).drop(columns="seed")
        mean.to_csv(Path(a.out) / "synthetic_heldout_mean.csv")
        print(f"mean over seeds {a.seeds}")
        print(mean[["final_error_m", "rmse_m", "error_at_30s_m", "return_to_start_est_m", "drift_pct"]].round(2))
        return

    if a.synthetic:
        seq = make_walk(seed=a.seed)
    elif a.euroc:
        seq = load_euroc(a.euroc)
    else:
        if not (a.imu and a.gt):
            raise SystemExit("give --synthetic, --euroc DIR, or both --imu and --gt (the comparison needs ground truth)")
        kw = {}
        if a.layout:
            with open(a.layout) as f:
                kw = json.load(f)
        seq = load_custom(a.imu, a.gt, **kw)
    out = Path(a.out) / seq.name
    out.mkdir(parents=True, exist_ok=True)
    print(f"{seq.name}: {len(seq.t)} samples, {seq.fs:.1f} Hz, {seq.t[-1]:.1f} s")

    df, results = evaluate(seq)
    df.to_csv(out / "metrics.csv", index=False)
    print(df.round(3).to_string(index=False))

    ref = results[-1]
    plots.trajectory_2d(results, seq.gt_p, out / "trajectory_2d.png")
    plots.trajectory_3d(results, seq.gt_p, out / "trajectory_3d.png")
    plots.error_time(results, seq.gt_p, out / "error_vs_time.png")
    plots.error_distance(results, seq.gt_p, out / "error_vs_distance.png")
    if seq.gt_q is not None:
        plots.orientation_error_plot(results[1:4], seq.gt_q, out / "orientation_error.png")
    gt_still = gt_stationary(seq.gt_v) if seq.gt_v is not None else None
    plots.zupt_plot(seq.t, seq.gyro, seq.acc, ref.still, gt_still, out / "zupt_detector.png")
    if gt_still is not None:
        print("ZUPT detector:", {k: round(v, 3) for k, v in detector_stats(ref.still, gt_still).items()})
    print("plots and metrics.csv written to", out)


if __name__ == "__main__":
    main()
