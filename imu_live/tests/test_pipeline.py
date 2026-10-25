import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imunav.ins import Strapdown  # noqa: E402
from imunav.io import Sequence  # noqa: E402
from imunav.metrics import position_metrics  # noqa: E402
from imunav.pipeline import Config, run_sequence  # noqa: E402
from imunav.synth import make_walk  # noqa: E402
from imunav.zupt import ZuptDetector, detector_stats, gt_stationary  # noqa: E402

G = 9.80665
IDEAL = dict(gyro_bias=(0, 0, 0), acc_bias=(0, 0, 0), gyro_nd=0, acc_nd=0, gyro_rw=0, acc_rw=0)


def test_zero_input_gives_no_motion():
    n = 4000
    t = np.arange(n) * 0.005
    seq = Sequence("static", t, np.zeros((n, 3)), np.tile([0, 0, G], (n, 1)), np.zeros((n, 3)))
    for mode in ("madgwick", "mahony", "ekf", "ins"):
        r = run_sequence(seq, Config(mode=mode, zupt=False))
        assert np.abs(r.p).max() < 1e-3, mode


def test_constant_velocity():
    n = 2000
    t = np.arange(n) * 0.005
    seq = Sequence("cv", t, np.zeros((n, 3)), np.tile([0, 0, G], (n, 1)), np.zeros((n, 3)))
    s = Strapdown(np.zeros(3), np.array([1.0, 0.5, 0.0]), G)
    q = np.array([1.0, 0, 0, 0])
    for _ in range(n):
        s.step(q, np.array([0, 0, G]), 0.005)
    assert np.allclose(s.p, [1.0 * n * 0.005, 0.5 * n * 0.005, 0.0], atol=1e-6)


def test_ideal_synthetic_path_is_recovered():
    seq = make_walk(**IDEAL)
    r = run_sequence(seq, Config(mode="ins", zupt=False))
    m = position_metrics(r.t, r.p, seq.gt_p)
    assert m["final_error_m"] < 0.02 and m["rmse_m"] < 0.02


def test_zupt_ekf_beats_baseline_and_is_accurate():
    seq = make_walk(seed=21)
    rb = run_sequence(seq, Config(mode="ins", zupt=False))
    rz = run_sequence(seq, Config(mode="ins"))
    base = position_metrics(rb.t, rb.p, seq.gt_p)
    zu = position_metrics(rz.t, rz.p, seq.gt_p)
    assert zu["rmse_m"] < 0.2 * base["rmse_m"]
    assert zu["drift_pct"] < 5.0


def test_detector_has_few_false_positives():
    seq = make_walk(seed=22)
    r = run_sequence(seq, Config(mode="ins"))
    st = detector_stats(r.still, gt_stationary(seq.gt_v))
    # weak spot: slow acceleration ramps look still; the EKF chi-square gate absorbs these
    assert st["false_positive_rate"] < 0.2


def test_pipeline_is_causal():
    """Samples after k must not influence the state at k."""
    seq = make_walk(seed=23)
    k = 6000
    full = run_sequence(seq, Config(mode="ins"))
    head = Sequence("head", seq.t[:k], seq.gyro[:k], seq.acc[:k], seq.gt_p[:k], seq.gt_q[:k], seq.gt_v[:k])
    part = run_sequence(head, Config(mode="ins"))
    assert np.allclose(full.p[:k], part.p, atol=1e-9)


def test_detector_flags_static_not_motion():
    det = ZuptDetector(fs=200)
    rng = np.random.default_rng(0)
    for _ in range(100):
        still = det.update(rng.normal(0, 1e-3, 3), np.array([0, 0, G]) + rng.normal(0, 0.02, 3))
    assert still
    for _ in range(100):
        moving = det.update(np.array([0.5, 0, 0]), np.array([0, 0, G]))
    assert not moving
