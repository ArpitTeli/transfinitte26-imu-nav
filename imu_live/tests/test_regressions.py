"""One test per bug fixed in the review pass."""
import io
import sys
import warnings
import zipfile
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from imunav import calib  # noqa: E402
from imunav.io import load_custom  # noqa: E402
from imunav.metrics import position_metrics  # noqa: E402
from imunav.pipeline import Config, make_navigator, run_sequence  # noqa: E402
from imunav.quat import q_from_gravity, quat_to_R  # noqa: E402
from imunav.synth import make_walk  # noqa: E402

G = 9.80665
LAYOUT = {"t": "t", "gyro": ["gx", "gy", "gz"], "acc": ["ax", "ay", "az"]}
GT_LAYOUT = {"t": "t", "p": ["x", "y", "z"]}


def test_sample_rate_is_taken_from_data():
    seq = make_walk(fs=100.0, seed=3)
    nav, _ = make_navigator(seq, Config(mode="ins"))
    assert nav.cfg.fs == pytest.approx(100.0)
    assert nav.det.n == 25  # 0.25 s window at 100 Hz
    r = run_sequence(seq, Config(mode="ins"))
    assert position_metrics(r.t, r.p, seq.gt_p)["drift_pct"] < 5.0


def test_error_horizon_counts_from_motion_start():
    t = np.arange(0, 60, 0.01)
    p_gt = np.zeros((len(t), 3))
    p_gt[:, 0] = np.clip(t - 10, 0, None)          # still for 10 s, then 1 m/s
    p_est = p_gt.copy()
    p_est[:, 1] = np.clip(t - 10, 0, None) * 0.1   # error grows 0.1 m/s once moving
    m = position_metrics(t, p_est, p_gt)
    assert m["motion_start_s"] == pytest.approx(10.05, abs=0.02)
    assert m["error_at_30s_m"] == pytest.approx(3.0, abs=0.02)


def test_custom_loader_crops_to_ground_truth_and_drops_duplicates(tmp_path):
    t = np.round(np.arange(0, 10, 0.01), 6)
    t = np.insert(t, 300, t[300])                   # a repeated timestamp
    imu = pd.DataFrame({"t": t, "gx": 0.0, "gy": 0.0, "gz": 0.0, "ax": 0.0, "ay": 0.0, "az": G})
    imu.to_csv(tmp_path / "imu.csv", index=False)
    tg = np.linspace(2, 8, 61)
    pd.DataFrame({"t": tg, "x": tg, "y": 0.0, "z": 0.0}).to_csv(tmp_path / "gt.csv", index=False)
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        seq = load_custom(tmp_path / "imu.csv", tmp_path / "gt.csv", LAYOUT, GT_LAYOUT)
    assert any("repeated timestamps" in str(x.message) for x in w)
    assert np.all(np.diff(seq.t) > 0)
    assert seq.t[-1] == pytest.approx(6.0, abs=0.011)             # cropped to GT span 2..8 s
    assert np.allclose(seq.gt_p[:, 0], seq.t + 2.0, atol=1e-6)    # no flat extrapolated ends


def test_allan_reads_random_walk_on_its_slope():
    rng = np.random.default_rng(4)
    fs, n_sigma, k_rw = 100.0, 1e-3, 1e-4
    n = 1_000_000
    x = rng.normal(0, n_sigma * np.sqrt(fs), n) + np.cumsum(rng.normal(0, k_rw / np.sqrt(fs), n))
    p = calib.allan_params(*calib.allan_deviation(x, fs))
    assert p["N"] == pytest.approx(n_sigma, rel=0.15)
    assert p["K"] == pytest.approx(k_rw, rel=0.35)


def test_static_check_accepts_large_bias():
    t = np.arange(0, 3, 0.005)
    gyro = np.full((len(t), 3), 0.08)               # big but constant bias
    acc = np.tile([0, 0, G], (len(t), 1))
    assert calib.static_calibration(t, gyro, acc)["looks_static"]
    gyro_moving = 0.3 * np.sin(2 * np.pi * t)[:, None] * np.ones(3)
    assert not calib.static_calibration(t, gyro_moving, acc)["looks_static"]


def test_gravity_alignment_with_x_axis_vertical():
    q = q_from_gravity(np.array([G, 0.0, 0.01]))
    assert np.all(np.isfinite(q))
    assert np.allclose(quat_to_R(q)[2, :], np.array([G, 0, 0.01]) / np.linalg.norm([G, 0, 0.01]))


def test_initial_velocity_from_ground_truth():
    seq = make_walk(seed=5)
    seq.gt_v = seq.gt_v.copy()
    seq.gt_v[0] = [0.5, 0.0, 0.0]
    nav, _ = make_navigator(seq, Config(mode="ins"))
    assert np.allclose(nav.v, [0.5, 0, 0])


def test_chi2_ppf_matches_scipy():
    from scipy.stats import chi2
    from imunav.ins import chi2_ppf
    for k in (1, 2, 3, 6):
        for p in (0.9, 0.99, 0.999, 0.9999):
            assert chi2_ppf(p, k) == pytest.approx(chi2.ppf(p, k), rel=1e-9)


def test_shoe_preset_recovers_from_bad_tilt_on_a_still_phone():
    """Filter starts 25 deg off in tilt on a phone lying still: it must not run away."""
    from imunav.pipeline import PRESETS, Navigator
    from imunav.quat import qexp
    cfg = PRESETS["shoe"].with_(fs=400.0)
    nav = Navigator(cfg, np.zeros(3), G, qexp([0.44, 0.0, 0.0]), np.zeros(3))
    rng = np.random.default_rng(0)
    for _ in range(4000):  # 10 s still
        nav.step(rng.normal(0, 0.002, 3), np.array([0, 0, G]) + rng.normal(0, 0.02, 3), 1 / 400)
    assert np.linalg.norm(nav.v) < 0.05
    assert np.linalg.norm(nav.p) < 2.0
    assert np.allclose(quat_to_R(nav.q)[2, :], [0, 0, 1], atol=0.02)
