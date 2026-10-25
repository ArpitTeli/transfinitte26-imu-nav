import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imunav import calib  # noqa: E402
from imunav.io import check_timestamps, load_custom  # noqa: E402
from imunav.orientation import AttitudeEKF, Madgwick, Mahony  # noqa: E402
from imunav.quat import (R_to_quat, q_from_gravity, qexp, qlog, qmul, quat_to_R,  # noqa: E402
                         yaw_of)

G = 9.80665


def test_quaternion_roundtrips():
    rng = np.random.default_rng(0)
    for _ in range(20):
        rv = rng.normal(size=3)
        q = qexp(rv)
        if np.linalg.norm(rv) < np.pi:
            assert np.allclose(qlog(q), rv, atol=1e-9)
        assert np.allclose(quat_to_R(R_to_quat(quat_to_R(q))), quat_to_R(q), atol=1e-9)
    a, b = qexp([0.3, 0, 0]), qexp([0, 0.4, 0])
    assert np.allclose(quat_to_R(qmul(a, b)), quat_to_R(a) @ quat_to_R(b))


def test_gravity_alignment():
    acc = np.array([1.0, -2.0, 9.5])
    q = q_from_gravity(acc, yaw=0.7)
    assert np.allclose(quat_to_R(q)[2, :], acc / np.linalg.norm(acc))
    assert yaw_of(q) == pytest.approx(0.7, abs=1e-9)


def test_timestamp_checks():
    t = np.arange(0, 10, 0.005)
    rep = check_timestamps(t, 200)
    assert rep["fs_ok"] and rep["n_gaps"] == 0
    t2 = np.delete(t, np.arange(500, 520))
    assert check_timestamps(t2)["n_gaps"] == 1
    with pytest.raises(ValueError):
        check_timestamps(np.array([0.0, 0.1, 0.1, 0.2]))


def test_custom_loader_units(tmp_path):
    t = np.arange(0, 2, 0.01)
    imu = tmp_path / "imu.csv"
    lines = ["time_ms,gx,gy,gz,ax,ay,az"]
    lines += [f"{ti * 1000},{57.29578},0,0,0,0,1.0" for ti in t]
    imu.write_text("\n".join(lines))
    layout = {"t": "time_ms", "gyro": ["gx", "gy", "gz"], "acc": ["ax", "ay", "az"], "time_scale": 1e-3}
    seq = load_custom(imu, imu_layout=layout, gyro_scale=np.pi / 180, acc_scale=G)
    assert seq.gyro[0, 0] == pytest.approx(1.0, abs=1e-4)
    assert seq.acc[0, 2] == pytest.approx(G)
    assert seq.fs == pytest.approx(100.0)


def test_static_calibration_and_six_position():
    rng = np.random.default_rng(1)
    t = np.arange(0, 5, 0.005)
    gyro = np.array([0.01, -0.02, 0.005]) + rng.normal(0, 1e-3, (len(t), 3))
    acc = np.array([0.1, 0.2, G]) + rng.normal(0, 0.02, (len(t), 3))
    c = calib.static_calibration(t, gyro, acc)
    assert np.allclose(c["gyro_bias"], [0.01, -0.02, 0.005], atol=2e-4)
    assert c["looks_static"]

    S_true = np.array([[1.02, 0.01, 0.0], [0.0, 0.98, 0.02], [0.01, 0.0, 1.01]])
    b_true = np.array([0.05, -0.03, 0.08])
    truth = G * np.array([[1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1]], float)
    means = truth @ S_true.T + b_true
    S, b = calib.six_position_fit(means)
    assert np.allclose(S, S_true, atol=1e-9) and np.allclose(b, b_true, atol=1e-9)
    assert np.allclose(calib.apply_acc_cal(means, S, b), truth, atol=1e-9)


def test_gyro_scale_fit():
    M_true = np.array([[1.03, 0.01, 0], [0, 0.97, 0.02], [0.01, 0, 1.0]])
    angles = np.array([np.pi, np.pi, np.pi])
    integrals = np.array([M_true[:, k] * angles[k] for k in range(3)])
    assert np.allclose(calib.gyro_scale_fit(integrals, angles), M_true)


def test_allan_white_noise():
    rng = np.random.default_rng(2)
    fs, sigma = 100.0, 0.01
    x = rng.normal(0, sigma, 400_000)
    taus, adev = calib.allan_deviation(x, fs)
    N = calib.allan_params(taus, adev)["N"]
    assert N == pytest.approx(sigma / np.sqrt(fs), rel=0.1)


@pytest.mark.parametrize("filt", [Madgwick, Mahony, AttitudeEKF])
def test_orientation_filters_converge_and_hold(filt):
    acc = np.array([0.0, 0.0, G])
    f = filt(qexp([0.3, -0.2, 0.0]), g=G, acc_gate=None)
    for _ in range(4000):
        f.update(np.zeros(3), acc, 0.005)
    assert np.allclose(quat_to_R(f.q)[2, :], [0, 0, 1], atol=1e-2)  # Mahony's I-term settles slowly
    q = f.q.copy()
    for _ in range(1000):
        f.update(np.zeros(3), acc, 0.005)
    assert np.allclose(f.q, q, atol=2e-3)
