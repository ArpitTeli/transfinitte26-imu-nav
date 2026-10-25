"""Compass: calibration, heading, and the filter's compass heading updates (imunav/compass.py)."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imunav.compass import (CompassCal, CompassCalibrator, mag_heading_h, north_angle,  # noqa: E402
                            phone_heading, wrap)
from imunav.pipeline import PRESETS, navigator_from_static  # noqa: E402
from imunav.quat import qexp, qmul, qnormalize, quat_to_R, yaw_of  # noqa: E402
from imunav.synth import EARTH_FIELD, magnetometer, make_foot_walk  # noqa: E402

OFFSET, SCALE = np.array([18.0, -31.0, 9.0]), np.array([1.04, 0.97, 1.0])


def _random_q(rng, n):
    q = rng.normal(size=(n, 4))
    return q / np.linalg.norm(q, axis=1, keepdims=True)


def _raw(q, rng, noise=0.3):
    return quat_to_R(q).T @ np.array(EARTH_FIELD) / SCALE + OFFSET + rng.normal(0, noise, 3)


def test_calibration_finds_offset_and_gains_once_every_direction_is_seen():
    rng = np.random.default_rng(1)
    cal = CompassCalibrator()
    for q in _random_q(rng, 60):                  # too few to cover the sphere
        cal.add(_raw(q, rng))
    assert not cal.progress()["done"]
    for q in _random_q(rng, 600):
        cal.add(_raw(q, rng))
    p = cal.progress()
    assert p["done"] and p["directions"] >= 20, p["message"]
    c = p["cal"]
    assert np.allclose(c.offset, OFFSET, atol=1.0)
    assert c.field == pytest.approx(np.linalg.norm(EARTH_FIELD), rel=0.05)
    m = np.array([np.linalg.norm(c.apply(_raw(q, rng, 0.0))) for q in _random_q(rng, 50)])
    assert m.std() / m.mean() < 0.01
    c2 = CompassCal.from_dict(c.to_dict())
    assert np.allclose(c2.offset, c.offset) and c2.field == c.field


def test_turning_about_one_axis_is_not_enough():
    rng = np.random.default_rng(2)
    cal = CompassCalibrator()
    for a in np.linspace(0, 6 * np.pi, 800):     # flat on a table, spun round and round
        cal.add(_raw(np.array([np.cos(a / 2), 0, 0, np.sin(a / 2)]), rng))
    p = cal.progress()
    assert not p["done"] and p["directions"] < 20


def test_heading_and_its_jacobian():
    rng = np.random.default_rng(3)
    f = np.array(EARTH_FIELD)
    for q in _random_q(rng, 20):
        m = quat_to_R(q).T @ f
        psi, _ = north_angle(q, m)
        assert psi == pytest.approx(math.atan2(f[1], f[0]), abs=1e-9)   # north is where the field says
        q2 = qnormalize(qmul(q, qexp(np.array([0.01, -0.02, 0.015]))))
        p0, H, _ = mag_heading_h(q2, m)
        for i in range(3):
            d = np.zeros(3); d[i] = 1e-6
            p1, _, _ = mag_heading_h(qnormalize(qmul(q2, qexp(d))), m)
            assert wrap(p1 - p0) / 1e-6 == pytest.approx(H[i], abs=1e-4)
    # a phone lying flat, top pointing along world +y (90 deg anticlockwise of north at +x): heading 270
    q = np.array([1.0, 0, 0, 0])
    assert phone_heading(q, 0.0)[0] == pytest.approx(270.0)


def _run(seq, mag, compass):
    cfg = PRESETS["shoe"].with_(fs=seq.fs)
    nav, _ = navigator_from_static(seq.t, seq.gyro, seq.acc, cfg, yaw_of(seq.gt_q[0]), seq.gt_p[0],
                                   mag=mag, compass=compass)
    for k in range(1, len(seq.t)):
        nav.step(seq.gyro[k], seq.acc[k], seq.t[k] - seq.t[k - 1], None if mag is None else mag[k])
    return nav


def test_compass_holds_the_heading_when_the_gyro_turns_wrongly():
    # a gyro that reads turns 3 % too large: every 90 deg corner adds 2.7 deg of heading error
    seq = make_foot_walk(fs=200.0, seed=5, legs=6, gyro_scale=(1.0, 1.0, 1.03))
    mag = magnetometer(seq, north=0.7)
    cal = CompassCal(OFFSET, SCALE, np.linalg.norm(EARTH_FIELD))
    err = lambda nav: abs(math.degrees(wrap(yaw_of(nav.q) - yaw_of(seq.gt_q[-1]))))
    plain, aided = _run(seq, None, None), _run(seq, mag, cal)
    assert err(plain) > 8.0
    assert err(aided) < 2.0
    end = lambda nav: np.linalg.norm(nav.p[:2] - seq.gt_p[-1, :2])
    assert end(aided) < end(plain)
    assert aided.ins.n_updates.get("mag", 0) > 50 and aided.ins.n_rejected.get("mag", 0) < 5


def test_recalibrating_mid_walk_takes_the_heading_from_the_compass():
    seq = make_foot_walk(fs=200.0, seed=6, legs=2)
    mag = magnetometer(seq, north=-1.2)
    cal = CompassCal(OFFSET, SCALE, np.linalg.norm(EARTH_FIELD))
    cfg = PRESETS["shoe"].with_(fs=seq.fs)
    nav, _ = navigator_from_static(seq.t, seq.gyro, seq.acc, cfg, yaw_of(seq.gt_q[0]), mag=mag, compass=cal)
    north = nav.north
    # standing still at the end, facing some other way; the old heading is forgotten (yaw0 deliberately wrong)
    k = np.searchsorted(seq.t, seq.t[-1] - 2.5)
    nav2, c = navigator_from_static(seq.t[k:], seq.gyro[k:], seq.acc[k:], cfg, yaw0=2.0, mag=mag[k:], compass=cal,
                                    north=north)
    assert c["compass_yaw"] is not None
    assert abs(math.degrees(wrap(yaw_of(nav2.q) - yaw_of(seq.gt_q[-1])))) < 1.5


def _feed(tr, seq, mag, k0, k1):
    for k in range(k0, k1):
        tr.add(seq.t[k], seq.gyro[k], seq.acc[k], 0.0, None if mag is None else mag[k])


@pytest.mark.parametrize("with_compass", [False, True])
def test_k_recalibrates_and_keeps_the_path(with_compass):
    from imunav.live.tracker import RECALIBRATING, TRACKING, LiveTracker
    seq = make_foot_walk(fs=200.0, seed=8, legs=3, tail_s=4.0)
    mag = magnetometer(seq, north=0.3) if with_compass else None
    tr = LiveTracker(PRESETS["shoe"], compass=CompassCal(OFFSET, SCALE, np.linalg.norm(EARTH_FIELD)) if with_compass else None)
    k_tail = len(seq.t) - int(3.5 * seq.fs)                 # the walker has stopped and stands still
    _feed(tr, seq, mag, 0, k_tail)
    before = tr.snapshot()
    assert before["state"] == TRACKING
    tr.recalibrate_keep()
    assert tr.status()["state"] == RECALIBRATING
    _feed(tr, seq, mag, k_tail, len(seq.t))
    after = tr.snapshot()
    assert after["state"] == TRACKING and after["recalibrations"] == 1
    assert after["last_recal"] == ("compass" if with_compass else "start direction")
    assert tr.gen == 2                                      # the path was extended, never replaced
    assert len(after["p"]) >= len(before["p"]) and np.allclose(after["p"][:len(before["p"])], before["p"])
    assert after["distance"] == pytest.approx(before["distance"], abs=0.05) and after["steps"] == before["steps"]
    # heading after K: from the compass it is right; without one, the walker is assumed to face the start way
    yaw_true, yaw_start = yaw_of(seq.gt_q[-1]), yaw_of(seq.gt_q[0])
    want = yaw_true if with_compass else yaw_start
    got = yaw_of(tr.nav.q) + yaw_start                      # the tracker starts at yaw 0 = the start heading
    assert abs(math.degrees(wrap(got - want))) < 2.0


def test_tracker_calibrates_the_compass_from_figure_8_readings():
    from imunav.live.tracker import COMPASS_CALIBRATING, COMPASS_OK, COMPASS_UNCALIBRATED, LiveTracker
    rng = np.random.default_rng(9)
    tr = LiveTracker(PRESETS["shoe"])
    tr.add_mag(0.0, _raw(_random_q(rng, 1)[0], rng))
    assert tr.status()["compass"]["state"] == COMPASS_UNCALIBRATED
    tr.calibrate_compass()
    assert tr.status()["compass"]["state"] == COMPASS_CALIBRATING
    for i, q in enumerate(_random_q(rng, 900)):
        tr.add_mag(0.02 * (i + 1), _raw(q, rng))
    c = tr.status()["compass"]
    assert c["state"] == COMPASS_OK and c["progress"]["done"] and tr.compass_cal is not None
    assert np.allclose(tr.compass_cal.offset, OFFSET, atol=1.0)
