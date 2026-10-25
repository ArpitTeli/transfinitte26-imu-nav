import socket
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imunav.live.align import StreamAligner  # noqa: E402
from imunav.live.protocol import ACC, GYRO, MAG, encode, decode  # noqa: E402
from imunav.live.recording import load_recording  # noqa: E402
from imunav.live.receiver import Receiver  # noqa: E402
from imunav.live.tracker import TRACKING, LiveTracker  # noqa: E402
from imunav.pipeline import PRESETS  # noqa: E402
from imunav.synth import make_foot_walk  # noqa: E402


def test_protocol_roundtrip_and_rejects_garbage():
    recs = [(ACC, 123456789012, 0.1, -0.2, 9.8), (GYRO, 123456790000, 1.0, 2.0, 3.0)]
    seq, out = decode(encode(7, recs))
    assert seq == 7 and len(out) == 2
    assert out[0][:2] == recs[0][:2] and np.allclose(out[1][2:], recs[1][2:])
    with pytest.raises(ValueError):
        decode(b"XXXX" + encode(1, recs)[4:])
    with pytest.raises(ValueError):
        decode(encode(1, recs)[:-3])


def test_aligner_interpolates_offset_streams():
    al = StreamAligner(fs=100.0)
    f = lambda t: np.array([t, 2 * t, -t])
    for k in range(200):
        t = k / 97.0                      # accel at ~97 Hz
        al.push(ACC, t, f(t))
        al.push(GYRO, t + 0.003, 3 * f(t + 0.003))  # gyro offset by 3 ms
    out = al.pop_ready()
    T = np.array([o[0] for o in out])
    assert np.allclose(np.diff(T), 0.01)
    assert all(np.allclose(a, f(t), atol=1e-9) and np.allclose(g, 3 * f(t), atol=1e-9) for t, g, a in out)
    al.push(ACC, 0.5, f(0.5))            # older than the last event
    assert al.n_late == 1


def _feed(tracker, seq):
    al = StreamAligner(400.0)
    for k in range(len(seq.t)):
        al.push(ACC, seq.t[k], seq.acc[k])
        al.push(GYRO, seq.t[k] + 0.0007, seq.gyro[k])
        for t, g, a in al.pop_ready():
            tracker.add(t, g, a)


def test_live_tracker_follows_synthetic_shoe_walk():
    seq = make_foot_walk(fs=400.0, seed=31)
    tr = LiveTracker(PRESETS["shoe"])
    _feed(tr, seq)
    s = tr.snapshot()
    assert s["state"] == TRACKING
    path_len = np.sum(np.linalg.norm(np.diff(seq.gt_p[:, :2], axis=0), axis=1))
    assert s["distance"] == pytest.approx(path_len, rel=0.05)
    assert np.linalg.norm(s["p"][-1, :2] - seq.gt_p[-1, :2]) < 0.02 * path_len
    n_strides = 4 * 7 + 3 * 2
    assert abs(s["steps"] - n_strides) <= 2


def test_tracker_waits_for_stillness():
    tr = LiveTracker(PRESETS["shoe"])
    rng = np.random.default_rng(0)
    for k in range(2000):  # 5 s of shaking
        tr.add(k / 400, rng.normal(0, 1.0, 3), np.array([0, 0, 9.8]) + rng.normal(0, 2.0, 3))
    assert tr.snapshot()["state"] != TRACKING


def test_udp_receiver_end_to_end(tmp_path):
    seq = make_foot_walk(fs=400.0, seed=32, strides_per_leg=2, legs=1, lead_s=3.0)
    tr = LiveTracker(PRESETS["shoe"])
    rec = tmp_path / "rec.csv"
    rx = Receiver(tr, port=0, fs=400.0, record_path=rec)
    port = rx.sock.getsockname()[1]
    rx.start()
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recs = []
    for k in range(len(seq.t)):
        t_ns = 10**12 + int(seq.t[k] * 1e9)
        recs += [(ACC, t_ns, *seq.acc[k]), (GYRO, t_ns + 700_000, *seq.gyro[k])]
        if k % 8 == 0:
            recs.append((MAG, t_ns + 300_000, 20.0, -5.0, -40.0))     # 50 Hz magnetometer
    for i, j in enumerate(range(0, len(recs), 20)):
        if i == 5:
            continue  # one lost packet
        tx.sendto(encode(i, recs[j:j + 20]), ("127.0.0.1", port))
        if i % 50 == 0:
            time.sleep(0.005)
    deadline = time.time() + 10
    while time.time() < deadline and (tr.snapshot()["state"] != TRACKING or rx.packets < i):
        time.sleep(0.05)
    time.sleep(0.3)
    rx.stop()
    assert rx.lost == 1
    s = tr.snapshot()
    assert s["state"] == TRACKING and s["distance"] > 2.0
    assert rec.read_text().splitlines()[0] == "t,gx,gy,gz,ax,ay,az,gap,mx,my,mz"
    df, info = load_recording(str(rec))
    assert info["has_mag"] and np.allclose(df[["mx", "my", "mz"]].dropna().iloc[-1], [20.0, -5.0, -40.0])
    assert s["compass"]["state"] == 1                   # readings arrive, but no calibration yet


def test_aligner_does_not_interpolate_across_a_hole():
    al = StreamAligner(fs=400.0, max_gap=0.05)
    f = lambda t: np.array([np.sin(7 * t), np.cos(5 * t), t])
    times = [k / 400 for k in range(4000) if not (4.0 <= k / 400 < 6.5)]   # 2.5 s of lost data
    for t in times:
        al.push(ACC, t, f(t))
        al.push(GYRO, t + 0.0007, f(t + 0.0007))
    out = al.pop_ready_gaps()
    T = np.array([o[0] for o in out]); gaps = np.array([o[3] for o in out])
    assert not np.any((T > 4.0) & (T < 6.5))                       # nothing invented inside the hole
    assert (gaps > 0).sum() == 1 and gaps.max() == pytest.approx(2.5, abs=0.01)
    assert al.n_gaps == 1 and al.gap_time == pytest.approx(2.5, abs=0.01)
    k = int(np.argmax(gaps > 0))
    assert np.allclose(out[k][2], f(out[k][0]), atol=1e-3)         # first sample after the hole is real data


@pytest.mark.parametrize("seed,lo,dur", [(33, 30.0, 0.2), (34, 23.1, 0.4), (33, 5.0, 0.4)])
def test_short_hole_mid_swing_does_not_corrupt_tilt(seed, lo, dur):
    """A short hole mid-swing loses 20-40 deg of foot pitch. The filter must re-level at the next stance
    (it used to keep the wrong tilt, reject every ZUPT and double the distance)."""
    seq = make_foot_walk(fs=400.0, seed=seed)
    hole = (seq.t >= lo) & (seq.t < lo + dur)
    tr = LiveTracker(PRESETS["shoe"])
    al = StreamAligner(400.0)
    for k in np.flatnonzero(~hole):
        al.push(ACC, seq.t[k], seq.acc[k])
        al.push(GYRO, seq.t[k] + 0.0007, seq.gyro[k])
        for t, g, a, gap in al.pop_ready_gaps():
            tr.add(t, g, a, gap)
    d = np.linalg.norm(np.diff(seq.gt_p[:, :2], axis=0), axis=1)
    i0, i1 = np.searchsorted(seq.t, [lo, lo + dur])
    truth = d.sum() - d[i0:i1].sum()
    assert tr.snapshot()["distance"] == pytest.approx(truth, rel=0.05)


def test_tracker_survives_a_gap_mid_walk():
    """A 3 s hole during walking must not run away (the old aligner integrated a straight line through it)."""
    seq = make_foot_walk(fs=400.0, seed=33)
    hole = (seq.t >= 12.0) & (seq.t < 15.0)
    tr = LiveTracker(PRESETS["shoe"])
    al = StreamAligner(400.0)
    for k in np.flatnonzero(~hole):
        al.push(ACC, seq.t[k], seq.acc[k])
        al.push(GYRO, seq.t[k] + 0.0007, seq.gyro[k])
        for t, g, a, gap in al.pop_ready_gaps():
            tr.add(t, g, a, gap)
    s = tr.snapshot()
    path_len = np.sum(np.linalg.norm(np.diff(seq.gt_p[:, :2], axis=0), axis=1))
    i0, i1 = np.searchsorted(seq.t, [12.0, 15.0])
    lost = np.sum(np.linalg.norm(np.diff(seq.gt_p[i0:i1, :2], axis=0), axis=1))
    assert s["gaps"] == 1 and s["gap_time"] == pytest.approx(3.0, abs=0.02)
    assert s["distance"] < path_len                     # distance walked during the hole is not invented
    assert s["distance"] > 0.8 * (path_len - lost)
    assert s["speed"] < 0.5                             # ends standing still, not running away
