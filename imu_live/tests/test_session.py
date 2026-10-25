import socket
import sys
import time
from pathlib import Path

import numpy as np
import pandas as pd
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from imunav.live.protocol import ACC, GYRO, encode  # noqa: E402
from imunav.live.receiver import Receiver  # noqa: E402
from imunav.live.session import PROTOCOL, Session  # noqa: E402
from imunav.live.tracker import TRACKING, LiveTracker  # noqa: E402
from imunav.pipeline import PRESETS  # noqa: E402


def _send(port, t0, seconds, seq0=0, fs=400.0):
    """stream `seconds` of a still phone (accel + gyro events) to the receiver"""
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recs = []
    for k in range(int(seconds * fs)):
        t_ns = int((t0 + k / fs) * 1e9)
        recs += [(ACC, t_ns, 0.0, 0.0, 9.81), (GYRO, t_ns + 700_000, 0.001, 0.0, 0.0)]
    seq = seq0
    for j in range(0, len(recs), 40):
        tx.sendto(encode(seq, recs[j:j + 40]), ("127.0.0.1", port)); seq += 1
        if seq % 20 == 0:
            time.sleep(0.003)
    tx.close()
    return seq


def _wait_rows(rx, n, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end and rx.rec_rows < n:
        time.sleep(0.02)


def _wait_packets(rx, n, timeout=5.0):
    """until all n packets sent so far have been received and processed"""
    end = time.time() + timeout
    while time.time() < end and rx.packets < n:
        time.sleep(0.02)
    time.sleep(0.05)                                # the last packet's rows are written after it is counted


def test_one_file_per_walk_with_markers(tmp_path):
    tr = LiveTracker(PRESETS["shoe"])
    rx = Receiver(tr, port=0, fs=400.0)
    port = rx.sock.getsockname()[1]
    rx.start()
    s = Session(rx, tr, tmp_path)
    assert s.walk[0] == "free" and s.next_code == "A1"
    seq = _send(port, 1000.0, 1.0)
    _wait_rows(rx, 350)
    free_path = s.path

    code, _, shape = s.next_walk()
    assert (code, shape) == ("A1", "cw") and s.path != free_path
    seq = _send(port, 1001.0, 1.0, seq)
    _wait_rows(rx, 350)
    s.marker("tripped at corner 2")
    s.next_walk()                                   # A2
    _send(port, 1002.0, 0.5, seq)
    _wait_rows(rx, 150)
    rx.stop(); s.close()

    files = sorted(tmp_path.glob("*.csv"))
    names = [f.name for f in files]
    assert any(n.endswith("_free.csv") for n in names)
    assert any(n.endswith("_A1.csv") for n in names) and any(n.endswith("_A2.csv") for n in names)
    a1 = pd.read_csv(next(f for f in files if f.name.endswith("_A1.csv")))
    assert list(a1.columns) == ["t", "gx", "gy", "gz", "ax", "ay", "az", "gap", "mx", "my", "mz"]
    assert a1.t.min() >= 1000.9 and len(a1) > 300          # A1 holds A1's data, not the free part
    m = pd.read_csv(next(f for f in files if f.name.endswith("_A1_markers.csv")))
    assert m.walk.tolist() == ["A1"] and m.note.tolist() == ["tripped at corner 2"]
    assert 1001.0 <= m.t_phone.iloc[0] <= 1002.1


def test_phone_stopped_and_started_between_walks(tmp_path):
    """The walker presses Stop at the end of a walk and Start at the next corner. The hole between walks
    is not a data gap of the new walk, and the new walk still calibrates and tracks."""
    tr = LiveTracker(PRESETS["shoe"])
    rx = Receiver(tr, port=0, fs=400.0)
    port = rx.sock.getsockname()[1]
    rx.start()
    s = Session(rx, tr, tmp_path)
    s.next_walk()                                   # A1
    sent = _send(port, 1000.0, 3.0)
    _wait_packets(rx, sent)                         # Stop pressed: every A1 packet has arrived
    s.next_walk()                                   # A2: n pressed while the phone is stopped
    _send(port, 1060.0, 3.0)                        # Start again: new socket, seq from 0, clock kept running
    _wait_rows(rx, 1150)
    snap = tr.snapshot()
    rx.stop(); s.close()
    assert snap["state"] == TRACKING
    assert snap["gaps"] == 0 and rx.lost == 0
    a2 = pd.read_csv(next(tmp_path.glob("*_A2.csv")))
    assert a2.t.min() >= 1060.0                     # nothing the aligner still held from A1
    assert len(a2) > 1100 and (a2.gap.iloc[1:] == 0).all()


def test_protocol_order_start_and_extra_walks(tmp_path):
    class Rx:
        last_t = None
        def set_record(self, path): self.path = path
    class Tr:
        resets = 0
        def reset(self): Tr.resets += 1
    s = Session(Rx(), Tr(), tmp_path, start="B2")
    walked = [s.next_walk()[0] for _ in range(len(PROTOCOL) - 3 + 2)]
    assert walked == ["B2", "C", "D", "E", "X1", "X2"]
    assert Tr.resets == len(walked)                 # every walk recalibrates
    with pytest.raises(ValueError):
        Session(Rx(), Tr(), tmp_path, start="Z9")


def test_track_is_turned_to_point_up():
    """No court in the live view: the track starts at the origin and its first 3 m point up the screen."""
    from imunav.live.session import heading_up
    ang = np.radians(30)                             # the phone's heading is arbitrary
    leg1 = np.column_stack([np.linspace(0, 10, 50) * np.cos(ang), np.linspace(0, 10, 50) * np.sin(ang), np.zeros(50)])
    right = leg1[-1] + np.column_stack([np.linspace(0, 4, 20) * np.sin(ang), -np.linspace(0, 4, 20) * np.cos(ang), np.zeros(20)])
    q = heading_up(np.vstack([leg1, right]) + [5.0, -2.0, 0.0])
    assert np.allclose(q[0], [0, 0]) and np.allclose(q[49], [0, 10], atol=1e-6)
    assert np.allclose(q[-1], [4, 10], atol=1e-6)    # a right turn goes right on screen (not mirrored)
    assert heading_up(leg1[:5]) is None              # under 3 m walked: not turned yet


def test_calibration_feedback_shows_how_still_the_shoe_is():
    """While calibrating, the tracker reports the spread the stillness check sees, so a loose or moving
    phone shows a number above the limit instead of an empty window."""
    from imunav.calib import GYRO_STILL_STD
    rng = np.random.default_rng(0)
    tr = LiveTracker(PRESETS["shoe"])
    for k in range(800):                             # 2 s wobbling like the 25 Oct recordings (0.026 rad/s)
        tr.add(k / 400, rng.normal(0, 0.03, 3), np.array([0, 0, 9.81]) + rng.normal(0, 0.05, 3))
    s = tr.snapshot()
    assert s["state"] == "calibrating" and s["motion"][0] > GYRO_STILL_STD
    for k in range(800, 1700):                       # then strapped tight and standing still
        tr.add(k / 400, rng.normal(0, 0.001, 3), np.array([0, 0, 9.81]) + rng.normal(0, 0.01, 3))
    assert tr.snapshot()["state"] == TRACKING


def test_phone_clock_going_back_restarts_tracking(tmp_path):
    """A phone that reboots (or a second phone) starts its clock again. The receiver must notice, also after a pause,
    or the filter would run with time going backwards."""
    tr = LiveTracker(PRESETS["shoe"])
    rx = Receiver(tr, port=0, fs=400.0)
    port = rx.sock.getsockname()[1]
    rx.start()
    s = Session(rx, tr, tmp_path)
    sent = _send(port, 1000.0, 3.0)
    _wait_packets(rx, sent)
    assert tr.snapshot()["state"] == TRACKING
    time.sleep(0.3)                                  # the phone pauses: everything has been passed on
    sent2 = _send(port, 5.0, 1.0)                    # its clock starts again near zero
    _wait_packets(rx, sent + sent2)
    snap = tr.snapshot()
    rx.stop(); s.close()
    assert snap["state"] != TRACKING                 # recalibrating on the new clock, not tracking backwards
    assert rx.last_t is not None and rx.last_t < 10.0
