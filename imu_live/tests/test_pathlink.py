"""The path sent back to the phone (pathlink.PathSender, PTH1 packets) is the path the laptop draws."""
import socket
import sys
import time
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imunav.live import pathlink  # noqa: E402
from imunav.live.pathlink import SPACING, PathSender  # noqa: E402
from imunav.live.protocol import (ACC, CALIBRATING_STATE, GYRO, MAX_POINTS, TRACKING_STATE, decode_path,  # noqa: E402
                                  encode, encode_path)
from imunav.live.receiver import Receiver  # noqa: E402
from imunav.live.session import heading_up  # noqa: E402
from imunav.live.tracker import TRACKING, LiveTracker  # noqa: E402
from imunav.pipeline import PRESETS  # noqa: E402
from imunav.synth import make_foot_walk  # noqa: E402


def test_path_packet_roundtrip_and_rejects_garbage():
    pts = [(0.5 * i, -0.25 * i) for i in range(MAX_POINTS)]
    data = encode_path(gen=3, state=1, flags=1, steps=12, distance=15.5, speed=1.25, motion=-1.0, cur=(1.5, 2.5),
                       n_total=400, start=200, points=pts)
    assert len(data) <= 1400
    d = decode_path(data)
    assert (d["gen"], d["state"], d["flags"], d["steps"], d["n_total"], d["start"], d["count"]) == \
           (3, 1, 1, 12, 400, 200, MAX_POINTS)
    assert d["distance"] == pytest.approx(15.5) and (d["x"], d["y"]) == (1.5, 2.5) and np.allclose(d["points"], pts)
    with pytest.raises(ValueError):
        decode_path(b"IMU1" + data[4:])
    with pytest.raises(ValueError):
        decode_path(data[:-3])
    with pytest.raises(ValueError):          # points past the end of the path
        decode_path(encode_path(1, 1, 0, 0, 0, 0, -1, (0, 0), n_total=10, start=5, points=pts[:6]))


class FakePhone:
    """Streams like IMU Stream and rebuilds the path from the packets that come back, as the app does (LivePath.kt).
    Drops a share of them, as Wi-Fi does."""

    def __init__(self, port, drop=0.0, seed=0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
        self.sock.settimeout(0.02)
        self.addr = ("127.0.0.1", port)
        self.rng = np.random.default_rng(seed)
        self.drop = drop
        self.gen, self.pts, self.last, self.got = None, {}, None, 0

    def send(self, i, recs):
        self.sock.sendto(encode(i, recs), self.addr)

    def listen(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            try:
                data, _ = self.sock.recvfrom(2048)
            except (socket.timeout, ConnectionResetError):
                continue
            if self.rng.random() < self.drop:
                continue
            d = decode_path(data)
            self.got += 1
            if d["gen"] != self.gen:
                self.gen, self.pts = d["gen"], {}
            for i, p in enumerate(d["points"]):
                self.pts[d["start"] + i] = p
            self.last = d

    def path(self):
        n = self.last["n_total"]
        return np.array([self.pts[i] for i in range(n)]) if all(i in self.pts for i in range(n)) else None


def _records(seq, t0_ns=10**12):
    recs = []
    for k in range(len(seq.t)):
        t_ns = t0_ns + int(seq.t[k] * 1e9)
        recs += [(ACC, t_ns, *seq.acc[k]), (GYRO, t_ns + 700_000, *seq.gyro[k])]
    return recs


@pytest.mark.parametrize("drop", [0.0, 0.3])
def test_phone_gets_the_path_the_laptop_draws(drop, monkeypatch):
    monkeypatch.setattr(pathlink, "LINK_S", 0.5)
    seq = make_foot_walk(fs=400.0, seed=33)
    tr = LiveTracker(PRESETS["shoe"])
    rx = Receiver(tr, port=0, fs=400.0)
    rx.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
    tx = PathSender(rx, tr, hz=50.0)
    rx.start()
    tx.start()
    phone = FakePhone(rx.sock.getsockname()[1], drop)
    try:
        recs = _records(seq)
        for i, j in enumerate(range(0, len(recs), 20)):
            phone.send(i, recs[j:j + 20])
            if i % 50 == 0:
                phone.listen(0.005)
        n_sent = i + 1
        deadline = time.time() + 20
        while time.time() < deadline and rx.packets < n_sent:
            phone.listen(0.05)
        phone.listen(0.3)                         # the last packets, and repairs (the link stays up 0.5 s)
        s = tr.snapshot()
        assert s["state"] == TRACKING and rx.packets == n_sent
        q = heading_up(s["p"])                    # what the laptop draws
        got = phone.path()
        assert got is not None and phone.gen == tx.gen and len(got) > 100
        # every point the phone has is a point of the laptop's path, about SPACING apart
        d = np.linalg.norm(q[:, None, :] - got[None, :, :], axis=2).min(0)
        assert d.max() < 1e-4
        step = np.linalg.norm(np.diff(got, axis=0), axis=1)
        assert step.min() >= SPACING - 1e-4 and step.max() < 3 * SPACING
        last = phone.last
        assert np.allclose([last["x"], last["y"]], q[-1], atol=1e-4)
        assert last["state"] == TRACKING_STATE and last["steps"] == s["steps"]
        assert last["distance"] == pytest.approx(s["distance"], rel=1e-6)

        # the phone stops streaming: nothing more is sent
        time.sleep(0.6)
        n = tx.sent
        time.sleep(0.3)
        assert tx.sent == n

        # recalibrate: the phone is told to drop the path
        gen = phone.gen
        tr.reset()
        still = [(ACC, 10**12 + int((seq.t[-1] + 1 + k / 400) * 1e9), 0.0, 0.0, 9.81) for k in range(20)]
        phone.send(n_sent, still)
        phone.listen(0.2)
        assert phone.gen != gen and phone.last["n_total"] == 0 and phone.last["state"] == CALIBRATING_STATE
    finally:
        tx.stop()
        rx.stop()
        phone.sock.close()
