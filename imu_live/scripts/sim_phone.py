"""Pretend to be the phone: stream a synthetic shoe-mounted walk to the live app over UDP, and report the path the
laptop sends back (as IMU Stream draws it).

Run:  Fusion Player, Live from the phone (one terminal)
      python scripts/sim_phone.py     (another terminal)
Accel and gyro are sent as separate, slightly offset event streams, like Android does, with a 50 Hz
magnetometer. --compass-cal first turns the phone through every orientation for 25 s, as for Calibrate compass
(click it in Fusion Player before starting this). --turn-error 0.03 gives the gyro a 3 % turn-rate error.
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imunav.live.protocol import ACC, GYRO, MAG, MAX_RECORDS, TRACKING_STATE, decode_path, encode  # noqa: E402
from imunav.quat import qexp, qmul, qnormalize, quat_to_R  # noqa: E402
from imunav.synth import EARTH_FIELD, magnetometer, make_foot_walk  # noqa: E402

MAG_OFFSET, MAG_SCALE = (18.0, -31.0, 9.0), (1.04, 0.97, 1.0)   # the simulated phone's own magnetism


def figure8(seconds, rng, fs=50.0):
    """raw magnetometer readings of a phone turned over and around (a slowly wandering spin, 2 rad/s)"""
    q, axis, out = np.array([1.0, 0, 0, 0]), rng.normal(size=3), []
    for _ in range(int(seconds * fs)):
        axis = axis + rng.normal(0, 0.3, 3)
        w = 2.0 * axis / np.linalg.norm(axis)
        q = qnormalize(qmul(q, qexp(w / fs)))
        m = quat_to_R(q).T @ np.array(EARTH_FIELD) / np.array(MAG_SCALE) + np.array(MAG_OFFSET)
        out.append(m + rng.normal(0, 0.3, 3))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--fs", type=float, default=400.0)
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed factor")
    ap.add_argument("--loss", type=float, default=0.0, help="fraction of packets to drop")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--compass-cal", action="store_true", help="first 25 s of figure-8 for the compass calibration")
    ap.add_argument("--turn-error", type=float, default=0.0, help="gyro turn-rate (z) gain error, e.g. 0.03")
    a = ap.parse_args()

    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    back = dict(packets=0, gen=None, points={}, last=None)    # the path the laptop sends back

    def read_back():
        while True:
            try:
                d = decode_path(sock.recv(2048))
            except (BlockingIOError, ConnectionResetError, ValueError):
                return
            back["packets"] += 1
            if d["gen"] != back["gen"]:
                back["gen"], back["points"] = d["gen"], {}
            back["points"].update((d["start"] + i, p) for i, p in enumerate(d["points"]))
            back["last"] = d
    seq = make_foot_walk(fs=a.fs, seed=a.seed, gyro_scale=(1.0, 1.0, 1.0 + a.turn_error))
    mag = magnetometer(seq, north=0.5, offset=MAG_OFFSET, scale=MAG_SCALE, seed=a.seed)
    rng = np.random.default_rng(a.seed)
    t0_ns = 123_456_000_000_000  # arbitrary "since boot" origin
    records = []
    if a.compass_cal:
        for i, m in enumerate(figure8(25.0, rng)):
            records.append((MAG, t0_ns + int(i / 50.0 * 1e9), *m))
        t0_ns += 26 * 10**9
    every = max(1, int(round(a.fs / 50)))                 # magnetometer at 50 Hz
    for k in range(len(seq.t)):
        t_ns = t0_ns + int(seq.t[k] * 1e9)
        records.append((ACC, t_ns, *seq.acc[k]))
        records.append((GYRO, t_ns + 700_000, *seq.gyro[k]))  # gyro event 0.7 ms later
        if k % every == 0:
            records.append((MAG, t_ns + 300_000, *mag[k]))
    t0_ns = records[0][1]
    per_packet = min(MAX_RECORDS, 20)
    start = time.perf_counter()
    n_pkt = 0
    print(f"streaming {'25 s of compass figure-8, then ' if a.compass_cal else ''}{seq.t[-1]:.0f} s of synthetic "
          f"shoe data to {a.host}:{a.port}")
    for i in range(0, len(records), per_packet):
        chunk = records[i:i + per_packet]
        due = (chunk[-1][1] - t0_ns) * 1e-9 / a.speed
        delay = start + due - time.perf_counter()
        if delay > 0:
            time.sleep(delay)
        if rng.random() >= a.loss:
            sock.sendto(encode(n_pkt, chunk), (a.host, a.port))
        n_pkt += 1
        read_back()
    print(f"done: {n_pkt} packets; true end position {np.round(seq.gt_p[-1, :2], 2)} m, "
          f"path {np.sum(np.linalg.norm(np.diff(seq.gt_p[:, :2], axis=0), axis=1)):.1f} m")
    time.sleep(0.5)
    read_back()
    d = back["last"]
    if d is None:
        print("no path came back (is Fusion Player's live mode running?)")
    else:
        have = sum(i in back["points"] for i in range(d["n_total"]))
        print(f"path sent back: {back['packets']} packets, {'tracking' if d['state'] == TRACKING_STATE else 'calibrating'}, "
              f"{d['steps']} steps, {d['distance']:.1f} m, {have} of {d['n_total']} points, "
              f"now at ({d['x']:.2f}, {d['y']:.2f}) m in the laptop's view, compass state {d['compass_state']}, "
              f"heading {d['compass']:.0f} deg")


if __name__ == "__main__":
    main()
