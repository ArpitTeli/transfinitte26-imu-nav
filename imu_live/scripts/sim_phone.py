"""Pretend to be the phone: stream a synthetic shoe-mounted walk to the live app over UDP, and report the path the
laptop sends back (as IMU Stream draws it).

Run:  Fusion Player, Live from the phone (one terminal)
      python scripts/sim_phone.py     (another terminal)
Accel and gyro are sent as separate, slightly offset event streams, like Android does.
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from imunav.live.protocol import ACC, GYRO, MAX_RECORDS, TRACKING_STATE, decode_path, encode  # noqa: E402
from imunav.synth import make_foot_walk  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--fs", type=float, default=400.0)
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed factor")
    ap.add_argument("--loss", type=float, default=0.0, help="fraction of packets to drop")
    ap.add_argument("--seed", type=int, default=0)
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
    seq = make_foot_walk(fs=a.fs, seed=a.seed)
    rng = np.random.default_rng(a.seed)
    t0_ns = 123_456_000_000_000  # arbitrary "since boot" origin
    records = []
    for k in range(len(seq.t)):
        t_ns = t0_ns + int(seq.t[k] * 1e9)
        records.append((ACC, t_ns, *seq.acc[k]))
        records.append((GYRO, t_ns + 700_000, *seq.gyro[k]))  # gyro event 0.7 ms later
    per_packet = min(MAX_RECORDS, 20)
    start = time.perf_counter()
    n_pkt = 0
    print(f"streaming {seq.t[-1]:.0f} s of synthetic shoe data to {a.host}:{a.port}")
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
              f"now at ({d['x']:.2f}, {d['y']:.2f}) m in the laptop's view")


if __name__ == "__main__":
    main()
