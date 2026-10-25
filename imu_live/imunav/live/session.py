"""Court recording session: one labelled laptop recording per protocol walk, plus timestamped markers.

The walker has the phone on the foot; whoever watches the laptop presses
  n  next protocol walk: closes the current file, opens <stamp>_<code>.csv, and recalibrates
     (press it before the walker presses Start on the phone and stands still on the start corner)
  m  marker: writes the time and a note to <recording>_markers.csv (e.g. "tripped", "lap 3 done")
The phone keeps its own complete copy of everything (IMU Stream saves to Download/IMU Stream), so the
laptop files are for watching, labelling and quick checks; score from the phone files.
"""
import datetime
import time
from pathlib import Path

import numpy as np

# code, description, walk shape ("cw" / "ccw": laps, "line": one long side walked there and back). The shape
# picks the lap counter in the live view and the walk type in scripts/court_eval.py, which reads the actual
# turn direction and first leg from the data. See docs/data_collection.md.
PROTOCOL = [
    ("A1", "5 laps clockwise, normal pace, from corner 1", "cw"),
    ("A2", "5 laps clockwise, normal pace, from corner 1 (test set)", "cw"),
    ("B1", "5 laps anticlockwise, normal pace, from corner 2", "ccw"),
    ("B2", "5 laps anticlockwise, normal pace, from corner 2 (test set)", "ccw"),
    ("C", "3 laps clockwise, slow, from corner 1", "cw"),
    ("D", "3 laps clockwise, fast walking, from corner 1", "cw"),
    ("E", "long side (13.4 m) there and back 5 times, from corner 1", "line"),
]
COURT_L, COURT_W = 13.40, 6.10        # doubles outer lines, m
LAP_M = 2 * (COURT_L + COURT_W)       # 39.0 m
ALIGN_M = 3.0                         # metres walked before the track is turned to point up


def heading_up(p):
    """start at the origin, rotated so the first ALIGN_M metres point up the screen (the phone does not know
    north, so "forward is up" is the natural way to show a walk). None until that far has been walked."""
    q = p[:, :2] - p[0, :2]
    d = np.linalg.norm(q, axis=1)
    if not np.any(d >= ALIGN_M):
        return None
    i = int(np.argmax(d >= ALIGN_M))
    a = np.pi / 2 - np.arctan2(q[i, 1], q[i, 0])
    c, s = np.cos(a), np.sin(a)
    return np.column_stack([c * q[:, 0] - s * q[:, 1], s * q[:, 0] + c * q[:, 1]])


class Session:
    def __init__(self, receiver, tracker, out_dir, protocol=PROTOCOL, start=None):
        self.rx, self.tr = receiver, tracker
        self.out = Path(out_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        self.protocol = list(protocol)
        codes = [p[0] for p in self.protocol]
        if start is not None and start not in codes:
            raise ValueError(f"unknown walk {start!r}; choose one of {', '.join(codes)}")
        self._next = codes.index(start) if start else 0
        self.walk = ("free", "not a protocol walk (press n to start the next walk)", None)
        self.n_extra = 0
        self.markers = 0
        self._open("free")

    @staticmethod
    def _stamp():
        return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    def _open(self, code):
        old, old_rows = getattr(self, "path", None), getattr(self.rx, "rec_rows", 1)
        self.path = self.out / f"{self._stamp()}_{code}.csv"
        self.markers_path = self.path.with_name(self.path.stem + "_markers.csv")
        self.rx.set_record(str(self.path))
        self.markers = 0
        if old is not None and old_rows == 0 and old.exists():
            old.unlink()                     # a file that never got a sample (e.g. the walk began at once)

    @property
    def next_code(self):
        return self.protocol[self._next][0] if self._next < len(self.protocol) else f"X{self.n_extra + 1}"

    def next_walk(self):
        """Start the next protocol walk (after the list runs out: extra walks X1, X2, ...)."""
        if self._next < len(self.protocol):
            self.walk = self.protocol[self._next]
            self._next += 1
        else:
            self.n_extra += 1
            self.walk = (f"X{self.n_extra}", "extra walk", None)
        self._open(self.walk[0])
        self.tr.reset()                       # the walker stands still on the start corner to recalibrate
        return self.walk

    def marker(self, note="mark"):
        """Append a timestamped note; phone time ties it to the recording rows."""
        first = not self.markers_path.exists()
        with open(self.markers_path, "a") as f:
            if first:
                f.write("t_phone,wall_time,walk,note\n")
            t = self.rx.last_t
            f.write(f"{'' if t is None else f'{t:.3f}'},{time.strftime('%H:%M:%S')},{self.walk[0]},{note}\n")
        self.markers += 1

    def close(self):
        self.rx.set_record(None)
