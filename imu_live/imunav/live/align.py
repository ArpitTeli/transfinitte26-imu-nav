"""Merge the phone's separate accel and gyro event streams onto one uniform time grid."""
from collections import deque

import numpy as np


class StreamAligner:
    """Push raw events, pop (t, gyro, acc) samples at `fs` Hz.

    A grid time T is emitted once both streams have an event at or after T, by
    linear interpolation between the events around T, so latency is about one
    event. Late (out-of-order) events are dropped and counted.

    Gaps: if either stream has no event for more than `max_gap` seconds around T
    (lost Wi-Fi packets), nothing is interpolated across the hole. The grid
    restarts where both streams have data again, and the first sample after the
    hole carries the length of the missing stretch (pop_ready_gaps). Filling a
    hole with a straight line would feed the filter motion that never happened.
    """

    def __init__(self, fs=400.0, max_gap=0.05):
        self.dt = 1.0 / fs
        self.max_gap = max_gap
        self.buf = {1: deque(), 2: deque()}  # type -> (t, xyz); 1 = acc, 2 = gyro
        self.T = None
        self.n_late = 0
        self.n_gaps = 0
        self.gap_time = 0.0
        self._pending_gap = 0.0
        self._hole_from = None

    def push(self, typ, t, xyz):
        b = self.buf[typ]
        if b and t <= b[-1][0]:
            self.n_late += 1
            return
        b.append((t, np.asarray(xyz, float)))

    @staticmethod
    def _advance(b, T):
        """drop events so that b[0] is the last event at or before T"""
        while len(b) >= 2 and b[1][0] <= T:
            b.popleft()

    @staticmethod
    def _interp(b, T):
        if len(b) == 1 or b[0][0] == T:
            return b[0][1].copy()
        (t0, x0), (t1, x1) = b[0], b[1]
        return x0 + (T - t0) / (t1 - t0) * (x1 - x0)

    def pop_ready(self):
        """[(t, gyro, acc), ...] with samples inside gaps left out"""
        return [(t, g, a) for t, g, a, _ in self.pop_ready_gaps()]

    def pop_ready_gaps(self):
        """[(t, gyro, acc, gap_s), ...]: gap_s > 0 on the first sample after missing data"""
        a, g = self.buf[1], self.buf[2]
        out = []
        if len(a) < 2 or len(g) < 2:
            return out
        if self.T is None:
            self.T = max(a[0][0], g[0][0])
        while a[-1][0] >= self.T and g[-1][0] >= self.T:
            resume, lost_from = None, None
            for b in (a, g):
                self._advance(b, self.T)
                if len(b) >= 2 and b[1][0] - b[0][0] > self.max_gap:   # hole around T in this stream
                    resume = b[1][0] if resume is None else max(resume, b[1][0])
                    lost_from = b[0][0] if lost_from is None else min(lost_from, b[0][0])
            if resume is not None:
                # the two streams resume a fraction of a ms apart, so one hole can be seen twice before any
                # sample is emitted: merge those sightings into one hole, from the first lost event on
                if self._hole_from is None:
                    self._hole_from = lost_from
                    self.n_gaps += 1
                else:
                    self._hole_from = min(self._hole_from, lost_from)
                self._pending_gap = resume - self._hole_from
                self.T = resume
                continue
            out.append((self.T, self._interp(g, self.T), self._interp(a, self.T), self._pending_gap))
            self.gap_time += self._pending_gap
            self._pending_gap, self._hole_from = 0.0, None
            self.T += self.dt
        return out
