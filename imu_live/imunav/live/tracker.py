"""Live tracker: calibrate while still, then run the navigator one sample at a time."""
import threading

import numpy as np

from ..calib import ACC_STILL_STD, GYRO_STILL_STD, static_calibration
from ..pipeline import navigator_from_static

CALIBRATING, TRACKING = "calibrating", "tracking"


class LiveTracker:
    """Thread-safe: add() from the network thread, snapshot() from the GUI thread.

    Starts in CALIBRATING: the shoe must be still for cfg.init_window seconds.
    If that window is not still, the oldest half is dropped and it keeps waiting.
    The path is stored at about `display_hz` for drawing; distance and step
    counts use every sample.
    """

    def __init__(self, cfg, display_hz=50.0):
        self.cfg = cfg
        self.display_dt = 1.0 / display_hz
        self.lock = threading.Lock()
        with self.lock:
            self._reset()

    def reset(self):
        """Forget everything and recalibrate."""
        with self.lock:
            self._reset()

    def clear(self):
        """Keep the calibration and filter state, but move the origin to the current position."""
        with self.lock:
            if self.nav is not None:
                self.nav.p[:] = 0.0
                self._p_last = self.nav.p.copy()
                self.path, self.path_t = [self._p_last], [self._t_last]
            self.steps, self.distance = 0, 0.0

    def _reset(self):
        self.state = CALIBRATING
        self.nav = None
        self.cal = None
        self._buf = []
        self.path, self.path_t = [], []
        self._p_last = None
        self._still_last = False
        self.steps = 0
        self.distance = 0.0
        self.gaps, self.gap_time = 0, 0.0
        self.message = "Hold the shoe still to calibrate"
        self.motion = None                  # while calibrating: (gyro, accel) spread over the last window
        self._motion_t = -1e9

    def add(self, t, gyro, acc, gap=0.0):
        """gap: seconds of data missing just before this sample (0 = none)."""
        with self.lock:
            if gap > 0:
                if self.state == CALIBRATING:
                    self._buf = []                  # a still window must not span a hole (no walking is lost:
                                                    # e.g. the phone was stopped between walks)
                else:
                    self.gaps += 1
                    self.gap_time += gap
                    self.nav.gap(gap)
                    self._t_last = t                # restart integration here; nothing spans the hole
                    self.message = f"Tracking - {self.gaps} data gap(s), {self.gap_time:.1f} s lost"
                    return
            if self.state == CALIBRATING:
                self._calibrate(t, gyro, acc)
                return
            dt = t - self._t_last
            self._t_last = t
            self.nav.step(gyro, acc, dt)
            p = self.nav.p
            self.distance += float(np.linalg.norm(p[:2] - self._p_last[:2]))
            self._p_last = p.copy()
            if self.nav.still and not self._still_last:
                self.steps += 1
            self._still_last = self.nav.still
            if t - self.path_t[-1] >= self.display_dt:
                self.path.append(self._p_last)
                self.path_t.append(t)

    def _calibrate(self, t, gyro, acc):
        self._buf.append((t, gyro, acc))
        if t - self._motion_t >= 0.1:
            self._motion_t = t
            self._measure_motion(t)
        if self._buf[-1][0] - self._buf[0][0] < self.cfg.init_window:
            return
        T = np.array([b[0] for b in self._buf])
        G = np.array([b[1] for b in self._buf])
        A = np.array([b[2] for b in self._buf])
        cal = static_calibration(T, G, A, self.cfg.init_window)
        if not cal["looks_static"]:
            self._buf = self._buf[len(self._buf) // 2:]
            self.message = "Moving - hold the shoe still to calibrate"
            return
        fs = 1.0 / float(np.median(np.diff(T)))
        self.nav, self.cal = navigator_from_static(T, G, A, self.cfg.with_(fs=fs))
        self._t_last = T[-1]
        self._p_last = self.nav.p.copy()
        self.path, self.path_t = [self._p_last], [T[-1]]
        self.state = TRACKING
        self.message = "Tracking - walk"
        self._buf = []

    def _measure_motion(self, t):
        """Feedback while calibrating: the spread the stillness check sees over the last init_window
        (a shoe that never settles below the limit is not rigidly strapped, or the walker is moving)."""
        i = len(self._buf) - 1
        while i > 0 and t - self._buf[i - 1][0] <= self.cfg.init_window:
            i -= 1
        if len(self._buf) - i < 20:
            self.motion = None
            return
        G = np.array([b[1] for b in self._buf[i:]])
        A = np.array([b[2] for b in self._buf[i:]])
        self.motion = (float(G.std(0).max()), float(A.std(0).max()))

    def snapshot(self):
        with self.lock:
            ins = getattr(self.nav, "ins", None) if self.nav else None
            return {
                "state": self.state, "message": self.message, "steps": self.steps,
                "distance": self.distance,
                "p": np.array(self.path) if self.path else np.zeros((0, 3)),
                "still": self._still_last,
                "speed": float(np.linalg.norm(self.nav.v[:2])) if self.nav else 0.0,
                "rejected": dict(ins.n_rejected) if ins else {},
                "gaps": self.gaps, "gap_time": self.gap_time,
                "motion": self.motion, "still_limits": (GYRO_STILL_STD, ACC_STILL_STD),
            }
