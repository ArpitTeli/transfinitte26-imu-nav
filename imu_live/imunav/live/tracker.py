"""Live tracker: calibrate while still, then run the navigator one sample at a time."""
import threading

import numpy as np

from ..calib import ACC_STILL_STD, GYRO_STILL_STD, static_calibration
from ..compass import CompassCalibrator, disturbed, north_angle, phone_heading
from ..pipeline import navigator_from_static
from ..quat import q_from_gravity

CALIBRATING, TRACKING, RECALIBRATING = "calibrating", "tracking", "recalibrating"
MIN_SWING = 0.15          # s of foot off the ground before a landing counts as a step (not a detector blip)
# compass states, as sent to the phone (protocol.py)
COMPASS_NONE, COMPASS_UNCALIBRATED, COMPASS_CALIBRATING, COMPASS_OK, COMPASS_DISTURBED = range(5)


class LiveTracker:
    """Thread-safe: add() and add_mag() from the network thread, snapshot() from the GUI thread.

    Starts in CALIBRATING: the shoe must be still for cfg.init_window seconds.
    If that window is not still, the oldest half is dropped and it keeps waiting.
    The path is stored at about `display_hz` for drawing; distance and step
    counts use every sample.

    recalibrate_keep() (K) calibrates again the same way, but keeps the path: the walk carries on from where it
    stands. The heading is then taken from the compass if there is a calibrated one, otherwise the walker faces
    the way they faced at the start.

    With a compass calibration (set_compass, or calibrate_compass() while the phone is turned through every
    orientation) the filter holds its heading to magnetic north at every stance.
    """

    def __init__(self, cfg, display_hz=50.0, compass=None):
        self.cfg = cfg
        self.display_dt = 1.0 / display_hz
        self.lock = threading.Lock()
        self.gen = 0                        # bumped whenever the path is replaced rather than extended
        self.compass_cal = compass
        self._calibrator = None
        self.compass_progress = None        # the calibrator's progress() while calibrating the compass
        self._mag_raw = None                # newest raw magnetometer reading
        self._mag_check_t = -1e9
        self._acc_last = None
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
                self.gen += 1
            self.steps, self.distance = 0, 0.0

    def recalibrate_keep(self):
        """K: stand still and calibrate again, keeping the path (pressed again: start the still window over)."""
        with self.lock:
            if self.state in (TRACKING, RECALIBRATING):
                self.state = RECALIBRATING
                self._buf = []
                self.message = "Recalibrating - stand still (the path is kept)"

    def _reset(self):
        self.gen += 1
        self.state = CALIBRATING
        self.nav = None
        self.cal = None
        self._buf = []
        self.path, self.path_t = [], []
        self._p_last = None
        self._still_last = False
        self._swing = 0.0
        self.steps = 0
        self.distance = 0.0
        self.gaps, self.gap_time = 0, 0.0
        self.recalibrations, self.last_recal = 0, None
        self.message = "Hold the shoe still to calibrate"
        self.motion = None                  # while calibrating: (gyro, accel) spread over the last window
        self._motion_t = -1e9

    # ---------------------------------------------------------------- compass
    def set_compass(self, cal):
        """Use a compass calibration from now on (it holds the heading from the next stance)."""
        with self.lock:
            self.compass_cal = cal
            if self.nav is not None and cal is not None:
                self.nav.set_compass(cal, None)

    def calibrate_compass(self):
        """Start collecting readings for a compass calibration: turn the phone over and around in a figure-8.
        Waving the phone about is not a walk: the path starts again, once the compass is done and the shoe is still."""
        with self.lock:
            self._reset()
            self._calibrator = CompassCalibrator()
            self.compass_progress = self._calibrator.progress()

    def cancel_compass_calibration(self):
        with self.lock:
            self._calibrator, self.compass_progress = None, None

    def add_mag(self, t, m):
        with self.lock:
            self._mag_raw = np.asarray(m, float)
            if self._calibrator is None:
                return
            self._calibrator.add(self._mag_raw)
            if t - self._mag_check_t >= 0.5:        # the fit is redone twice a second
                self._mag_check_t = t
                self.compass_progress = p = self._calibrator.progress()
                if p["done"]:
                    self._calibrator = None
                    self.compass_cal = p["cal"]
                    if self.nav is not None:
                        self.nav.set_compass(p["cal"], None)

    def _compass_status(self):
        """dict(state, heading (deg, phone top or back, as measured), filter_heading, axis, field)"""
        out = dict(state=COMPASS_NONE, heading=None, filter_heading=None, axis=None, field=None,
                   progress=self.compass_progress)
        if self._calibrator is not None:
            out["state"] = COMPASS_CALIBRATING
            return out
        if self._mag_raw is None:
            return out
        cal = self.compass_cal
        if cal is None:
            out["state"] = COMPASS_UNCALIBRATED
            return out
        m = cal.apply(self._mag_raw)
        out["field"] = float(np.linalg.norm(m))
        nav = self.nav
        if nav is not None and nav.compass is not None and nav.compass_psi is not None:
            out["heading"], out["axis"] = phone_heading(nav.q, nav.compass_psi)
            if nav.north is not None:
                out["filter_heading"] = phone_heading(nav.q, nav.north[0])[0]
            out["state"] = COMPASS_DISTURBED if nav.compass_disturbed else COMPASS_OK
        elif self._acc_last is not None:           # not tracking yet: tilt from the accelerometer
            q = q_from_gravity(self._acc_last, 0.0)
            psi, u = north_angle(q, m)
            out["heading"], out["axis"] = phone_heading(q, psi)
            out["state"] = COMPASS_DISTURBED if disturbed(u, cal.field) else COMPASS_OK
        else:
            out["state"] = COMPASS_OK
        return out

    # ---------------------------------------------------------------- samples
    def add(self, t, gyro, acc, gap=0.0, mag=None):
        """gap: seconds of data missing just before this sample (0 = none). mag: newest raw magnetometer reading."""
        with self.lock:
            self._acc_last = acc
            if gap > 0:
                if self.state in (CALIBRATING, RECALIBRATING):
                    self._buf = []                  # a still window must not span a hole (no walking is lost:
                                                    # e.g. the phone was stopped between walks)
                else:
                    self.gaps += 1
                    self.gap_time += gap
                    self.nav.gap(gap)
                    self._t_last = t                # restart integration here; nothing spans the hole
                    self.message = f"Tracking - {self.gaps} data gap(s), {self.gap_time:.1f} s lost"
                    return
            if self.state in (CALIBRATING, RECALIBRATING):
                self._calibrate(t, gyro, acc, mag)
                return
            dt = t - self._t_last
            self._t_last = t
            self.nav.step(gyro, acc, dt, mag)
            p = self.nav.p
            self.distance += float(np.linalg.norm(p[:2] - self._p_last[:2]))
            self._p_last = p.copy()
            if self.nav.still:
                if not self._still_last and self._swing >= MIN_SWING:
                    self.steps += 1
                self._swing = 0.0
            else:
                self._swing += dt
            self._still_last = self.nav.still
            if t - self.path_t[-1] >= self.display_dt:
                self.path.append(self._p_last)
                self.path_t.append(t)

    def _calibrate(self, t, gyro, acc, mag):
        if self._calibrator is not None:            # the phone is being turned for the compass: not yet
            self._buf = []
            self.message = "Calibrating the compass - then strap the phone on and stand still"
            return
        self._buf.append((t, gyro, acc, mag))
        if t - self._motion_t >= 0.1:
            self._motion_t = t
            self._measure_motion(t)
        if self._buf[-1][0] - self._buf[0][0] < self.cfg.init_window:
            return
        T = np.array([b[0] for b in self._buf])
        G = np.array([b[1] for b in self._buf])
        A = np.array([b[2] for b in self._buf])
        M = np.array([b[3] if b[3] is not None else (np.nan,) * 3 for b in self._buf], float)
        cal = static_calibration(T, G, A, self.cfg.init_window)
        keep = self.state == RECALIBRATING
        if not cal["looks_static"]:
            self._buf = self._buf[len(self._buf) // 2:]
            self.message = ("Moving - stand still to recalibrate (the path is kept)" if keep
                            else "Moving - hold the shoe still to calibrate")
            return
        cfg = self.cfg.with_(fs=1.0 / float(np.median(np.diff(T))))
        if keep:
            # carry on from here: same position; heading from the compass, else the start direction (yaw 0)
            old = self.nav
            self.nav, self.cal = navigator_from_static(T, G, A, cfg, yaw0=0.0, p0=old.p.copy(), mag=M,
                                                       compass=self.compass_cal,
                                                       north=old.north if old.compass is not None else None)
            self.recalibrations += 1
            self.last_recal = "compass" if self.cal["compass_yaw"] is not None else "start direction"
            self._p_last = self.nav.p.copy()
            self.message = f"Tracking - recalibrated (heading from the {self.last_recal}), path kept"
        else:
            self.nav, self.cal = navigator_from_static(T, G, A, cfg, mag=M, compass=self.compass_cal)
            self._p_last = self.nav.p.copy()
            self.path, self.path_t = [self._p_last], [T[-1]]
            self.gen += 1
            self.message = "Tracking - walk"
        self._t_last = T[-1]
        self._still_last, self._swing = True, 0.0
        self.state = TRACKING
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

    # ---------------------------------------------------------------- reading
    def path_since(self, gen, k):
        """(generation, [(x, y), ...] from path point k on), for a reader that follows the path as it grows.
        When `gen` is no longer the current generation the path was replaced, and all of it is returned."""
        with self.lock:
            if gen != self.gen:
                k = 0
            return self.gen, [(float(p[0]), float(p[1])) for p in self.path[k:]]

    def status(self):
        """snapshot() without the path"""
        with self.lock:
            return self._status()

    def snapshot(self):
        with self.lock:
            s = self._status()
            s["p"] = np.array(self.path) if self.path else np.zeros((0, 3))
            return s

    def _status(self):
        ins = getattr(self.nav, "ins", None) if self.nav else None
        return {
            "state": self.state, "message": self.message, "steps": self.steps,
            "distance": self.distance,
            "still": self._still_last,
            "speed": float(np.linalg.norm(self.nav.v[:2])) if self.nav else 0.0,
            "rejected": dict(ins.n_rejected) if ins else {},
            "updates": dict(ins.n_updates) if ins else {},
            "gaps": self.gaps, "gap_time": self.gap_time,
            "motion": self.motion, "still_limits": (GYRO_STILL_STD, ACC_STILL_STD),
            "recalibrations": self.recalibrations, "last_recal": self.last_recal,
            "compass": self._compass_status(),
        }
