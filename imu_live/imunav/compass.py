"""The phone's compass (magnetometer): calibration, tilt-compensated heading, and what the filter uses it for.

Calibration. A magnetometer reads the earth's field plus the phone's own magnetism (hard iron: a constant offset)
and a slightly different gain per axis (soft iron, kept axis-aligned here). Turning the phone through every
orientation (a figure-8, over and around) traces an ellipsoid; fitting it gives the offset and gains:
    m_cal = (m_raw - offset) * scale
Calibrated readings have the same length (the local field, ~45 uT in India) whichever way the phone points.
The phone's offset belongs to the phone, so calibrating it in the hand before strapping it on is right.

Heading. The filter knows the phone's tilt, so it can turn a calibrated reading into the world frame and read
off where magnetic north lies (north_angle). That is only trusted while the foot is flat (no swing, no heel
strike) and while the field looks like the earth's: the right strength and dip (disturbed() checks both).

World frame: z up, angles anticlockwise from x (as the filter). Compass headings for people are degrees
clockwise from magnetic north.
"""
import json
import math
import os
import time

import numpy as np

from .quat import quat_to_R

# 26 directions (cube faces, edges, corners): calibration has seen a direction once 3 readings point that way
DIRECTIONS = np.array([(i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1) if (i, j, k) != (0, 0, 0)],
                      float)
DIRECTIONS /= np.linalg.norm(DIRECTIONS, axis=1, keepdims=True)
NEED_DIRECTIONS = 20      # of 26
NEED_SPREAD = 0.03        # calibrated field strength varies by less than 3 %
FIELD_RANGE = (20.0, 80.0)  # uT: the earth's field anywhere; outside it something magnetic is nearby
NORM_TOL = 0.15           # a reading more than 15 % off the calibrated strength is disturbed ...
DIP_TOL = math.radians(10)  # ... and so is one whose dip is 10 deg off the reference


class CompassCal:
    """A fitted calibration: m_cal = (m_raw - offset) * scale; `field` is the earth's field strength (uT)."""

    def __init__(self, offset, scale, field, spread=0.0, directions=0, when=None):
        self.offset = np.asarray(offset, float)
        self.scale = np.asarray(scale, float)
        self.field = float(field)
        self.spread = float(spread)
        self.directions = int(directions)
        self.when = when or time.strftime("%Y-%m-%d %H:%M")

    def apply(self, m):
        return (np.asarray(m, float) - self.offset) * self.scale

    def to_dict(self):
        return dict(offset=self.offset.tolist(), scale=self.scale.tolist(), field=self.field, spread=self.spread,
                    directions=self.directions, when=self.when)

    @classmethod
    def from_dict(cls, d):
        return cls(d["offset"], d["scale"], d["field"], d.get("spread", 0.0), d.get("directions", 0), d.get("when"))

    def save(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=1)

    @classmethod
    def load(cls, path):
        """None if there is no (readable) calibration at `path`"""
        try:
            with open(path) as f:
                return cls.from_dict(json.load(f))
        except (OSError, ValueError, KeyError, TypeError):
            return None


def fit_ellipsoid(M):
    """Axis-aligned ellipsoid through the readings M (n x 3): (offset, scale, field, spread).
    Solves a x^2 + b y^2 + c z^2 + d x + e y + f z = 1 by least squares."""
    M = np.asarray(M, float)
    x, y, z = M.T
    A = np.column_stack([x * x, y * y, z * z, x, y, z])
    p, *_ = np.linalg.lstsq(A, np.ones(len(M)), rcond=None)
    a, b, c, d, e, f = p
    if min(a, b, c) <= 0:
        raise ValueError("not an ellipsoid")
    off = np.array([-d / (2 * a), -e / (2 * b), -f / (2 * c)])
    g = 1 + a * off[0] ** 2 + b * off[1] ** 2 + c * off[2] ** 2
    radii = np.sqrt(g / np.array([a, b, c]))
    field = float(np.cbrt(np.prod(radii)))
    scale = field / radii
    r = np.linalg.norm((M - off) * scale, axis=1)
    return off, scale, field, float(r.std() / r.mean())


class CompassCalibrator:
    """Collects raw readings while the phone is turned through every orientation; done when they cover at least
    NEED_DIRECTIONS of 26 directions and fit an ellipsoid well, at a believable field strength."""

    def __init__(self, max_samples=6000):
        self.M = []
        self.max_samples = max_samples
        self._last = None

    def add(self, m):
        m = np.asarray(m, float)
        if self._last is not None and np.linalg.norm(m - self._last) < 0.5:
            return                                  # the phone has hardly moved: nothing new to learn
        self._last = m
        self.M.append(m)
        if len(self.M) > self.max_samples:
            self.M = self.M[::2]

    def progress(self):
        """dict(n, directions, spread, field, done, cal, message)"""
        n = len(self.M)
        out = dict(n=n, directions=0, spread=None, field=None, done=False, cal=None)
        if n < 30:
            out["message"] = "Turn the phone over and around in a figure-8"
            return out
        M = np.array(self.M)
        try:
            off, scale, field, spread = fit_ellipsoid(M)
        except (ValueError, np.linalg.LinAlgError):
            off, scale, field, spread = M.mean(0), np.ones(3), float("nan"), float("nan")
        u = (M - off) * scale
        u /= np.linalg.norm(u, axis=1, keepdims=True)
        hits = np.bincount(np.argmax(u @ DIRECTIONS.T, axis=1), minlength=len(DIRECTIONS))
        dirs = int((hits >= 3).sum())
        ok_field = FIELD_RANGE[0] < field < FIELD_RANGE[1]
        done = dirs >= NEED_DIRECTIONS and spread < NEED_SPREAD and ok_field and n >= 150
        out.update(directions=dirs, spread=spread, field=field, done=done)
        if done:
            out["cal"] = CompassCal(off, scale, field, spread, dirs)
            out["message"] = f"Compass calibrated: field {field:.0f} uT, spread {100 * spread:.1f} %"
        elif dirs < NEED_DIRECTIONS:
            out["message"] = (f"Keep turning the phone over and around: {dirs} of {NEED_DIRECTIONS} directions "
                              "covered")
        elif not ok_field:
            out["message"] = f"Field {field:.0f} uT is not the earth's: move away from metal and magnets"
        else:
            out["message"] = f"Fit spread {100 * spread:.1f} % (needs under {100 * NEED_SPREAD:.0f} %): keep turning, slowly"
        return out


def north_angle(q, m_cal):
    """World angle (rad, anticlockwise from x) of magnetic north, from attitude q and a calibrated reading.
    Also returns the field in the world frame."""
    u = quat_to_R(q) @ m_cal
    return math.atan2(u[1], u[0]), u


def dip_of(u):
    """dip of a world-frame field vector (rad, positive = pointing down)"""
    return math.atan2(-u[2], math.hypot(u[0], u[1]))


def disturbed(u, field, dip_ref=None):
    """True if a world-frame reading u does not look like the earth's field (strength, or dip vs dip_ref)"""
    n = float(np.linalg.norm(u))
    if abs(n - field) > NORM_TOL * field:
        return True
    return dip_ref is not None and abs(dip_of(u) - dip_ref) > DIP_TOL


def phone_heading(q, psi_n):
    """Compass heading of the phone's top edge (its +y axis): degrees clockwise from magnetic north, like
    Android's compass. If the top points nearly straight up or down, the back of the phone is used instead.
    Returns (degrees, "top" or "back")."""
    R = quat_to_R(q)
    axis, name = R[:, 1], "top"
    if math.hypot(axis[0], axis[1]) < 0.35:
        axis, name = -R[:, 2], "back"
    psi = math.atan2(axis[1], axis[0])
    return math.degrees(psi_n - psi) % 360.0, name


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def mag_heading_h(q, m_cal):
    """Measurement model for the filter: psi_n(q) and its Jacobian with respect to the body-frame attitude
    error (q_true = q exp(dtheta)). Rotating the world by phi moves the field u by phi x u, which turns its
    horizontal angle by g . (phi x u) with g = (-u_y, u_x, 0) / |u_xy|^2; and phi = R dtheta."""
    R = quat_to_R(q)
    u = R @ m_cal
    hh = u[0] ** 2 + u[1] ** 2
    g = np.array([-u[1], u[0], 0.0]) / hh
    ux = np.array([[0.0, -u[2], u[1]], [u[2], 0.0, -u[0]], [-u[1], u[0], 0.0]])
    return math.atan2(u[1], u[0]), -(g @ ux @ R), u
