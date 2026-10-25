"""Causal navigation pipeline: one sample in, one state out.

Navigator.step() never looks ahead and does no I/O, so it can be ported to a
microcontroller. run_sequence() is only a driver loop around it.

Modes
  madgwick | mahony | ekf : orientation filter + strapdown (+ ZUPT velocity reset)
  ins                      : 15-state EKF with ZUPT / ZARU / gravity updates
Aiding sources used: none (IMU + zero-velocity updates only).
"""
import warnings
from dataclasses import dataclass, field, replace

import numpy as np

from .calib import static_calibration
from .ins import TH_, V_, InsEKF, Strapdown
from .orientation import FILTERS
from .quat import q_from_gravity, qexp, qmul, qnormalize, quat_to_R, yaw_of
from .zupt import ZuptDetector


@dataclass
class Config:
    mode: str = "madgwick"
    zupt: bool = True
    fs: float = 200.0
    # orientation filters
    beta: float = 0.05
    kp: float = 1.0
    ki: float = 0.02
    acc_gate: float = 1.0
    acc_always: bool = False  # True: also correct tilt from the accel while moving
    # stationary detector
    zupt_win: float = 0.25
    zupt_gyro_thr: float = 0.06
    zupt_acc_var_thr: float = 0.05
    zupt_res_thr: float = 0.3
    zupt_override: float = float("inf")  # s of attitude-test veto on a clearly still IMU before it is ignored
    att_p_bias: float = 1e-9  # attitude-EKF gyro-bias prior variance (bias already calibrated)
    # ins EKF
    sigma_g: float = 1.7e-4
    sigma_a: float = 2.0e-3
    sigma_bg: float = 1.9e-5
    sigma_ba: float = 3.0e-3
    q_scale: float = 1.0
    sigma_zupt: float = 0.05
    zupt_period: float = 0.1  # seconds between stationary updates (consecutive samples are correlated)
    use_zaru: bool = True
    sigma_zaru: float = 2e-3  # per-sample gyro noise while still (rad/s); grows with sample rate
    # ZARU only while |gyro - bias| is below this (rad/s). A foot pivoting on the ground at a corner passes the
    # still test (zero velocity is right) but is turning: a ZARU there would learn the turn as gyro bias.
    zaru_gyro_thr: float = float("inf")
    use_gravity: bool = True
    gate_prob: float = 0.999
    force_after: float = 0.3  # s of rejected ZUPTs while still ...
    v_diverged: float = float("inf")  # ... with |v| above this (m/s) means the filter has diverged: re-open v and tilt
    p_att: float = 1e-5
    p_bg: float = 1e-7
    p_ba: float = 2.5e-3
    sigma_grav: float = 0.05
    # data gaps: a time step longer than max_gap (s) is lost data; after it, continuous stillness of restart_s
    # is needed before the filter restarts at the new stance
    max_gap: float = 0.05
    restart_s: float = 0.1
    # initialisation
    init_window: float = 2.0
    calibrate: bool = True

    def with_(self, **kw):
        return replace(self, **kw)


# Shoe-mounted IMU: short stance phases (~0.3-0.5 s), so a short window and frequent updates.
# First tuned on make_foot_walk seeds 1-3 (scripts/tune.py), then on the real court walks of 2026-10-25
# (scripts/court_tune.py). A real foot is never perfectly still at stance and the phone gyro is noisier than
# assumed, so the ZUPT is softer and the gyro noise larger; ZARU is tighter. Tuned on A1, B1, C, D, E first: RMS
# corner error 3.9 -> 0.7 m, held-out A2 1.26 -> 0.66 m and B2 2.79 -> 0.59 m, leave-one-walk-out 3.94 -> 0.81 m.
# Retuning on all 7 walks then added: no gravity update (a foot rolling at stance makes the gravity reading wrong,
# and the ZUPTs already hold the tilt) and stance updates every 0.01 s; 0.69 -> 0.62 m. No walk is held out now.
PRESETS = {
    "shoe": Config(mode="ins", zupt_win=0.08, zupt_gyro_thr=0.3, zupt_acc_var_thr=2.0, zupt_period=0.01,
                   sigma_zupt=0.3, sigma_g=1.5e-3, sigma_zaru=1e-3, use_gravity=False,
                   # divergence recovery, chosen on the first real phone recording (20261024_035346);
                   # a foot never ramps gently, so this does not trigger in normal stance
                   zupt_override=0.3, v_diverged=0.5),
    "body": Config(mode="ins"),
}


class Navigator:
    def __init__(self, cfg, gyro_bias, gravity, q0, p0, v0=None):
        self.cfg = cfg
        self.bias = np.array(gyro_bias, float)
        v0 = np.zeros(3) if v0 is None else np.array(v0, float)
        self.det = ZuptDetector(cfg.fs, cfg.zupt_win, cfg.zupt_gyro_thr, cfg.zupt_acc_var_thr,
                                cfg.zupt_res_thr, gravity, cfg.zupt_override)
        self.still = False
        self._wait_stance = False
        self._run_n, self._run_acc, self._run_t = 0, np.zeros(3), 0.0
        self.n_gaps, self.gap_time = 0, 0.0
        self._k = 0
        self._every = max(1, int(round(cfg.zupt_period * cfg.fs)))
        self._force_n = max(1, int(round(cfg.force_after / cfg.zupt_period)))
        self._streak = 0
        if cfg.mode == "ins":
            self.ins = InsEKF(q0, p0, v0, gyro_bias, g=gravity, sigma_g=cfg.sigma_g, sigma_a=cfg.sigma_a,
                              sigma_bg=cfg.sigma_bg, sigma_ba=cfg.sigma_ba, q_scale=cfg.q_scale,
                              gate_prob=cfg.gate_prob, p_att=cfg.p_att, p_bg=cfg.p_bg, p_ba=cfg.p_ba)
            self.orient = None
        else:
            kw = {"madgwick": {"beta": cfg.beta}, "mahony": {"kp": cfg.kp, "ki": cfg.ki}, "ekf": {"p_bias": cfg.att_p_bias}}[cfg.mode]
            self.orient = FILTERS[cfg.mode](q0, g=gravity, acc_gate=cfg.acc_gate, **kw)
            self.strap = Strapdown(p0, v0, gravity)

    def step(self, gyro, acc, dt):
        cfg = self.cfg
        if self._wait_stance:
            self._step_waiting(gyro, acc, dt)
            return
        up = quat_to_R(self.q)[2, :]
        if cfg.mode == "ins":
            acc_d = acc - self.ins.ba
            gyro_d = gyro - self.ins.bg
        else:
            acc_d, gyro_d = acc, gyro - self.bias
        self.still = self.det.update(gyro_d, acc_d, up)
        if cfg.mode == "ins":
            e = self.ins
            e.predict(gyro, acc, dt)
            self._k += 1
            if self.still and cfg.zupt and self._k % self._every == 0:
                if cfg.use_gravity:
                    e.gravity(acc, cfg.sigma_grav)
                if e.zupt(cfg.sigma_zupt):
                    self._streak = 0
                else:
                    self._recover_if_diverged(acc)
                if cfg.use_zaru and np.linalg.norm(gyro_d) < cfg.zaru_gyro_thr:
                    e.zaru(gyro, cfg.sigma_zaru)
        else:
            q_prev = self.orient.q
            self.orient.update(gyro - self.bias, acc, dt, use_acc=self.still or cfg.acc_always)
            q_mid = qnormalize(q_prev + self.orient.q)
            self.strap.step(q_mid, acc, dt, self.still and cfg.zupt)

    def gap(self, dur):
        """`dur` seconds of data were lost: the motion in between is unknown and is not integrated.
        Until the foot is detected still again only the attitude follows the gyro (so heading stays
        continuous through the rest of the stride); position and velocity are frozen. At that stance the
        filter restarts from the last position: velocity 0, start-up covariance, the biases it had learnt.
        Widening the covariance instead (and integrating on) made every later ZUPT shift the position,
        inflating distance by ~60 % in tests. Heading change during the hole itself is unrecoverable."""
        self.n_gaps += 1
        self.gap_time += dur
        self._streak = 0
        self._wait_stance = True
        if self.cfg.mode == "ins":
            self.ins.v[:] = 0.0
        else:
            self.strap.v[:] = 0.0

    def _step_waiting(self, gyro, acc, dt):
        """after a gap: attitude only, until the next stance; then restart the error state there.
        The rotation during the hole is lost (a foot pitches 20-40 deg mid-swing), so the attitude is not
        trusted: the detector runs without its gravity-residual test, and at the stance tilt is re-levelled
        from the accelerometer like at start-up (heading kept as the best guess)."""
        cfg = self.cfg
        # without the residual test the detector can mistake the slow end of a swing for stance, so ask for
        # restart_s of continuous stillness (a real stance lasts 0.3-0.5 s) and level on its mean accel
        ba = self.ins.ba if cfg.mode == "ins" else 0.0
        bg = self.ins.bg if cfg.mode == "ins" else self.bias
        self.still = self.det.update(gyro - bg, acc - ba, None)
        if self.still:
            self._run_n += 1
            self._run_acc += acc - ba
            self._run_t += dt
        else:
            self._run_n, self._run_acc, self._run_t = 0, np.zeros(3), 0.0
        planted = self._run_t >= cfg.restart_s
        if cfg.mode == "ins":
            e = self.ins
            e.q = qnormalize(qmul(e.q, qexp((gyro - e.bg) * dt)))
            if planted:
                e.q = q_from_gravity(self._run_acc / self._run_n, yaw_of(e.q))
                e.v[:] = 0.0
                keep = slice(9, 15)                                  # gyro and accel bias blocks
                bias_P = e.P[keep, keep].copy()
                e.P = e.P0.copy()
                e.P[keep, keep] = bias_P
                self._wait_stance = False
        else:
            self.orient.update(gyro - self.bias, acc, dt, use_acc=False)
            if planted:
                self.orient.q = q_from_gravity(self._run_acc / self._run_n, yaw_of(self.orient.q))
                self.strap.v[:] = 0.0
                self._wait_stance = False
        if not self._wait_stance:
            self._run_n, self._run_acc, self._run_t = 0, np.zeros(3), 0.0

    def _recover_if_diverged(self, acc):
        """A rejected ZUPT is normal at a slow start (the gate's job). But if the detector keeps
        saying still while the filter believes |v| > v_diverged, the filter has diverged and the
        gate would lock it out for good: re-open velocity and tilt and accept the measurements.
        Gyro bias is never re-opened, because a false 'still' would corrupt it."""
        cfg, e = self.cfg, self.ins
        self._streak += 1
        if self._streak < self._force_n or float(np.linalg.norm(e.v)) < cfg.v_diverged:
            return
        e.reopen([V_, TH_], [float(e.v @ e.v) + 0.01, 0.3 ** 2], "diverged")
        if cfg.use_gravity:
            e.gravity(acc, cfg.sigma_grav)
        e.zupt(cfg.sigma_zupt)
        self._streak = 0

    @property
    def q(self):
        return self.ins.q if self.cfg.mode == "ins" else self.orient.q

    @property
    def p(self):
        return self.ins.p if self.cfg.mode == "ins" else self.strap.p

    @property
    def v(self):
        return self.ins.v if self.cfg.mode == "ins" else self.strap.v


@dataclass
class Result:
    name: str
    t: np.ndarray
    q: np.ndarray
    p: np.ndarray
    v: np.ndarray
    still: np.ndarray
    calib: dict = field(default_factory=dict)
    n_updates: dict = field(default_factory=dict)
    n_rejected: dict = field(default_factory=dict)


def navigator_from_static(t, gyro, acc, cfg, yaw0=0.0, p0=None, v0=None):
    """Calibrate on the first cfg.init_window seconds (platform still) and build a Navigator."""
    cal = static_calibration(t, gyro, acc, cfg.init_window)
    bias = cal["gyro_bias"] if cfg.calibrate else np.zeros(3)
    q0 = q_from_gravity(cal["acc_mean"], yaw0)
    p0 = np.zeros(3) if p0 is None else p0
    return Navigator(cfg, bias, cal["gravity"], q0, p0, v0), cal


def make_navigator(seq, cfg):
    """Build a Navigator from the start of a sequence. The sample rate is taken from the data."""
    cfg = cfg.with_(fs=seq.fs)
    yaw0 = yaw_of(seq.gt_q[0]) if seq.gt_q is not None else 0.0
    p0 = seq.gt_p[0] if seq.gt_p is not None else None
    v0 = seq.gt_v[0] if seq.gt_v is not None else None
    nav, cal = navigator_from_static(seq.t, seq.gyro, seq.acc, cfg, yaw0, p0, v0)
    if not cal["looks_static"]:
        warnings.warn(f"{seq.name}: first {cfg.init_window} s do not look stationary; "
                      "gyro bias and initial tilt will be wrong")
    return nav, cal


def run_sequence(seq, cfg, label=None):
    nav, cal = make_navigator(seq, cfg)
    cfg = nav.cfg
    n = len(seq.t)
    Q, Pp, V, S = np.zeros((n, 4)), np.zeros((n, 3)), np.zeros((n, 3)), np.zeros(n, bool)
    dts = np.diff(seq.t, prepend=seq.t[0] - 1.0 / cfg.fs)
    gap_dt = max(cfg.max_gap, 5.0 / cfg.fs)
    for k in range(n):
        if dts[k] > gap_dt:                   # lost data: never integrate across it
            nav.gap(dts[k])
        else:
            nav.step(seq.gyro[k], seq.acc[k], dts[k])
        Q[k], Pp[k], V[k], S[k] = nav.q, nav.p, nav.v, nav.still
    ins = getattr(nav, "ins", None)
    return Result(label or f"{cfg.mode}{'+zupt' if cfg.zupt else ''}", seq.t, Q, Pp, V, S, cal,
                  ins.n_updates if ins else {}, ins.n_rejected if ins else {})
