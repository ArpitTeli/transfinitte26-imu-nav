"""Sample pipeline: phone samples -> axis mapping -> unit fix -> fusion filter. No GUI code, so it can be tested headless."""
import collections, math
import numpy as np
from fusion_py import Filter

G = 9.80665
# phone axes (x right, y up the screen, z out of the screen) -> filter body frame (x fwd, y left, z up)
MAPPINGS = {
    "Flat on palm, top edge = forward": lambda v: (v[1], -v[0], v[2]),
    "Upright, screen to you, back = forward": lambda v: (-v[2], -v[0], v[1]),
    "Phone axes as-is (x fwd, y left, z up)": lambda v: (v[0], v[1], v[2]),
}
STILL_S = 2.0          # startup still period used for gravity direction + gyro bias
WINDOW_S = 20.0        # seconds kept for the scrolling plots


class Engine:
    def __init__(self, params=None, mapping=None):
        self.params = params or {}
        self.map = MAPPINGS[mapping or next(iter(MAPPINGS))]
        self.reset()

    def set_mapping(self, name):
        self.map = MAPPINGS[name]
        self.reset()

    def reset(self):
        self.filt = None
        self.gyr = None            # latest gyro (held), phone frame
        self.mag = None
        self.use_tacc = False
        self.acc_scale = None      # 1 or G, decided from the first samples
        self.calib = []            # (t, a, g) during the still period
        self.t_prev = None
        self.t0 = None
        self.state = "waiting for data"
        self.warn = ""
        self.counts = collections.Counter()
        self.rate = 0.0
        W = int(WINDOW_S * 120)
        self.buf = {k: collections.deque(maxlen=W) for k in
                    ("t", "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz", "anorm",
                     "roll", "pitch", "yaw", "v")}
        self.track = [], []
        self.last = {}
        self.log = []              # rows for CSV recording

    # -- ingest -----------------------------------------------------------------------------------------
    def feed(self, sensor, t, x, y, z):
        self.counts[sensor] += 1
        v = (x, y, z)
        if sensor == "gyr":
            self.gyr = v
        elif sensor == "mag":
            self.mag = v
        elif sensor in ("acc", "tacc"):
            if sensor == "tacc":
                self.use_tacc = True
            elif self.use_tacc:
                return                       # prefer the gravity-including 'totalacceleration' stream
            self._accel(t, v)

    def _accel(self, t, a):
        if self.gyr is None:
            return
        if self.t0 is None:
            self.t0 = t
        if self.acc_scale is None:
            self.calib.append((t, a, self.gyr))
            if len(self.calib) < 20:
                return
            n = float(np.median([np.linalg.norm(c[1]) for c in self.calib]))
            self.acc_scale = G if n < 3.0 else 1.0           # some phones report g instead of m/s^2
            self.calib = [(c[0], tuple(self.acc_scale * q for q in c[1]), c[2]) for c in self.calib]
            if np.median([np.linalg.norm(c[1]) for c in self.calib]) < 5:
                self.warn = "accelerometer looks gravity-free: enable 'Total Acceleration' (or the raw accelerometer)"
        else:
            a = tuple(self.acc_scale * q for q in a)

        if self.filt is None:
            if self.calib and self.calib[-1][0] != t:
                self.calib.append((t, a, self.gyr))
            if t - self.calib[0][0] < STILL_S:
                self.state = "calibrating: hold the phone still"
                return
            A = np.array([self.map(c[1]) for c in self.calib]); Gy = np.array([self.map(c[2]) for c in self.calib])
            still = float(np.linalg.norm(Gy.std(axis=0))) < 0.02
            self.filt = Filter(self.params)
            self.filt.init(A.mean(0), Gy.mean(0) if still else None)
            self.state = "running" + ("" if still else " (phone was moving at start: gyro bias not preset)")
            self.t_prev = t
            self.calib = []
            return

        dt = t - self.t_prev
        self.t_prev = t
        if not (0 < dt < 0.5):
            return
        self.rate = 0.95 * self.rate + 0.05 / dt if self.rate else 1 / dt
        am, gm = self.map(a), self.map(self.gyr)
        mm = self.map(self.mag) if self.mag is not None else None
        o = self.filt.step(am, gm, mm, dt)
        self.last = o
        ts = t - self.t0
        b = self.buf
        b["t"].append(ts)
        for k, v in zip(("ax", "ay", "az"), am): b[k].append(v)
        for k, v in zip(("gx", "gy", "gz"), gm): b[k].append(v)
        mv = mm or (math.nan,) * 3
        for k, v in zip(("mx", "my", "mz"), mv): b[k].append(v)
        b["anorm"].append(math.sqrt(sum(q * q for q in am)))
        b["roll"].append(math.degrees(o["roll"])); b["pitch"].append(math.degrees(o["pitch"]))
        b["yaw"].append(math.degrees(o["yaw"])); b["v"].append(o["v"])
        self.track[0].append(o["x"]); self.track[1].append(o["y"])
        self.log.append((ts, *am, *gm, *mv, o["x"], o["y"], o["yaw"], o["v"], o["roll"], o["pitch"]))

    def save_csv(self, path):
        hdr = "t,ax,ay,az,gx,gy,gz,mx,my,mz,x,y,yaw,v,roll,pitch"
        np.savetxt(path, np.array(self.log), delimiter=",", header=hdr, comments="", fmt="%.6g")
