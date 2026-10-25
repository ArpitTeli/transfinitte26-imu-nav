"""Live mode for Fusion Player: the phone's IMU stream (IMU Stream app over Wi-Fi) through the foot filter, as it
happens. The receiver, tracker and recording session are imu_live's (imunav.live)."""
import collections, time
from pathlib import Path
import numpy as np
import foot  # noqa: F401  (puts imu_live on sys.path)
from imunav.live.receiver import Receiver, local_ip
from imunav.live.session import Session
from imunav.live.tracker import TRACKING, LiveTracker
from imunav.pipeline import PRESETS

FS = 400.0               # the filter's sample grid
STRIP_S = 30.0           # seconds of raw sensor magnitudes kept for the live strip charts
SPEED_S = 3.0            # walking speed = distance over the last few seconds
OUT_DIR = Path.home() / "Documents" / "IMU Live"


class StripTracker(LiveTracker):
    """LiveTracker that also keeps the last STRIP_S seconds of |gyro| (deg/s), |accel| (m/s^2) and foot-flat"""

    def __init__(self, cfg):
        super().__init__(cfg)
        self.strip = collections.deque(maxlen=int(STRIP_S * FS))

    def add(self, t, gyro, acc, gap=0.0):
        super().add(t, gyro, acc, gap)
        with self.lock:
            flat = self.state == TRACKING and self._still_last
            self.strip.append((t, np.degrees(np.linalg.norm(gyro)), float(np.linalg.norm(acc)), float(flat)))

    def strips(self):
        with self.lock:
            return np.array(self.strip) if self.strip else np.zeros((0, 4))


class LiveRun:
    """One live session: listens on `port`, tracks, records every sample to OUT_DIR. Raises OSError if the port is taken."""

    def __init__(self, port=5555):
        self.port = port
        self.tracker = StripTracker(PRESETS["shoe"])
        self.rx = Receiver(self.tracker, port, FS)
        self.session = Session(self.rx, self.tracker, OUT_DIR)
        self.rx.start()
        self.ip = local_ip()
        self._dist = collections.deque()          # (wall time, distance) for the walking speed

    def stop(self):
        self.rx.stop()
        self.session.close()

    def snapshot(self):
        s = self.tracker.snapshot()
        now = time.monotonic()
        self._dist.append((now, s["distance"]))
        while self._dist and now - self._dist[0][0] > SPEED_S:
            self._dist.popleft()
        t0, d0 = self._dist[0]
        s["walk_speed"] = (s["distance"] - d0) / (now - t0) if now - t0 > 0.5 else 0.0
        s["link_age"] = now - self.rx.last_rx if self.rx.last_rx else None
        s["rate"], s["lost"] = self.rx.acc_rate, self.rx.lost
        s["file"], s["rows"] = Path(self.session.path).name, self.rx.rec_rows
        return s
