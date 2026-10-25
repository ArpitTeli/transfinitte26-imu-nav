"""Send the live path back to the phone, so IMU Stream draws what the laptop draws (PTH1 packets, protocol.py).

Phone sensors -> laptop (filter, laptop graph) -> this -> phone graph. The packets go to the address the phone
streams from, through the receiver's socket, so the phone needs no setting of its own.
"""
import collections
import math
import threading
import time

from .protocol import CALIBRATING_STATE, FLAG_STILL, MAX_POINTS, TRACKING_STATE, encode_path
from .session import ALIGN_M
from .tracker import TRACKING

SPACING = 0.1          # m between the path points sent to the phone
SPEED_S = 3.0          # walking speed = distance over the last few seconds, as on the laptop
LINK_S = 2.0           # stop sending once the phone has been silent this long


class PathSender(threading.Thread):
    """Every 1/hz s while the phone streams: one packet with the newest points, and one with a block of the older
    ones, taken in turn (a lost packet is repaired within a few seconds, and a phone that joins late gets all of it).

    The path is shown as on the laptop (session.heading_up): the start at the origin, turned so the first ALIGN_M
    metres point up once they have been walked. Turning it starts a new generation, as recalibrating does.
    """

    def __init__(self, receiver, tracker, hz=10.0):
        super().__init__(daemon=True)
        self.rx, self.tr, self.period = receiver, tracker, 1.0 / hz
        self.running = True
        self.sent = 0
        self.gen = 0                  # generation as the phone sees it
        self._tgen = None             # the tracker's generation being followed
        self._k = 0                   # tracker path points read so far
        self.raw = []                 # path points kept, SPACING apart, in the filter's frame
        self._p0 = None
        self._rot = None              # (cos, sin) of the turn that points the first ALIGN_M metres up
        self._cur = None              # newest position, filter frame
        self._repair = 0
        self._dist = collections.deque()

    def run(self):
        while self.running:
            time.sleep(self.period)
            try:
                self.tick()
            except OSError:
                pass                  # the network hiccupped or the socket is closing; try again next time

    def stop(self):
        self.running = False
        if self.is_alive():
            self.join(timeout=2.0)

    def _follow(self):
        gen, new = self.tr.path_since(self._tgen, self._k)
        if gen != self._tgen:
            self._tgen, self._k, self.raw, self._p0, self._rot, self._cur = gen, 0, [], None, None, None
            self.gen += 1
        self._k += len(new)
        for x, y in new:
            if self._p0 is None:
                self._p0 = (x, y)
                self.raw.append((x, y))
            elif math.hypot(x - self.raw[-1][0], y - self.raw[-1][1]) >= SPACING:
                self.raw.append((x, y))
            if self._rot is None and math.hypot(x - self._p0[0], y - self._p0[1]) >= ALIGN_M:
                a = math.pi / 2 - math.atan2(y - self._p0[1], x - self._p0[0])
                self._rot = (math.cos(a), math.sin(a))
                self.gen += 1
        if new:
            self._cur = new[-1]

    def view(self, pts):
        """filter frame -> the laptop's view frame"""
        x0, y0 = self._p0
        c, s = self._rot or (1.0, 0.0)
        return [(c * (x - x0) - s * (y - y0), s * (x - x0) + c * (y - y0)) for x, y in pts]

    def tick(self):
        self._follow()
        st = self.tr.status()
        now = time.monotonic()
        self._dist.append((now, st["distance"]))
        while now - self._dist[0][0] > SPEED_S:
            self._dist.popleft()
        t0, d0 = self._dist[0]
        speed = max(0.0, (st["distance"] - d0) / (now - t0)) if now - t0 > 0.5 else 0.0
        if self.rx.peer is None or now - self.rx.last_rx > LINK_S:
            return                    # the phone is not streaming
        tracking = st["state"] == TRACKING
        n = len(self.raw)
        head = dict(gen=self.gen, state=TRACKING_STATE if tracking else CALIBRATING_STATE,
                    flags=FLAG_STILL if tracking and st["still"] else 0, steps=st["steps"],
                    distance=st["distance"], speed=speed, motion=st["motion"][0] if st["motion"] else -1.0,
                    cur=self.view([self._cur])[0] if self._cur else (0.0, 0.0), n_total=n)
        a = max(0, n - MAX_POINTS)
        self._send(head, a, self.raw[a:])
        if a > 0:
            if self._repair >= a:
                self._repair = 0
            b = min(self._repair + MAX_POINTS, a)
            self._send(head, self._repair, self.raw[self._repair:b])
            self._repair = b

    def _send(self, head, start, pts):
        self.rx.send(encode_path(start=start, points=self.view(pts) if pts else [], **head))
        self.sent += 1
