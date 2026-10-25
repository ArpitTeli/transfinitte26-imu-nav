"""Headless test: run the simulated phone through Receiver -> Engine and print the result."""
import queue, subprocess, sys, time, math
from receiver import Receiver
from engine import Engine

q = queue.Queue(); r = Receiver(q, 8123); r.start()
sim = subprocess.Popen([sys.executable, "simulate_phone.py", "8123", "40"])
e = Engine(mapping="Flat on palm, top edge = forward", params={"mag": 1, "zupt": 1})
t_end = time.time() + 42
while time.time() < t_end and sim.poll() is None:
    try:
        e.feed(*q.get(timeout=0.2))
    except queue.Empty:
        pass
while not q.empty(): e.feed(*q.get())
r.stop()
o = e.last
print("state:", e.state, "| counts:", dict(e.counts), "| rate %.1f Hz" % e.rate, "| warn:", e.warn)
print({k: round(v, 3) for k, v in o.items()})
print("true final yaw (deg): sim integrates rate; filter yaw (deg): %.1f" % math.degrees(o["yaw"]))
