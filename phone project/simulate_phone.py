"""Pretend to be the phone: walks a figure-of-eight-ish path and POSTs Sensor Logger style JSON.
    python simulate_phone.py [port] [seconds]
Phone held flat, top edge pointing in the walking direction (phone x right, y forward, z up)."""
import json, math, sys, time, urllib.request
import numpy as np

port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
dur = float(sys.argv[2]) if len(sys.argv) > 2 else 60
URL = f"http://127.0.0.1:{port}/data"
HZ = 50
rng = np.random.default_rng(1)
G = 9.80665
B = np.array([20.0, 0.0, -42.0])           # earth field, uT (north, east?, down) in a world frame x=E? kept simple
t0 = time.time(); k = 0; yaw = 0.0; prev_yaw = 0.0
while time.time() - t0 < dur:
    t = k / HZ
    # 6 s still, then walk: 1.3 m/s with sinusoidal turning
    walking = t > 6
    rate = 0.5 * math.sin(2 * math.pi * (t - 6) / 20) if walking else 0.0   # rad/s, CCW
    yaw += rate / HZ
    fwd_acc = 0.8 * math.sin(2 * math.pi * 1.8 * t) if walking else 0.0     # step bounce, forward
    acc = [rng.normal(0, .05), fwd_acc + rng.normal(0, .05), G + (0.6 * math.sin(2 * math.pi * 3.6 * t) if walking else 0) + rng.normal(0, .05)]
    gyr = [rng.normal(0, .003), rng.normal(0, .003), rate + 0.004 + rng.normal(0, .003)]
    # magnetometer in the body frame: field rotates opposite to yaw
    c, s = math.cos(-yaw), math.sin(-yaw)
    mag = [c * B[0] - s * B[1] + rng.normal(0, .3), s * B[0] + c * B[1] + rng.normal(0, .3), B[2] + rng.normal(0, .3)]
    now = int((t0 + t) * 1e9)
    payload = [{"name": n, "time": now, "values": dict(zip("xyz", v))}
               for n, v in (("accelerometer", acc), ("gyroscope", gyr), ("magnetometer", mag))]
    k += 1
    try:
        req = urllib.request.Request(URL, json.dumps({"payload": payload}).encode(), {"Content-Type": "application/json"})
        urllib.request.urlopen(req, timeout=1).read()
    except Exception as e:
        print("post failed:", e); time.sleep(1)
    time.sleep(max(0, t0 + (k / HZ) - time.time()))
