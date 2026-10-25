"""Record the firmware's raw stream (build env "lograw") to a CSV for imu_calib.py (and later for replay).
usage: python log_capture.py COM5 calib_log.csv [--baud 921600] [--seconds 300]      (Ctrl-C stops early)
Output columns: t (s), ax, ay, az, gx, gy, gz (raw MPU6050 counts), cL, cR (encoder counts).
Then:  python imu_calib.py calib_log.csv --counts      -> calib.json + calib.h (copy calib.h to firmware/src/)"""
import argparse, time, serial

ap = argparse.ArgumentParser(); ap.add_argument("port"); ap.add_argument("out")
ap.add_argument("--baud", type=int, default=921600); ap.add_argument("--seconds", type=float, default=300)
A = ap.parse_args()
n, t0, last = 0, None, 0.0
with serial.Serial(A.port, A.baud, timeout=1) as s, open(A.out, "w") as f:
    f.write("t,ax,ay,az,gx,gy,gz,cL,cR\n")
    print(f"recording {A.port} @ {A.baud} for up to {A.seconds:.0f} s. Keep the IMU still for the first 30 s.")
    start = time.time()
    try:
        while time.time() - start < A.seconds:
            parts = s.readline().decode(errors="ignore").strip().split(",")
            if len(parts) != 9: continue                  # boot messages, header, partial lines
            try: v = [int(x) for x in parts]
            except ValueError: continue
            if t0 is None: t0 = v[0]
            ts = (v[0] - t0) * 1e-6
            f.write(f"{ts:.6f}," + ",".join(map(str, v[1:])) + "\n"); n += 1
            if ts - last >= 10: last = ts; print(f"  {ts:5.0f} s, {n} samples ({n / max(ts, 1e-9):.0f} Hz)", flush=True)
    except KeyboardInterrupt:
        pass
print(f"wrote {n} samples to {A.out}")
