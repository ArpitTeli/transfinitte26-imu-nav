"""What calibration buys in navigation (sim, 60 m loop). Per trial: draw MPU6050-scale IMU errors, record a synthetic
calibration session, calibrate with imu_calib.py, then drive the loop with the RAW sensor, the CALIBRATED sensor and an
IDEAL sensor (no scale/misalignment errors, small bias), and score each with the tier metrics.
usage: python nav_calib_check.py [trials]"""
import sys, os, numpy as np
from concurrent.futures import ProcessPoolExecutor
import sim, imu_calib as ic, calib_sim as cs
from targets import sim_job
CFG = [("IMU only", "enc_v=0 enc_w=0 mag=0 zupt=0"), ("IMU + wheels", "enc_v=1 enc_w=0 mag=0 zupt=1")]

def prepare(k):
    rng = np.random.default_rng(2000 + k)
    pa, pg, bg = cs.true_params(rng)
    t, a, w = cs.session(rng, pa, pg, bg)
    c = ic.calibrate(t, a, w, verbose=False)
    err = dict(Ca=ic.accel_matrix(pa), ba=pa[6:9], Cg=ic.gyro_matrix(pg))
    sim.make(500 + k, ".", imu_err=err, tag=f"cal{k}_raw")
    sim.make(500 + k, ".", tag=f"cal{k}_ideal")
    d = np.loadtxt(f"run_cal{k}_raw.csv", delimiter=",", skiprows=1)
    d[:, 1:4], d[:, 4:7] = ic.apply(c, d[:, 1:4], d[:, 4:7])
    np.savetxt(f"run_cal{k}_cal.csv", d, delimiter=",", fmt="%.6f", header="t,ax,ay,az,gx,gy,gz,mx,my,mz,vL,vR", comments="")
    open(f"gt_cal{k}_cal.csv", "w").write(open(f"gt_cal{k}_raw.csv").read())
    return k

if __name__ == "__main__":
    trials = int(sys.argv[1]) if len(sys.argv) > 1 else 12
    with ProcessPoolExecutor(12) as p:
        list(p.map(prepare, range(trials)))
        jobs = [(f"cal{k}_{v}", n, c) for k in range(trials) for v in ("raw", "cal", "ideal") for n, c in CFG]
        R = list(p.map(sim_job, jobs))
    print(f"{trials} trials, 60 m loop. Mean / worst over trials.")
    print(f"{'config':14s} {'sensor':12s} {'first 30 s max m':>17s} {'final m':>14s} {'final %':>14s} {'max m':>14s} {'return-to-start m':>18s}")
    for n, _ in CFG:
        for v, label in (("raw", "raw"), ("cal", "calibrated"), ("ideal", "ideal")):
            r = [x for x in R if x[1] == n and x[0].endswith("_" + v)]
            f = lambda i: f"{np.mean([x[i] for x in r]):6.2f} / {max(x[i] for x in r):6.2f}"
            print(f"{n:14s} {label:12s} {f(2):>17s} {f(3):>14s} {f(4):>14s} {f(5):>14s} {f(6):>18s}")
    for k in range(trials):
        for v in ("raw", "cal", "ideal"):
            for s in ("run", "gt"): os.remove(f"{s}_cal{k}_{v}.csv")
