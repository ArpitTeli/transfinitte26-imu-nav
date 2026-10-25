"""Synthetic differential-drive run: 60 m square loop, known ground truth.
Writes run_<seed>.csv (sensor log) and gt_<seed>.csv. Imperfections: IMU bias+noise+mount tilt,
wheel-diameter mismatch, encoder quantisation, a wheel-slip burst, a magnetic disturbance, stops.
"""
import numpy as np, sys

def make(seed, out=".", disturb=0.08, bias_rw=2e-5, imu_err=None, tag=None):
    rng = np.random.default_rng(seed)
    dt, G, b = 0.005, 9.81, 0.15
    # ---- command profile: (v_cmd, w_cmd, duration)
    segs = [(0,0,3.0)]
    for k in range(4):
        segs += [(0.5,0,30.0), (0,0,1.0 if k!=1 else 3.0), (0,0.5,np.pi/2/0.5), (0,0,1.0)]
    segs += [(0,0,3.0)]
    vc, wc = [], []
    for v, w, T in segs:
        n = int(T/dt); vc += [v]*n; wc += [w]*n
    vc, wc = np.array(vc), np.array(wc); n = len(vc)
    t = np.arange(n)*dt
    # first-order smoothing (tau 0.3 s) -> continuous accelerations
    al = dt/0.3; v = np.zeros(n); w = np.zeros(n)
    for i in range(1, n):
        v[i] = v[i-1] + al*(vc[i]-v[i-1]); w[i] = w[i-1] + al*(wc[i]-w[i-1])
    psi = np.cumsum(w)*dt; x = np.cumsum(v*np.cos(psi))*dt; y = np.cumsum(v*np.sin(psi))*dt
    a_fwd = np.gradient(v, dt); a_lat = v*w

    # ---- IMU with mount tilt
    th, ph = np.deg2rad(1.0), np.deg2rad(0.5)
    u = np.array([-np.sin(th), np.sin(ph)*np.cos(th), np.cos(ph)*np.cos(th)])
    xs = np.array([1.0, 0, 0]); fs = xs - (xs@u)*u; fs /= np.linalg.norm(fs); ls = np.cross(u, fs)
    acc_bias = np.array([0.05, -0.03, 0.08])
    f_true = G*u[None,:] + a_fwd[:,None]*fs[None,:] + a_lat[:,None]*ls[None,:]
    w_true = w[:,None]*u[None,:]
    if imu_err is not None:   # uncalibrated sensor: raw = C^-1 true + b  (C, b as estimated by imu_calib.py)
        f_true = f_true @ np.linalg.inv(imu_err["Ca"]).T; w_true = w_true @ np.linalg.inv(imu_err["Cg"]).T
        acc_bias = imu_err["ba"]
    acc = f_true + acc_bias + rng.normal(0, 0.04, (n,3))          # (random draws kept in the original order)
    gb = np.array([0.010, -0.008, 0.030]) + np.cumsum(rng.normal(0, bias_rw*np.sqrt(dt), (n,3)), axis=0)
    gyr = w_true + gb + rng.normal(0, 0.003, (n,3))
    # ---- magnetometer (hard-iron offset, dip angle, disturbance 40-48 s)
    Bh, Bv, thn = 0.20, 0.35, 0.8
    hard = np.array([0.05, -0.03, 0.02])
    ang = thn - psi
    mag = Bh*(np.cos(ang)[:,None]*fs + np.sin(ang)[:,None]*ls) - Bv*u + hard + rng.normal(0, 0.004, (n,3))
    d = (t > 40) & (t < 48)
    D_world = np.array([0.08, 0.0])   # steel object: extra horizontal field in a fixed world direction
    dang = 0.0 - psi
    mag[d] += disturb*(np.cos(dang[d])[:,None]*fs + np.sin(dang[d])[:,None]*ls)
    # ---- wheel encoders: right wheel 0.4% larger, quantised counts, slip burst
    mpp = np.pi*0.065/2800
    vL_t = v - w*b/2; vR_t = (v + w*b/2)*1.004
    slip = (t > 25) & (t < 27); vL_t = vL_t.copy(); vL_t[slip] += 0.3
    def counts(vt):
        c = np.floor(np.cumsum(vt)*dt/mpp + rng.uniform()); dc = np.diff(c, prepend=c[0]); return dc*mpp/dt
    vL, vR = counts(vL_t), counts(vR_t)
    run = np.column_stack([t, acc, gyr, mag, vL, vR])
    tag = seed if tag is None else tag
    np.savetxt(f"{out}/run_{tag}.csv", run, delimiter=",", fmt="%.6f", header="t,ax,ay,az,gx,gy,gz,mx,my,mz,vL,vR", comments="")
    np.savetxt(f"{out}/gt_{tag}.csv", np.column_stack([t, x, y, psi]), delimiter=",", fmt="%.6f", header="t,x,y,yaw", comments="")
    return float(np.sum(np.abs(v))*dt)

if __name__ == "__main__":
    for s in range(int(sys.argv[1]) if len(sys.argv) > 1 else 6):
        print(s, "distance m:", round(make(s), 1))
