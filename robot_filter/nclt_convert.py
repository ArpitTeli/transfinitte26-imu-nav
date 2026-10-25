"""Convert NCLT sensors (ms25.csv + wheels.csv) to the replay CSV format and calibrate track width / mag offset.
usage: python nclt_convert.py <sen_dir> <groundtruth.csv> [out_prefix]
Writes <prefix>_run.csv, <prefix>_gt.csv (t,x,y,yaw) and <prefix>_params.txt
MS25 frame is x fwd, z DOWN -> converted to body x fwd, y left, z up by negating y and z of accel/gyro/mag.
"""
import sys, numpy as np
if len(sys.argv) < 3: sys.exit(__doc__)              # no arguments: show the usage, not a traceback
sen, gtf = sys.argv[1], sys.argv[2]
pre = sys.argv[3] if len(sys.argv) > 3 else "nclt"
ms = np.loadtxt(f"{sen}/ms25.csv", delimiter=",")
wh = np.loadtxt(f"{sen}/wheels.csv", delimiter=",")
gt = np.loadtxt(gtf, delimiter=",")
gt = gt[np.isfinite(gt).all(1)]                      # some sessions start with -nan rows
t0 = ms[0, 0]
t = (ms[:, 0] - t0) / 1e6
flip = np.array([1, -1, -1])
m = ms[:, 1:4] * flip; a = ms[:, 4:7] * flip; g = ms[:, 7:10] * flip
tw = (wh[:, 0] - t0) / 1e6
vL = np.interp(t, tw, wh[:, 1]); vR = np.interp(t, tw, wh[:, 2])

# startup still period: until the wheels first move (smoothed |v| > 0.1 m/s), minus a 2 s margin
k = max(1, int(1.0 / np.median(np.diff(t))))
vs = np.convolve(np.abs(vL) + np.abs(vR), np.ones(k) / k, mode="same")
t_move = t[np.argmax(vs > 0.2)]
still = t < max(2.0, t_move - 2.0)
gb = g[still].mean(0)
wz = g[:, 2] - gb[2]
dv = vR - vL
mv = (np.abs(vL) + np.abs(vR) > 0.6)
slope = np.sum(dv[mv] * wz[mv]) / np.sum(dv[mv] ** 2) if mv.sum() > 100 else 0.0   # = 1/b
track = 1.0 / slope if slope else float("nan")
if not 0.1 < track < 3.0:            # robot never drove, or a wild fit: the Segway's width is about 0.59 m
    print(f"track width fit failed ({track:.3f} m); using 0.59 m")
    track = 0.59
# magnetometer hard-iron offset: sphere fit  |m - c|^2 = r^2  ->  linear least squares
A = np.column_stack([2 * m, np.ones(len(m))]); bvec = (m ** 2).sum(1)
sol = np.linalg.lstsq(A, bvec, rcond=None)[0]
moff = sol[:3]
mn = np.linalg.norm(m - moff, axis=1)
print(f"track width {track:.3f} m  (regression, vs gyro)")
print(f"mag offset {moff.round(4)}  |m-off| mean {mn.mean():.3f} std {mn.std():.3f}  (raw |m| std {np.linalg.norm(m,axis=1).std():.3f})")
print(f"still rows (t<{t[still][-1]:.1f} s, first motion {t_move:.1f} s): {still.sum()}   gyro bias {gb.round(4)}")

np.savetxt(f"{pre}_run.csv", np.column_stack([t, a, g, m, vL, vR]), delimiter=",", fmt="%.6f",
           header="t,ax,ay,az,gx,gy,gz,mx,my,mz,vL,vR", comments="")
tg = (gt[:, 0] - t0) / 1e6
np.savetxt(f"{pre}_gt.csv", np.column_stack([tg, gt[:, 1], gt[:, 2], np.unwrap(gt[:, 6])]), delimiter=",", fmt="%.6f",
           header="t,x,y,yaw", comments="")
open(f"{pre}_params.txt", "w").write(f"still={int(still.sum())} track={track:.4f} moff={moff[0]:.5f},{moff[1]:.5f},{moff[2]:.5f}\n")
open(f"{pre}_meta.txt", "w").write(f"{t_move:.3f}\n")
print("wrote", f"{pre}_run.csv", f"{pre}_gt.csv", f"{pre}_params.txt")
