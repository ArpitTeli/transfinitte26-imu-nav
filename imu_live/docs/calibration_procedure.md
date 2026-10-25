# Calibration procedure

Anyone should be able to repeat this with the same board. Every step lists what to do, the expected output and the check that it worked.

## 0. Before you start

- Mount the IMU rigidly on the platform (no foam or loose cables between IMU and chassis).
- Let the board warm up for 5 minutes. Bias drifts with temperature.
- Log at a fixed rate (200 Hz). Confirm with `check_timestamps` that the median rate is within 5% and there are no gaps.
- Confirm units: gyro in rad/s (use `--gyro_scale` 0.01745 if the logger writes deg/s), accel in m/s² (multiply by 9.80665 if in g). A level, still board must read about +9.8 on the axis pointing up.

## 1. Static bias (every recording, automatic)

1. Keep the platform still and level for at least 2 s at the start of every recording.
2. `imunav.calib.static_calibration` averages the window and returns `gyro_bias`, `acc_mean`, `gravity`.
3. **Check:** `looks_static` is true. Gyro std is near the datasheet noise, and gyro bias magnitude is below about 0.05 rad/s. If it fails, the platform moved during the window; re-record.

The accelerometer bias cannot be separated from tilt with one pose. The pipeline uses the measured |gravity| as local g and lets the EKF estimate the rest.

## 2. Start-up routine for accelerometer bias (recommended)

Horizontal accel bias and tilt error look identical while the platform stays at one heading. Rotating changes that:

1. Stand still 3 s (pose A).
2. Rotate in place by 90° to 180°, then stand still 3 s (pose B).
3. Start moving.

In synthetic tests the first leg's error peaked near 7 m because no rotation had happened yet; after the first turn the bias estimate improved and later legs were better.

## 3. Six-position accelerometer calibration (own hardware)

1. Place the board on a flat surface with each axis pointing up (+x, −x, +y, −y, +z, −z), in that order. Hold each pose still for 20 s.
2. Take the mean of each pose to form a 6×3 matrix.
3. `S, b = imunav.calib.six_position_fit(means)`. The model is `acc_raw = S · a_true + b`. Apply with `apply_acc_cal(acc, S, b)`.
4. **Check:** after calibration, each pose's mean reads (±9.806, 0, 0) etc. within 0.02 m/s², and `S` is close to the identity (diagonal 0.98–1.02 for a decent part).
5. Save `S` and `b` with the date and board serial.

## 4. Gyro scale and misalignment (own hardware)

1. Record gyro with the board still for 5 s to get the bias.
2. Rotate the board by a known angle about each axis in turn (for example a 180° turn against a fixed stop, or a turntable). End still.
3. Integrate the bias-removed gyro over each rotation with `integrate_rotation`.
4. `M = gyro_scale_fit(integrals, angles)`. Apply with `apply_gyro_cal(gyro, M, bias)`.
5. **Check:** repeat one rotation and confirm the integrated angle is within 1% of the true angle.

## 5. Allan variance (noise parameters for filter tuning)

1. Record a static, level, undisturbed log. Longer is better: 2 h or more shows bias instability, 20 min shows noise density.
2. `python scripts/run_allan.py log.csv --fs 200` writes plots and prints N (noise density at τ = 1 s), B (bias instability) and K (random walk).
3. **Check:** the log-log curve has a −½ slope at short τ. A flat minimum gives B.
4. Use N for `sigma_g` / `sigma_a` and K for `sigma_bg` / `sigma_ba` in `Config`. Datasheet values are the fallback; the defaults are EuRoC (ADIS16448) values.

## 6. Validate

Run `python scripts/run_compare.py --imu ... --gt ... --layout ...` on a short known walk. A pass means the plots show the trajectory in the right frame (no mirrored or rotated path) before any tuning.
