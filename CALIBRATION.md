# IMU calibration procedure

Calibrates the accelerometer and gyroscope for **bias, scale factor and axis misalignment** without any equipment:
you turn the board by hand. Method: Tedaldi, Pretto and Menegatti, *A robust and easy to implement method for IMU
calibration without external equipments*, ICRA 2014.

## What gets calibrated

Each sensor is corrected in its own frame as `calibrated = C · (raw − b)`:

| Sensor | Bias `b` | Scale (diagonal of `C`) | Misalignment (off-diagonal of `C`) | Parameters |
|---|---|---|---|---|
| Accelerometer | 3 | 3 | 3 (upper triangular) | 9 |
| Gyroscope | 3 | 3 | 6 (full) | 12 |

The accelerometer's axes, made orthogonal, define the calibrated frame. The gyro misalignment is measured relative to
that frame, so both sensors end up in one common frame.

How it is solved:
1. **Gyro bias:** the mean gyro reading over the first 30 s, while the board is still.
2. **Accelerometer:** in every still pose the true reading has magnitude g. A least-squares fit of the 9 accelerometer
   parameters makes `|C (a − b)| = g` hold across all poses.
3. **Gyro scale and misalignment:** the calibrated accelerometer gives gravity's direction in each pose. Integrating
   the gyro between two consecutive poses must rotate the first gravity direction into the second. A least-squares
   fit of the 9 gyro scale and misalignment parameters makes that hold across all pose pairs.

The turn-on gyro bias of a MEMS IMU changes at every power-up. The firmware therefore still averages the gyro during
its 2 s still period at boot and removes what is left of the bias.

## What you need

- The ESP32 board with the MPU6050, connected by USB.
- The raw-logging firmware: `firmware/bin/lograw/` (prebuilt), or `pio run -e lograw -t upload` from `firmware/`.
- Python with `numpy`, `scipy` and `pyserial`.
- A table. Nothing else.

## Steps

1. **Flash the logging firmware.** From `firmware/` (in a folder path without an apostrophe):
   `pio run -e lograw -t upload`. Or flash the prebuilt binaries with esptool: `bootloader.bin` at 0x1000,
   `partitions.bin` at 0x8000, `firmware.bin` at 0x10000. In this mode the motors never run.
2. **Let it warm up for about 5 minutes,** powered. MPU6050 biases move with temperature; calibrate near the
   temperature you will run at.
3. **Start recording:** `python log_capture.py COM5 calib_log.csv --seconds 300` (use your port).
4. **Hold still for 30 s or more** with the board flat on the table. Don't touch it.
5. **Make 30 to 40 poses.** For each pose:
   - Turn the board smoothly to a new orientation over 1 to 3 s.
   - Hold it completely still for about 3 s. Rest your elbow or the board itself on the table.
   - Cover all six faces (each axis pointing up and down) plus many tilted orientations in between.
   - Turn about all three axes. Turns only about the vertical axis don't constrain the gyro.
6. **Stop** (Ctrl-C, or wait for `--seconds`).
7. **Run the calibration:** `python imu_calib.py calib_log.csv --counts`
   (add `--g 9.78` or your local gravity if you know it).
8. **Check the report** against the acceptance values below. Repeat the recording if any check fails.
9. **Install it:** copy the generated `calib.h` to `firmware/src/calib.h`, then build and flash the normal firmware
   (`pio run -e esp32dev -t upload`).

### Acceptance values

| Report line | Accept if | Typical (validation below) |
|---|---|---|
| `still poses used` | 20 or more (30 to 40 is better) | 36 |
| `orientation coverage` | above 0.1 | 0.24 |
| accel `\|a\|-g RMS` after | below 0.01 m/s² | 0.002 m/s² |
| gyro pose-to-pose error after | below 0.2° | 0.07 to 0.09° |

## Validation (synthetic sessions with known errors)

`calib_sim.py` generates calibration sessions whose sensor errors are drawn at MPU6050 datasheet scale: scale
±2% (1σ), misalignment 1° (1σ), accelerometer bias 0.3 m/s² (1σ) and gyro bias 0.05 rad/s (1σ). The sessions include
hand tremor during the still poses, hand acceleration during the turns, int16 quantisation and slow gyro-bias drift.
It then compares the recovered parameters with the truth.

Run: `python calib_sim.py 20 36` (20 sessions of 36 poses).

| | Uncalibrated (datasheet nominal) | Calibrated, 36 poses (median / worst) |
|---|---|---|
| Accelerometer error on a 1 g reading | 1.0 / 1.4 m/s² | **0.003 / 0.005 m/s²** |
| Heading error per 360° turn | 16 / 23° | **0.19 / 0.40°** |
| Accelerometer scale error | — | 0.009 / 0.025 % |
| Accelerometer misalignment error | — | 0.008 / 0.020° |
| Accelerometer bias error | — | 0.0008 / 0.0016 m/s² |
| Gyro scale error | — | 0.025 / 0.061 % |
| Gyro misalignment error | — | 0.020 / 0.055° |

Fewer poses degrade the worst case. With 20 poses the worst values are 0.009 m/s² and 0.38°; with 12 poses they are
0.093 m/s² and 1.5°.

### What it buys in navigation (sim, 60 m loop)

`python nav_calib_check.py 12` draws IMU errors for each trial, calibrates them with a synthetic session, then drives
the loop with the raw, the calibrated and an ideal IMU. Mean / worst over 12 trials:

| Filter | Sensor | Final error | Final / distance | Return to start |
|---|---|---|---|---|
| IMU + wheels | raw | 0.94 / 1.89 m | 1.57 / 3.15 % | 0.95 / 1.92 m |
| IMU + wheels | calibrated | **0.36 / 0.76 m** | **0.61 / 1.27 %** | **0.37 / 0.80 m** |
| IMU + wheels | ideal | 0.36 / 0.77 m | 0.60 / 1.28 % | 0.36 / 0.80 m |
| IMU only | raw | 13.2 / 34.3 m | 22.0 / 57.1 % | 13.2 / 34.3 m |
| IMU only | calibrated | **1.0 / 3.2 m** | **1.7 / 5.3 %** | **1.0 / 3.2 m** |
| IMU only | ideal* | 2.0 / 3.0 m | 3.4 / 5.0 % | 2.0 / 3.0 m |

\*The ideal sensor keeps the sim's small uncorrected accelerometer bias, which is why calibrated beats it.

With wheels, calibration brings the raw sensor to ideal-sensor accuracy, mainly by fixing the gyro scale error that
otherwise skews every turn.

## Limits and next steps

- **IMU-only error in the first 30 s is not a calibration problem.** It stays about 5 m whether the sensor is raw,
  calibrated or ideal, against Tier 1's 1 m. Part of it is the tilt filter (Madgwick) leaning toward the accelerometer
  while the robot accelerates: removing that correction cuts the 30 s error to about 3 m but lets tilt drift over a
  full run. Fixing this is filter work: tilt handling during acceleration, stop detection from the IMU alone, and
  smoothing for recorded traces.
- **Temperature:** biases move with temperature. Calibrate warm, near operating temperature. A temperature model is
  not included.
- **Mounting alignment** (IMU frame against robot frame) is separate. `imu_to_body()` handles axis remapping, and the
  boot still period handles tilt. A small yaw misalignment of the board on the robot is not estimated yet.
- **Magnetometer** hard- and soft-iron (ellipsoid) calibration is not part of this procedure.

## Files

| File | Purpose |
|---|---|
| `imu_calib.py` | Calibration tool: log → `calib.json` (parameters and report) + `calib.h` (firmware constants) |
| `log_capture.py` | Records the firmware's raw serial stream to CSV |
| `calib_sim.py` | Synthetic calibration sessions and the parameter-recovery validation |
| `nav_calib_check.py` | Navigation impact of calibration in the sim |
| `firmware/src/calib.h` | Calibration constants used by the firmware (`IMU_CALIBRATED 0` until you install one) |
| `main.c.txt` | Firmware: `imu_calibrate()` applies the constants; `LOG_RAW` build streams raw data |
