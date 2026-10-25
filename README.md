# IMU + wheel-encoder + magnetometer fusion (2D)

Files
- fusion.h / fusion.c : portable filter (Madgwick tilt + 7-state EKF + ZUPT + gating). Same code runs on PC and ESP32.
- replay.c            : run the filter on a CSV log.      gcc -O2 -o replay replay.c fusion.c -lm
- sim.py / evaluate.py: synthetic runs with ground truth + ablation table.
- main.c              : ESP32 (ESP-IDF v5) firmware. NOT compiled by me - build it and send me the errors.

Quick start (PC)
    python3 sim.py 6                 # makes run_0..5.csv + gt_0..5.csv
    python3 evaluate.py 0 1 2        # TUNE on these
    python3 evaluate.py 3 4 5        # TEST (report this, never tune on it)

Replay CSV: t, ax,ay,az (m/s^2), gx,gy,gz (rad/s), mx,my,mz, vL,vR (m/s). Body frame x fwd, y left, z up.
No magnetometer: write nan in mx,my,mz.   Params: key=value on the command line (see replay.c header).

State: [x, y, yaw, v, gyro-z bias, accel-fwd bias, wheel-scale mismatch]
Updates: wheel speed -> v | (vR-vL)/b -> gyro bias + wheel mismatch | magnetometer -> yaw | ZUPT -> v, biases
Every update is innovation-gated (|y| > gate*sqrt(S) => rejected, counted in n_rej) -> this is the slip / disturbance defence.

## Build & run on Windows (no gcc needed)
    pip install ziglang
    python -m ziglang cc -O2 -o replay.exe replay.c fusion.c      (copy the .txt files to .c/.h first)
    python sim.py 6 ; python evaluate.py 3 4 5 ; python plot.py 3          # sim, ablation table, 4-panel PNG

## Real data: NCLT (UMich) - sessions 2012-01-08 and 2012-02-04 (12.1 km, 171 min)
    python nclt_convert.py nclt_data/2012-01-08 "Sample datasets/groundtruth_2012-01-08.csv" s0108   # -> s0108_run/_gt/_params/_meta
    python final_report.py nclt_tuned.json s0108 s0204       # ablation table + result_<session>.png
    python tune_multi.py out.json s0108,s0204 start=nclt_tuned.json      # re-tune (parallel coordinate descent)
    python sweep.py nclt_tuned.json mag_ref_s 60,120,300                # sweep one parameter
Run the scripts from the folder holding replay.exe and the converted CSVs (work/).
- Converter: MS25 z points down (y,z flipped); still period auto-detected; track width + mag hard-iron offset fitted from
  the sensor data (not GT); -nan GT rows dropped.
- Evaluation: rigid (+mirror) alignment on the first 60 s of motion only, so later drift is not hidden.
- Tuning: objective = RMSE over the first 40 min of each session; the rest is held out. Tuned settings: nclt_tuned.json.

Results, full filter (wheel speed + ZUPT + magnetometer):
| session    | RMSE  | first 40 min | after 40 (held out) | final error       |
| 2012-01-08 | 10.8 m | 9.2 m       | 11.9 m              | 11.9 m (0.18 %)   |
| 2012-02-04 | 20.2 m | 7.4 m       | 27.9 m              | 59.4 m (1.05 %)   |
IMU only: 801-1304 m RMSE. Wheel speed only: 61-156 m RMSE.

What mattered on real data (all now in fusion.c / main.c):
- Gate lock-out: a 3-sigma gate rejected every wheel update once v drifted -> gate_recover_s accepts after 1 s.
- Magnetometer reference averaged over mag_ref_s = 250 s of driving (gyro carries yaw meanwhile) instead of the first
  samples at the start spot: halved the error. Field-strength reference is averaged over the same window.
- ZUPT: zupt_w = 0.005 rad/s or slow pivot turns are learnt as gyro bias; zupt_s in seconds (rate-independent).
  On NCLT, ZUPT with these settings never fires at all (log: zupt 0/0) - it adds nothing there; the start-up
  gyro bias comes from the still-period average instead.
- Wheel yaw-rate vs gyro (enc_w) hurts on the Segway -> off. gate = 8 also means short wheel-slip bursts are no longer
  rejected (sim: 399 -> 0 rejected); fine on NCLT, re-check on a robot that slips.

## ZUPT position-jump fix (pos_consider)
Found on set 1 (2012-01-15): one ZUPT update moved the position 1,290 m, because x,y are never measured and their
covariance (and correlation with velocity) grows without bound. Fix: for sensors in the pos_consider bitmask the Kalman
gain rows for x,y are zeroed (Schmidt "consider" states) and P is updated in Joseph form. Default = ZUPT only (bit 8).
Chosen and validated on the training sessions + sim only (validate_consider.py); set 1 was NOT re-scored.
- ZUPT forced to fire on training data: max position jump 127 m -> 0.38 m (01-08), 146 m -> 0.76 m (02-04).
- Frozen settings: training results identical (ZUPT never fires there). Sim full filter unchanged.
- Protecting wheel/mag updates too (pos_consider=15) also kills small jumps but loses the useful heading->position
  coupling: full-filter RMSE 10.8 -> 14.2 m on 01-08, 0.22 -> 0.29 m on the sim. Not used.
Still open: ZUPT learns gyro bias from short stops (noisy); without the magnetometer that still hurts.

## ZUPT gyro-bias learning (zupt_bias_mode)
Old: during a stop, one gyro-bias update per sample, each treated as independent noise - so a 2 s stop looked like a
precise bias measurement, although a stopped robot's wobble is slow and correlated.
New (default, mode 1): collect the stop's gyro rate and apply ONE update when the stop ends (or every zupt_bias_max_s = 30 s
on a long stop), using the stop's mean rate with variance = spread/n + zupt_bias_floor^2 (3e-4 rad/s). Stops shorter than
zupt_bias_min_s = 1 s are ignored, and the newest zupt_bias_lag_s = 0.5 s of each stop is dropped because the detector
notices motion late. zb_mode=0 on the replay command line restores the old behaviour.
Validated on the training sessions + sim only (validate_zb.py / validate_zb.log); set 1 was NOT re-scored. RMSE vs old:
- Training sessions, frozen settings: identical (ZUPT never fires).
- Training sessions, ZUPT forced to fire: 01-08 wheel+ZUPT 187 -> 189 m, full 29.2 -> 20.8 m;
  02-04 wheel+ZUPT 515 -> 348 m, full 42.7 -> 15.2 m.
- Sim, default drift (seeds 0-5): better or equal in all three configs (e.g. wheel+ZUPT 0.188 -> 0.149 m).
- Sim with 5x gyro-bias drift (seeds 300-311): WORSE (wheel+ZUPT 0.316 -> 0.516 m): its stops are 1 s long and the
  new method skips them, while the sim's white gyro noise makes short stops reliable. Real-data noise is not white.
- Lag-only per-sample variant was tried and rejected (worse on real data).

## IMU calibration (bias, scale, misalignment)
See CALIBRATION.md: hand-held procedure (no equipment), imu_calib.py, log_capture.py, firmware LOG_RAW build env and
calib.h. Validated on synthetic sessions (calib_sim.py) and for navigation impact in the sim (nav_calib_check.py).
