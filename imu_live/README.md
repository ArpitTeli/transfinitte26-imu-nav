# IMU-based indoor positioning (Embien Transfinitte'26, Problem 4)

Real-time dead reckoning from an IMU with zero-velocity updates. Python, numpy only for the filter code. **Aiding sources used: none.** The only external information is the zero-velocity, zero-rotation-rate and gravity constraints applied while the platform is detected as stationary.

## What is in here

| Module | Purpose |
|---|---|
| `imunav/io.py` | Configurable CSV loaders (EuRoC and custom layouts), timestamp checks, gap resampling, ground-truth interpolation |
| `imunav/calib.py` | Static bias, six-position accel fit, gyro scale/misalignment fit, Allan variance |
| `imunav/orientation.py` | Madgwick, Mahony, multiplicative attitude EKF (all one sample at a time) |
| `imunav/zupt.py` | Causal stationary detector and its evaluation against ground truth |
| `imunav/ins.py` | Strapdown integration and the 15-state error-state EKF (ZUPT, ZARU, gravity updates, chi-square gating) |
| `imunav/pipeline.py` | `Navigator.step()`: one sample in, one state out, no I/O and no look-ahead |
| `imunav/metrics.py`, `plots.py` | Final/RMSE/30 s error, return-to-start, drift %, all plots |
| `imunav/synth.py` | Synthetic walks (body-mounted and shoe-mounted) with known path, bias, noise and scale error |
| `imunav/live/` | UDP protocol, accel/gyro stream alignment, live tracker and receiver for the phone app |
| `android/` | IMU Stream Android app (Kotlin, no libraries) |

## Live demo (phone on the shoe)

An Android app (`android/`) streams the phone's IMU over Wi-Fi to `scripts/live.py`, which draws the walked path in real time. See **[docs/live_demo.md](docs/live_demo.md)** for installing the app, the network setup and scoring walks with `scripts/replay.py`.

## Setup

```bash
pip install -r requirements.txt
python scripts/check_env.py
python -m pytest
```

## Run

```bash
python scripts/run_compare.py --synthetic                              # all filters, plots in results/synthetic
python scripts/run_compare.py --synthetic --seeds 10 11 12 13 14       # mean table over held-out seeds
python scripts/extract_euroc.py machine_hall.zip MH_01_easy            # after downloading the 12 GB zip by hand
python scripts/run_compare.py --euroc data/euroc/MH_01_easy
python scripts/run_compare.py --imu imu.csv --gt gt.csv --layout configs/layout.json   # Embien traces
python scripts/tune.py --synthetic-seeds 1 2 3 --mode ins --grid "zupt_res_thr=0.2,0.3" "gate_prob=0.99,0.999"
python scripts/run_allan.py static_log.csv --fs 200
python scripts/bench_realtime.py
```

For Embien's traces, copy `configs/layout.example.json` and edit the column names, time scale (`time_scale` converts the time column to seconds), and unit scales (`gyro_scale` 0.01745 for deg/s, `acc_scale` 9.80665 for g).

### Data format

IMU CSV: time, gyro x/y/z (rad/s after scaling), accel x/y/z (m/s² after scaling), ideally 200 Hz. Ground-truth CSV: time, position x/y/z (m), and optionally quaternion w/x/y/z (body to world) and velocity. Frame: world z is up; the start position and heading are taken from the first ground-truth sample.

## Method in one paragraph

Gyro bias and local gravity come from the stationary start. A causal detector flags still periods (gyro magnitude, accel-magnitude variance, and specific force not explained by gravity). The 15-state EKF integrates the IMU and, while still, applies zero-velocity, zero-angular-rate and gravity-vector measurements at about 10 Hz. Each measurement goes through a chi-square gate, so false "still" detections during slow starts get rejected, and the counts are logged. The Madgwick and Mahony pipelines (orientation filter plus strapdown plus velocity reset) are kept as comparisons.

## Results so far (synthetic data only)

Synthetic rectangular walk: 48 m path, four stops, EuRoC-grade noise, gyro bias about 0.002 rad/s, accel bias about 0.04 m/s². Filter settings were tuned on seeds 1 to 3. The table is the mean over **held-out seeds 10 to 14**.

| Pipeline | Final error | RMSE | Max error | Error 30 s after motion starts | Drift % of distance |
|---|---|---|---|---|---|
| Madgwick, no ZUPT | 65.0 m | 29.1 m | 65.0 m | 11.8 m | 135 % |
| Madgwick + ZUPT | 9.3 m | 5.6 m | 9.5 m | 4.8 m | 19 % |
| Mahony + ZUPT | 11.0 m | 6.5 m | 11.0 m | 5.4 m | 23 % |
| Attitude-EKF + ZUPT | 5.3 m | 15.2 m | 23.4 m | 23.4 m | 11 % |
| 15-state EKF, no ZUPT | 111.1 m | 45.0 m | 111.1 m | 9.2 m | 230 % |
| **15-state EKF + ZUPT** | **0.66 m** | **2.5 m** | **7.8 m** | **5.1 m** | **1.4 %** |

Other measured facts:
- Per-sample update time: worst case about 1.1 ms, p99 under 0.4 ms (budget 5 ms), on this laptop's Python.
- Mid-run error is much larger than final error: the EKF's worst point averages 7.8 m, inside the first leg. The 30 s error is measured from the start of motion, as the problem statement defines it, and averages 5.1 m, so **the core-tier "under 1 m over the first 30 s of motion" target is not met on synthetic data**. See `docs/calibration_procedure.md` section 2 for why and the mitigation.
- The stationary detector flags about 12 % of moving samples as still (mostly slow ramps) and misses about 20 % of real stops (turns in place). The EKF gate absorbs most of the false positives.

**Not yet done:** EuRoC and Embien traces have not been run. EuRoC is a flying drone with almost no stops, so ZUPT will help little there. Expect large drift on it and use it for calibration and orientation checks only. Real performance on Embien's marked-path traces is the number that matters.

## Honest limitations

- Yaw is not observable without a heading aid; it drifts with residual gyro bias. Over a 50 m path this is small but real.
- Accuracy depends on stops. With no stops the error grows quickly.
- Accel bias and tilt are only separable after the platform has rotated between two stops (see calibration doc).
- The synthetic generator is easier than a real walk or robot; the real traces decide the outcome.
- No hardware logger, fixed-point port or own-hardware data yet.

## Tools, libraries and datasets (state these in the report)

numpy, scipy (chi-square quantile), pandas (CSV loading), matplotlib (plots), pytest. Datasets: synthetic generator (own), EuRoC MAV (if used), Embien traces (if used). The code was written with AI assistance (Claude Code); disclose that per the hackathon's own-work rule.

## Next steps

1. Run on Embien's traces and tune on a subset only; keep one trace held out for the final number.
2. Pick one aiding source if a bonus tier is wanted (UWB or tag anchors), and declare it.
3. Port `Navigator.step` to C for ESP32 or STM32 (the filter has no I/O and fixed-size 15×15 matrices); compare against this Python reference.
