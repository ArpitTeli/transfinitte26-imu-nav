# Wheeled-robot filter: IMU + wheel encoders + magnetometer

A small Kalman filter in portable C (`fusion.c`) that tracks a wheeled robot from its IMU, wheel speeds and
magnetometer. The same code runs on a PC, through `replay.exe`, and on the ESP32 (see [`../firmware/`](../firmware/)).

On four sessions of the NCLT robot dataset (25.9 km), the full filter stays within 12.5 to 26.6 m RMS over runs of
about 6 km each. The IMU alone drifts by kilometres.

Run every tool from this folder. They call `replay.exe` and read their data files from here.

## Files

| File | Purpose |
|---|---|
| `fusion.c`, `fusion.h` | The filter. Copies live in `../firmware/src/` and `../fusion_player/filter/`; keep all three identical. |
| `replay.c`, `replay.exe` | Runs the filter on a CSV log. Rebuild with `python -m ziglang cc -O2 -o replay.exe replay.c fusion.c`. |
| `sim.py`, `evaluate.py`, `plot.py` | Synthetic 60 m robot loop with known truth, an ablation table, and a plot. |
| `nclt_convert.py`, `multi.py` | Convert an NCLT session to the filter's format, and score a run against ground truth. |
| `eval4.py`, `final_report.py`, `targets.py` | Ablation tables, plots per session, and scoring against the challenge tiers. |
| `tune_all.py`, `tune_multi.py`, `sweep.py` | Tuning: all sessions jointly, a train/hold-out split, or one setting swept. |
| `imu_calib.py`, `log_capture.py`, `calib_sim.py`, `nav_calib_check.py` | IMU calibration (bias, scale, misalignment). See [CALIBRATION.md](CALIBRATION.md). |
| `settings/tune4.json` | Final settings, tuned on all four NCLT sessions. `settings/nclt_tuned.json` was tuned on the first two. |
| `results/` | Final ablation numbers (`tune4_eval4.json`, `final_report.txt`) and plots. |

## How the filter works

The filter tracks seven numbers:
- position x and y, heading and speed;
- three slowly changing errors it learns as it goes: the gyro bias, the forward accelerometer bias, and the
  left/right wheel-size mismatch.

At every sample (200 per second on the robot):

1. **Tilt:** a Madgwick filter tracks roll and pitch, so gravity can be removed from the accelerometer.
2. **Predict:** the gyro turns the heading, the accelerometer changes the speed, and the speed moves the position.
3. **Correct:**
   - the wheel speed corrects the speed;
   - the magnetometer corrects the heading;
   - when the robot is stopped, a zero-velocity update sets the speed to zero and re-learns the gyro bias.

What made it work on real data:

- **Outlier check.** Every correction is compared with what the filter expects, and anything too far off is
  rejected. This handles wheel slip and magnetic disturbances. After 1 s of rejections the filter accepts again, so
  it can never lock itself out.
- **Magnetometer reference.** It is averaged over the first 250 s of driving, while the gyro carries the heading.
  Readings with the wrong field strength or angle are ignored. This halved the error.
- **Position shielded from stop corrections.** Before this, one stop correction once moved the position by 1.29 km.
  Position is now a "consider" state for zero-velocity updates.
- **One gyro-bias update per stop,** from the whole stop's average, instead of one per sample. A short, wobbly stop
  can no longer teach the filter a wrong bias.

## Quick start

**Simulation (no data needed):**

```
python sim.py 6              # run_0..5.csv and gt_0..5.csv
python evaluate.py 3 4 5     # ablation table on seeds 3-5
python plot.py 3             # plot_3.png
```

**NCLT data:**

1. Download a session from the [NCLT site](https://robots.engin.umich.edu/nclt/).
2. Take `ms25.csv` and `wheels.csv` out of its `<date>_sen.tar.gz` into a folder.
3. Convert and score:

```
python nclt_convert.py <that folder> groundtruth_2012-01-08.csv s0108
python final_report.py settings/tune4.json s0108         # ablation table + result_s0108.png
python eval4.py settings/tune4.json s0108 s0204 s0115 s0122
python targets.py settings/tune4.json                    # tiers; needs all four sessions and sim.py 6
python tune_all.py out.json s0108,s0204,s0115,s0122 start=settings/tune4.json   # re-tune
```

The converter learns everything from the sensor data, not the ground truth:
- the still period at the start (for the gyro bias);
- the wheel track width;
- the magnetometer's offset.

Scoring lines the estimate up with the ground truth using the first 60 s of driving only, so later drift is not
hidden.

## Results (settings tuned on all four sessions)

RMS position error over each whole session:

| Session | Length | IMU only | IMU + wheels | Full filter | Full filter, final error |
|---|---|---|---|---|---|
| 2012-01-08 | 6.5 km | 2,350 m | 56 m | **12.5 m** | 0.27 % of distance |
| 2012-02-04 | 5.6 km | 670 m | 162 m | **26.6 m** | 1.70 % |
| 2012-01-15 | 7.6 km | 10,918 m | 201 m | **18.6 m** | 0.20 % |
| 2012-01-22 | 6.2 km | 3,428 m | 121 m | **13.6 m** | 0.81 % |

Against the challenge tiers, with wheels and no magnetometer:
- **Tier 1:** the error in the first 30 s was 0.29 to 0.76 m.
- **Tier 2:** 92 % of 50 m stretches stayed within 3 %.

The magnetometer is what keeps heading from drifting over an hour. A robot rarely stops, so an IMU alone has nothing
to cancel its drift.

## Replay CSV format

```
t, ax,ay,az (m/s²), gx,gy,gz (rad/s), mx,my,mz, vL,vR (m/s)
```

The body frame is x forward, y left, z up. With no magnetometer, write `nan` for `mx,my,mz`. Filter options are
passed as `key=value` on the `replay.exe` command line; `replay.c` lists them.
