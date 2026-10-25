# Fusion Player

Replays an NCLT dataset through the IMU + wheel-encoder + magnetometer fusion filter and draws the result as it
unfolds, at the robot's own pace (1×) or faster.

## Run

    pip install -r requirements.txt
    run.bat            (or: python app.py)

## Use

1. **Open dataset…** (Ctrl+O). Pick the two files exactly as downloaded from the NCLT site:
   the sensor archive `<date>_sen.tar.gz` and `groundtruth_<date>.csv`. Choose the filter settings profile and how the
   estimate is aligned to ground truth, then **Process**. The first run of a dataset takes 10–30 s; it is cached in
   `cache/`, and **Recent** reopens it instantly.
2. **Play** (Space). **Speed** sets the playback rate: 1× is real time, as the robot drove it; up to 600×.
   Drag the slider to jump anywhere. **Restart** goes back to the start.

## Graphs

| Tab | What it shows |
|---|---|
| Trajectories | One map per condition. Ground truth (dark) and the estimate (colour) are drawn as they happen, with dots on the current positions. The whole ground-truth path is shown faintly for context (toggle in the side panel). **Trajectories follow the robot** zooms in and tracks it. The maps pan and zoom together. |
| Position error | Error against time and against distance travelled, all conditions on one plot (log scale toggle). |
| Heading | One plot per condition, each with the ground-truth heading, wrapped to ±180°. |
| Speed | One plot per condition, each with the ground-truth speed. A condition that runs off the scale (IMU only) is pinned to the plot edge and its title says so. |

Heading and speed scroll with playback; **Heading and speed time axis** in the side panel sets the window (last 2, 10
or 30 min, or the whole run). Long runs are drawn as a min/max envelope, so spikes stay visible at any zoom.
| Sensors (live) | Scrolling last 60 s of the gyro yaw rate, forward acceleration, and both wheel speeds with ground-truth speed. |
| Summary | Whole-run RMSE, max and final error per condition, the self-calibration values, filter settings and update counts. |

The side panel shows the clock, distance travelled, and for each condition the error now, the RMSE so far and the
worst error so far.

## Conditions

Defined in the settings profile (`settings/*.json`): IMU only, IMU + wheels, IMU + wheels + ZUPT, and the full filter
(+ magnetometer). Add a profile by copying a JSON file and editing `args` (any `replay` option) or `conditions`.

## How it works

`engine.py` reads `ms25.csv` and `wheels.csv` straight from the archive, converts them to the filter's input,
self-calibrates from the sensor data only (still period, track width, magnetometer offset), runs `filter/replay.exe`
once per condition, aligns each estimate to ground truth and resamples everything onto a 10 Hz timeline.
`filter/replay.exe` is built from the same `fusion.c` as the ESP32 firmware:

    python -m ziglang cc -O2 -o filter/replay.exe filter/replay.c filter/fusion.c
