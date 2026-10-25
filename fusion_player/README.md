# Fusion Player

Watch IMU navigation unfold, from recorded data or live from the phone on the shoe. The start page offers three ways in:

- **Recorded data:** a phone walk recording (`.csv`) through the foot filter, or an NCLT robot dataset through the
  IMU + wheel-encoder + magnetometer filter. Each is computed once, then played back at its own pace (1×) or faster.
- **Live from the phone:** the IMU Stream app's Wi-Fi stream through the foot filter, as the walker moves.
- **Demo:** a sample recording that ships with the app (`demo/court_walk_5_laps.csv`: five laps of a 13.4 × 6.1 m
  badminton court, phone on the shoe), processed and played back like any recorded walk. The first time takes about
  20 s; after that it opens from the cache.

## Run

    pip install -r requirements.txt
    run.bat            (or: python app.py)

Shortcuts: `python app.py walk.csv` opens a walk, `python app.py sen.tar.gz groundtruth.csv` an NCLT dataset, and
`python app.py --live` goes straight to live mode.

## Recorded data

1. **Open recorded data…** (Ctrl+O) and pick the kind:
   - **Phone walk recording:** the phone's own copy (`Download/IMU Stream/imu_….csv`) or the laptop's copy of a live
     session (`Documents/IMU Live/….csv`). The walk must start with the shoe still for 2 s. A 45-minute recording takes a few
     minutes the first time.
   - **NCLT robot dataset:** the sensor archive `<date>_sen.tar.gz` and `groundtruth_<date>.csv`, exactly as downloaded,
     and how the estimate is aligned to ground truth. The filter always uses the settings tuned on all 4 NCLT sessions
     (`settings/tuned_4_sessions.json`). The first run takes 10–30 s.
2. Results are cached in `cache/`; **Recent** reopens them instantly.
3. **Play** (Space). **Speed** sets the playback rate: 1× is real time, up to 600×. Drag the slider to jump anywhere.

| Tab | Phone walk | NCLT dataset |
|---|---|---|
| Path / Trajectories | The foot path from the start (first 3 m pointing up). The dot is green while the foot is flat, orange while it swings. | One map per condition, with ground truth drawn as it happens. |
| Position error | (no ground truth for a free walk) | Error against time and against distance, all conditions. |
| Heading, Speed | The filter's heading and walking speed. | Each condition against the ground truth. |
| Sensors | Rotation rate, acceleration and foot-flat, last 30 s. | Gyro yaw rate, forward acceleration and wheel speeds, last 60 s. |
| Summary | Distance, steps, start-up calibration, data gaps, filter settings. | RMSE, max and final error, self-calibration, settings, update counts. |

Court walks are scored against the court with `imu_live/scripts/court_eval.py`.

## Live from the phone

1. Turn on the phone's hotspot and connect the laptop to it.
2. Click **Live from the phone** (Ctrl+L). The side panel shows the address to type into IMU Stream (port 5555).
3. In IMU Stream: that address, **Fastest**, **Start**. The panel shows **LINK OK** with the rate. The path is sent
   back to the phone, which draws the same graph and compass under its controls (IMU Stream 1.3 or later).
4. **Calibrate compass** (once per phone, on open ground): click it, turn the phone in your hand over and around in a
   figure-8 until the panel says **CALIBRATED**. It is saved for next time (`%LOCALAPPDATA%\Fusion Player\compass.json`).
5. Strap the phone to the shoe; the walker stands still. After a compass calibration tracking starts by itself,
   otherwise press **R**. The panel shows the shoe's motion against the limit; after 2 s still it switches from
   **CALIBRATING** to **TRACKING**: go.

With a calibrated compass the filter pulls its heading back towards north at every step, so long walks stop
turning. The panel shows the compass heading (where the top of the phone points) next to the filter's, on a dial.

Keys: **K** recalibrate and keep the path (stand still 2 s; heading from the compass, or face the start direction
without one), **R** new path, **C** make the current position the start, **N** next labelled walk and **M** marker
(data collection). **Stop live** returns to the start page. Every live session is recorded to `Documents/IMU Live`,
compass readings included, and the phone keeps its own complete copy.

## How it works

- `engine.py` (NCLT): reads `ms25.csv` and `wheels.csv` straight from the archive, self-calibrates from the sensor data
  only (still period, track width, magnetometer offset), runs `filter/replay.exe` once per condition, aligns each
  estimate to ground truth and resamples onto a 10 Hz timeline. `filter/replay.exe` is built from the same `fusion.c`
  as the ESP32 firmware: `python -m ziglang cc -O2 -o filter/replay.exe filter/replay.c filter/fusion.c`
- `foot.py` (phone walks): loads the recording with `imu_live`'s loader, calibrates on the first still 2 s, runs the
  foot filter (`imunav`, shoe preset; with the saved compass calibration when the recording has compass readings)
  and builds a 20 Hz timeline.
- `livemode.py` (live): `imu_live`'s receiver, tracker, recording session and path sender (`imunav.live`). The
  sender sends the path back to the phone ten times a second, over the same UDP link.

The phone parts need the `imu_live` folder next to this one.

## Build the Windows installer

`python packaging/build_windows.py` (needs `pip install pyinstaller` and Inno Setup 6). It writes
`build/installer/Fusion-Player-Setup-<version>.exe`, with the phone APK and the guides included when they have been
built. On a PC that still has the older IMU Live app, the installer removes it; recordings in `Documents\IMU Live` are
kept.
