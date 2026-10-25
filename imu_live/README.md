# IMU Live: the foot filter, the phone app and the walk tools

This folder holds the pieces behind shoe-mounted tracking:

- **`imunav`**, the Python foot filter;
- **IMU Stream**, the Android app that streams and records the phone's sensors;
- the **tools** that check, score and tune walks.

[Fusion Player](../fusion_player/) uses all of them for its live mode and walk playback. The root
[README](../README.md) explains how to use the apps end to end.

## Contents

| Path | What it is |
|---|---|
| `imunav/` | The filter: calibration (`calib.py`), stillness detection (`zupt.py`), the 15-state Kalman filter (`ins.py`) and one-sample-at-a-time navigation (`pipeline.py`, with the tuned `shoe` preset). |
| `imunav/live/` | The live link: packet format (`protocol.py`), the UDP receiver, lining up the two sensor streams, the live tracker, labelled recording sessions, and loading recordings. |
| `android/` | **IMU Stream** (Kotlin, no libraries). |
| `scripts/` | Tools, listed below. |
| `tests/` | 47 tests: `python -m pytest tests`. |
| `docs/data_collection.md` | How to record court walks: mounting, the walking routine, the walk list and a log sheet. |
| `docs/calibration_procedure.md` | Sensor calibration options for the foot filter: static bias, six-position, gyro scale, Allan variance. |
| `results/court_20261025/` | The court session: tracks and scores before tuning (`untuned`), after tuning on 5 walks (`tuned`) and the final settings (`final`), plus the tuning logs. |
| `results/synthetic/`, `results/*.csv` | Early comparison of filters on synthetic walks. |
| `configs/layout.example.json` | Column layout for loading other people's IMU logs with `run_compare.py`. |

## How the foot filter works

1. **Start-up.** The shoe must be still for 2 s. The filter measures the gyro bias and the direction of gravity,
   which sets the initial tilt.
2. **Every sample.** The gyro updates the orientation. The accelerometer, with gravity removed, updates the velocity,
   and the velocity moves the position.
3. **Every step.** For about 0.3 s the shoe is flat and still on the ground. A detector spots this from low rotation,
   steady acceleration, and an acceleration that gravity alone explains.
4. **While the foot is flat:**
   - **Zero velocity:** the speed is set to zero, which cancels the drift built up during the step.
   - **Zero rotation:** the gyro bias is re-learned.
5. **Safety checks.** Every correction is tested against what the filter expects. Outliers are rejected, and a
   diverged filter re-opens and recovers.
6. **Data gaps.** When Wi-Fi drops packets, nothing is integrated across the hole. The filter restarts at the next
   flat foot.

The `shoe` preset was tuned on the seven court walks of 25 Oct 2026. Two things mattered most:
- **Trusting each zero-velocity update less.** A real foot is never perfectly still at stance.
- **Assuming a noisier gyro than the datasheet suggests.**

The changes and the tuning record are in the preset's comment in `imunav/pipeline.py` and in
`results/court_20261025/`.

The phone cannot tell where north is, so heading drifts slowly. That is the main limit on accuracy.

## Tools

Run them from this folder.

| Script | What it does |
|---|---|
| `scripts/replay.py rec.csv --loop` | Runs a recording through the live tracker, prints distance and the final or return-to-start error, and saves a plot. |
| `scripts/diagnose.py rec.csv` | Physics checks with no ground truth, stride by stride: one stance per stride, sensible stride length, speed near zero at each stance. |
| `scripts/court_eval.py phone.csv windows.json` | Scores court walks against the court. Corners are found from the turns and are the known truth. Gives Tier 1, Tier 2, return-to-start, and per-walk plots of error against distance and time. |
| `scripts/court_tune.py phone.csv windows.json --train …` | Tunes the filter on chosen walks only, all walks in parallel. Stages: search, refine, ablate, leave-one-out. |
| `scripts/sim_phone.py` | Pretends to be the phone: streams a synthetic walk to Fusion Player's live mode. |
| `scripts/bench_realtime.py` | Measures how long the filter takes per sample, against the live budget. |
| `scripts/run_compare.py --synthetic` | Compares the filter options on synthetic walks. |
| `scripts/tune.py`, `scripts/run_allan.py`, `scripts/check_env.py` | Synthetic tuning, Allan variance of a static recording, and an environment check. |

`windows.json` names each walk and when it happened on the phone's clock, for example
`{"A1": [332080.4, 332344.0], ...}`. The laptop's per-walk files give these times: the first and last `t` of each file.

**Example:** score the court session, then look at `results/court_20261025/final/`.

```
python scripts/court_eval.py recordings/imu_20261025_041004.csv results/court_20261025/walk_windows.json --out results/court_20261025/final
```

Recordings are not in the repository. Put your own in `recordings/` (ignored by git).

## IMU Stream (the phone app)

- Reads the accelerometer and the raw (uncalibrated) gyroscope as fast as the phone allows.
- Saves everything to `Download/IMU Stream/imu_<date>_<time>.csv` on the phone.
- With a laptop address, it also streams over UDP (port 5555).
- Holds a Wi-Fi lock and keeps the screen on while recording.

**Packet format** (little-endian, must match `imunav/live/protocol.py`):

```
"IMU1" | uint32 seq | uint16 count | count × [uint8 type, int64 t_ns, float32 x, y, z]
type 1 = accelerometer (m/s², gravity included), 2 = gyroscope (rad/s)
```

**The phone's own file:**
- starts with `#` header lines (phone model, sensor names and ranges);
- then has rows of `type,t_ns,x,y,z`.

`imunav.live.recording.load_recording` reads it, and the laptop's CSVs too.

**Build:** `cd android`, then `gradlew.bat assembleDebug`. The APK is in `app/build/outputs/apk/debug/`.

## Results

**Court walks** (final settings, 7 walks, 1,000 m in all):
- typical error at the court corners 0.49 m;
- Tier 1 (first 30 s) 0.15 to 0.84 m;
- every 50 m stretch within 3 % of the distance;
- return to start 0.34 to 1.65 m.

**Synthetic walks** (early comparison, held-out seeds; 48 m rectangle with four stops):

| Filter | Final error | Drift |
|---|---|---|
| Madgwick, no zero-velocity updates | 65.0 m | 135 % |
| Madgwick + zero-velocity updates | 9.3 m | 19 % |
| Mahony + zero-velocity updates | 11.0 m | 23 % |
| **15-state Kalman filter + zero-velocity updates** | **0.66 m** | **1.4 %** |

That comparison is why the 15-state filter runs live.
