# IMU navigation: a phone on the shoe, tracked live on a laptop

Position tracking from motion sensors alone (accelerometer and gyroscope), built for the Embien Transfinitte'26
challenge.

An Android phone strapped to a shoe streams its sensors to a laptop over Wi-Fi. The laptop app, **Fusion Player**,
draws the walker's path as they walk. The filter cancels drift every time the foot is flat on the ground.

On real walks around a badminton court (13.40 × 6.10 m), the estimated position was typically **0.5 m** from the
truth after 100 to 200 m of walking. Every 50 m stretch stayed within **3 %** of the distance walked.

The repo also holds a second filter, for a wheeled robot (IMU + wheel encoders + magnetometer), written in C for an
ESP32. It is tested on the public NCLT robot dataset, which Fusion Player can play back too.

## What's in the repo

| Folder | What it is |
|---|---|
| [`fusion_player/`](fusion_player/) | **Fusion Player**, the Windows desktop app: live tracking from the phone, or playback of recordings and NCLT robot data. |
| [`imu_live/`](imu_live/) | The **foot filter** (Python), the **IMU Stream** Android app (`android/`), tools to score and tune court walks, and the data-collection guide. |
| [`robot_filter/`](robot_filter/) | The **wheeled-robot filter** (`fusion.c`, portable C) with its tools: simulation, NCLT conversion, tuning, tier scoring, IMU calibration. |
| [`firmware/`](firmware/) | **ESP32 firmware** that runs the robot filter on the board (PlatformIO). |

## Track a walk live, end to end

You need a Windows laptop and an Android phone (accelerometer and gyroscope; most phones have both).

### 1. Install the two apps

- **Laptop:** download `Fusion-Player-Setup-2.0.0.exe` from this repo's **Releases** page and run it.
  - No admin rights are needed.
  - Windows may warn about an unrecognised app, because the installer is not code-signed: click **More info**, then
    **Run anyway**.
  - To run from source instead, see [Run from source](#run-from-source).
- **Phone:** download `ImuStream.apk` from **Releases**, copy it to the phone and open it.
  - Allow "Install unknown apps" for your file manager when Android asks.
  - To build it yourself, see [Build from source](#build-from-source).

### 2. Connect the phone and laptop

1. Turn on the phone's **Wi-Fi hotspot** and connect the laptop to it.
   - Campus and office Wi-Fi often block phone-to-laptop traffic; a hotspot always works.
2. The first time Fusion Player listens for the phone, Windows asks about the firewall. Allow **Private networks**.

### 3. Strap the phone to the shoe

- Strap it **tight on the shoe**, not the ankle: on top over the laces (easiest), or on the outer side below the ankle
  bone. Use a running armband, a velcro strap or strong tape.
- It must not wobble or touch the leg. The filter relies on the moment in every step when the shoe is flat and still;
  the ankle never stops moving.
- Any orientation works, as long as it stays the same during a walk.

### 4. Start live tracking

1. On the laptop, open **Fusion Player** and click **Live from the phone**. The side panel shows the laptop's address.
2. On the phone, open **IMU Stream**:
   - type that address under **Laptop IP**;
   - choose **Fastest**;
   - press **Start**.
3. The phone shows **STREAMING + RECORDING**. The laptop shows **LINK OK** with the sample rate (about 250 Hz on a
   typical phone).
4. Keep IMU Stream open on screen while walking: Wi-Fi stays fastest with the screen on.

### 5. Calibrate, then walk

1. The walker stands **still**, with the phone on the shoe.
2. Press **R** on the laptop (Recalibrate). The panel shows **CALIBRATING** and the shoe's motion.
3. Once the shoe has stayed below the limit (0.02 rad/s) for 2 s, the panel turns green: **TRACKING**. Walk.
4. The path starts at a dot. Once 3 m have been walked, it turns so that direction points up the screen. The dot is
   orange while the foot swings and green when it is flat.

| Key | Does |
|---|---|
| **R** | Recalibrate: the walker stands still, and a new path starts. |
| **C** | Keep tracking, but make the current position the start. |
| **N**, **M** | Data collection only: start the next labelled walk, or timestamp an event (see the guide). |

### 6. Stop

- Press **Stop** in IMU Stream, then **Stop live** in Fusion Player.
- Both sides keep a recording:
  - the phone's own complete copy is in `Download/IMU Stream`;
  - the laptop's copy is in `Documents\IMU Live`.

## Record without a laptop, play back later

1. In IMU Stream, leave **Laptop IP** empty and press **Start**. The phone shows **RECORDING ON THE PHONE**.
2. Stand still for about 5 s, walk, stand still again, then press **Stop**.
3. Copy the file from the phone's `Download/IMU Stream` folder to the laptop (USB, Quick Share or Drive).
4. In Fusion Player, choose **Recorded data**, then **Phone walk recording**, and pick the file.
   - It is processed once; a 45-minute walk takes a few minutes the first time.
   - After that it opens instantly from **Recent**.
5. Press **Play**. **Speed** runs from real time up to 600×, and the slider jumps anywhere.

The tabs show the path, heading, walking speed, live sensor traces and a summary. For walks on a marked court, the
step-by-step protocol (start corner, pauses, laps) is in [`imu_live/docs/data_collection.md`](imu_live/docs/data_collection.md).

## Play back the wheeled-robot data (NCLT)

1. Download a session from the [NCLT dataset](https://robots.engin.umich.edu/nclt/): its sensor archive
   (`<date>_sen.tar.gz`) and ground truth (`groundtruth_<date>.csv`). Use the files exactly as downloaded.
2. In Fusion Player, choose **Recorded data**, then **NCLT robot dataset**, pick both files and press **Process**.
3. Play. Four sensor combinations run side by side against the ground truth:
   - IMU only;
   - IMU + wheels;
   - IMU + wheels + stop detection;
   - the full filter with the magnetometer.

## Results

**Shoe-mounted phone, court walks of 25 Oct 2026** (Samsung Galaxy M17, 7 walks, 1,000 m in all):

| | Result |
|---|---|
| Typical error at the court corners (median of 100) | 0.49 m |
| Tier 1: error in the first 30 s | 0.15 to 0.84 m on every walk (target: under 1 m) |
| Tier 2: 50 m stretches within 3 % of distance | 100 % of 69 stretches (median 0.6 %) |
| Return to the start corner | 0.34 to 1.65 m after 107 to 195 m |

Details, per-walk plots and the tuning record are in
[`imu_live/results/court_20261025/`](imu_live/results/court_20261025/).

**Wheeled-robot filter on NCLT** (4 sessions, 25.9 km in all): 12.5 to 26.6 m RMS error over runs of about 6 km
each, against hundreds of metres to kilometres for the IMU alone. See [`robot_filter/`](robot_filter/).

## Run from source

Python 3.12 on Windows.

```
pip install -r fusion_player/requirements.txt
pip install -r imu_live/requirements.txt
python fusion_player/app.py              # or fusion_player/run.bat
```

`python fusion_player/app.py --live` goes straight to live mode, and `python fusion_player/app.py walk.csv` opens a
recording.

To try live mode without a phone, run `python imu_live/scripts/sim_phone.py` in a second terminal. It streams a
synthetic walk to the laptop.

## Build from source

| What | How |
|---|---|
| Fusion Player installer | `python fusion_player/packaging/build_windows.py` (needs `pip install pyinstaller` and Inno Setup 6) |
| IMU Stream APK | `cd imu_live/android` then `gradlew.bat assembleDebug` (needs the Android SDK); the APK lands in `app/build/outputs/apk/debug/` |
| Robot filter (`replay.exe`) | `python -m ziglang cc -O2 -o replay.exe replay.c fusion.c` in `robot_filter/` (needs `pip install ziglang`) |
| ESP32 firmware | `pio run -e esp32dev` in `firmware/` (needs PlatformIO; build from a folder path without an apostrophe) |

Tests: `python -m pytest imu_live/tests` (47 tests).

## Tools and data

- **Libraries:** numpy, pandas, PySide6, pyqtgraph, matplotlib, scipy and pytest.
- **Android app:** Kotlin, no libraries.
- **Robot filter and firmware:** C (ESP-IDF on the ESP32).
- **Data:** the NCLT dataset (University of Michigan), and our own court recordings.
- Built with AI assistance (Claude Code).
