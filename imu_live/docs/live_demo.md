# Live demo: phone on the shoe, path on the laptop

> The laptop app is now **Fusion Player** (`fusion_player/`, installer `Fusion-Player-Setup-2.0.0.exe`), which
> replaces IMU Live: its start page offers recorded data or **Live from the phone**, with the same receiver, filter and
> recordings. See `fusion_player/README.md`. The IMU Live notes below still apply to `python scripts/live.py`.

```
Android phone (strapped to shoe)  --UDP over Wi-Fi-->  laptop: scripts/live.py
  IMU Stream app                                        align -> EKF + ZUPT -> live plot
  accel + uncalibrated gyro, 400+ Hz                    records every session to results/live/
```

## 0. Install the laptop app (Windows)

Run `IMU-Live-Setup-1.0.0.exe`. It installs **IMU Live** with Start-menu and desktop shortcuts and an uninstaller, and no Python is needed. Recordings go to `Documents\IMU Live`. The phone APK is included: Start menu, then IMU Live, then "Phone app (APK) folder".

- A normal install is per-user and needs no admin rights. Windows then asks about the firewall the first time IMU Live runs; allow **Private** networks.
- If you choose "Install for all users", the installer can add the firewall rule for you (UDP 5555, private networks only), and the uninstaller removes it.
- SmartScreen may warn about an unrecognised app because the installer is not code-signed. Click **More info**, then **Run anyway**.

To rebuild the installer: `python packaging/build_windows.py` (needs `pip install pyinstaller` and Inno Setup 6). Without installing anything, `python scripts/live.py` does the same as the app.

## 1. Install the app on the phone (once)

The built APK is `android/app/build/outputs/apk/debug/app-debug.apk`.

- **USB:** enable Developer options and USB debugging on the phone, plug it in, then run
  `C:\Android\Sdk\platform-tools\adb.exe install -r android\app\build\outputs\apk\debug\app-debug.apk`
- **No cable:** copy the APK to the phone and open it (allow "install unknown apps" for the file manager).

To rebuild after changing the Kotlin code: `cd android` then `gradlew.bat assembleDebug`.

## 2. Network

Phone and laptop must be on the same network, and the phone must be able to reach the laptop.

- **Recommended: phone hotspot.** Turn on the phone's hotspot and connect the laptop to it. Campus and hackathon Wi-Fi often block device-to-device traffic (client isolation), and a hotspot avoids that.
- The first time `live.py` runs, Windows asks whether Python may use the network. Allow **Private** networks. If you clicked Cancel, allow `python.exe` for UDP port 5555 in Windows Defender Firewall.

## 3. Run

1. Laptop: `python scripts/live.py`. It prints and shows `Phone <laptop IP>:5555`.
2. Phone: open **IMU Stream**, type that IP, choose **Fastest**, press **Start**. The phone shows the accel and gyro rates (expect 400 Hz or more on most recent phones), the accelerometer range, and the file it is saving to. Leave the IP empty to record on the phone only (no laptop needed). Keep the app's screen open while walking: Wi-Fi stays fastest with the screen on.
3. Strap the phone firmly to the shoe, touching only the shoe (on top over the laces, or on the outer side below the ankle bone; see [data_collection.md](data_collection.md), section 4). Not on the ankle: the lower leg is never still while walking, and the filter needs the foot-flat moment of every step. It must not wobble.
4. **Stand still for 2 s.** The laptop switches from CALIBRATING to TRACKING.
5. Walk. The dot is green while the foot is detected still and orange while it swings.

Keys in the laptop window: `r` recalibrate (stand still again), `c` make the current position the origin, `q` quit.

## 3b. Court session: one person walks, one watches the laptop

The walker has the phone on the left foot; you stay at the laptop beside the court. The walking routine, court
corners and walk list are in [data_collection.md](data_collection.md): one phone recording per walk (Start on the
corner, 5 s still, 2-3 s still at every corner, 5 s still at the end, Stop).

1. **Network:** turn on the walker's phone hotspot and connect the laptop to it (the court is well within range).
   Campus Wi-Fi often blocks phone-to-laptop traffic.
2. **Laptop:** `python scripts/live.py` (to continue a protocol already started: `--start B1`). Note the address it shows.
3. **Phone:** IMU Stream, type that address, **Fastest**. Keep the app's screen open while walking.
4. **Each walk:** when the walker is on the start corner, you press **n**; then the walker presses **Start** and stands
   still. The window shows the walk (A1, A2, B1, B2, C, D, E, in that order, with its start corner), CALIBRATING,
   then TRACKING. The walker finishes the 5 s and goes. At the end they stand still 5 s and press **Stop**; LINK LOST
   between walks is normal.
5. **While they walk**, watch:
   - **LINK OK** (green) with about 400 Hz. LINK LOST (red) mid-walk means Wi-Fi dropped; the phone's copy is still complete.
   - The **track on the court outline** (drawn once 3 m are walked) and the **lap count**.
   - Press **m** to mark anything worth knowing later (a trip, a missed corner pause, a re-mount); it goes into
     `<walk>_markers.csv` with the time.
6. After the last walk press **q**. The laptop files are `results/live/<date_time>_<walk>.csv`, one per walk; each
   starts when n was pressed. Afterwards copy the phone's files (`Download/IMU Stream`) to the laptop too: score from those.

## 4. Score a walk on the marked path

Every session is saved twice:
- **On the phone** (the complete copy): `Download/IMU Stream/imu_<date>_<time>.csv`. Copy it to the laptop by USB or any file share. Wi-Fi losses do not affect it, so **score walks from this file**.
- On the laptop: `results/live/<date_time>.csv` (installed app: `Documents\IMU Live`). Packets lost on Wi-Fi are marked as gaps in its `gap` column; nothing is invented to fill them.

Replay either file through the exact same tracker (the format is detected automatically):

```
python scripts/replay.py imu_20261025_101500.csv --align --end 50 0     # straight 50 m path
python scripts/replay.py imu_20261025_101500.csv --loop                 # walked back to the start
python scripts/diagnose.py imu_20261025_101500.csv                      # stride-by-stride checks, no ground truth needed
```

`--align` rotates the track so its first 3 m point along +x, because the phone does not know which way the path runs. The script prints the final error, drift as a percentage of distance, and the return-to-start error for loops, and saves a plot next to the CSV.

Tune on some walks and score on others. Example of trying a setting:
`python scripts/replay.py rec.csv --align --end 50 0 --set zupt_gyro_thr=0.6`

## 5. Without a phone

`python scripts/sim_phone.py` streams a synthetic shoe walk in the same packet format. Run it while `live.py` is open. `python scripts/live.py --headless 60` runs without a window and saves `results/live/dashboard.png`.

## Known limits

- The shoe preset is tuned on real court walks: walks A1, B1, C, D and E of 25 Oct 2026, with A2 and B2 held out for testing. Tuning uses `scripts/court_tune.py` and scoring `scripts/court_eval.py`. On the held-out walks, the RMS error at the court corners fell from 1.3 and 2.8 m to 0.7 and 0.6 m. It was tuned on one phone (a Galaxy M17) and one walker, so retune with those scripts if either changes.
- Heading is relative to the start; there is no magnetometer, so slow heading drift is possible.
- Wi-Fi drops show as "lost pkts" and "data gaps" on the laptop. The tracker does not integrate across a gap: it holds the position, re-levels at the next foot-flat, and carries on, but the distance walked during the gap is lost and heading may shift. Use the phone's own file for scoring.
- Recordings made before gap marking (Oct 2026) silently filled lost data with straight lines; `imunav.live.recording.load_recording` finds those stretches and turns them back into gaps.
