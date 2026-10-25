# Phone IMU Viewer

Desktop app that shows live IMU data (accelerometer, gyroscope, magnetometer) streamed from a phone, and runs the
project's own fusion filter on it (`filter/fusion.c` + `fusion.h`, unmodified copies of `firmware/src/`:
Madgwick tilt + 7-state EKF + ZUPT + innovation gating + magnetometer disturbance checks).

## Run
    pip install -r requirements.txt      (numpy, matplotlib; ziglang only if you want to rebuild the DLL)
    run.bat                               (or: python app.py)

Press **Demo (simulated phone)** to try it with no phone.

## Phone setup (Android or iOS)
1. Install **Sensor Logger** (free, by Kelvin Choi) on the phone. Phone and PC must be on the same Wi-Fi.
2. The app's header shows the URL, e.g. `http://192.168.1.20:8000/data`.
3. In Sensor Logger: *Settings > Data Streaming > Push to HTTP*, paste that URL, enable it. Turn on Accelerometer,
   Gyroscope and Magnetometer (set rate 50-100 Hz). If it offers "Total Acceleration", turn that on too - the filter
   needs gravity included in the acceleration.
4. Start recording in the phone app. Hold the phone **still for ~2 s**: the filter uses that to set gravity direction
   and gyro bias ("calibrating" in the header). Then move.
5. Pick **Phone orientation** in the app to match how you hold it (this maps phone axes to the filter's
   x-forward / y-left / z-up frame). Changing it resets the filter.
6. Windows Firewall may ask to allow Python on private networks - allow it, or the phone can't reach the PC.

## Tabs
- **Raw IMU** - scrolling last 20 s of accel, gyro, mag and |accel|.
- **Filtered (fusion.c)** - heading, roll/pitch, speed and the dead-reckoned track from the filter.
- Status line: rate, gyro bias, stationary flag, accepted/rejected magnetometer and ZUPT updates.
- **Save CSV...** writes the mapped samples + filter output.

## Notes / limits
- A phone has no wheel encoders, so the filter runs with `enc_v=0 enc_w=0`: IMU + magnetometer + ZUPT only. Heading
  and tilt are good; **position and speed from a phone IMU alone drift quickly** (that is the "IMU only" case in the main README).
- Gyro is held between samples (accelerometer is the clock), so use similar rates for all three sensors.
- Phone units assumed: accel m/s^2 (g is auto-detected), gyro rad/s, mag uT.
- Filter settings: `PARAMS` in `app.py`. Keys are listed in `filter/fusion_dll.c` (`fl_set`). Rebuild after editing C:
  `cd filter && python -m ziglang cc -shared -O2 -o fusion.dll fusion.c fusion_dll.c`

## Files
`app.py` GUI | `engine.py` sample pipeline | `fusion_py.py` ctypes binding | `receiver.py` HTTP server for the phone |
`simulate_phone.py` fake phone | `test_headless.py` no-GUI check | `filter/` C filter + DLL wrapper
