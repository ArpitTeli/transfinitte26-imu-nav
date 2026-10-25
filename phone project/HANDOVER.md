# Handover: foot-mounted phone navigation

Status as of 2026-10-25. Part 1 is built and tested on simulated data. Part 2 onward is not started.

## 1. Goal
Phone strapped to the **left foot** (screen facing outward/left, top edge up the shin). Stream IMU to the PC, show it live,
and estimate foot position, heading and distance using a filter built on the project's existing filter work.

## 2. Done (in `phone project/`)
| File | Purpose |
|---|---|
| `app.py` | Tk + matplotlib desktop app: raw IMU tab, filtered tab, CSV save, Demo button |
| `engine.py` | Samples -> axis mapping -> unit fix -> filter. Accelerometer is the clock; gyro/mag held between samples |
| `receiver.py` | HTTP server for the **Sensor Logger** app's "Push to HTTP" stream (`http://<pc-ip>:8000/data`) |
| `fusion_py.py` + `filter/fusion_dll.c` | ctypes binding to `filter/fusion.c` (unmodified copy of `firmware/src/fusion.c`), built with ziglang |
| `simulate_phone.py`, `test_headless.py` | Fake phone and a no-GUI end-to-end test |
| `README.md` | Phone setup and usage |

Verified only with the simulator: heading matched truth to 0.1 deg on clean data, GUI tabs render. **Never run with a real phone
or the real Sensor Logger app**, so its JSON field names, units (accel in g vs m/s^2, gyro in rad/s) and timestamps are unconfirmed.

## 3. Why the current filter is not enough
`fusion.c` is a **2D wheeled-robot** EKF (state x, y, yaw, forward speed, biases). A foot moves in 3D with sideways and vertical
motion and heavy impacts. Its tuned parameters (`nclt_tuned.json`, `fusion_player/settings/*.json`) come from a Segway at 100 Hz
and do not transfer. Kept and reused: the ZUPT idea, gyro-bias learning from stops, innovation gating, the replay/tuning tooling.

## 4. To do (in order)
1. **Verify the real phone link.** Run with Sensor Logger, 50-100 Hz, accelerometer + gyroscope + magnetometer. Check axis signs
   (stand still: the up axis should read +9.8 m/s^2), units, and timestamp jitter. Fix `receiver.py` / `engine.py` as needed.
2. **Left-foot axis mapping** in `engine.MAPPINGS`: forward = -phone x, left = +phone z, up = +phone y
   (derived for: screen facing left, top edge up). Check it against a real recording; if the strap differs, redo it.
3. **New 3D foot-mounted filter** as a separate module (do not edit `fusion.c`; keep the old one for comparison):
   - Strapdown INS: position, velocity, orientation, gyro and accel biases.
   - Stance detector (e.g. SHOE test on accel + gyro energy over a short window).
   - ZUPT (zero velocity) at each stance, plus zero-angular-rate update for gyro bias.
   - Magnetometer **off by default** (unreliable at foot level); optional, heavily gated.
   - Expose through the same C-wrapper / ctypes route, or pure NumPy if it is fast enough at 100 Hz.
4. **Court evaluator** (`evaluate_court.py`): rigid-fit the estimated path to the court rectangle and report
   shape/scale error, loop-closure error (% of distance), vertical drift, step count and stance-detection sanity.
5. **Record data** (see section 5), then **tune** noise, stance thresholds and bias parameters. Adapt `tune_multi.py` / `sweep.py`.
   Tune on the tuning set only; report on the held-out test set only.
6. **GUI additions:** left-foot mode, stance indicator, step count, distance, and a court overlay on the track.

## 5. Ground truth and recording protocol
Standard badminton court, doubles **13.40 m x 6.10 m** (singles width 5.18 m); one lap of the outer lines = **39.0 m**. Confirm the
real court matches. Walk the lines to about 0.3 m; use several laps so that error is small relative to distance.
Each recording starts and ends with ~5 s standing still (gyro-bias calibration).
- 5 laps clockwise, normal pace; 5 laps counterclockwise, normal pace.
- 5 laps slow; 5 laps fast walking.
- Out-and-back along the 13.4 m length, 5 times (cleanest step-length check).
- Keep at least one full recording as a **test set**; never tune on it.
Why a rectangle and not only a loop: a pure scale error still closes a loop, so the known dimensions are needed to see it.

## 6. Risks and limits
- Phone accelerometer range is often +/-8 g: heel-strike clipping hurts stance detection, worse at speed.
- Rates of 50-100 Hz over Wi-Fi with timestamp jitter, versus 200 Hz+ in dedicated foot units.
- Heading drift will dominate; realistic target is roughly 1-3 % of distance error, not a guarantee.
- Strap must be rigid; wobble looks like foot motion.
- Ground truth tolerance (~0.3 m) caps how finely small errors can be resolved per lap.
- Running is out of scope unless recorded.

## 7. Open questions
- Does the court match the standard size, and is the Sensor Logger Push-to-HTTP rate high enough on this phone?
- Exact strap pose: phone vertical on the outer side of the foot, or on the shin?
- Does the user want offline replay of recorded CSVs as well as live use?

## 8. How to run what exists
    cd "phone project"
    pip install -r requirements.txt
    run.bat                      (Demo button = simulated phone)
    python test_headless.py      (no GUI)
Rebuild the filter DLL after editing C: `cd filter && python -m ziglang cc -shared -O2 -o fusion.dll fusion.c fusion_dll.c`
