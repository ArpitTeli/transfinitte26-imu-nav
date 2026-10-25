# Data collection guide: phone on the foot, badminton court

This guide covers everything from installing the app to handing the files over. There are two ways to record, and the walking routine is the same for both:

- **Phone only:** you walk alone, and the phone saves everything. No laptop is needed.
- **Live link:** a friend walks with the phone and you watch the track on the laptop. The phone still saves its own copy, and that copy is the one used for scoring.

Make **one recording per walk**: press Start on the start corner and Stop when the walk is finished.

---

## 1. What to bring

- An Android phone with **IMU Stream** installed (`ImuStream.apk` from the repo's Releases page, or built from `imu_live/android`).
- Something that holds the phone rigidly on the shoe: a running armband, a velcro strap, or strong tape such as athletic tape or duct tape (section 4).
- Masking tape or chalk to mark the two start corners.
- A standard badminton court. The outer doubles lines are 13.40 × 6.10 m, so one lap is 39.0 m.
- The phone charged above 60%.
- Paper or a second phone for the log sheet (section 8).
- **Live link only:** the laptop with the project, and the walker's phone hotspot.

## 2. Install the app (once)

1. Copy `ImuStream.apk` to the phone by USB, Drive or Quick Share.
2. Open it on the phone. If Android asks, allow "Install unknown apps" for the file manager.
3. An older version, if installed, is updated.

## 3. Phone settings (check before every session)

- **Battery saver:** off.
- **Do Not Disturb:** on, so that calls and pop-ups can't cover the Stop button.
  - For **phone-only** sessions you can use airplane mode instead.
  - For **live-link** sessions, don't use airplane mode, because it turns the hotspot off.
- **Auto-rotate:** off.
- **Clock:** the phone's date and time must be correct, because the files are named by them.
- **During a walk:** keep IMU Stream open on screen and don't lock the phone or switch apps. The app keeps the screen on by itself.

In the app:
- **Laptop IP:** leave it empty for phone-only. For the live link, type the laptop's address.
- **Sample rate:** choose **Fastest**.

After **Start**, the screen should show:
- **RECORDING ON THE PHONE** (or **STREAMING + RECORDING** with the live link).
- `accel ... Hz` and `gyro ... Hz`, both 200 Hz or more (about 250 Hz on a Galaxy M17, up to 400+ on others).
- `saved: Download/IMU Stream/imu_YYYYMMDD_HHMMSS.csv`. Write this time in the log.
- `events saved`, which keeps counting up.

The file appears in the phone's `Download/IMU Stream` folder **only after you press Stop**.

## 4. Mounting the phone: on the shoe, not the ankle

**Why the shoe:** the filter gets position by integrating acceleration twice, and the phone's small sensor errors grow with the square of time:

| Time since the last reset | Error from a 0.5° tilt error |
|---|---|
| 1 s | 4 cm |
| 10 s | 4.3 m |

The filter keeps errors small by resetting often. Every step, the shoe sits flat and completely still on the ground for a fraction of a second. The filter uses that moment to reset the speed to zero and correct its tilt, so errors never build up for more than one step.

**Why not the ankle:** your ankle and lower leg are never still while you walk. Even with the foot planted, the lower leg rolls forward over it. A phone taped at the ankle could only reset at your 2 to 3 s corner pauses, which are 5 to 10 s of walking apart. That means metres of error on every side of the court, against a target of 1.5 m over 50 m. There are methods made for ankle sensors (counting steps and estimating step length), but they are less accurate, and we haven't built one.

**How to mount it on the shoe:** use the **left** shoe, with the phone touching only the shoe. Two ways work:

1. **On top of the shoe, over the laces (easiest):**
   - Lay the phone lengthwise, screen up, top edge towards the toes, centred on the laces.
   - Hold it with a running armband, a velcro strap or tape wrapped round the middle of the shoe: over the phone and under the arch.
   - Don't wrap anything under the heel or the ball of the foot, because walking would wear through it.
2. **On the outer side of the shoe:** lengthwise, top edge towards the toes, screen facing out. Only use this if the phone fits **below the ankle bone**.

**Check before every walk:**
- Push the phone with a finger. It must not move or rock relative to the shoe.
- Take a few steps and bend your knee forward. The phone must not touch your leg at any point.
- The orientation doesn't matter to the filter: the 5 s still at the start tells it which way is down, and the first 3 m tell it which way you're walking. It must be the same for every recording, though.

**If you have to re-mount it** during a session, write that in the log.

## 5. The court

Mark two corners at the **same end** of the court with tape, and label them 1 and 2:

```
                 face this way at the start  ---->

   corner 1  *------------------------------------------*
   (A, C,    |                                          |
    D, E)    |            13.40 m x 6.10 m              |  6.10 m
             |                                          |
   corner 2  *------------------------------------------*
   (B)                         13.40 m
```

At both corners you face the same way, along the long side:
- **From corner 1** the court is on your **right**, so every turn is a right turn. This is clockwise.
- **From corner 2** the court is on your **left**, so every turn is a left turn. This is anticlockwise.

## 6. The routine for every walk

1. Stand on the start corner with your **left foot on the corner point**, facing along the long side.
2. Press **Start**: bend down, tap it, and stand up straight. Keep the left foot planted.
3. **Stand completely still for 5 s.** Count them. Don't shift your weight.
4. Walk at the pace for this walk, with your **left foot on the line** at every step.
5. **At every corner** (every turn):
   - stop with your left foot on the corner point;
   - **stand still for 2 to 3 s**;
   - then turn and carry on.
6. **At the end of the last lap**, stop on the start corner and **stand still for 5 s**.
7. Press **Stop**. Write the walk code, the file time and anything unusual in the log.

**Counting laps:** each lap ends at the start corner. At the end of laps 1 to 4, pause 2 to 3 s there as at any other corner, then carry on. After the last lap, stand still for 5 s instead.

**Don't stop anywhere else.** If you trip, stop by accident or lose count of laps, carry on to the end. Then write down what happened and how many laps you actually did.

**Paces:**
- **Normal:** your everyday walking speed.
- **Slow:** about half speed. Take proper steps; don't shuffle.
- **Fast:** brisk walking, as if you are late. Never jog.

## 7. Session plan

### At home, before the court

| Code | What | Phone | How long |
|---|---|---|---|
| TEST | Test recording | On the foot | 1 min |
| CAL | Calibration | In your hand, **not** on the foot | 5 min |
| STATIC | Optional noise recording | On a table | 20 min |

**TEST:** Mount the phone and follow the routine (5 s still), walk around the room, then 5 s still, then Stop. Check three things:
- The rate was 200 Hz or more.
- The screen showed an `accel range` line. Note the value: ±8 g or more is good.
- The file is in `Download/IMU Stream`.

If you will use the live link at the court, do this test with the live link too (section 9).

**CAL:** this is for the Tier 1 calibration (bias, scale, misalignment).
1. Press Start with the laptop IP empty (phone only).
2. Lay the phone flat on a table, screen up, and **don't touch it for 40 s**. It needs at least 30 s.
3. Then put it in **30 to 40 different positions**, holding each **completely still for at least 3 s**.
   - Prop it against books or a box rather than holding it in the air, because a hand shakes.
   - Turn it smoothly from one position to the next, and don't drop it into place.
   - Cover every face pointing down (screen down, screen up, and each of the four edges), plus many tilted positions in between.
   - Make the turns between positions go about all three axes: flip it end over end, roll it sideways, and spin it flat.
4. Finish lying flat and still, then press Stop.

**STATIC (optional):**
1. Press Start.
2. Put the phone screen up on a steady table, away from fans and washing machines.
3. Don't touch it for 20 minutes, then press Stop.

This measures the sensor noise, which helps tune the filter.

### At the court, in this order

| Code | Walk | Start corner | About | Used for |
|---|---|---|---|---|
| A1 | 5 laps clockwise, normal pace | 1 | 4 min | Tuning |
| A2 | 5 laps clockwise, normal pace | 1 | 4 min | **Test (never tuned on)** |
| B1 | 5 laps anticlockwise, normal pace | 2 | 4 min | Tuning |
| B2 | 5 laps anticlockwise, normal pace | 2 | 4 min | **Test (never tuned on)** |
| C | 3 laps clockwise, slow | 1 | 3 min | Tuning (slow walking) |
| D | 3 laps clockwise, fast walking | 1 | 2 min | Tuning (hard heel strikes) |
| E | One long side there and back, 5 times | 1 | 3 min | Step-length check |

**Walk E:**
1. From corner 1, walk along the long side to the far corner (13.40 m).
2. Stand still for 2 to 3 s, turn round to your right, and walk back to corner 1.
3. Stand still for 2 to 3 s and turn round to your right again.
4. Do this 5 times there and back, finishing on corner 1 with 5 s still.

**Between walks:** the phone is stopped. Walk to the next start corner and check that the strap is still tight.

That is about 25 minutes of walking, or about an hour with setup and breaks. If court time is short, the minimum useful set is **A1, A2, B1 and E**.

## 8. Log sheet

Fill in one row per recording. The file time is the `HHMMSS` in the `saved:` line. If you redo a walk, write down both attempts and say which one counts.

| Code | File time (HHMMSS) | Laps done | Notes (tripped, re-mounted, missed a pause, ...) |
|---|---|---|---|
| TEST | | | |
| CAL | | | |
| STATIC | | | |
| A1 | | | |
| A2 | | | |
| B1 | | | |
| B2 | | | |
| C | | | |
| D | | | |
| E | | | |

## 9. Live link: a friend walks, you watch the laptop

The walking routine is exactly the same. In addition:

1. **Network:** turn on the walker's phone hotspot and connect the laptop to it.
   - Campus Wi-Fi often blocks phone-to-laptop traffic.
   - The first time, Windows asks about the firewall; allow **Private** networks.
2. **Laptop:** open **Fusion Player** and click **Live from the phone**. The side panel shows the address to use.
3. **Phone:** type that address into IMU Stream and choose Fastest.
4. **Each walk:**
   1. When the walker is on the start corner, **press N** (Next labelled walk). The panel shows the walk's code.
   2. **Then** the walker presses Start and stands still. The panel shows CALIBRATING and then TRACKING.
   3. The walker finishes the 5 s and sets off.
   4. While it calibrates, the panel shows the **shoe motion**. It must stay below 0.02 rad/s for 2 s.
      - If it stays above that while the walker stands still, the phone isn't tight on the shoe: re-strap it.
      - If TRACKING still doesn't appear, press **R** (New path), and they stand still for another 5 s.
5. **While they walk, watch:**
   - **LINK OK** (green) at about 250 Hz (the phone's rate).
   - The path, which starts at a dot and turns to point up the screen after 3 m. The phone draws the same path
     under its controls (IMU Stream 1.2 or later), so the walker can check it too.
   - Press **M** to timestamp anything worth knowing, such as a trip, a missed pause or a re-mount.
6. **At the end**, the walker stands still for 5 s and presses Stop. The panel then shows **LINK LOST**, which is normal between walks.
7. After the last walk, click **Stop live**.

If you forget N until after Start, press it while the walker is still standing, and have them stand for another 5 s.

The labels start again at A1 each time live mode starts. If you have to restart partway through, write down which walk you were on.

The laptop's copy is in `Documents\IMU Live`: one `<date_time>_<walk>.csv` per walk, plus `_markers.csv` for walks with markers.

## 10. After the session

1. Copy the phone's `Download/IMU Stream` folder to the laptop, by USB ("File transfer" mode), Quick Share or Drive.
2. Put the files in `imu_live/recordings/<date>_court/`, together with the log sheet. A photo of the sheet or a typed `log.txt` is fine.
3. For live-link sessions, leave `Documents\IMU Live` as it is.
4. Score the walks with `scripts/court_eval.py` (see `imu_live/README.md`). It finds each walk's corners, scores them against the court and plots the tracks. To tune, use some walks (for example A1, B1, C, D and E) and keep others (A2, B2) for testing.

## 11. If something looks wrong

| You see | Do |
|---|---|
| The rate is below 200 Hz | Check that Fastest is selected and close other apps. The filter needs about 100 Hz at the very least. |
| `accel range` is below ±8 g | Carry on, but tell me. Hard heel strikes may clip. |
| An `error:` line in the app | Write it down. If `events saved` stops rising, stop and start again. |
| The phone slipped or the strap loosened | Stop, re-mount, and redo the whole walk. Note it. |
| No file in `Download/IMU Stream` | It appears only after Stop. Check the `saved:` name on the screen. |
| LINK LOST in the middle of a walk | Keep walking; the phone's copy is complete. Move the laptop closer if it keeps happening. |
