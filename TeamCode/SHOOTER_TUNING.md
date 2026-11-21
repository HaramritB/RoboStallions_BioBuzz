# Shooter PIDF tuning and distance calibration

Current V2 TeleOp trial: **P=180, I=D=0, reference F=12.71 at 13.3 V** (P was 90 until 2026-11-12), set in `Robot.java` as
`V2_TELEOP_SHOOTER_P` and `V2_TELEOP_SHOOTER_F`. Only Blue/Red V2 TeleOp use these; the original
OpModes and AutoPath keep `SHOOTER_P` / `SHOOTER_F`. **Flywheel Tuning + Burst Logger** starts from
the same reference F, so its results go straight back into those two constants.

The latest calibrated distance table is:

| Camera-to-tag distance (m) | Base RPM |
| --- | --- |
| 1.39 | 3215 |
| 1.52 | 3220 |
| 1.62 | 3235 |

The tuner cycles **3215 -> 3220 -> 3235 -> 3300 -> 3215**: every table speed plus 3300 as
headroom above the farthest shot so far.

## Where the logs are

Both files are on the **Control Hub**, in `/sdcard/FIRST/java/src/Datalogs/`:

| File | Written when | Contents |
| --- | --- | --- |
| `shooter_volleys.csv` | Every volley, recording or not | One row per volley: P, F, battery, target, launch RPM of balls 1-4, ball count, min RPM, drop, recovery time |
| `shooter_burst_<time>.csv` | While recording (A) | Every loop sample: RPM, error, battery, P/F, gate/feeder state, loop time, volley number, dip count, events |

`shooter_volleys.csv` keeps growing across runs (new rows are appended), so one pull covers a
whole test session. Telemetry shows both full paths under **FILES**.

**Getting them onto a laptop** (robot on, laptop on the robot's Wi-Fi):

```sh
~/Library/Android/sdk/platform-tools/adb connect 192.168.43.1:5555
~/Library/Android/sdk/platform-tools/adb pull /sdcard/FIRST/java/src/Datalogs ~/Downloads/Datalogs
```

Or in Android Studio: **View > Tool Windows > Device Explorer**, open
`/sdcard/FIRST/java/src/Datalogs`, right-click the files, **Save As**.

## Voltage scaling

```text
applied F = reference F * 13.3 / measured battery voltage
```

13.3 V is a fixed normalization point, not a battery reading; never change it. At 12.8 V the tuner
applies 12.71 x 13.3 / 12.8 = **13.21**. Always save **reference F**, never applied F. Scaling keeps
the steady speed right as the battery drops, but a lower battery still has less power left over for
recovering between balls, so recovery looks worse on a low battery. That is real; compare P values
on the same battery.

## Tuner controls (gamepad 1)

| Input | Action |
| --- | --- |
| Left stick click | Flywheel on/off |
| Y | Next test RPM (3215 / 3220 / 3235 / 3300) |
| **Right stick click** | **P preset: 90 -> 150 -> 180 -> 90** |
| D-pad up/down | P up/down by the step size |
| D-pad right/left | Reference F up/down by the step size |
| B | Step size: 0.1 -> 0.01 -> 0.001 -> 0.0001 -> 10 -> 1 -> 0.1 |
| Back | P to zero (F-only tuning) |
| A | Start/stop the sample CSV (does not touch the flywheel) |
| X | Gate open/closed (opening backs the transmission off 100 ms, feeding waits 250 ms) |
| RT / LT (hold) | Load inward / eject outward |
| RB (hold) | Feed a volley (needs flywheel ON and gate OPEN) |
| LB (hold) | Mark a volley fed some other way (no motors) |

It starts with **P=0**, reference F **12.71**, step **0.1**, flywheel off and gate closed.
It does not drive, aim the turret or move the flower.

## What the readouts mean

- **Target / actual / error**: live speed.
- **Velocity steps**: the smallest change the hub's speed reading can show. If it says
  **20 ticks/s = 42.9 RPM**, readings jump in ~43 RPM steps, so a +/-10 RPM window cannot be judged.
- **Loop**: how often the tuner samples. Lower is better for seeing each ball's dip.
- **VOLLEYS**: one line per volley, newest first, for example
  `#3 P=150 F=12.71 @3235: launch 3248/3205/3161/3120 (1st-last 128) min 2990 drop 258 rec 310 ms`
  - **launch**: speed just before each ball hit the wheel (ball 1 / 2 / 3 / 4).
  - **1st-last**: how much slower the last ball launched than the first. **This is the main number.**
  - **min / drop**: lowest speed in the volley and how far below the starting speed that was.
  - **rec**: time from the last ball's dip until speed is back within +/-50 RPM (held 150 ms).
  - Fewer than 4 launch numbers for 4 balls means some dips merged: the wheel never sped back up
    between those balls. That is useful information in itself.
  - `launch none` means no dip was bigger than 60 RPM; compare **min / drop** instead and send the
    sample CSV.

## The P comparison test (full steps)

Goal: find the lowest P that keeps ball 4 launching about as fast as ball 1. Takes ~30 minutes.

### 0. Prepare

1. Build and deploy. Charge or swap to a full battery if you can (12.8 V resting works, but use the
   **same battery for every trial**). Note the voltage on telemetry at the start and end.
2. Stand the robot where balls can fly safely (ideally at a spot you shoot from, aimed at the HIVE,
   so you can count makes too). The tuner does not aim the turret: point it by hand or with
   V2 TeleOp first, then switch OpModes.
3. Have 4 balls ready for every volley, loaded the same way each time.

### 1. Check feedforward (P = 0), about 3 minutes

1. Select **Flywheel Tuning + Burst Logger** (Testing group), INIT, START.
2. Check telemetry: **P / reference F = 0 / 12.71**, **Battery** shows a real voltage.
3. Click the **left stick**: flywheel spins to **3215**. Wait 3 seconds.
4. Watch **error**. If it averages within about **+/-50 RPM**, F is fine. Note **Velocity steps**.
5. Press **Y** to step through 3220, 3235, 3300, waiting ~3 s at each.
6. Only if error is consistently one-sided at all speeds: D-pad right (too slow) or left (too fast),
   press B once for 0.01 steps when close. Write down the final reference F.

### 2. Compare P values with four-ball volleys, about 20 minutes

Use **3235 RPM** (your farthest shot so far); press Y until target reads 3235.

For **each volley**:

1. Click the **right stick** until **P preset** shows the P for this trial (90, 150 or 180).
2. Gate closed. Hold **RT** to load 4 balls, release, wait **2 seconds** for the speed to settle.
3. Press **A** to start a sample recording (optional; the volley summary is saved anyway).
4. Press **X** to open the gate, then hold **RB** until all 4 balls are out (~1 s). Release RB.
5. Press **X** to close the gate. Wait until the volley line appears under **VOLLEYS** (up to
   1.5 s). Press **A** to stop recording if you started one.
6. Write down the volley line and, if aimed, how many of the 4 went in.

Run **3 volleys per P in this order** so battery drain affects every P equally:

```text
90, 150, 180,   180, 150, 90,   90, 150, 180
```

Optional: repeat the order at **3300**.

### 3. Pick P

Compare the **1st-last** numbers (and makes):

- Pick the **lowest P whose 1st-last stays under ~50-75 RPM**, with recovery under ~300 ms and no
  hunting (speed swinging above and below target) while idle.
- If even **P=180** shows ball 4 launching **100+ RPM slower**, the balls come faster than the wheel
  can recover. More P will not fix it: space the balls out (slower feed) or raise the RPM target
  briefly while feeding. Send the logs and we will change that next.
- If **1st-last is small but ball 4 still falls short** when shooting, the cause is mechanical (the
  last ball has nothing pushing it into the wheel), not the PIDF.

Then set `V2_TELEOP_SHOOTER_P` (and `V2_TELEOP_SHOOTER_F` if step 1 changed it) in `Robot.java`,
rebuild, deploy, and re-check a few volleys in V2 TeleOp. Pull `shooter_volleys.csv` and send it
along if you want a second look. Live tuner changes are never saved on the robot: they reset when
the OpMode restarts, and `Robot.java` is the only place gains are stored.

## 4. Calibrate distance to RPM in V2

1. Select the matching Blue/Red V2 TeleOp. Park at the first measured distance, stay still,
   and confirm the turret is pointing at the upward CELL.
2. Press **A** to turn on the shooter. **X** toggles the gate and **RT** feeds inward.
3. Click the **left stick** to enter calibration and freeze the table's base RPM. Begin with
   the existing RPM for that distance. D-pad up/down trims by **10 RPM** per press; hold **LB**
   while pressing up/down for **1 RPM**. Back clears trim; left stick click exits calibration.
   `Speed stable` indicates measured speed stayed within **+/-10 RPM for 150 ms**. Aim and gate
   calibration are also needed for READY/rumble. Manual X/RT feeding does not wait for this.
   If the tuner shows **Velocity steps 20 ticks/s**, this +/-10 window may rarely or never pass.
4. Check aim first. For consistent short shots, increase RPM; for consistent long shots,
   decrease RPM. Sideways misses require aim correction rather than RPM changes.
5. Validate the chosen RPM with at least **three four-ball volleys**. Recheck aim when the HIVE
   tips. Record the V2 **Shot distance** in meters, final **commanded RPM**, battery voltage,
   and makes/attempts. With a HIVE tag visible, click the **right stick** to log the current
   distance/RPM to `/sdcard/FIRST/biobuzz_shot_table.csv`; telemetry confirms the save.
   Use the displayed camera/tag distance consistently, not floor distance.
6. Repeat at the three calibrated distances and your usual farther shooting spots, about every
   **0.3-0.5 m**. Outside 1.62 m, the current table clamps at 3235 RPM until you trim it.
7. Add the measured points to `SHOOTER_DISTANCE_METERS` and `SHOOTER_DISTANCE_RPM` in
   `Robot.java`. Keep matching entries in ascending distance order, then rebuild/deploy.
   Exit calibration and clear RPM trim (Back or restart TeleOp), then validate between table
   points and at lower battery voltage. Logging alone does not update the table.
   The shot-table file is pulled the same way as the tuner logs: `adb pull /sdcard/FIRST/biobuzz_shot_table.csv`.

V2 keeps the one-person drive/feed layout with the calibration additions above; the tuner has
its own controls. Auto keeps its +/-50 RPM window.
