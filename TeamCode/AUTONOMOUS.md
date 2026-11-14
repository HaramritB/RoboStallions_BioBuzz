# BioBuzz autonomous and mechanism testing

This branch integrates the mechanism controllers from main commit
`b41fb87d8ae5875d51bb5b3a497c32fb7e62c168` into PradhyunAuto. This open-loop intake
follow-up is based on `cd5952fb199036fdc5fb02c911faa41724a37dab`. Only PradhyunAuto is published.
Main is not modified. The newer separate **BioBuzz Auto** route from the teammate is preserved.

## Current scoring entry: AutoPath (BioBuzz FTC 2026)

Select **AutoPath** for the working route with shooter/turret/intake/feeder integration.
Select **AutoPath (Drive Only)** to run the same route and pauses without
initializing mechanism hardware. The separate **BioBuzz Auto** from the latest pull is
unchanged. The older BioBuzz Pedro Auto description below refers to its own implementation.

AutoPath follows the supplied eight-segment route. Every segment runs at the 85% modeled
speed except the two that drive into balls: **path 2** (into the four wall balls) and
**path 7** (FLOWER intake), which use 35% / 18 in/s. AutoPath (Drive Only) uses the same
speeds so it rehearses the real run.

1. INIT at (56, 8), heading 270°. A selects BLUE and B selects RED for tag filtering.
   With the turret pointing straight backward (180 deg, BioBuzzAutoConfig.TURRET_ZERO_DIRECTION_DEG), BACK zeros it.
2. Shoot the four preloads at START before driving. The flywheel and turret stay active, and
   the **intake runs inward the entire time** from START to the end of the route (same
   direction as the feed), including during volleys. While driving, the transmission stacks
   balls against the closed gate. For each volley it stops during spin-up, briefly backs the
   balls off the gate (0.10 s outward) as the gate opens, and never pushes balls into a moving
   gate. **Turret pre-aim (2026-11-12, fixed 2026-11-13):** whenever the chassis
   has been still for 0.25 s and a HIVE tag is in view (any time, not only during a volley, and
   the tag need not be centered), AutoPath records where the HIVE is on the field (Pedro pose +
   turret angle at frame capture + tag offset and distance; running average of ~20 sightings).
   Whenever the tag is out of view, the turret turns to the angle odometry predicts for the
   HIVE instead of sweeping, and counter-rotates against chassis turns. It sweeps only if
   stopped to shoot and nothing shows up within 1 s of reaching the prediction.
   **Why volley 2 missed (found 2026-11-13):** the team zeros the turret pointing straight
   BACKWARD (180 deg) at START, but the HIVE memory assumed encoder 0 meant forward. The START
   lock (encoder ~+32) was read as a HIVE in front of the robot, so the estimate landed mirrored
   behind START at about (17, -55) instead of (95, 71), and at spot 2 the turret pre-aimed ~134
   deg away from the real HIVE (the same wrong direction every run). With no tag in view there it
   fell back to the sweep, and the 2 s aim grace opened the gate mid-sweep. Fix:
   `BioBuzzAutoConfig.TURRET_ZERO_DIRECTION_DEG = 180` converts encoder angles; INIT telemetry
   says to zero backward. If you ever zero it pointing forward, set that to 0. Also: the HIVE is learned from any still sighting, pre-aim feeds forward chassis rotation at up to
   0.45 power, autonomous tracking uses 0.45 power when more than 10 deg off, and the 2 s
   unaimed fallback applies only when the turret is blind (SEARCH / no camera). While it is
   still turning toward a known or visible HIVE (PRE-AIM, UNWIND, TRACK, REACQUIRE), the gate
   waits until the 3.5 s force-feed time (spot 2 already stops for 4 s). Telemetry shows
   `HIVE at (x, y)` and the turret state under Shooter / aiming / gate.
   **Run log (2026-11-13):** every AutoPath run writes one CSV row per loop (pose, turn rate,
   turret angle/speed/state/power, tag ids and offsets, distance, HIVE estimate, pre-aim angle,
   shot phase, gate) to `/sdcard/FIRST/java/src/Datalogs/autopath_<millis>.csv` on the Control
   Hub. Telemetry shows the file name. Pull with
   `adb pull /sdcard/FIRST/java/src/Datalogs/` after a run.
3. Collect on paths 1–2 and through the one-second stop at the wall after path 2. Path 2 drives
   all the way to (7.994, 0) and ends on Pedro's end-of-path signal; there is no early
   stop. As a safety net, if balls block the robot first, it continues after 1 s without 0.25 in
   of progress, so the wall can never hang the route.
4. Follow paths 3–5 to the second shooting spot. At path 5's endpoint, deploy the flower
   lever and shoot, with a four-second minimum stop.
5. Collect on paths 6–7 and for two seconds at FLOWER. The existing four-second
   no-progress recovery uses the measured departure pose if path 7 stalls.
6. Follow path 8, shoot the final volley, and clean up the mechanisms and drivetrain.

Mechanisms use Robot.initMechanismsOnly, keeping Pedro as the sole drive controller.
Both full Robot and mechanism-only initialization select AUTO hub bulk caching and clear
old cached reads. Shooter power is explicitly zeroed before applying the existing encoder
mode/PIDF. This keeps a previous OpMode's manual cache or running output from carrying into
INIT. No drive motor directions, distance calibration or PIDF gains are changed.
Each active loop first updates Pedro, then supplies its CW-converted heading/turn rate to
Robot.updateMechanismSensors. Robot.aimTurret(true, 0), distanceBasedShooterRpm and
updateShooter(baseRpm) reuse TeleOp's aiming and measured RPM interpolation/PIDF.
Collection uses Robot.updateManualFeeder with the calibrated gate closed, in the **same
gamepad 1 RIGHT-trigger direction as the shooting feed** (intake -1, transmission -0.6), so
balls stack against the closed gate. Shooting is a
single continuous burst using the same manual feeder motor control:

- Between volleys (driving, intake, pauses) the flywheel holds the calibrated RPM and the
  turret tracks, with the gate closed. Volleys start warm and aimed; finishing a volley or
  an intake stops only the gate/intake/transmission, never the flywheel.
- Keep the gate CLOSED and intake/transmission stopped until ready: measured speed within
  **50 RPM** of the calibrated target for **300 ms**, at least five samples, no sampling gap
  over 150 ms, and at least **300 ms** stopped at the spot (turret settle). A speed dip,
  overshoot, invalid aim/distance or target change over 25 RPM resets the stable window.
- Fallbacks instead of aborting: with no fresh aimed tag after **2.0 s**
  (SHOOT_AIM_GRACE_SECONDS) and the turret blind (sweeping / no camera; a turret still turning
  toward the HIVE waits until 3.5 s), shoot at the calibrated-table RPM (last filtered distance, else
  the 3215 no-tag default) once the flywheel is stable. If the flywheel still is not in the
  band after **3.5 s** (e.g. low battery) but is at least 85% of target, feed anyway. If a
  volley cannot start within **6 s** (broken flywheel/gate), skip it and continue the route.
- **Gate first, then feed:** command calibrated OPEN, wait **GATE_OPEN_WAIT_SECONDS = 0.25 s**
  for the servo to travel (AutoPath backs the transmission outward for the first
  **GATE_RELIEF_SECONDS = 0.10 s** to unload the gate), then start both motors together
  in **gamepad 1 RIGHT-trigger** direction: intake -INTAKE_POWER and transmission
  -TRANSMISSION_INDEX_POWER. If the tag is lost while the gate is open, the turret holds
  still instead of sweeping.
- Once feeding starts, hold the gate open and both motors running for
  **SHOOT_BURST_SECONDS = 2.0 seconds**, including normal flywheel RPM dips. The feed window
  starts after the gate-open wait, not when spin-up begins.
- At the end of that window, close the gate and stop the feed motors before another actuator
  write, settle for 0.20 seconds, then continue the route. Ball-clear counts are advisory:
  tightly packed balls may not produce four distinct sensor edges.
- Losing the tag during the short feed window keeps feeding at the latched RPM and is counted
  in telemetry ("aim lost N feed loops"); it no longer ends the autonomous. Invalid feeder
  sensor samples are treated as "no ball" and reported. STOP closes the gate and stops
  every motor. The diagnostic OpModes (BioBuzz Pedro Auto, mechanism test) keep the strict
  abort-on-failure behavior.

The shooter uses the same measured distance/RPM interpolation and tuned PIDF as TeleOp:
**1.39 m -> 3215 RPM, 1.52 m -> 3220, 1.62 m -> 3235**. Interpolate between
points and clamp beyond the measured endpoints, as in Robot.rpmForDistance. Auto uses its
stricter 50-RPM stable-speed qualification; TeleOp's existing 100-RPM readiness tolerance is
unchanged. Auto looks up the table with the **median of the last five fresh tag distances**,
so one bad Limelight frame cannot change the shot, and holds that distance for up to 1 s of
missing frames before using the 3215 no-tag default. Once the gate opens, latch that RPM for
the stationary burst. No fixed 4000 RPM/color-based shooting target replaces the measured table.

AutoPath commands the `flower` servo OFF as soon as INIT is pressed (and again at START). It stays
OFF through the end of path 5, deploys at the second shooting spot,
and stays ON through volley 2, paths 6–8 and the final volley. Only the end of the OpMode,
STOP or a failure retracts it. FlowerLever shares TeleOp's ON=0.435/OFF=0.1 positions and
PWM-enable/restore behavior. AutoPath (Drive Only) never maps or commands it.

Scoring needs intake, transmission, turret, shooter, torqueservo1, **flower**, a working Limelight and
the configured color/range feeder sensor. Missing devices or invalid gate calibration fail at
INIT. During the run, AutoPath reports invalid sensor samples, lost aim and skipped volleys
under **Mechanism warning** and keeps driving; STOP cleans up every output.
The drive-only entry does not require any of these mechanism devices.

Shutdown closes the mechanism controller before cancelling Ivy commands, so normal STOP or
cleanup does not throw `Mechanism command interrupted: SHOOT`. If a run fails, Driver Station
gets the original exception; additional cleanup failures are attached without replacing it.
Read `AutoPath failure` for the cause. Invalid sensor errors also include sampled controller
diagnostics, including feeder distance, tag distance and aim error.

Validate physical exits, collection yield and total match time on the robot. The mechanism
integration preserves the route the team reports working; it does not establish new physical
clearance. Use the **AutoPath** telemetry for alliance, mechanism state, RPM/aim/gate diagnostics
and failures. **Feed RPM: start / target / minimum** records the actual speed when feeding
begins, its qualified target, and the lowest sampled speed during the burst. These values
persist after the burst until a new one starts. If the start speed is correct but minimum
speed falls severely, the remaining issue is speed recovery under feed load; record these
values before changing calibrated speeds or PIDF. Software regression tests cover both normal and recovered FLOWER departures,
all three volleys, the one-/four-second pauses, late/failed shots, fresh sensor failure,
stop during intake, feed-window output ordering, missing sensor counts, slow spin-up,
flower deployment/retraction/PWM restoration, and
cleanup even when a device close or flower retraction fails.

## Older entry: BioBuzz Pedro Auto

**BioBuzz Pedro Auto runs all five fixed paths with drivetrain and Pinpoint only.**
INIT initializes localization without driving. START begins the inward move before path 1.
There is no gamepad mode or alliance selection. The OpMode does not initialize shooting,
intake, the gate, turret, Limelight, or the feeder color/range sensor.

The team measured 427.284 mm from the back to the front of the intake guard, 448.7 mm
side to side, and 213.642 mm from the tracked center to the intake tip. The runtime offset
is 8.411102 inches forward with the centered-intake assumption of zero lateral offset.
This places the FLOWER chassis center at (46.872269, 131.819817) inches, heading 90.572°.
Path 5 starts at that same pose. The original visualizer JSON remains the route source.

## Scoring sequence retained for future integration

This plan is inactive in BioBuzz Pedro Auto.

1. After START, hold the starting pose and shoot **4 preloaded balls before path 1**.
   Move inward to y = 14.447 in, turn to 179.7546°, then return to the original path-1
   entry XY at that heading. This places the rotation away from the lower wall.
2. Drive the original straight path 1 at 179.7546° with intake active. Keep intake running
   until settled path arrival, then stop intake and begin path 2. No stored-ball measurement,
   presence transition or inventory target is used to decide when collection finishes.
3. Drive path 2, hold its endpoint and shoot **4 balls**.
4. Hold that position while turning from **-84.7123 to 0 degrees**; verify settling.
5. Drive path 3, run intake during the path-4 approach, then hold position with intake for
   FLOWER_INTAKE_SECONDS at FLOWER. The initial test window is **2.0 seconds**, configurable
   and not physically tuned. Finish the timed attempt without claiming four stored balls.
6. Drive path 5; hold its endpoint, shoot **4 balls**, then stop all outputs.

**The older BioBuzz Pedro Auto and stationary mechanism test use intake only.**
The current AutoPath scoring entry also operates the flower lever as described above;
Flower Calibration remains a separate utility.

## Software and hardware

- FTC SDK 12.0.0; PedroPathing core/revhub 3.0.2; Ivy core/pedro 1.1.1. Versions unchanged.
- Existing Constants.create(HardwareMap): Mecanum, PinpointLocalizer, Foresight.
- Drive names frontleft/frontright/backleft/backright, all FORWARD in Pedro Constants.
  Robot's separate TeleOp mixer uses left REVERSE plus explicit negative left outputs.
  Neither mixer, motor directions, PID coefficients nor localizer configuration was retuned.
- Pinpoint device pinpoint, goBILDA 4-bar pods, X reversed/Y forward, offsets -3.5625/-4.125 in.
- Mechanisms use intake, transmission, turret, shooter, torqueservo1; optional LED and
  Limelight. Sensor names colorsensor and legacy colorSensor are both supported by main.
- The latest Robot gate controller loads saved calibration, honoring invalid/disabled settings;
  its main-code measured defaults are OPEN .41/CLOSED .59. Auto cleanup uses the loaded
  calibrated CLOSED endpoint, including overrides, rather than writing a fixed constant.
- Shooter PIDF remains 180/0/0/13.83. Auto reuses main's measured distance/RPM interpolation
  and target selection, not a new flywheel controller. Target distance must be finite/positive,
  aiming fresh and within tolerance, and stable measured RPM before feeding. Feeding starts
  after stable-speed qualification and the gate-open wait, and remains continuous through
  normal RPM dips and brief aim loss (AutoPath falls back after 2 s without a tag).
- AutoPath completes a bounded timed feed attempt after readiness, with ball counts only
  in telemetry. The separate counted stationary test still requires four feeder-clear events.
  Neither mode can establish actual scoring in the goal without physical observation.

Robot.initMechanismsOnly preserves main's mechanism initialization but never maps/configures
drivetrain motors. The full autonomous supplies Pedro's fresh pose/rate to Robot's turret
controller (converted to clockwise degrees). Robot.updateMechanismSensors does not update
Pinpoint again. The stationary test has no competing localization/drivetrain controller.

## Intake policy: no stored-ball measurement

The user explicitly chose to run intake and then shoot, without measuring whether four balls
are stored. The old counted-collection capability, observation counter, startup check and
COUNTED_COLLECTION_VALIDATED approval gate have been removed, rather than marked verified.

Path 1 owns intake lifetime: drive and intake together, stop intake at verified arrival, then
start path 2. A stalled/interrupted path, invalid localization, timeout or FTC Stop still stops
both controllers and blocks further autonomous actions. The FLOWER action runs intake during path 4 and while
holding the waypoint for FLOWER_INTAKE_SECONDS (initial test value 2.0 s), with a separate
10-second hard timeout. Timed intake stops outputs before the first control update at/after
the duration; a delay is an intake attempt, not proof of inventory. No FLOWER lever is used.

Intake calls the same Robot.updateManualFeeder method as TeleOp's gamepad 1 RIGHT trigger,
with the calibrated gate closed: intake and transmission both run inward and stack the balls
against the gate. Shooting then opens the gate and runs the same direction. Auto reads Limelight through Robot.updateMechanismSensors, uses
Robot.aimTurret for target tracking and Robot.distanceBasedShooterRpm for flywheel speed.
Pinpoint supplies the chassis pose. Ball sensing remains in the feeder/shooting interfaces.
AutoPath now uses timed continuous shooting so missed individual ball-clear events do not
block navigation after the load has left. The older counted stationary test keeps strict
shot-clear verification for diagnostics. Verify pickup yield, feed-window duration and actual
scoring on the robot. Tune the FLOWER window and full-route timing physically.

## Stationary test: BioBuzz Auto Mechanism Test

This is a Development **TeleOp**, separately registered and selectable. It never controls drive
motors or the FLOWER lever. Place the stationary robot in a suitable mechanism test position,
verify turret reference and linkage clearance, and provide a safe shooting target.

1. Select **BioBuzz Auto Mechanism Test**, then INIT. Missing required mechanisms, camera,
   ball-distance/color sensor, invalid sensor data or invalid saved gate calibration blocks it.
   INIT observes and zeros motors; it does not command the gate open/closed.
2. Before START: D-pad down selects manual intake (default); D-pad up selects a four-shot volley.
   A selects BLUE targeting; B selects RED. Alliance selection does not mirror any route.
3. Press START. Outputs remain idle until **gamepad 1 LB+A is held**.
4. Intake mode: intake runs while held, using main's existing feeder/indexing logic. Observe
   actual storage behavior and sensor telemetry. It stops at release or the 10-second bound;
   reaching the bound is not a successful four-ball collection.
5. Shoot mode: load the requested four balls and keep LB+A held. The controller aims, spins up,
   feeds only with readiness, verifies four shot events, then stops. Missing readiness/ball
   events fails at seven seconds. Releasing either held control interrupts and stops outputs.
6. One attempt per INIT. Re-INIT for another attempt. FTC Stop always cancels/zeros mechanisms
   and closes the camera. No indefinite retry or automatic rearming occurs.

Telemetry reports mode, status/failure, RPM/target RPM, vision/aim/gate readiness, ball presence,
observed ball exits and the open-loop intake policy. A successful software volley cannot itself
prove four goals were scored: verify physical exits, RPM recovery, aiming and gate travel.

## Route source and geometry

Source: TeamCode/routes/BioBuzz_AutoPath.pp.json, retained unchanged.
SHA-256: a61a3d550df9bbd607a2752377f7f146ff9c3287f79398649f6a459dd5936f3b.
The separately supplied (1) variant is not substituted: it changes path 1 to tangential heading
and supplies modeled dimensions, unlike the explicit original requested here.

Robot-center inches, origin at the lower-left of the supplied field frame, +X right, +Y up;
heading CCW from +X. Start at **(57.6323, 9.0657), physically forward +Y (90 degrees)**.
Setting the pose in software does not verify the physical starting location.

| Segment | Start -> endpoint (inches, degrees) | Geometry/heading |
| --- | --- | --- |
| Path 1 | (57.6323,9.0657,90) -> (9.5657,9.2715,179.7546) | Line, linear heading |
| Path 2 | Previous end -> (32.2934,127.6559,-84.7123) | Quadratic control (40.923,34.4143), linear heading |
| Turn | Same XY, -84.7123 -> 0 | Explicit follower.hold; measured settling, no zero-length path |
| Path 3 | (32.2934,127.6559,0) -> (46.9373,125.3042,90) | Quadratic control (40.2631,108.1664), linear heading |
| Path 4 | Previous end -> (46.7883,140.2305,90.572) | Line, linear heading |
| Path 5 | Previous end -> (38.0943,127.3583,-66.9637) | Quadratic control (45.4429,110.0765), linear heading |

The table records the raw visualizer export. Runtime path 1 retains its straight XY line at
a constant 179.7546° after the inward alignment. The original heading change on that line
put the measured rectangular envelope about 3.03 in past the lower wall. The inward turn
location accounts for the footprint half-diagonal, the 2-inch position tolerance, and a
0.25-inch nominal margin. Path 2 already travels backward and inward from the pickup endpoint.

Sampling the five corrected segments with the measured 16.8222 × 17.6654 in envelope found
no field-boundary intersection. The straight pickup line has about 0.20 in minimum nominal
lower-wall clearance. The rectangle overlaps the Scoring-Side FLOWER during approach and
departure, where the measured length includes the intake guard. Physical contact geometry,
tracking error and full-run timing require a robot trial. The checked-in audit is the raw
export audit, with the measured offset and heading alignment applied at runtime.

## Field test: BioBuzz Pedro Auto

1. Remove any configured REV Color/Range Sensor entry if that sensor is absent. Leave
   Pinpoint and the built-in IMU configured.
2. Place the robot center at **(57.6323, 9.0657) inches** with its front facing **+Y (90°)**.
3. Select **BioBuzz Pedro Auto** in Driver Station and tap **INIT**. Confirm
   **FIXED ROUTE (drive only)**, Pinpoint READY, Localization ready true, and Failure none.
4. Tap **START**. Expect the inward move, alignment turn, return to the path-1 entry,
   paths 1 and 2, the in-place turn, then paths 3, 4, and 5. Keep a person at STOP and
   observe the narrow lower-wall clearance and the FLOWER approach/departure.
5. If the route stops early, record Status, Route segment, Pinpoint, and Failure telemetry.

The fixed route uses 36 in/s and a 0.60 modeled speed fraction on paths 2 and 3.
The inward move, return, path 1, and FLOWER paths 4 and 5 use 18 in/s and a 0.35
modeled speed fraction. Actual speed depends on tracking, braking, and available distance.
Each waypoint requires <=2 in position error, <=3 degrees
heading error, <=2 in/s speed and <=8 degrees/s turn rate, sustained 0.20 s. Path deadlines
are 15 s, turn deadlines 5 s, and the run deadline 27 s. Path skipping is disabled.
Verify physical tracking and total elapsed time on the field.

Fresh Pinpoint READY and finite pose/velocity are checked before navigation output. Arrival,
heading, settled motion, unexpected path replacement, cancellation, and timeouts prevent
subsequent movements after failure. Stop clears follower state, writes zero drivetrain
power, and cancels scheduler commands before reset.

## Direct route: AutoPath

**AutoPath** and **AutoPath (Drive Only)** follow the same eight-segment geometry and
heading profiles and speeds. Path 2 runs to its end at (7.994, 0), with a 1-second
no-progress safety stop. The second shooting stop is after path 5. Path 7 targets (46.5109, 139.7571);
if the approach stalls for four seconds without 0.25 inch of new progress, path 8 starts
from the measured pose. Scoring retains the two-second stopped intake window before
departure. Verify wall and FLOWER contact and total run time on the field.

## Earlier scoring integration

This follow-up changes autonomous/BioBuzzAutoConfig.java (path-owned/timed intake settings,
no inventory gate), autonomous/BioBuzzAutoMechanisms.java (remove stored counts, add bounded
intake actions), autonomous/BioBuzzAutoRoutine.java (finish travel intake on path arrival),
autonomous/BioBuzzAuto.java and BioBuzzMechanismTest.java (remove inventory preflight/telemetry),
BioBuzzAutoRegression.java and MechanismIsolationRegression.java (open-loop intake checks),
and this document. All prior main-code integrations below remain intact.

All Java paths below begin TeamCode/src/main/java/org/firstinspires/ftc/teamcode/.

| File(s) | Purpose |
| --- | --- |
| Robot.java | Latest main gate/vision/turret/shooter interfaces; isolated mechanism init and Pedro-fed sensor update |
| BioBuzzTeleOp.java; BioBuzzBlueTeleOp.java; BioBuzzRedTeleOp.java | Import compatible main consumers and alliance-specific registrations |
| FlywheelTuning.java; RobotTest.java; TurretLimitsTest.java; TurretTrackingTuner.java | Import main's matching tuning/test clients |
| GateCalibration.java; FlowerCalibration.java | Import main's existing calibration utilities; flower utility never called by Auto |
| autonomous/BioBuzzAuto.java | Use current Robot interfaces and Pedro chassis state; capability preflight; diagnostics; remove obsolete TeleOp preselection |
| autonomous/BioBuzzAutoConfig.java | Remove stale component approval gates; require measured intake-tip coordinates for FLOWER |
| autonomous/BioBuzzAutoMechanisms.java | Calibrated gate cleanup, distance-based RPM/readiness, manual test ownership, sensor-source isolation and diagnostics |
| autonomous/BioBuzzMechanismTest.java | Stationary operator-held intake/four-shot test, bounded interruption and cleanup |

| Other file(s) | Purpose |
| --- | --- |
| TeamCode/CONTROLS.md; TeamCode/TURRET_TRACKING.md | Current main controller instructions |
| TeamCode/tests/RobotControlsRegression.java; run_robot_controls.py | Current main gate/feed/indicator regression and runner |
| TeamCode/tests/TurretTrackingRegression.java; run_turret_tracking.py | Current main turret regression and runner |
| TeamCode/tests/MechanismIsolationRegression.java | Isolated initialization, calibrated endpoint cleanup, real intake controls, no FLOWER mapping, no duplicate Pinpoint update |
| TeamCode/tests/MechanismTestRegression.java | Actual stationary OpMode lifecycle, held controls, four shots, release/timeout/Stop/sensor failures |
| TeamCode/tests/run_autonomous.py | Compile modified/new production OpModes and run added test |
| TeamCode/AUTONOMOUS.md | This current operational handoff |

## Earlier scoring validation

The measured update passed source/API compilation and the hardware-free Java and route tests
using official FTC 12.0.0 AARs, the Android API 30 jar and tagged PedroPathing 3.0.2/Ivy 1.1.1
sources. The previous 06db929 revision built on macOS. The current Linux wrapper attempt
stopped while downloading Gradle because Java reported network unreachable.

```sh
python3 TeamCode/tests/run_robot_controls.py
python3 TeamCode/tests/run_turret_tracking.py
python3 TeamCode/tests/run_autonomous.py
python3 -m unittest discover -s TeamCode/tests -p test_route_source.py -v
./gradlew :TeamCode:assembleDebug
# Prior integration also attempted with a locally downloaded, unmodified Gradle 9.1.0 distribution:
/tmp/biobuzz-gradle-dist/gradle-9.1.0/bin/gradle \
  :TeamCode:assembleDebug :TeamCode:testDebugUnitTest --no-daemon
git diff --check
```

Build command for the configured macOS Gradle cache:

```sh
GRADLE_USER_HOME=/tmp/biobuzz-gradle-home \
JAVA_HOME='/Applications/Android Studio.app/Contents/jbr/Contents/Home' \
./gradlew :TeamCode:assembleDebug :TeamCode:testDebugUnitTest --offline --no-daemon
```

Geometry tests verify the measured FLOWER center, path-4/path-5 continuity, straight pickup
line and the alignment maneuver before path 1. Lifecycle tests cover enabled registration,
full-route default, no INIT movement, missing-reach failure, controlled test selection and
full scoring START with the measured reach and recording hardware.

| Check | Result |
| --- | --- |
| Main gate/robot controls | 82 checks PASS (previous stale missing-member suite now fixed by main integration) |
| Main turret tracking | 65 checks PASS |
| Autonomous sequence/navigation/mechanisms | 161 checks PASS |
| Mechanism isolation/cleanup/heading ownership | 30 checks PASS |
| Full route OpMode lifecycle | 13 checks PASS |
| Stationary mechanism test lifecycle | 10 checks PASS |
| Route-source/audit generation | 8 tests PASS; route unchanged |
| Changed Java API compilation | PASS against real FTC AARs, Android jar and tagged Pedro/Ivy sources |
| Current assembleDebug | Gradle download blocked by network access before packaging |
| Prior macOS assembleDebug at 06db929 | BUILD SUCCESSFUL, including APK packaging |
| Physical robot tests/deployment | NOT PERFORMED |

Build PradhyunAuto in Android Studio before installing. Verify the corrected route and complete
routine timing on the robot.
