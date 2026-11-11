# BioBuzz autonomous and mechanism testing

This branch integrates the mechanism controllers from main commit
`b41fb87d8ae5875d51bb5b3a497c32fb7e62c168` into PradhyunAuto. This open-loop intake
follow-up is based on `cd5952fb199036fdc5fb02c911faa41724a37dab`. Only PradhyunAuto is published.
Main is not modified. The disabled legacy encoder Auto is preserved.

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

**FLOWER is intake only. The autonomous and stationary test never map or command its lever.**
The imported Flower Calibration utility and competition TeleOp controls are separate main-code
features; they are not called by autonomous.

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
  aiming fresh and within tolerance, and RPM ready before feeding. Readiness must settle for
  the existing .20-second policy before opening. An already open normal feed pulse continues
  through the expected RPM dip; loss of valid aiming closes the gate and stops feeding.
- Shooting completion requires four feed-cycle counts AND corresponding observed ball-clear
  events. The motor/gate stop at the requested counter limit while remaining verification is
  pending. A timer or spin-up delay alone cannot count as a successful volley. Actual scoring
  in the goal still requires observation on the robot.

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

Intake calls the same Robot.updateManualFeeder method as TeleOp's forward intake, with the
calibrated gate closed. The transmission indexes until the feeder sensor sees a ball and the
intake continues running. Auto reads Limelight through Robot.updateMechanismSensors, uses
Robot.aimTurret for target tracking and Robot.distanceBasedShooterRpm for flywheel speed.
Pinpoint supplies the chassis pose. Ball sensing remains in the feeder/shooting interfaces. Shooting still requests four
shots and requires its existing readiness and shot-clear verification. If fewer balls were
acquired and four shots cannot be verified, the bounded shooting timeout fails the routine;
this update changes intake completion, not the shooting failure policy. Verify pickup yield
and actual scoring on the robot. Tune the FLOWER window and full-route timing physically.

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
feed cycles and the open-loop intake policy. A successful software volley cannot itself
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

The fixed route uses the existing controlled-test velocity constraints of 12 in/s and a
0.25 modeled speed fraction. Each waypoint requires <=2 in position error, <=3 degrees
heading error, <=2 in/s speed and <=8 degrees/s turn rate, sustained 0.20 s. Path deadlines
are 15 s, turn deadlines 5 s, and the run deadline 27 s. Path skipping is disabled.
Verify physical tracking and total elapsed time on the field.

Fresh Pinpoint READY and finite pose/velocity are checked before navigation output. Arrival,
heading, settled motion, unexpected path replacement, cancellation, and timeouts prevent
subsequent movements after failure. Stop clears follower state, writes zero drivetrain
power, and cancels scheduler commands before reset.

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
