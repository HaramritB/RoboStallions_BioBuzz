# BioBuzz autonomous and mechanism testing

This branch integrates the mechanism controllers from main commit
`b41fb87d8ae5875d51bb5b3a497c32fb7e62c168` into PradhyunAuto. This open-loop intake
follow-up is based on `cd5952fb199036fdc5fb02c911faa41724a37dab`. Only PradhyunAuto is published.
Main is not modified. The disabled legacy encoder Auto is preserved.

**Shooting/intake controllers are implemented. Intake is open-loop by user instruction: no
stored-ball counter or inventory approval is required. The full driving/scoring OpMode remains
disabled pending physical hardware, tuning and route-clearance checks.**
The stationary mechanism test is separately selectable and requires held operator controls.
No robot deployment or motor activation was performed from the development environment.

## Confirmed routine

1. After START, hold the starting pose and shoot **4 preloaded balls before path 1**.
2. Drive path 1 with intake active to attempt pickup of the staged balls. Keep intake running
   until settled path arrival, then stop intake and begin path 2. No stored-ball measurement,
   presence transition or inventory target is used to decide when collection finishes.
3. Drive path 2, hold its endpoint and shoot **4 balls**.
4. Hold that position while turning from **-84.7123 to 0 degrees**; verify settling.
5. Drive path 3 (approach), then path 4; hold position and run intake for
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
both controllers and blocks further autonomous actions. The FLOWER action runs intake while
holding the waypoint for FLOWER_INTAKE_SECONDS (initial test value 2.0 s), with a separate
10-second hard timeout. Timed intake stops outputs before the first control update at/after
the duration; a delay is an intake attempt, not proof of inventory. No FLOWER lever is used.

Ball sensing remains in the existing feeder/shooting interfaces. Shooting still requests four
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

These are confirmed route inputs, not measured physical clearance. Path 4's center ends inside
Scoring-Side FLOWER, 1.2695 in below the supplied 141.5-inch reference boundary; path 5 starts
inside FLOWER. Intended intake contact does not approve center penetration. Measure footprint
and intake reach, then approve an approach outside solid geometry. For vertical half extent at
heading h use abs(sin(h))*length/2 + abs(cos(h))*width/2, plus a measured margin. No numeric
correction is invented and source geometry is preserved. The sampled audit is not a continuous
collision proof.

## Full autonomous: BioBuzz Pedro Auto

The full OpMode remains @Disabled. Hardware/calibration, footprint/route-clearance/hash review,
FLOWER approach and mechanism validation gates remain unconfirmed. The
main-controller source/API integration gate is now complete; that is not physical validation.
Do not mark missing measurements or capabilities confirmed just to make the Auto selectable.

After the documented physical checks, controlled first-N **movement-only** mode is the first
route test: default path 1, D-pad +/- chooses N, Y selects the separate full scoring plan.
Movement tests skip all mechanisms, including preload shooting. N>=3 includes the explicit
turn. A/B chooses BLUE/RED mechanism tag filtering without mirroring geometry.

Each waypoint requires <=2 in position error, <=3 deg heading error, <=2 in/s speed and
<=8 deg/s turn rate, sustained .20 s. Path deadlines 15 s, turn 5 s, whole run 27 s. Requests
12 in/s and .25 modeled max path speed with path skipping disabled; these are velocity requests,
not hard motor-power caps. Full-route timing with three volleys must be measured and tuned to
fit the competition period; increasing the deadline does not solve an overlong route.

Fresh Pinpoint READY and finite pose/velocity are checked before navigation output. Arrival,
heading, settled motion, unexpected path replacement, cancellation, timeouts and mechanism
failures latch failure and block subsequent actions. Follower.stop only clears state in Pedro
3.0.2, so cleanup also calls drivetrain.stop and independent raw zero fallbacks. Mechanisms
stop independently even if one write fails. Scheduler commands are cancelled before reset.

## Changed files in this update

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
| autonomous/BioBuzzAutoConfig.java | Mark controller source/API integration done; physical validation gates remain closed |
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

## Validation of this update

The earlier parent commit built an APK on macOS. This session runs on a new Linux machine;
that old successful APK build is not evidence of packaging this update.

```sh
python3 TeamCode/tests/run_robot_controls.py
python3 TeamCode/tests/run_turret_tracking.py
python3 TeamCode/tests/run_autonomous.py --upstream-source /tmp/biobuzz-upstream
python3 -m unittest discover -s TeamCode/tests -p test_route_source.py -v
./gradlew :TeamCode:assembleDebug
# Prior integration also attempted with a locally downloaded, unmodified Gradle 9.1.0 distribution:
/tmp/biobuzz-gradle-dist/gradle-9.1.0/bin/gradle \
  :TeamCode:assembleDebug :TeamCode:testDebugUnitTest --no-daemon
git diff --check
```

The current test environment is ../validation-env: JAVA_HOME points to its java/ launcher
for the installed OpenJDK 21 compiler module, GRADLE_USER_HOME to gradle/ (FTC caches),
ANDROID_HOME to sdk/, and --upstream-source to upstream/. These are local verification inputs,
not robot dependencies or committed build changes. The official Android API 30 jar is used.
FTC RobotCore/Hardware 12.0.0 are actual published AARs. Pedro and Ivy were compiled from
unmodified upstream tags v3.0.2 and v1.1.1 (95 main-source files), rather than unavailable
resolved Maven archives. This checks actual source/API compatibility, not Android APK packaging.
The source-mode runner intentionally skips the unchanged Tuning.java client, which depends on
the separately distributed tuning artifact. All modified/new production OpModes are compiled.

| Check | Result |
| --- | --- |
| Main gate/robot controls | 82 checks PASS (previous stale missing-member suite now fixed by main integration) |
| Main turret tracking | 65 checks PASS |
| Autonomous sequence/navigation/mechanisms | 138 checks PASS |
| Mechanism isolation/cleanup/heading ownership | 29 checks PASS |
| Full route OpMode lifecycle | 8 checks PASS |
| Stationary mechanism test lifecycle | 10 checks PASS |
| Route-source/audit generation | 8 tests PASS; route unchanged |
| Changed Java API compilation | PASS against real FTC AARs, Android jar and tagged Pedro/Ivy sources |
| Android Gradle wrapper | FAILED downloading Gradle: Java network access unavailable |
| Prior integration: locally downloaded Gradle build | FAILED resolving Android Gradle plugin 8.13.2 before Java compilation |
| Physical robot tests/deployment | NOT PERFORMED |

No dependency version or robot tuning change was made to hide environment failures. Build this
branch in the team's configured Android Studio environment before installing. First test gate,
turret reference, intake/storage behavior and a stationary four-shot volley, then verify
route clearance and mechanism timing before full-route testing.
