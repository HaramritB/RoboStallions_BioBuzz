# BioBuzz Pedro autonomous foundation

## Status

Implemented on branch `PradhyunAuto`, based on clean commit `62189dd`.
The editable checkout is `/Users/pradhyun/Documents/ChatGPT/FTC/RoboStallions_BioBuzz1`.
The original checkout at `/Users/pradhyun/StudioProjects/RoboStallions_BioBuzz1` was read
but cannot be edited under this session's filesystem permissions. No commit, merge,
push, deployment, or physical robot execution was performed.

**The new OpMode is disabled. This is a movement-only software foundation with example
coordinates, not a validated competition route.** The existing `BioBuzz Auto` is a separate,
unchanged encoder-based OpMode; do not confuse it with the new Pedro OpMode.

## Repository findings

| Item | Evidence / configuration |
| --- | --- |
| FTC SDK | `build.dependencies.gradle`: FTC artifacts 12.0.0 |
| Build tools | Gradle wrapper 9.1.0, Android Gradle Plugin 8.13.2; Java 8 source/target; compile SDK 30, target 28, min 24 |
| Package | `org.firstinspires.ftc.teamcode`; existing tuner classes in `.procedures`; new code in `.autonomous` |
| Pedro | Existing `com.pedropathing:revhub:3.0.2`, `com.pedropathing:tuning:1.0.0`; no upgrade |
| Ivy | Previously absent; added `com.pedropathing.ivy:pedro:1.1.1` only to TeamCode. Upstream v1.1.1 integration compiles against Pedro core 3.0.0 and works with the used 3.0.2 APIs. Published binary resolution still requires Gradle sync outside this sandbox. |
| Drivetrain | Four-wheel mecanum; names `frontleft`, `frontright`, `backleft`, `backright` in both Robot and Constants |
| Directions | Pedro Constants: all FORWARD. Robot: left REVERSE, right FORWARD, plus explicit negative left wheel powers in `drive()`. Different mixers make a simple direction comparison insufficient; verify actual wheel response using Pedro. No directions were changed. |
| Localizer | `PinpointLocalizer`, device `pinpoint`, goBILDA 4-bar pods; X reversed, Y forward |
| Offsets/units | X pod -3.5625 in; Y pod -4.125 in; global and offset units INCH, preserved exactly |
| Follower factory | `Constants.create(HardwareMap)` constructs Pinpoint + Mecanum + Foresight with existing coefficients |
| Calibration evidence | Specific tuned coefficients in Constants, recent commits `7eb34dc` (new constants) and `67c574f` (pedropathing done), plus localizer/drivetrain/Foresight tuning procedures. Physical verification, logs and route approval are not present. |
| Existing OpModes | BioBuzz Auto, BioBuzz TeleOp, Robot Test, Flywheel Tuning + Burst Logger, Turret Limits (calibration), Pedro tuning entry point |
| Existing route | Old Auto explicitly labels its 24 in forward / 24 in strafe distances as placeholders. No verified field waypoints or field diagram found. Tuner test lines/curves are test geometry. |

The old controls documentation and gate regression harness describe calibration methods
that the current `Robot.java` no longer contains. Current source is authoritative; that
mismatch is unrelated to this integration and is left unchanged.

## Files

| File | Purpose |
| --- | --- |
| `TeamCode/build.gradle` | Adds the Ivy 1.1.1 Pedro integration dependency |
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/BioBuzzAuto.java` | Disabled iterative FTC OpMode, lifecycle, hardware checks, localization guard, telemetry and cleanup |
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/BioBuzzAutoConfig.java` | Three disabled review gates; timeout, waypoint and settling policies |
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/BioBuzzAutoPaths.java` | Example poses and fresh line/Bezier path factories; explicit constant/linear heading |
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous/BioBuzzAutoRoutine.java` | Ivy sequence with verified arrival, failure latching and cancellation handling |
| `TeamCode/tests/BioBuzzAutoRegression.java` | Hardware-free checks using real Ivy scheduler/Pedro paths and simulated localization/drivetrain |
| `TeamCode/tests/run_autonomous.py` | Compiles/runs those checks against cached binaries or explicitly supplied tagged upstream sources |
| `TeamCode/AUTONOMOUS.md` | Findings, route, validation, operating instructions and extension guidance |

Constants, Robot, existing autonomous, TeleOp, and all tuning values remain unchanged.

## Exact routine

1. INIT resets Ivy, checks approval gates, and (only after approval gates are true)
   resolves drive hardware and initializes the existing factory. Set the start pose.
2. INIT LOOP updates only localization. It never executes Ivy or follower drive control.
3. START requires READY Pinpoint status and a finite, stationary starting pose. Schedule once.
4. Follow start-to-score. Wait for tracking to finish and the robot to be within arrival
   and velocity tolerances. There is no scoring mechanism action.
5. Follow score-to-park under the same arrival checks.
6. Complete and explicitly zero drivetrain output. No automatic retry.

| Segment | Start (X in, Y in, heading deg) | End | Geometry | Heading | Validation |
| --- | --- | --- | --- | --- | --- |
| Start to score | (24, 24, 0) | (48, 48, 90) | Straight line | Linear 0 to 90 deg | EXAMPLE ONLY |
| Score to park | (48, 48, 90) | (72, 48, 90) | Straight line | Constant 90 deg | EXAMPLE ONLY |

These share the exact same `scorePose` endpoint. Origin is the lower-left corner on the
chosen field drawing, +X right, +Y up, inches, positive rotation counterclockwise; zero
heading faces +X. Pose refers to robot center as configured by Pinpoint offsets. At the
example start, the robot's forward axis faces +X. No alliance, starting wall, obstacle
clearance, robot footprint clearance, or physical starting location has been established.
Measure and record those before accepting the route. There is no automatic alliance mirror.

`straight(start, end, constantHeading)` supports constant or linear headings.
`bezier(start, end, constantHeading, controlPoses...)` supports curves when measured geometry
justifies them; control headings do not define interpolated robot headings. Constant heading
rejects endpoints with inconsistent headings. Initial paths deliberately have no controls.

## Safety and lifecycle details

- Both `@Disabled` and three compile-time false review flags block this OpMode. Removing
  only the annotation cannot enable movement; unapproved INIT does not initialize hardware.
- Hardware names come directly from existing Constants. Missing/wrong hardware catches an
  initialization exception. Partially resolved drive motors receive independent stop writes.
- Existing encoder modes are not changed. Each drive motor must already be in
  RUN_WITHOUT_ENCODER: Pedro's Mecanum constructor does not set that mode. Any other mode
  blocks initialization; verify the robot setup rather than silently overriding it here.
- The existing factory's localizer, drivetrain and algorithm are reused. A Localizer decorator
  validates each *fresh* sample inside follower.update before control reaches motor output.
  Pinpoint must be READY, with finite position, heading and velocity. INIT can wait through
  calibration; START and running faults fail closed. Finite but frozen readings cannot be
  universally identified from pose alone; segment/overall deadlines bound such a failure.
- Ivy v1.1.1 source uses `atParametricEnd()` in its default `follow` command, despite the
  current website's description of completion. This routine overrides completion and adds
  a follower requirement: it waits for HOLD plus position <=2 in, heading <=5 deg,
  linear speed <=2 in/s and angular speed <=10 deg/s. These are initial test policy values,
  not measured tuning. Tracking loss, an unexpected path replacement, or cancellation fails.
- A segment that cannot arrive/settle fails after 10 s. The whole routine fails after 27 s.
  Failed prerequisites latch failure and no later path can start.
- FTC Stop, completion and exceptions cancel active Ivy work, reset static scheduler state,
  clear follower tracking and stop drivetrain output. In Pedro 3.0.2 `Follower.stop()` only
  changes state, so `follower.drivetrain.stop()` and independent zero motor writes are also
  necessary. Cleanup errors are reported without skipping remaining stop attempts.
- No Robot.init or mechanism update runs here, avoiding a second drivetrain controller,
  turret re-zero, gate command, or camera startup. No mechanism is activated by this Auto.
- Telemetry transmission is limited to 100 ms: pose, targets and wrapped errors, distance,
  segment/stage, follower mode/tracking, elapsed time, completion, readiness, failure and stop.
- The pre-existing PinpointLocalizer constructor internally sleeps 500 ms during initialization.
  This integration adds no sleeps or movement polling loops; runtime control is nonblocking.

## Validation performed

Commands below ran from the editable checkout. JDK for the direct Java checks was Android
Studio's bundled JBR; cached FTC SDK 12.0.0 AARs and installed Android 36 android.jar supplied
FTC/Android types. New production sources, existing Constants, and actual tagged upstream
Pedro v3.0.2 / Ivy v1.1.1 sources were compiled together at Java 8 source/target level.
Upstream source files were fetched through the GitHub connector into temporary storage;
no replacement library, fake API stubs, or vendored dependency was added to the project.

| Command | Result |
| --- | --- |
| `GRADLE_USER_HOME=/tmp/biobuzz-gradle-home JAVA_HOME='/Applications/Android Studio.app/Contents/jbr/Contents/Home' ./gradlew :TeamCode:assembleDebug --offline --no-daemon` | BLOCKED before project configuration/Java compilation: FileLockContentionHandler creation fails with `java.net.SocketException: Operation not permitted`. Same failure on the unmodified baseline. |
| `GRADLE_USER_HOME=/tmp/biobuzz-gradle-home JAVA_HOME='/Applications/Android Studio.app/Contents/jbr/Contents/Home' ./gradlew :TeamCode:testDebugUnitTest --offline --no-daemon` | BLOCKED before test discovery by the same Gradle lock-socket restriction. |
| `python3 TeamCode/tests/run_autonomous.py --upstream-source /tmp/biobuzz-upstream` | PASS: production Java/source API compilation and 34 autonomous regression checks; only Java 8 bootstrap/obsolete-option warnings |
| `python3 TeamCode/tests/run_robot_controls.py` | PRE-EXISTING FAILURE: 21 javac errors for missing gate calibration members and turretLimitBlocked in unchanged Robot.java |
| `git diff --check` | PASS |

Autonomous checks cover endpoints and heading interpolation, fresh path creation, curves,
invalid route rejection, wrapped angles, path ordering, settling, success cleanup, missed
waypoints, timeouts, follower interruption, Ivy cancellation, explicit Stop, invalid start,
and rejecting non-ready/nonfinite localization *before* drivetrain output.

**No successful Gradle/APK build is claimed.** The shell also cannot resolve Maven hosts,
and Pedro/Ivy binaries are absent from its cache. Source compatibility checks cannot verify
published transitive dependency resolution, Android packaging, or physical behavior.
The supplied upstream source directory is temporary; normal reusable verification after
Gradle sync is `python3 TeamCode/tests/run_autonomous.py`.

On the development machine with normal network and Gradle socket permissions, run:

```sh
./gradlew :TeamCode:assembleDebug
./gradlew :TeamCode:testDebugUnitTest
python3 TeamCode/tests/run_autonomous.py
```

The custom hardware-free harness is the autonomous test suite; the repository previously
had no Gradle unit tests under src/test. Reconcile the older RobotControlsRegression/source
mismatch separately before treating all repository tests as green.

## Driver Station and competition readiness

The new **BioBuzz Pedro Auto (UNVALIDATED)** is intentionally absent from Driver Station
selection while annotated `@Disabled`. The old **BioBuzz Auto** is not this implementation.
Before changing the annotation or flags:

1. Verify four motor names, Pedro wheel directions and axis/rotation behavior. The existing
   Robot mixer differs from Pedro; do not copy TeleOp's direction values without wheel tests.
   Confirm drive run modes are correct and no controller is competing for the drivetrain.
2. Verify Pinpoint device/pod type, encoder signs, pod installation/offsets, units, startup
   READY status and actual measured forward/strafe/rotation localization. Confirm the existing
   Foresight tuning on the current weight, battery, wheel and mechanism configuration.
3. Measure the selected alliance/start pose, robot forward orientation, scoring and parking
   poses, path clearance for the robot's footprint, and field obstacles. Replace all example
   poses; verify adjacent endpoints and headings and record the measurements.
4. Validate the arrival/settling tolerances and deadlines during controlled tests. Existing
   Foresight velocity settings are preserved; they are NOT a newly imposed slow-speed cap.
   Use verified Pedro route constraints for initial reduced-speed tests before full-speed runs.
5. Only after those checks, set the three corresponding flags in BioBuzzAutoConfig true,
   remove @Disabled, rename the OpMode to remove UNVALIDATED, and rebuild/install using the
   team's normal process. Select that exact Pedro OpMode, INIT, check telemetry says READY,
   place the robot at the measured pose and orientation, then press START in a clear test area.
   Exercise Stop during each segment before competition use. No automatic deployment is provided.

## Shooter/intake integration and later cycles

Current Robot.java supplies:

| Mechanism | Real interface / readiness |
| --- | --- |
| Flywheel `shooter` | shooterOn, updateShooter(), setShooterRpm(), shooterRpm(), shooterAtSpeed(); velocity PIDF already set by Robot.init |
| Turret `turret` | aimTurret(), turretToAngle(), zeroTurret(), aimedAtTarget(); Limelight/alliance tag filtering |
| Feeder `transmission` and gate `torqueservo1` | updateFeeder(wantShoot, intakePower), readyToShoot(), shotsFired, ballPresent; timed feed progression requires update calls |
| Intake `intake` | Power passed through updateFeeder() / existing feeder motor logic |
| Sensors | updateSensors(); Limelight target freshness/aim, optional color/distance sensor |
| Cleanup | Robot.stop() zeros drivetrain and mechanism motors and stops Limelight |

The source is sufficient to identify these methods but not to approve their complete
physical sequence. Gate source and documentation disagree; verify travel/endpoints, feeding,
ball sensing, flywheel readiness, turret zero/limits, tag freshness and timeout behavior first.
Do not assume the older documented gate calibration API is available.

Insert a verified Ivy command at the marked point between movement commands. Require the
specific mechanism objects so parallel commands cannot own the same actuator. Run each
mechanism controller exactly once per FTC loop while its command is active. Its completion
must verify success (not just a timer); on a failure call routine.fail(reason), then clean up
mechanisms on interruption, failure and FTC Stop. Avoid raw waitMs as proof that scoring worked.
Do not call Robot.init after creating Pedro: it changes the same drivetrain directions/modes.
Before reusing Robot wholesale, introduce an explicit mechanism-only initialization boundary
or deliberately order initialization and verify sole drivetrain ownership.

To extend to start -> launch -> pickup -> launch -> park, add measured poses/path methods,
reuse the same follower/OpMode, and append guarded movement commands plus verified scoring
and intake commands to the sequence. The private movement helper can be reused inside
BioBuzzAutoRoutine; it accepts path, expected start, target, and stage. Add corresponding
stage names and update the overall time budget. Use sequential for dependent actions and
parallel/deadline groups only for mechanisms with distinct ownership and explicit cleanup.

## Primary API references

- https://pedropathing.com/docs/pathing/guide/setting-up-auto
- https://pedropathing.com/docs/pathing/tuning/constants
- https://pedropathing.com/docs/pathing/guide/pose-creation
- https://pedropathing.com/docs/pathing/guide/path-creation
- https://pedropathing.com/docs/pathing/guide/path-following
- https://pedropathing.com/docs/pathing/guide/auto-usage
- https://pedropathing.com/docs/ivy
- https://github.com/Pedro-Pathing/PedroPathing/blob/v3.0.2/core/src/main/java/com/pedropathing/follower/Follower.java
- https://github.com/Pedro-Pathing/Ivy/blob/v1.1.1/pedro/src/main/java/com/pedropathing/ivy/pedro/PedroCommands.java
- https://github.com/Pedro-Pathing/Ivy/blob/v1.1.1/core/src/main/java/com/pedropathing/ivy/Scheduler.java
