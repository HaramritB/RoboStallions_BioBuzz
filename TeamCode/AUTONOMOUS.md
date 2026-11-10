# BioBuzz autonomous integration

The integrated Java is in the editable checkout
`/Users/pradhyun/Documents/ChatGPT/FTC/RoboStallions_BioBuzz1`, branch `PradhyunAuto`.
The original `/Users/pradhyun/StudioProjects/RoboStallions_BioBuzz1` is clean at `e80d1e5`
but outside this session's writable roots. A guarded incremental patch/installer is supplied
beside this editable checkout. No branches were merged or force-pushed; no robot was deployed.

**Android debug build: passed. Physical execution: intentionally disabled. This is a complete
five-path driving implementation with reusable mechanism commands, not yet a verified scoring auto.**

## Inputs and repository findings

- The requested `BioBuzz_AutoPath.pp.json` is retained unchanged. Its SHA-256 is
  `a61a3d550df9bbd607a2752377f7f146ff9c3287f79398649f6a459dd5936f3b`.
  It matches the requested explicit headings and has an empty robot block. A separately
  supplied `(1)` variant has a tangential first path and modeled 16-inch dimensions;
  those differ from this request and do not establish physical measurements. It was not
  substituted for the explicitly requested original route.
- Read-only upstream inspection on 2026-11-10 found main at
  `b41fb87d8ae5875d51bb5b3a497c32fb7e62c168` (tracking/TeleOp merge). This branch
  remains based on its original controllers. No pull/merge was performed because the active
  request prohibits merging. Main adds saved gate calibration (defaults OPEN .41/CLOSED .59,
  versus this branch's .14/0), revised turret/vision controls, distance-based RPM, and a
  TeleOp flower lever named `flower` (ON .38/OFF .03). Do not copy just these numbers into
  the older adapter: reconcile the complete relevant controller interface and rerun its
  regressions first. ROBOT_CONTROLLER_REVISION_VALIDATED remains false for this reason.
- FTC SDK 12.0.0; Pedro `revhub`/`core` 3.0.2; Ivy `pedro`/`core` 1.1.1;
  tuning 1.0.0. Gradle 9.1.0, AGP 8.13.2, Java 8 source/target. Versions are unchanged.
- Four-wheel mecanum: `frontleft`, `frontright`, `backleft`, `backright`.
  Pedro Constants sets all FORWARD. Robot's TeleOp mixer uses left REVERSE and explicit
  negative left-wheel powers. Different mixers mean actual wheel behavior must be checked;
  neither configuration was changed.
- `Constants.create()` supplies the existing Pinpoint + Mecanum + Foresight objects.
  Device `pinpoint`, goBILDA 4-bar pods, X encoder reversed and Y forward; offsets -3.5625
  and -4.125 inches. Existing calibrated coefficients and units are unchanged. Recent tuning
  commits support the presence of tuning work but do not independently verify current behavior.
- No measured chassis length/width or verified multi-ball inventory interface was found.
  The old encoder Auto labels its 24-inch distances as placeholders and shoots preloads
  after that drive. This does not establish where to shoot or collect on this different route.
- Existing mechanisms: `shooter`, `turret`, `intake`, `transmission`, gate `torqueservo1`,
  optional `Limelight`, color/distance sensor and LED. Current Robot source is authoritative;
  older CONTROLS.md and RobotControlsRegression reference removed gate-calibration methods.

## Changed files

All Java paths below are relative to `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

| File | Purpose |
| --- | --- |
| `autonomous/BioBuzzAuto.java` | Single iterative Pedro OpMode; test/full selection, initialization, telemetry and safe cleanup |
| `autonomous/BioBuzzAutoConfig.java` | Closed physical-review gates, route-hash approval, test policy and confirmed four-ball preload/pickup/shot plan |
| `autonomous/BioBuzzAutoPaths.java` | Generated geometry with all original control points and explicit headings |
| `autonomous/BioBuzzAutoRoutine.java` | Five guarded drives, explicit turn, optional guarded mechanism commands, timeouts and failure state |
| `autonomous/BioBuzzAutoMechanisms.java` | Nonblocking shoot/collect/center commands plus mechanism-only Robot adapter; no drivetrain ownership |
| `AutoPath.java` | Deprecated disabled compatibility shim; delegates to the single Pedro implementation |
| `BioBuzzAuto.java` | Old encoder Auto retained and explicitly disabled/marked Legacy |
| `Robot.java` | Extracts original mechanism initialization into a shared method and adds initMechanismsOnly; original initialization block preserved byte-for-byte |

Other repository paths:

| File | Purpose |
| --- | --- |
| `TeamCode/routes/BioBuzz_AutoPath.pp.json` | Unchanged authoritative available source input |
| `TeamCode/routes/BioBuzz_route_audit.json` | Reproducible clearance findings, missing measurements and explicit turn |
| `TeamCode/tools/sync_autonomous_route.py` | Source validation, Java generation, audit and stale-artifact check; no runtime parser |
| `TeamCode/tests/BioBuzzAutoRegression.java` | Integrated real-Pedro/Ivy route and mechanism command checks |
| `TeamCode/tests/AutoPathRegression.java` | Compatibility and unique autonomous registration checks |
| `TeamCode/tests/MechanismIsolationRegression.java` | SDK hardware proxies verify initialization isolation, unchanged mechanism setup and cleanup after a failed write |
| `TeamCode/tests/AutoLifecycleRegression.java` | Actual OpMode callbacks: INIT/START/Stop, gamepad selection, repeated initialization and scheduler reset |
| `TeamCode/tests/test_route_source.py` | Source/audit consistency and malformed route rejection |
| `TeamCode/tests/run_autonomous.py` | Runs all Java checks with cached actual binary dependencies; optional tagged-source mode retained |
| `TeamCode/AUTONOMOUS.md` | This handoff |
| `.gitignore` | Excludes Python utility/test bytecode caches |

No TeleOp source, motor direction, encoder conversion, localizer offset, PID coefficient or
library version was changed. Whole Robot.init() retains its previous sequence and mechanism
configuration. The new mechanism-only initializer never maps a drivetrain motor or resets a
turret reference unless explicitly requested (Auto passes false).

## Exact sequence

| Action | Start -> end (inches, degrees) | Heading/geometry |
| --- | --- | --- |
| Path 1 | (57.6323, 9.0657, 90) -> (9.5657, 9.2715, 179.7546) | Line, linear heading |
| Path 2 | Previous endpoint -> (32.2934, 127.6559, -84.7123) | Quadratic Bezier, control (40.923, 34.4143), linear heading |
| Turn before path 3 | Same XY, -84.7123 -> 0 deg | Explicit hold-position command; measured heading/velocity completion |
| Path 3 | (32.2934, 127.6559, 0) -> (46.9373, 125.3042, 90) | Quadratic Bezier, control (40.2631, 108.1664), linear heading |
| Path 4 | Previous endpoint -> (46.7883, 140.2305, 90.572) | Line, linear heading |
| Path 5 | Previous endpoint -> (38.0943, 127.3583, -66.9637) | Quadratic Bezier, control (45.4429, 110.0765), linear heading |

The user's corrected strategy is encoded by `BioBuzzAutoConfig.scoringPlan`:

1. After START, hold the starting pose and shoot **4 preloaded balls before path 1**.
2. Drive path 1 with intake active, collecting **4 staged balls during travel**.
   Require both verified four-ball pickup and settled arrival before beginning path 2.
   If collection finishes first, stop intake and finish the path. If arrival occurs first,
   hold the endpoint until collection succeeds or its timeout fails the routine.
3. Finish path 2; hold its pose and shoot **4 balls**.
4. Hold the same XY and turn from -84.7123 to 0 degrees; verify settling.
5. Finish path 3 (approach), then path 4; intake **4 balls at FLOWER**.
   **Intake only: never initialize, deploy, or command the flower lever.**
6. Finish path 5; hold its pose and shoot **4 balls**, then stop all outputs.

All three counts are confirmed as 4. The simulated complete run verifies 12 shot events.
The current Robot exposes presence at one feeder, and its indexing stops when that feeder
is occupied. It does not expose a verified four-ball storage count or indexing controller.
The mechanism port now supports a monotonic count of verified newly stored balls and explicitly
advertises whether counted collection is supported. It must come from actual storage/indexing
feedback; do not implement it by counting presence-sensor flicker or by elapsed intake time.
The existing Robot adapter advertises unsupported and blocks full scoring hardware initialization,
even if someone incorrectly sets COUNTED_COLLECTION_VALIDATED=true. A real compatible counted
indexing adapter must be implemented and tested before full scoring can run.

A first-N test stops after N drives with ALL mechanisms inactive, including no preload shots;
N>=3 includes the turn. Selecting five paths with D-pad is a movement-only test. Y selects
the separate full scoring plan, which requires all physical/controller approvals and a verified
counted collection interface. No mechanism hardware is initialized in movement-test mode.

Coordinates are robot-center inches: +X right, +Y up, positive heading CCW from +X. The actual
start requires the configured robot center at (57.6323, 9.0657), physically forward along +Y
(90 deg) in that same field frame. Setting a localization pose cannot verify physical placement.
Selecting RED/BLUE changes only mechanism tag filtering; it never mirrors this geometry.

The same follower persists through movement and mechanisms. Every waypoint requires <=2 in
position error, <=3 deg heading error, <=2 in/s linear and <=8 deg/s angular speed, sustained
for 0.20 s. Paths time out after 15 s, turns after 5 s, the entire run after 27 s. These are
initial test policies, not measured precision or an assurance the full routine fits match time.
The added three volleys and slow path speed require physical timing measurements; do not
increase the whole-run limit to hide an overlong competition routine.
Path-scoped settings request <=12 in/s and <=25% modeled max path speed, disable path skipping,
and restore when a path is released/cancelled. These are velocity requests, not hard motor caps.

The installed Ivy default follow command checks atParametricEnd. This implementation replaces
that completion check with arrival/settling. A fresh localizer wrapper checks Pinpoint READY
and finite pose/velocity inside follower.update(), before any controller output. Unexpected
path replacement, interruption, cancellation, invalid starts or failed actions latch failure.
Stop also explicitly stops drivetrain output because Pedro 3.0.2 Follower.stop() only clears
navigation state. Independent raw zero writes remain as a cleanup fallback.

## Clearance blockers

The saved robot block is empty. No BioBuzz footprint was invented. The audit uses 1001
samples per Bezier parameter per drive and can additionally check an explicitly supplied
rectangular footprint. Sampling is not continuous collision proof and does not validate
contact intentions, overhangs, mechanisms or actual field placement.

- Path 4's endpoint is inside the supplied Scoring-Side FLOWER polygon.
- Path 5 starts inside that polygon.
- The inspected visualizer uses a 141.5-inch reference frame; point 4 is only **1.2695 in**
  below its upper boundary. Earlier discussion of 3.7695 in assumed a 144-inch frame and
  must not be used for this file's reference geometry. Verify the real field/frame physically.
- The user confirmed FLOWER pickup is intended. That intent does not establish that the
  robot center may enter the polygon or cross the wall. Actual footprint, intake/lever reach,
  and allowed contact geometry are still unknown.

Proposed resolution, pending measurement/approval: if FLOWER is solid and approached from
below, place the robot center below the polygon's lower extent (137.7 in) by its projected
half-length/half-width plus a measured margin. For heading h, vertical half-extent is
`abs(sin(h))*length/2 + abs(cos(h))*width/2`. The wall constraint is center Y no greater than
`141.5 - halfExtent - margin` in the reference frame. The robot may need a different approach
if a mechanism, rather than its center, must reach FLOWER. Do not use an invented numeric target.
The original file/coordinates remain unchanged until that intent and footprint are confirmed.

## Driver Station procedure

The single intended name is **BioBuzz Pedro Auto**, group Development. It is absent from the
selection list while @Disabled remains. Both the compatibility shim and old Legacy Auto are disabled.

Before enabling physical testing, confirm the actual hardware/directions, drive run modes,
Pinpoint readiness, units/signs/offsets, current Foresight tuning and physical start pose. Measure
ROBOT_LENGTH_IN and ROBOT_WIDTH_IN. Review clearance through the selected first N paths and set
VALIDATED_PATH_COUNT accordingly. Set the appropriate approval flags in BioBuzzAutoConfig, and
set REVIEWED_ROUTE_SHA256 to the exact generated source hash only for the route actually reviewed.
Removing @Disabled by itself cannot bypass these gates. Auto does not change drive run modes:
all four motors must already be RUN_WITHOUT_ENCODER, as expected by the existing Pedro setup.

After those reviews, remove @Disabled from autonomous/BioBuzzAuto.java, build/install through
the team's normal process, select BioBuzz Pedro Auto and INIT:

- Default: controlled first-path-only test, movement only.
- Gamepad 1 D-pad up/down: first N paths, 1..5; switches to controlled test mode.
- Gamepad 1 Y: toggles full route (all five paths) vs first-path controlled test.
- Gamepad 1 A/B: BLUE/RED tag filtering for mapped mechanism commands; no route mirroring.

Check telemetry for the selected mode, route hash, approval blockers, Pinpoint READY and measured
start orientation. START schedules once. FTC Stop cancels and zeros outputs. Exercise Stop during
each movement/action before competition. Full operation additionally requires FULL_ROUTE_VALIDATED,
review of the unresolved FLOWER approach, counted inventory/indexing, current controller reconciliation,
and mechanism readiness/cleanup tests.

## Mechanism integration

`Action.shoot(ballCount, timeout)`, `Action.collect(ballCount, timeout)` and
`Action.centerTurret(timeout)` are composed by the confirmed scoringPlan factory. Each
command owns the mechanism port and runs controllers once per active FTC iteration. Read-only
sensor updates are allowed in INIT; no shooter/turret/intake/gate control runs there.

The adapter reuses updateSensors, aimTurret/turretToAngle, updateShooter and updateFeeder, preserving
Robot's tuned velocity PIDF and feeder sequence. Stale/un-aimed targeting closes the gate; speed
readiness before opening remains in the existing feeder. Its timed feed pulse may continue through
normal flywheel RPM dip. Shooting completion requires both the existing shot counter AND observed
ball-present -> absent transitions for the requested count, so feed time alone cannot prove success.
The legacy single-ball collection command rejects an already occupied feeder and requires
sustained new detection. Four-ball commands instead require a supported monotonic stored-ball
counter delta of exactly 4, plus settling; a reset or over-count aborts. Four timed shot counts
without corresponding sensor-clear events cannot advance, and feeding stops once the requested
counter limit is reached to prevent an additional ball while verification is pending.
Invalid/unavailable distance data fails before mechanism output. Turret centering requires its
encoder tolerance. Once a provisional result is detected, outputs stop immediately during the
settling check to prevent additional feeding/collection. Loss of the held waypoint or motion
settling aborts before the next mechanism controller output.

Mechanisms require separately verified gate travel/endpoints, turret reference, sensors, flywheel
behavior and deadlines, plus MECHANISMS_VALIDATED and MECHANISM_SEQUENCE_VALIDATED. Gate cleanup is
issued only after a mechanism action has actually used the hardware; INIT and unused cleanup never
move it. Independent zero writes ensure one failed actuator write cannot skip the others. The
Limelight remains available between commands and is closed on overall completion/failure/Stop.

Counts and action locations are confirmed by the user: four preloads, two four-ball pickups,
three four-ball volleys, intake-only FLOWER. No remaining ball-count or lever clarification is
needed. Confirm actual aiming visibility at the starting pose and points 2/5, turret
reference, RPM selection and sensor behavior at points 2 and 5. No software test proves that
a ball scored in the goal. The result is not yet a physically verified scoring autonomous.

## Verification

All final checks ran from the editable checkout. Direct Java checks used actual cached Maven
binaries for FTC SDK 12.0.0, Pedro 3.0.2 and Ivy 1.1.1; SDK hardware interfaces were simulated with
recording proxies. Tagged upstream sources remain an optional independent API check.

```sh
python3 TeamCode/tools/sync_autonomous_route.py --check
python3 -m unittest discover -s TeamCode/tests -p test_route_source.py -v
python3 TeamCode/tests/run_autonomous.py
GRADLE_USER_HOME=/tmp/biobuzz-gradle-home \
JAVA_HOME='/Applications/Android Studio.app/Contents/jbr/Contents/Home' \
./gradlew :TeamCode:assembleDebug :TeamCode:testDebugUnitTest --offline --no-daemon
python3 TeamCode/tests/run_robot_controls.py
```

| Check | Result |
| --- | --- |
| Source/generation/audit synchronization | PASS |
| Python route-source suite | 8 tests PASS |
| Integrated route/action regression | 136 checks PASS |
| SDK mechanism initialization/isolation/cleanup regression | 17 checks PASS |
| Iterative FTC OpMode lifecycle regression | 8 checks PASS |
| Compatibility and autonomous registration | PASS |
| Android assembleDebug | BUILD SUCCESSFUL; real TeamCode Java compilation, dex and APK packaging completed |
| Gradle testDebugUnitTest | Successful task graph, NO-SOURCE; the repository's actual autonomous tests are the custom harness above |
| Existing RobotControlsRegression | Pre-existing failure: 21 javac missing-member errors for gatePosition, gate calibration methods and turretLimitBlocked absent from current Robot.java |
| Original mechanism init block / TeleOp preservation | Original mechanism block byte-identical after extraction; TeleOp and Constants unchanged; simulated whole vs mechanism-only init agrees |
| Physical robot test | NOT PERFORMED |

The initial normal-network build failed resolving Maven hosts; after refreshing the isolated
Gradle cache from the machine's installed dependencies, the offline build passed. Java 8/JDK 21,
Gradle deprecation, native symbol stripping and blocked Android analytics/FSEvents messages were
non-fatal. No plugin/dependency upgrade or Android project build-setting change was needed.

APK: `TeamCode/build/outputs/apk/debug/TeamCode-debug.apk`. It contains a disabled autonomous
and has not been installed or deployed. Keep the disabled state until the physical reviews pass.

Edit geometry only in TeamCode/routes/BioBuzz_AutoPath.pp.json, then run the generator without
--check. Its source hash changes; previous route approval must be revisited. The original
JSON is not parsed on the robot. New cycles can reuse the same guarded drive/turn and action
interfaces; extend the reviewed source and regenerate rather than duplicating follower control.
