# Turret tracking setup

The turret uses a 1150 RPM Yellow Jacket with a 12-tooth motor pinion driving the
84-tooth turret gear. The motor's exact internal ratio gives 145.0909 encoder ticks
per output revolution and **2.82121 ticks per turret degree** after the external gearing.
The observed 272 ticks for an estimated 90-degree turn is about 7% higher; check the
physical angle accurately before substituting an empirical conversion.

The measured endpoints are **-724/+563 raw ticks** from forward zero. Software adds
15 ticks of inward clearance at each end, giving **-709/+548**. Turret encoder angles
remain unwrapped across the full measured travel, which exceeds 360 degrees.

## Run on the robot

1. Deploy the debug APK. Check `Robot code` reads `BIOBUZZ-FLOWER-ON0.32-OFF0-2026-11-11`.
2. Select **BioBuzz Blue TeleOp** or **BioBuzz Red TeleOp** for the correct alliance.
   During INIT, point the shooter physically forward
   and press **gamepad 2 Back** to establish the same zero used to measure the limits.
3. Check pipeline 4 and `Heading compensation`. Chassis compensation uses the existing
   **GoBilda Pinpoint named `pinpoint`** in the robot configuration. It must report
   `READY` and finite heading/rate. The code reads it without resetting pose or IMU.
   Missing, calibrating, or faulted Pinpoint data falls back to vision/encoder tracking.
4. Start with a stationary chassis. Auto-aim searches before its first detection.
   A tag slightly right of the crosshair should have negative TX and cause positive
   turret movement. The verified vision multiplier remains -1.
5. Acquire a tag, then rotate the chassis slowly both ways. CW chassis heading/rate
   should increase during a right turn; the turret should counter-rotate to hold aim.
   Check `TRACK`, the compensated goal, and requested/applied power. Brief dropouts use
   `REACQUIRE LAST TAG` for up to 400 ms from the observation's estimated capture time.

Aim now locks one alliance tag, initially choosing a tag nearest the crosshair. It
keeps that ID as the number/order of other detections changes. If the selected tag
disappears, it waits 250 ms before selecting another visible alliance tag. The
`Aim target` line identifies the selected tag; distance/face-angle telemetry uses
that same tag. This prevents a changing multi-tag average from moving the aim point.

Search reverses near either software endpoint. Readings outside a software limit get
a short inward `LIMIT RECOVERY` movement. A target beyond an endpoint can use an
equivalent angle within the measured travel (`UNWIND`); the turret cannot maintain
continuous visual lock while physically turning through a cable-limited full turn.
Expired observations resume search, and TX displays as unavailable when no matching
tag is detected. Manual stick movement and centering discard the previous aim goal.

Tracking includes first-order camera-latency correction and chassis turn-rate
feedforward. `TURRET_KV` starts from nominal no-load speed; its final value and the
tracking power cap still need verification under the real turret load. Heading
compensation preserves bearing during rotation; it does not predict a new target
bearing during long periods of robot translation without vision.

## Recheck calibration

Use **Turret Limits (calibration)**. It bypasses the software limits: stop before hard
stops or cable tension. Point forward, release the bumper, and press **gamepad 1 Back**.
Hold **right bumper + right stick X** to jog. Release the bumper and press **A** at
the negative endpoint, **B** at the positive endpoint. Copy the raw measured endpoints
into `TURRET_MEASURED_MIN_TICKS/MAX_TICKS`; the clearance is calculated separately.
Those marks are not saved automatically. Re-establish forward zero after calibration.

## Hardware-free verification

With FTC SDK 12.0.0 cached and the Android SDK installed:

```sh
python3 TeamCode/tests/run_turret_tracking.py
```

These checks run the production controller and motor-limit methods. They cover
acquisition, direction, duplicate frames, heading changes/wrap, short dropouts,
expiry, latency, feedforward, multi-turn travel, and manual control. On-robot testing
is required to verify actual heading direction, mechanical clearance, and tuning.

## Live tuning for stationary jitter

Select **Turret Tracking (tuning)** in the Test group. This uses the production
tracker, software limits, and heading compensation. The shooter/intake/transmission
stay off. All gain edits last only for that OpMode; record the winning values and
copy them to the `TURRET_*` defaults in Robot.java afterward. `SHOOTER_*` gains
control the separate flywheel loop.

The tuner starts with these **candidates**, which still require measurement:

| Gain | Starting value | Units |
| --- | ---: | --- |
| P | 0.012 | motor power / degree |
| I | 0, fixed | integral disabled |
| D | 0 | motor power / (degree/second) |
| kS | 0.020 | motor power |
| kV | 0.00101449 | motor power / (degree/second) |
| Deadband | 1.5 | degrees |

I stays disabled while friction and measurement noise are characterized. D, when
enabled, damps measured turret-plus-chassis angular velocity and avoids a derivative
kick when a new camera goal arrives. Once stationary within the deadband, the
controller waits for an error above deadband + 0.5 degrees before resuming correction.
Chassis rotation or ongoing turret motion immediately releases that quiet hold.

### Controls

- **Gamepad 1 X/B during INIT:** choose BLUE/RED.
- **Gamepad 2 Back with right bumper released:** zero with the turret physically forward.
- **Hold gamepad 2 right bumper:** enable automatic turret tracking. Release to stop.
- **Right bumper + gamepad 2 right stick X:** manually jog at up to 12% power.
- **Gamepad 2 D-pad up/down:** select P, D, kS, kV, or deadband.
- **D-pad left/right with right bumper released:** decrease/increase the selected gain.
- **Gamepad 2 X/Y with right bumper released:** load quiet candidates/competition defaults.
- **Gamepad 2 A:** reset RMS/peak error and TX range measurements.
- **Hold gamepad 1 left bumper:** enable 25% driving/rotation with its sticks.

The live gain step sizes are P=0.001, D=0.0001, kS=0.005, kV=0.00005, deadband=0.25 degrees.

### Test sequence

1. Keep the chassis stationary at a normal shooting distance. Acquire a tag and
   verify `Aim target` stays on one ID while `Raw tags` fluctuates. Let it settle,
   press gamepad 2 A, hold aim for five seconds, then release. Record the gains,
   RMS/peak error, TX range, and whether physical jitter remains. Repeat at least
   three times; keep distance, camera settings, and battery charge comparable.
2. For sharp repeated kicks near the target, decrease kS by 0.005 per trial. Use
   the smallest value that still corrects a small displacement reliably both ways.
   If it sticks outside the deadband, restore one increment and retry.
3. For broad alternating overshoot, decrease P by 0.002 per trial. If this makes
   acquisition too sluggish, restore P and try D=0.0001, then 0.0002. Increase D
   only while overshoot improves; reduce it if measured-velocity noise adds jitter.
4. Use a 5-to-10-degree manual displacement, then release the stick while holding
   aim. Check both directions for settling, repeated crossings, and persistent
   error. Deadband=1.5 degrees is the starting tolerance, not a measured optimum.
   Only tighten it after the aim point and stationary output are stable.
5. Once stationary tracking is quiet, rotate the chassis slowly both ways, then
   stop. Keep kV unchanged if rotation tracking remains good. For consistent lag
   during steady turns, increase kV by 0.00005 per trial; decrease for leading.
   A pinned 0.22 tracking power indicates saturation and limits what gains can fix.

Send the selected tag ID, gain values, RMS error, TX range, and the remaining motion
pattern after the first trials. Those observations determine the next adjustment;
the starting candidates alone do not establish final tuned constants.
