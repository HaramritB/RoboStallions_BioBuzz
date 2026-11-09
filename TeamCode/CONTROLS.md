# Competition TeleOp, gate calibration and turret controls

Choose **BioBuzz Blue TeleOp** or **BioBuzz Red TeleOp** on the Driver Station.
The selected OpMode fixes the alliance automatically, including the HIVE tag filter;
X/B during INIT cannot change it. Both entries share the same driving, shooter,
turret, and gate controls. The old generic BioBuzz TeleOp entry is removed.

BioBuzz Auto still uses X/B during INIT to choose its alliance. After Auto, select
its matching Blue or Red TeleOp; Auto no longer preselects the removed generic entry.

The redesigned gate's measured code defaults are **OPEN=0.41** and **CLOSED=0.59**.
Both competition TeleOps, Auto, Robot Test, and Flywheel Tuning share these values. A complete
saved calibration overrides the code defaults. With no saved pair, fresh installs
use the measured defaults; loading them sends no position command during INIT.
The configured gate servo remains named `torqueservo1`.

In either competition TeleOp, **gamepad 2 X** toggles OPEN/CLOSED. Telemetry identifies whether
the active endpoints came from code defaults or saved calibration. A partially saved
or invalid pair blocks the gate instead of mixing it with defaults.

The calibration tool is still available for future model changes. **Left bumper + Back**
in Gate Calibration disables both saved endpoints and code defaults persistently until
another complete pair is saved, so clearing calibration cannot revive old settings.

## Establish a known reference first

A standard positional servo does not report its shaft angle to this code. With no
known safe command, tiny numeric increments cannot guarantee a tiny first movement.
Temporarily free the linkage or mounting so the servo can move without driving the
gate into the frame. **Do not energize a guessed midpoint with the linkage constrained.**

Use **Gate Calibration** in the Test group. It maps only the gate and disables its
PWM on INIT. No position is sent during INIT or automatically at START.

All calibration controls use **gamepad 1**:

1. With the mechanism freed, START and hold **left bumper**, then tap **A** to command
   the explicit **0.5000 reference**. Verify it moves to a position and holds. If it
   rotates continuously, press **B** to release PWM and configure positional mode.
2. Keep that reference powered while reconnecting/repositioning the linkage with
   the gate clear of the frame and with room to move in both directions.
3. Hold **right bumper** and tap **D-pad left/right** for **-0.01/+0.01 per press**.
   Add **left bumper** for **0.0005** fine steps. Holding the D-pad does not repeat.
4. Find OPEN with enough clearance for the ball, without reaching frame contact.
   Tap **X** to mark it. Find CLOSED where the gate blocks the ball while still clear
   of the frame; tap **Y** to mark it. OPEN may be numerically lower or higher.
5. Tap **A without left bumper** to save **both** marked endpoints. Saving sends no
   position command. Values persist on the Robot Controller and load in all gate modes.
6. In either competition TeleOp, **gamepad 2 X** toggles the calibrated gate OPEN/CLOSED.

**B immediately releases just the gate's PWM**. STOP also releases it in calibration.
Release invalidates the current calibration command reference; a subsequent jog cannot
silently re-energize it. Re-establish the reference with the mechanism freed if needed.
**Left bumper + Back** clears the saved endpoints. Merely marking one endpoint does
not overwrite the saved pair. There is no automatic search for frame contact, torque
reduction, or stall detection; stop short of contact and leave visible clearance.

Servo direction, hub PWM configuration, servo programming, horn alignment, and mounting
must remain the same after calibration. Recalibrate after changing any of them or the
mechanism. The tool preserves the configured PWM range rather than expanding travel.

SWYFT lists its Torque servo as programmable, including a continuous mode. Check the
[manufacturer's servo information](https://swyftrobotics.com/electrical/swyft-servos)
if its current mode is uncertain. The code cannot identify that programmed mode.

For the turret, **gamepad 2 right stick X** overrides auto-aim. Gamepad 2 **Y** toggles
auto-aim; hold **B** to center. With the turret physically pointing straight forward,
**gamepad 2 Back** resets its encoder zero. Software limits are **-709 to +548 ticks**,
inside the measured physical endpoints of -724/+563. Positive ticks turn right from above.

Telemetry shows requested/applied power, raw encoder ticks, limit blocking, and Pinpoint
heading-compensation status. See [Turret tracking setup](TURRET_TRACKING.md) for deployment,
calibration, and the turret-specific regression command.

Robot Test uses **gamepad 1 X/B** to nudge by 0.002 inside the saved endpoint interval.
Its first requested movement uses the calibrated closed position as the command reference.
Use Gate Calibration to change endpoints; Robot Test cannot expand the saved travel.

## Hardware-free regression checks

After syncing FTC SDK 12.0.0 and installing the Android SDK:

```sh
python3 TeamCode/tests/run_robot_controls.py
```

This exercises production gate/turret methods against recording hardware proxies. It does
not validate actual motor wiring, servo travel, mechanical clearance, or on-robot behavior.
