# Competition TeleOp, gate calibration and turret controls

**One-person layout:** select the original **BioBuzz Blue/Red TeleOp (Single Controller)**
or **BioBuzz Blue/Red TeleOp V2**. Both use gamepad 1: left stick drive/strafe, right stick X
rotate, LB slow drive, tap RT / LT to toggle intake in / out, A shooter toggle, X gate toggle
(V2: X toggles the whole shooting sequence: gate open, intake in, LED purple; starts only within +/-50 RPM),
Y auto-aim toggle, B hold to center the turret, D-pad left/right hold to jog the turret,
D-pad up/down RPM trim, and RB flower toggle. Back zeros the turret during INIT only.
The gamepad 2 references below describe the original two-person TeleOps and calibration tools.
V2 details are in [V2.md](V2.md).

V2 calibration additions: up/down trims **10 RPM**, or **1 RPM** with LB held; left stick click
freezes/unfreezes the table RPM; right stick click logs a point while calibrating; Back after
START clears trim only. V2 TeleOp's ready indication uses **+/-10 RPM for 150 ms**, while X/RT
retain manual gate/feed control. Original TeleOp trim remains 50 RPM per press.

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

In both competition TeleOps, **gamepad 2 right bumper** toggles the flower lever
**ON=0.435 / OFF=0.1** (`FlowerLever`) using the Servo named **`flower`**. It moves to OFF as
soon as INIT is pressed (and again at START), holds its position with PWM enabled while
TeleOp runs, and displays its commanded state. These measured
values are fixed TeleOp constants; calibration preferences do not override them.

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

## TeleOp shooter RPM by AprilTag distance

For PIDF tuning and distance calibration, use [the shooter tuning walkthrough](SHOOTER_TUNING.md).
The tuner tests 3215, 3220, 3235 and 3300 RPM, starts from V2 TeleOp's reference F, and saves
a per-volley summary to `/sdcard/FIRST/java/src/Datalogs/shooter_volleys.csv`.

Both competition TeleOps select the base shooter RPM from the fresh, locked HIVE
AprilTag's camera-to-tag distance in meters:

| Distance (m) | Base RPM |
| --- | --- |
| 1.39 | 3215 |
| 1.52 | 3220 |
| 1.62 | 3235 |

Values between points are linearly interpolated. Closer than the table uses 3215;
farther than the table uses 3235. A missing tag or unavailable/invalid pose immediately
uses **3215 base RPM**, even during the brief retained turret-target visibility period.
The distance is from the camera to the locked alliance HIVE tag, matching the
existing Limelight distance telemetry. Shooter telemetry identifies the RPM source.

**Gamepad 2 D-pad up/down** still adds/subtracts **50 RPM** of manual trim; at zero
trim, motor targets match the table and fallback exactly. **A** and the right trigger
retain their shooter on/off behavior. Distance selection alone does not start the
shooter. Auto and Robot Test retain their existing shooter speed selection.

## Flower lever calibration

Configure the SWYFT torque servo as **Servo**, named **`flower`**. Select
**Flower Calibration** in the **Test** group. It maps only `flower`; INIT sends
no position and disables its PWM. The configured PWM range/direction are preserved.
Use positional mode for this calibration; a continuously rotating servo needs its
mode changed before numeric positions can represent lever ON and OFF.

All controls use **gamepad 1**, after START:

1. Free the linkage or mounting first, then hold **LB** and tap **A** to command
   the **0.5000 reference**. Reattach the lever in a clear position while holding
   that reference. A positional servo cannot report its unknown starting angle.
2. Hold **RB** and tap **D-pad left/right** to move **-0.05/+0.05** per press.
   Add **LB** for **0.005** fine steps. Holding the D-pad does not repeat.
3. At the desired ON position, tap **X**. At OFF, tap **Y**. ON can be numerically
   above or below OFF. Stop before the lever binds or hits a hard stop.
4. Tap **A without bumpers** to save both marked values on this Robot Controller.
   The screen shows current command, marked positions, and saved positions.
5. With a powered reference, hold **RB** and tap **D-pad up/down** to verify the
   saved **ON/OFF** positions. **B** releases PWM immediately; STOP also releases it.
   After release, jogging and recall require the explicit reference command again.

After a position is commanded, the OpMode keeps the PWM hold enabled until **B**
or STOP; marking/saving endpoints and releasing the bumpers keep the hold active.
Telemetry shows **Position hold** and **PWM signal**. These confirm the software
request and PWM state, not measured servo voltage/current. Larger steps change
the target position, not servo torque or supply power.

Flower calibration values use separate preferences (`biobuzz_flower_calibration_v1`,
keys `on` and `off`) and do not change gate calibration. Competition TeleOp uses
the configured **ON=0.435 / OFF=0.1** `FlowerLever` constants; saving a new calibration pair does
not automatically change those competition endpoints.

## Pollen indicator setup

Use the **goBILDA RGB Indicator Light (3118-0808-0002)**. On the Driver Station,
configure **Expansion Hub -> Servos -> port 3** as **Servo**, named **`led`**.
Configure the sensor on its connected **I2C bus** as **REV Color Sensor V3**,
named **`colorsensor`**. The older name **`colorSensor`** is also accepted.

The light is **green whenever pollen is detected**, otherwise **red**, independent
of pollen color, shooter readiness, gate position, or visible HIVE tags. It starts
red on INIT and refreshes with every sensor update, including competition INIT loops.
Both **BioBuzz Blue TeleOp** and **BioBuzz Red TeleOp** show **Pollen LED** during
INIT and driving, including a message if the LED or color sensor is missing.
The V3 uses proximity for presence, so any nearby object can trigger the indicator.
A missing color sensor leaves the light red.

Start by holding pollen in its intended position and checking distance in **Robot
Test (every feature)**. The starting detection threshold is **less than 4.0 cm**
(`BALL_PRESENT_CM` in Robot.java); tune it for your mounting so the empty mechanism
does not trigger it. Nonfinite, zero, and negative distances count as no detection.
Robot Test displays the detected presence as **Pollen LED** along with color/distance.

The LED alone uses the standard FTC 600-2400 us servo PWM range. Green is 0.500
(1500 us); red is 0.280 (1104 us), slightly above the OFF boundary. See the
[manufacturer's PWM chart](https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/2275/15126/3118-0808-0002-Product-Insight-4__88285.1757516465.png?c=1).
This behavior runs while an OpMode using Robot's sensor updates is initialized or
active; software cannot keep updating the light after the FTC runtime disables PWM.

## Hardware-free regression checks

After syncing FTC SDK 12.0.0 and installing the Android SDK:

```sh
python3 TeamCode/tests/run_robot_controls.py
```

This exercises production gate/turret/indicator methods against recording hardware proxies. It does
not validate actual motor wiring, servo travel, mechanical clearance, or on-robot behavior.
