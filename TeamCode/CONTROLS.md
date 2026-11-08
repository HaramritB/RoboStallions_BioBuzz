# Gate calibration and turret controls

Use **BioBuzz TeleOp**, press INIT, then START. INIT sends no gate position.
With no calibration saved, starting TeleOp also leaves the gate where it is.

With the gate currently physically open, use **gamepad 2**:

1. Hold **left trigger** and tap **X** to save the current command as **OPEN**.
2. Tap **D-pad right/left** to nudge the gate by **+0.005/-0.005** per press.
   Find which direction closes it, and stop before the linkage binds or the servo strains.
3. Hold **left trigger** and tap **B** to save that command as **CLOSED**.
4. **Left bumper** recalls OPEN; **right bumper** recalls CLOSED. Neither needs a tag.

Both endpoints save on the Robot Controller and are reused by TeleOp, Auto, and Robot Test,
including after restarting the app. Identical endpoints do not enable automatic feeding.
Recalibrate if the servo, linkage, direction, or configured port changes.

These are commanded servo positions, not measured shaft positions. Physically pushing the gate
by hand does not update its command. The initial reference uses the last servo command, falling
back to `0.0` (the old TeleOp's idle command) if unavailable.

Manual adjustments hold their position and stop automatic transmission. Release and press
**gamepad 2 right trigger** again to resume automatic feeding. Automatic feeding requires both
saved endpoints, a visible alliance tag, correct turret aim, and shooter speed.

For the turret, hold **gamepad 1 D-pad left/right** for low-power manual movement, or use
**gamepad 2 right stick X**. Both override auto-aim. Gamepad 2 **Y** toggles auto-aim;
hold **B** to center. With the turret physically pointing straight forward, **Back** resets
its encoder zero. The soft limits remain at +/-150 degrees.

Telemetry shows turret control source, requested/applied power, encoder ticks, motor enable
state, and a soft-limit warning. If applied power is nonzero but the turret does not move,
check the configured `turret` motor port, motor wiring, and mechanical binding. If a soft
limit is shown, move inward; reset zero only with the turret physically centered.

Robot Test uses **gamepad 1 X/B** to nudge the gate by the same small step. Save endpoints
in BioBuzz TeleOp before testing its automatic shot sequence.

## Hardware-free regression checks

After syncing FTC SDK 12.0.0 and installing the Android SDK:

```sh
python3 TeamCode/tests/run_robot_controls.py
```

This exercises production gate/turret methods against recording hardware proxies. It does
not validate actual motor wiring, servo travel, mechanical clearance, or on-robot behavior.
