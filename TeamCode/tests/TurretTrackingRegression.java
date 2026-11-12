package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;

/** Runs the production turret controller with a deterministic clock and recording motor. */
public final class TurretTrackingRegression {
    private static int checks;

    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static void near(double actual, double expected, String message) {
        check(Math.abs(actual - expected) < 1e-6, message + ": " + actual + " != " + expected);
    }

    private static double field(Object controller, String name) throws Exception {
        Field field = controller.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return field.getDouble(controller);
    }

    private static String text(Object controller, String name) throws Exception {
        Field field = controller.getClass().getDeclaredField(name);
        field.setAccessible(true);
        return (String) field.get(controller);
    }

    private static final class Aim {
        double now = 1000;
        final Robot.TurretAimController controller;

        Aim() {
            this(Robot.TURRET_MIN_DEG, Robot.TURRET_MAX_DEG);
        }

        Aim(double min, double max) {
            this(min, max, new Robot.TurretGains());
        }

        Aim(double min, double max, Robot.TurretGains gains) {
            controller = new Robot.TurretAimController(min, max, () -> now, gains);
        }

        double update(double angle, boolean visible, double error, double stamp,
                      double heading, double rate) {
            return controller.update(angle, visible, error, stamp, true, heading, rate, 0, 0);
        }
    }

    private static final class Motor {
        int ticks;
        double power;
        final DcMotorEx device = (DcMotorEx) Proxy.newProxyInstance(
                DcMotorEx.class.getClassLoader(), new Class<?>[]{DcMotorEx.class},
                (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "getCurrentPosition": return ticks;
                        case "setPower": power = (double) args[0]; return null;
                        case "getPower": return power;
                        case "getVelocity": return power * 1000; // Freely moving: never a stall.
                        case "setMode": return null;
                        default: throw new AssertionError("Unexpected motor call: " + method.getName());
                    }
                });
    }

    public static void main(String[] args) throws Exception {
        // Main's turret update replaced the single-tag AimTagLock with averaging all visible
        // alliance tags inside Robot.updateVision, so the old lock-selection checks were removed.

        near(Robot.TURRET_MOTOR_TICKS_PER_REV, 145.0909090909091, "1150 RPM encoder ratio");
        near(Robot.TURRET_TICKS_PER_DEG, 2.8212121212121213, "12:84 external gearing");
        check(Robot.TURRET_MIN_TICKS == -709 && Robot.TURRET_MAX_TICKS == 548,
                "Soft limits must have clearance inside measured -724/+563 endpoints");
        check(Robot.TURRET_MAX_DEG - Robot.TURRET_MIN_DEG > 360,
                "Measured multi-turn travel must remain available");

        // Current design (main's turret commit): with a known heading the turret holds its field
        // bearing and never sweeps; it searches only when the heading is unavailable (NaN).
        Aim aim = new Aim();
        near(aim.update(0, false, Double.NaN, 0, 0, 0), 0,
                "With heading and no tag yet, hold the starting direction instead of sweeping");
        check(text(aim.controller, "state").equals("FIELD HOLD"), "No-tag hold reports FIELD HOLD");
        check(aim.update(0, false, Double.NaN, 0, 10, 0) < 0,
                "Field hold counter-rotates the turret when the chassis turns CW");

        aim = new Aim();
        double none = Double.NaN;
        check(aim.update(0, false, Double.NaN, 0, none, 0) > 0,
                "Without heading, auto aim searches before any tag detection");
        check(aim.update(Robot.TURRET_MAX_DEG - 1, false, Double.NaN, 0, none, 0) < 0,
                "Search must reverse before the positive limit");
        check(aim.update(Robot.TURRET_MIN_DEG + 1, false, Double.NaN, 0, none, 0) > 0,
                "Search must reverse before the negative limit");
        check(aim.update(Robot.TURRET_MIN_DEG - 2, false, Double.NaN, 0, none, 0) > 0,
                "Below the negative limit, recovery must move inward");
        check(aim.update(Robot.TURRET_MAX_DEG + 2, false, Double.NaN, 0, none, 0) < 0,
                "Above the positive limit, recovery must move inward");

        Aim auto = new Aim();
        auto.controller.searchWhenTagLost = true;
        check(auto.update(0, false, Double.NaN, 0, 0, 0) > 0,
                "AutoPath opt-in sweeps for the HIVE before the first tag even with heading");
        check(auto.update(0, true, 10, 1, 0, 0) > 0, "AutoPath opt-in tracks a seen tag");
        auto.now += 401;
        near(auto.update(10, false, Double.NaN, 1, 0, 0), Robot.TURRET_SEARCH_POWER,
                "AutoPath opt-in sweeps again once the tag is lost, instead of holding a stale bearing");

        aim = new Aim();
        double rightTagError = -8 * Robot.TURRET_VISION_POWER_SIGN;
        check(aim.update(0, true, rightTagError, 1, 0, 0) > 0,
                "Verified negative TX for a right-hand tag must command rightward motion");
        near(aim.update(3, true, rightTagError, 1, 0, 0), 0.14,
                "Repeated reads of a frame must not move its encoder goal");
        near(aim.update(8, true, rightTagError, 1, 0, 0), 0,
                "Stop when the correctly scaled encoder reaches the frame's goal");

        aim = new Aim();
        near(aim.update(0, true, 0, 1, 0, 0), 0, "Centered stationary target");
        aim.now += 100;
        check(aim.update(0, false, Double.NaN, 1, 10, 0) < 0,
                "CW chassis rotation during dropout must command CCW turret correction");
        near(field(aim.controller, "lastTargetAngle"), -10, "Hold the field bearing through dropout");
        near(aim.update(-10, false, Double.NaN, 1, 10, 0), 0,
                "Reaching the compensated angle must stop correction");
        aim.now += 301;
        near(aim.update(-10, false, Double.NaN, 1, 10, 0), 0,
                "With heading, an expired observation keeps holding the field bearing (no sweep)");
        check(text(aim.controller, "state").equals("FIELD HOLD"), "Expired target reports FIELD HOLD");
        near(aim.update(-10, false, Double.NaN, 1, Double.NaN, 0), Robot.TURRET_SEARCH_POWER,
                "Without heading, an expired observation releases tracking and resumes search");

        aim = new Aim();
        aim.update(0, true, 0, 1, 0, 0);
        aim.now += 100;
        check(aim.update(0, true, 0, 1, 10, 0) < 0,
                "Heading compensation must work between duplicate camera frames too");
        aim.now += 301;
        check(aim.update(0, true, 0, 1, 10, 0) < 0, "Keep holding the compensated bearing");
        near(aim.controller.observationAgeMs(), 401,
                "Duplicate visible frames must not extend the observation timeout");

        aim = new Aim();
        aim.update(0, true, 0, 1, 179, 0);
        aim.update(0, false, Double.NaN, 1, -179, 0);
        near(field(aim.controller, "lastTargetAngle"), -2, "Heading wrap must not create a 358-degree jump");

        aim = new Aim();
        aim.update(0, true, 0, 1, 0, 0);
        check(aim.update(0, true, 0, 1, 0, 60) < 0,
                "Turn-rate feedforward must counter chassis rotation even at zero error");
        check(aim.update(0, true, 0, 1, 0, -60) > 0,
                "Feedforward must also counter CCW chassis rotation");

        aim = new Aim();
        aim.controller.update(0, true, 15, 1, true, 0, 20, 10, 100);
        near(field(aim.controller, "lastTargetAngle"), 12,
                "Capture-time correction must include both turret and chassis rotation");
        near(aim.controller.observationAgeMs(), 100, "Frame latency counts toward target age");
        aim = new Aim();
        aim.controller.update(0, true, 15, 1, true, 0, 0, 0, 300);
        near(field(aim.controller, "lastTargetAngle"), 0,
                "A frame older than the latency budget creates no goal; the turret holds its start direction");
        check(Double.isInfinite(aim.controller.observationAgeMs()),
                "A stale frame is not counted as an observation");

        aim = new Aim();
        aim.update(190, true, 10, 1, 0, 0);
        near(field(aim.controller, "lastTargetAngle"), -160,
                "A target beyond the cable limit must select its legal equivalent angle");
        check(aim.update(190, true, 10, 1, 0, 0) < 0, "Unwind toward that equivalent angle");
        aim = new Aim(-150, 150);
        check(aim.update(149, true, 30, 1, 0, 0) > 0,
                "An unreachable target moves only up to the nearest legal limit");
        near(aim.update(150, false, Double.NaN, 1, 0, 0), 0,
                "At the positive limit the turret never pushes outward");
        check(text(aim.controller, "state").equals("FIELD HOLD LIMIT"),
                "An out-of-reach field target reports FIELD HOLD LIMIT");

        aim = new Aim();
        check(aim.update(0, true, 10, 1, Double.NaN, Double.NaN) > 0,
                "Vision feedback must still work without Pinpoint");
        aim.update(0, false, Double.NaN, 1, 120, 0);
        near(field(aim.controller, "lastTargetAngle"), 10,
                "Heading recovery must not apply an unknown accumulated heading jump");
        near(aim.controller.update(0, true, 0, 2, false, 120, 0, 0, 0), Robot.TURRET_TRACK_MAX_POWER,
                "With heading, an unusable camera keeps holding the last field bearing");
        near(aim.controller.update(0, true, 0, 3, false, Double.NaN, 0, 0, 0), Robot.TURRET_SEARCH_POWER,
                "Without heading, an unusable camera releases the old target and searches");
        aim.controller.cancelReturn();
        near(aim.update(0, false, Double.NaN, 1, 120, 0), 0,
                "Manual override discards the old goal; the turret holds where it was left");
        near(field(aim.controller, "lastTargetAngle"), 0,
                "After an override the held bearing is the current angle, not the old target");
        near(aim.update(Double.NaN, false, Double.NaN, 0, 0, 0), 0, "Reject invalid encoders");
        near(aim.update(0, true, Double.NaN, 1, 0, 0), 0, "Reject invalid visual corrections");

        aim = new Aim();
        near(aim.update(0, true, 0, 1, 0, 0), 0, "Enter quiet hold at the target");
        near(aim.update(0, true, 1.2, 2, 0, 0), 0,
                "Hysteresis must suppress noise just outside the entry deadband");
        check(aim.update(0, true, 1.6, 3, 0, 0) > 0,
                "A real displacement beyond the exit deadband must resume correction");
        near(aim.update(0, true, 0.2, 4, 0, 0), 0, "Reenter quiet hold after correction");
        check(aim.update(0, true, 0.2, 4, 0, 60) < 0,
                "Quiet hold must not suppress counter-rotation feedforward");

        Robot.TurretGains gains = new Robot.TurretGains();
        gains.d = 0.001;
        aim = new Aim(Robot.TURRET_MIN_DEG, Robot.TURRET_MAX_DEG, gains);
        aim.update(0, true, 4, 1, 0, 0);
        double withoutMotion = aim.update(0, true, 4, 1, 0, 0);
        double withMotion = aim.controller.update(0, true, 4, 1, true, 0, 0, 50, 0);
        near(field(aim.controller, "dTerm"), -0.05, "D damps measured turret angular velocity");
        check(withMotion < withoutMotion, "D must reduce overshoot-driving power");
        aim.update(0, true, 20, 2, 0, 0);
        near(field(aim.controller, "dTerm"), 0, "A vision target step must not cause a D kick");
        gains.quietStart();
        near(gains.p, 0.012, "Quiet P starting candidate");
        near(gains.kS, 0.02, "Quiet static-friction starting candidate");
        near(gains.deadbandDeg, 1.5, "Quiet deadband starting candidate");
        near(gains.d, 0, "Candidate starts with D disabled until overshoot is measured");
        check(new Robot().turretGains.p == Robot.TURRET_KP,
                "Tuning one instance must not overwrite competition defaults");

        Robot robot = new Robot();
        Motor motor = new Motor();
        robot.turret = motor.device;
        motor.ticks = Robot.TURRET_MAX_TICKS;
        robot.aimTurret(false, 0.3);
        near(motor.power, 0, "Motor-level positive soft limit");
        robot.aimTurret(false, -0.3);
        near(motor.power, -0.3, "Manual inward movement at positive limit");
        motor.ticks = Robot.TURRET_MIN_TICKS;
        robot.aimTurret(false, -0.3);
        near(motor.power, 0, "Motor-level negative soft limit");
        robot.aimTurret(false, 0.3);
        near(motor.power, 0.3, "Manual inward movement at negative limit");
        motor.ticks = 254;
        near(robot.turretDegrees(), 254 / Robot.TURRET_TICKS_PER_DEG, "Production encoder conversion");
        robot.zeroTurret();
        near(motor.power, 0, "Re-zero must stop the motor first");
        robot.targetTx = 10.06;
        robot.targetFresh = true;
        robot.targetVisible = true;
        robot.updateSensors();
        check(Double.isNaN(robot.targetTx), "Lost vision must clear the misleading retained TX");
        check(!robot.targetFresh && !robot.targetVisible, "Lost vision must clear target flags");
        check(!robot.aimedAtTarget(), "A stale centered offset must not authorize a shot");
        System.out.println("Passed " + checks + " turret tracking regression checks.");
    }
}
