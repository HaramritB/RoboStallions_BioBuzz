package org.firstinspires.ftc.teamcode.v2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Measures turret encoder ticks per degree using the Pinpoint heading as the reference.
 *
 * Robot.TURRET_TICKS_PER_DEG is calculated as 2.8212 from the gear ratio, but an earlier hand
 * measurement suggested ~3.02 (7% more). Odometry aiming relies on this number, so check it:
 *
 *  1. Choose BLUE (X) or RED (B) in INIT. Park about 1.5-2 m from your HIVE.
 *  2. START. The turret tracks the HIVE tag automatically (Y toggles tracking).
 *  3. Stop moving. When "Ready to mark" says YES, press A.
 *  4. Slowly rotate the chassis in place (gamepad 1 right stick, 30% power) at least 180 degrees,
 *     keeping the tag in view, then stop. When ready, press B.
 *  5. Read "Measured". Repeat in the other direction. Copy a consistent value to
 *     TURRET_MOTOR_TICKS_PER_REV / gear constants (or tell us) only if it differs by more than ~2%.
 */
@TeleOp(name = "Turret Scale Check V2", group = "Test")
public final class TurretScaleCheckV2 extends LinearOpMode {
    private static final double DRIVE_SCALE = 0.3;
    private static final double MAX_MARK_ERROR_DEG = 1.0;
    private static final double MAX_MARK_TURN_DEG_S = 2.0;

    @Override
    public void runOpMode() {
        RobotV2 v2 = new RobotV2();
        v2.initTeleOp(hardwareMap);
        Robot.Alliance alliance = Robot.lastAlliance;
        while (opModeInInit()) {
            if (gamepad1.xWasPressed()) alliance = Robot.Alliance.BLUE;
            if (gamepad1.bWasPressed()) alliance = Robot.Alliance.RED;
            v2.setAlliance(alliance);
            v2.beginLoop();
            v2.readPinpoint();
            v2.readVision();
            telemetry.addData("Alliance", "%s (gamepad 1 X blue / B red)", alliance);
            telemetry.addData("Pose", v2.poseStatus);
            telemetry.addData("Vision", v2.visionStatus);
            telemetry.update();
        }

        boolean track = true;
        double unwrappedHeadingDeg = 0;
        double lastHeading = Double.NaN;
        boolean startMarked = false;
        double startTicks = 0, startHeading = 0, startError = 0;
        String result = "Press A to mark the start";

        try {
            while (opModeIsActive()) {
                v2.beginLoop();
                v2.readPinpoint();
                v2.readVision();
                if (v2.poseValid) {
                    if (Double.isFinite(lastHeading)) {
                        double d = v2.headingRad - lastHeading;
                        unwrappedHeadingDeg += Math.toDegrees(Math.atan2(Math.sin(d), Math.cos(d)));
                    }
                    lastHeading = v2.headingRad;
                }
                v2.robot.drive(-gamepad1.left_stick_y * DRIVE_SCALE, gamepad1.left_stick_x * DRIVE_SCALE,
                        gamepad1.right_stick_x * DRIVE_SCALE);
                if (gamepad1.yWasPressed()) track = !track;
                v2.driveTurret(track, false, 0);

                double error = v2.visionErrorDeg();
                boolean ready = v2.poseValid && v2.frameFresh && Double.isFinite(error)
                        && Math.abs(error) < MAX_MARK_ERROR_DEG
                        && Math.abs(Math.toDegrees(v2.omegaRadS)) < MAX_MARK_TURN_DEG_S;
                int ticks = v2.robot.turret.getCurrentPosition();

                if (gamepad1.aWasPressed() && ready) {
                    startMarked = true;
                    startTicks = ticks;
                    startHeading = unwrappedHeadingDeg;
                    startError = error;
                    result = "Start marked. Rotate the chassis >= 180 deg, stop, press B";
                }
                if (gamepad1.bWasPressed() && ready && startMarked) {
                    double dTicks = ticks - startTicks;
                    // The turret must turn clockwise by the chassis' CCW heading change to keep aim.
                    double dTrueDeg = (unwrappedHeadingDeg - startHeading) - (error - startError);
                    if (Math.abs(dTrueDeg) < 90) {
                        result = String.format(java.util.Locale.US, "Rotated only %.0f deg: rotate >= 180 deg", dTrueDeg);
                    } else {
                        double measured = dTicks / dTrueDeg;
                        double pct = 100.0 * (measured - Robot.TURRET_TICKS_PER_DEG) / Robot.TURRET_TICKS_PER_DEG;
                        result = String.format(java.util.Locale.US,
                                "Measured %.4f ticks/deg over %.0f deg (code %.4f, %+.1f%%)%s", measured, dTrueDeg,
                                Robot.TURRET_TICKS_PER_DEG, pct,
                                measured < 0 ? "  NEGATIVE: check heading/turret sign conventions" : "");
                    }
                    startMarked = false;
                }

                telemetry.addData("Result", result);
                telemetry.addData("Ready to mark", ready ? "YES" : "no (need fresh tag, |error| < 1 deg, stopped)");
                telemetry.addData("Turret", "%d ticks (%.1f deg by code scale), tracking %s", ticks,
                        v2.robot.turretDegrees(), track ? "ON" : "OFF");
                telemetry.addData("Heading (unwrapped)", "%.1f deg; turn rate %.1f deg/s", unwrappedHeadingDeg,
                        Math.toDegrees(v2.omegaRadS));
                telemetry.addData("Vision error", Double.isFinite(error) ? String.format(java.util.Locale.US, "%.2f deg", error) : "no tag");
                v2.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            v2.robot.stop();
        }
    }
}
