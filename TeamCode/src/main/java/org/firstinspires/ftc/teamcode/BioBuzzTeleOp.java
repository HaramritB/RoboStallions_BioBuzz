package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Driver 1 (gamepad1)
 *   Left stick          drive + strafe
 *   Right stick X       turn
 *   Left bumper (hold)  slow mode
 *   Right trigger       intake
 *   Left trigger        outtake (intake + transmission in reverse)
 *
 * Driver 2 (gamepad2)
 *   Right trigger (hold) shoot. Turns the flywheel on; balls feed one at a time, only while our
 *                        HIVE tag is visible, the turret is aimed, and the flywheel is at speed
 *                        for that ball's color
 *   A                    flywheel on / off (spin it up early so the first shot is fast)
 *   Y                    turret auto-aim on / off
 *   Right stick X        manual turret (overrides auto-aim while held)
 *   B (hold)             turret back to center
 *   Dpad up / down       shooter rpm trim, +/- 50 rpm on every color *   Back                 re-zero the turret (only with the turret pointing straight forward)
 *
 * During INIT: X = blue alliance, B = red alliance (defaults to whatever Auto last ran).
 */
@TeleOp(name = "BioBuzz TeleOp", group = "Competition")
public class BioBuzzTeleOp extends LinearOpMode {

    static final double SLOW_MODE_SCALE = 0.4;
    static final double STICK_DEADBAND = 0.1;

    @Override
    public void runOpMode() {
        Robot robot = new Robot();
        robot.init(hardwareMap, false);
        boolean autoAim = true;

        while (opModeInInit()) {
            if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) robot.alliance = Robot.Alliance.BLUE;
            if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) robot.alliance = Robot.Alliance.RED;
            Robot.lastAlliance = robot.alliance;

            robot.updateSensors();
            telemetry.addData("Alliance", "%s   (X = blue, B = red)", robot.alliance);
            telemetry.addData("HIVE tag", robot.targetVisible ? "ID " + robot.targetId : "not visible");
            telemetry.addData("Turret", "%.1f deg", robot.turretDegrees());
            telemetry.update();
        }

        while (opModeIsActive()) {
            robot.updateSensors();

            // ---------------- Driver 1: drive + intake ----------------
            double scale = gamepad1.left_bumper ? SLOW_MODE_SCALE : 1.0;
            robot.drive(-gamepad1.left_stick_y * scale,
                        gamepad1.left_stick_x * scale,
                        gamepad1.right_stick_x * scale);

            double intakePower = 0;
            if (gamepad1.right_trigger > 0.2) intakePower = Robot.INTAKE_POWER;
            else if (gamepad1.left_trigger > 0.2) intakePower = -Robot.INTAKE_POWER;

            // ---------------- Driver 2: shooter + turret ----------------
            boolean wantShoot = gamepad2.right_trigger > 0.5;
            if (gamepad2.aWasPressed()) robot.shooterOn = !robot.shooterOn;
            if (wantShoot) robot.shooterOn = true;
            if (gamepad2.yWasPressed()) autoAim = !autoAim;
            if (gamepad2.dpadUpWasPressed()) robot.rpmTrim += Robot.RPM_TRIM_STEP;
            if (gamepad2.dpadDownWasPressed()) robot.rpmTrim -= Robot.RPM_TRIM_STEP;            if (gamepad2.backWasPressed()) robot.zeroTurret();

            double manualTurret = gamepad2.right_stick_x;
            if (gamepad2.b) {
                robot.turretToAngle(0);
            } else if (Math.abs(manualTurret) > STICK_DEADBAND) {
                robot.aimTurret(false, manualTurret);
            } else {
                robot.aimTurret(autoAim, 0);
            }

            robot.updateShooter();
            robot.updateFeeder(wantShoot, intakePower);
            robot.updateLed();

            // ---------------- Telemetry ----------------
            telemetry.addData("Alliance", robot.alliance);
            telemetry.addData("HIVE tag", robot.targetVisible
                    ? String.format("ID %d   tx %.1f deg   %s", robot.targetId, robot.targetTx,
                                    robot.aimedAtTarget() ? "AIMED" : "aiming")
                    : "not visible, gate locked");
            telemetry.addData("Turret", "%.1f deg   %s", robot.turretDegrees(), autoAim ? "AUTO" : "MANUAL");
            telemetry.addData("Shooter", "%s   %.0f / %.0f rpm   trim %+.0f   %s",
                    robot.shooterOn ? "ON" : "OFF", robot.shooterRpm(), robot.targetRpm + robot.rpmTrim,
                    robot.rpmTrim, robot.shooterAtSpeed() ? "AT SPEED" : "");            telemetry.addData("Ball", "%s   hue %.0f   %.1f cm", robot.ballColor, robot.ballHue, robot.ballDistanceCm);
            telemetry.addData("Gate", robot.gateOpen ? "OPEN" : "closed");
            telemetry.addData("Shots", robot.shotsFired);
            telemetry.update();
        }

        robot.stop();
    }
}
