package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * Tests every mechanism from gamepad1 alone. Nothing moves until you press something: motors
 * start with no power, and the gate receives no position command until explicitly requested.
 *
 * Gamepad1
 *   Left stick / right stick X   drive, strafe, turn
 *   Right trigger / left trigger intake in / out (harder press = faster)
 *   Right bumper / left bumper   transmission up / down
 *   A                            shooter on / off (starts at 1000 rpm)
 *   Dpad up / down               shooter target +/- 250 rpm
 *   Right stick button           shooter target: POLLEN rpm <-> NECTAR rpm from Robot.java
 *   Dpad right / left (hold)     turret right / left (soft limits +/-150 deg from where it started)
 *   Left stick button            turret auto-aim on / off (needs the Limelight)
 *   X / B                        nudge gate +/-0.002 inside saved calibration limits
 *   Hold Y                       the real competition shot sequence (needs the Limelight + tag)
 *   Back                         WHEEL TEST mode on / off. In it, hold X / Y / A / B to spin ONE
 *                                wheel forward: frontleft / frontright / backleft / backright
 *
 * Motor directions come from Robot.java. If something runs backward, flip it there.
 */
@TeleOp(name = "Robot Test (every feature)", group = "Test")
public class RobotTest extends LinearOpMode {

    static final double WHEEL_TEST_POWER = 0.3;
    static final double TURRET_TEST_POWER = 0.3;
    static final double STICK_DEADBAND = 0.1;
    static final double RPM_STEP = 250;
    static final double MAX_RPM = 6000;

    @Override
    public void runOpMode() {
        Robot robot = new Robot();
        robot.init(hardwareMap, false);
        robot.zeroTurret();     // doesn't move it; wherever the turret is now becomes 0 deg

        boolean wheelTestMode = false;
        boolean testShooterOn = false;
        double testRpm = 1000;  // start slow so you can check the spin direction safely
        double peakRpm = 0;
        boolean autoAim = false;
        boolean wasRunningSequence = false;

        telemetry.addLine("Nothing moves until you press a button. Everything is on gamepad1.");
        telemetry.addData("Limelight", robot.Limelight != null ? "found" : "not in config, skipped");
        telemetry.addData("Color sensor", robot.colorSensor != null ? "found" : "not in config, skipped");
        telemetry.addData("LED", robot.led != null ? "found" : "not in config, skipped");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            robot.updateSensors();

            if (gamepad1.backWasPressed()) {
                wheelTestMode = !wheelTestMode;
                robot.drive(0, 0, 0);
            }

            // Read every edge once per loop, so a press isn't lost to the wrong mode.
            boolean xPressed = gamepad1.xWasPressed();
            boolean bPressed = gamepad1.bWasPressed();
            boolean aPressed = gamepad1.aWasPressed();

            // ---------------- Drivetrain ----------------
            if (wheelTestMode) {
                robot.frontLeft.setPower(gamepad1.x ? WHEEL_TEST_POWER : 0);
                robot.frontRight.setPower(gamepad1.y ? WHEEL_TEST_POWER : 0);
                robot.backLeft.setPower(gamepad1.a ? WHEEL_TEST_POWER : 0);
                robot.backRight.setPower(gamepad1.b ? WHEEL_TEST_POWER : 0);
            } else {
                robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

            // ---------------- Turret ----------------
            if (gamepad1.leftStickButtonWasPressed()) autoAim = !autoAim;
            if (gamepad1.dpad_right) robot.aimTurret(false, TURRET_TEST_POWER);
            else if (gamepad1.dpad_left) robot.aimTurret(false, -TURRET_TEST_POWER);
            else robot.aimTurret(autoAim, 0);

            // ---------------- Shooter settings ----------------
            if (aPressed && !wheelTestMode) {
                testShooterOn = !testShooterOn;
                peakRpm = 0;
            }
            if (gamepad1.dpadUpWasPressed()) testRpm = Math.min(MAX_RPM, testRpm + RPM_STEP);
            if (gamepad1.dpadDownWasPressed()) testRpm = Math.max(0, testRpm - RPM_STEP);
            if (gamepad1.rightStickButtonWasPressed()) {
                testRpm = testRpm == Robot.RPM_YELLOW ? Robot.RPM_RED : Robot.RPM_YELLOW;
            }

            // ---------------- Gate settings ----------------
            if (!wheelTestMode && (xPressed || bPressed)) {
                robot.gateController.jogWithinSaved(xPressed ? 1 : -1);
            }

            double intakePower = 0;
            if (gamepad1.right_trigger > STICK_DEADBAND) intakePower = gamepad1.right_trigger * Robot.INTAKE_POWER;
            else if (gamepad1.left_trigger > STICK_DEADBAND) intakePower = -gamepad1.left_trigger * Robot.INTAKE_POWER;

            boolean runSequence = !wheelTestMode && gamepad1.y;
            if (runSequence) {
                // Competition logic: shooter speed by ball color, gate only opens on our HIVE tag.
                robot.shooterOn = true;
                robot.updateShooter();
                robot.updateFeeder(true, intakePower);
                wasRunningSequence = true;
            } else {
                if (wasRunningSequence) {
                    // The sequence left the gate closed; pick up manual control from there.
                    robot.shooterOn = false;
                    robot.gateOpen = false;
                    robot.gateController.commandSaved(false);
                    wasRunningSequence = false;
                }

                if (testShooterOn) robot.setShooterRpm(testRpm);
                else robot.shooter.setPower(0);

                robot.intake.setPower(intakePower);
                double transmissionPower = 0;
                if (gamepad1.right_bumper) transmissionPower = Robot.TRANSMISSION_FEED_POWER;
                else if (gamepad1.left_bumper) transmissionPower = -Robot.TRANSMISSION_FEED_POWER;
                robot.transmission.setPower(transmissionPower);
            }

            double rpm = robot.shooterRpm();
            peakRpm = Math.max(peakRpm, rpm);

            // ---------------- Telemetry ----------------
            telemetry.addData("MODE", wheelTestMode
                    ? "WHEEL TEST: hold X=FL  Y=FR  A=BL  B=BR   (Back to exit)"
                    : "normal   (Back = wheel test)");

            telemetry.addLine("--- Drivetrain (power / encoder) ---");
            addMotor("frontleft", robot.frontLeft);
            addMotor("frontright", robot.frontRight);
            addMotor("backleft", robot.backLeft);
            addMotor("backright", robot.backRight);

            telemetry.addLine("--- Intake / transmission ---");
            addMotor("intake", robot.intake);
            addMotor("transmission", robot.transmission);

            telemetry.addLine("--- Shooter ---");
            if (runSequence) {
                telemetry.addData("Mode", "SEQUENCE   target %.0f rpm for %s",
                        robot.targetRpm, robot.shotColor);
            } else {
                telemetry.addData("Mode", "MANUAL   %s   target %.0f rpm", testShooterOn ? "ON" : "OFF", testRpm);
            }
            telemetry.addData("Actual", "%.0f rpm   peak %.0f   power %.2f", rpm, peakRpm,
                    robot.shooter.getPower());

            telemetry.addLine("--- Turret ---");
            telemetry.addData("Angle", "%.1f deg   %s   power %.2f", robot.turretDegrees(),
                    autoAim ? "AUTO-AIM" : "manual", robot.turret.getPower());

            telemetry.addLine("--- Gate (torqueservo1) ---");
            if (runSequence) {
                telemetry.addData("Gate", "%s (sequence)%s", robot.gateOpen ? "OPEN" : "closed",
                        robot.targetVisible ? "" : "   locked: no HIVE tag");
            } else {
                telemetry.addData("Gate", Double.isFinite(robot.gateController.command)
                        ? String.format("command %.4f", robot.gateController.command) : "not commanded");
            }
            robot.addGateTelemetry(telemetry);

            telemetry.addLine("--- Sensors ---");
            telemetry.addData("Limelight", robot.Limelight == null ? "not in config"
                    : robot.targetVisible ? String.format("tag %d   tx %.1f deg", robot.targetId, robot.targetTx)
                    : "no HIVE tag for " + robot.alliance);
            telemetry.addData("Color sensor", robot.colorSensor == null ? "not in config"
                    : String.format("%s   hue %.0f   %.1f cm", robot.ballColor, robot.ballHue, robot.ballDistanceCm));
            telemetry.addData("Pollen LED", robot.led == null ? "not in config"
                    : robot.ballPresent ? "GREEN: pollen detected" : "RED: no pollen detected");
            telemetry.update();
        }

        robot.stop();
    }

    private void addMotor(String name, DcMotorEx motor) {
        telemetry.addData(name, "%.2f   %d", motor.getPower(), motor.getCurrentPosition());
    }
}
