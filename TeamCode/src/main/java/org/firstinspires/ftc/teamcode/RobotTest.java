package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

/**
 * Tests every mechanism one at a time. Nothing moves until you press something: motors start
 * with no power, and the gate servo stays unpowered where it is until you first command it.
 *
 * Gamepad1: drivetrain, intake, transmission
 *   Left stick / right stick X   drive, strafe, turn
 *   Hold X / Y / A / B           spin ONE wheel forward: frontleft / frontright / backleft / backright
 *   Right trigger                intake in (harder press = faster)
 *   Left trigger                 intake out
 *   Right bumper / left bumper   transmission up / down
 *
 * Gamepad2: shooter, turret, gate
 *   A                            shooter on / off (starts at 1000 rpm)
 *   Dpad up / down               shooter target +/- 250 rpm
 *   Left bumper / right bumper   shooter target = POLLEN rpm / NECTAR rpm from Robot.java
 *   Right stick X                turret left / right (soft limits +/-150 deg from where it started)
 *   Back                         re-zero turret here
 *   Left stick button            turret auto-aim on / off (needs the Limelight)
 *   X / B                        gate to GATE_OPEN / GATE_CLOSED
 *   Dpad left / right            nudge gate -/+ 0.02, to find the open and closed numbers
 *   Hold right trigger           the real competition shot sequence (needs the Limelight + tag)
 *
 * Motor directions come from Robot.java. If something runs backward, flip it there.
 */
@TeleOp(name = "Robot Test (every feature)", group = "Test")
public class RobotTest extends LinearOpMode {

    static final double WHEEL_TEST_POWER = 0.3;
    static final double STICK_DEADBAND = 0.1;
    static final double RPM_STEP = 250;
    static final double MAX_RPM = 6000;
    static final double GATE_STEP = 0.02;

    @Override
    public void runOpMode() {
        Robot robot = new Robot();
        robot.init(hardwareMap, false);
        robot.zeroTurret();     // doesn't move it; wherever the turret is now becomes 0 deg

        boolean testShooterOn = false;
        double testRpm = 1000;  // start slow so you can check the spin direction safely
        double peakRpm = 0;
        boolean autoAim = false;
        boolean gateCommanded = false;
        double gatePosition = Robot.GATE_CLOSED;
        boolean wasRunningSequence = false;

        telemetry.addLine("Nothing moves until you press a button.");
        telemetry.addData("Limelight", robot.limelight != null ? "found" : "not in config, skipped");
        telemetry.addData("Color sensor", robot.colorSensor != null ? "found" : "not in config, skipped");
        telemetry.addData("LED", robot.led != null ? "found" : "not in config, skipped");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            robot.updateSensors();

            // ---------------- Gamepad1: drivetrain ----------------
            boolean singleWheel = gamepad1.x || gamepad1.y || gamepad1.a || gamepad1.b;
            if (singleWheel) {
                robot.frontLeft.setPower(gamepad1.x ? WHEEL_TEST_POWER : 0);
                robot.frontRight.setPower(gamepad1.y ? WHEEL_TEST_POWER : 0);
                robot.backLeft.setPower(gamepad1.a ? WHEEL_TEST_POWER : 0);
                robot.backRight.setPower(gamepad1.b ? WHEEL_TEST_POWER : 0);
            } else {
                robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }

            // ---------------- Gamepad2: turret ----------------
            if (gamepad2.backWasPressed()) robot.zeroTurret();
            if (gamepad2.leftStickButtonWasPressed()) autoAim = !autoAim;
            double turretStick = gamepad2.right_stick_x;
            if (Math.abs(turretStick) > STICK_DEADBAND) robot.aimTurret(false, turretStick);
            else robot.aimTurret(autoAim, 0);

            // ---------------- Gamepad2: shooter settings ----------------
            if (gamepad2.aWasPressed()) {
                testShooterOn = !testShooterOn;
                peakRpm = 0;
            }
            if (gamepad2.dpadUpWasPressed()) testRpm = Math.min(MAX_RPM, testRpm + RPM_STEP);
            if (gamepad2.dpadDownWasPressed()) testRpm = Math.max(0, testRpm - RPM_STEP);
            if (gamepad2.leftBumperWasPressed()) testRpm = Robot.RPM_YELLOW;
            if (gamepad2.rightBumperWasPressed()) testRpm = Robot.RPM_RED;

            // ---------------- Gamepad2: gate settings ----------------
            if (gamepad2.xWasPressed()) {
                gatePosition = Robot.GATE_OPEN;
                gateCommanded = true;
            }
            if (gamepad2.bWasPressed()) {
                gatePosition = Robot.GATE_CLOSED;
                gateCommanded = true;
            }
            if (gamepad2.dpadRightWasPressed()) {
                gatePosition = Range.clip(gatePosition + GATE_STEP, 0, 1);
                gateCommanded = true;
            }
            if (gamepad2.dpadLeftWasPressed()) {
                gatePosition = Range.clip(gatePosition - GATE_STEP, 0, 1);
                gateCommanded = true;
            }

            double intakePower = 0;
            if (gamepad1.right_trigger > STICK_DEADBAND) intakePower = gamepad1.right_trigger * Robot.INTAKE_POWER;
            else if (gamepad1.left_trigger > STICK_DEADBAND) intakePower = -gamepad1.left_trigger * Robot.INTAKE_POWER;

            boolean runSequence = gamepad2.right_trigger > 0.5;
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
                    gatePosition = Robot.GATE_CLOSED;
                    gateCommanded = true;
                    wasRunningSequence = false;
                }

                if (testShooterOn) robot.setShooterRpm(testRpm);
                else robot.shooter.setPower(0);

                if (gateCommanded) robot.gate.setPosition(gatePosition);

                robot.intake.setPower(intakePower);
                double transmissionPower = 0;
                if (gamepad1.right_bumper) transmissionPower = Robot.TRANSMISSION_FEED_POWER;
                else if (gamepad1.left_bumper) transmissionPower = -Robot.TRANSMISSION_FEED_POWER;
                robot.transmission.setPower(transmissionPower);
            }
            robot.updateLed();

            double rpm = robot.shooterRpm();
            peakRpm = Math.max(peakRpm, rpm);

            // ---------------- Telemetry ----------------
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
                telemetry.addData("Gate", gateCommanded ? String.format("position %.2f", gatePosition)
                                                        : "not moved yet (unpowered)");
            }

            telemetry.addLine("--- Sensors ---");
            telemetry.addData("Limelight", robot.limelight == null ? "not in config"
                    : robot.targetVisible ? String.format("tag %d   tx %.1f deg", robot.targetId, robot.targetTx)
                    : "no HIVE tag for " + robot.alliance);
            telemetry.addData("Color sensor", robot.colorSensor == null ? "not in config"
                    : String.format("%s   hue %.0f   %.1f cm", robot.ballColor, robot.ballHue, robot.ballDistanceCm));
            telemetry.update();
        }

        robot.stop();
    }

    private void addMotor(String name, DcMotorEx motor) {
        telemetry.addData(name, "%.2f   %d", motor.getPower(), motor.getCurrentPosition());
    }
}
