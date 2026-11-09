package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/** Live adjustment of the production turret controller; gains last only for this OpMode. */
@TeleOp(name = "Turret Tracking (tuning)", group = "Test")
public class TurretTrackingTuner extends LinearOpMode {
    private static final String[] NAMES = {"P", "D", "kS", "kV", "deadband deg"};
    private static final double[] STEPS = {0.001, 0.0001, 0.005, 0.00005, 0.25};
    private static final double[] MAXIMUMS = {0.05, 0.005, 0.10, 0.005, 5.0};
    private int selected;
    private int samples;
    private double squaredError;
    private double peakError;
    private double minTx = Double.POSITIVE_INFINITY;
    private double maxTx = Double.NEGATIVE_INFINITY;

    @Override
    public void runOpMode() {
        Robot robot = new Robot();
        robot.init(hardwareMap, false);
        robot.turretGains.quietStart();
        // This test operates only the turret and an explicitly enabled slow drivetrain.
        robot.drive(0, 0, 0);
        robot.turret.setPower(0);
        robot.shooter.setPower(0);
        robot.intake.setPower(0);
        robot.transmission.setPower(0);
        telemetry.setMsTransmissionInterval(100);

        try {
            while (opModeInInit()) {
                if (gamepad1.xWasPressed()) robot.alliance = Robot.Alliance.BLUE;
                if (gamepad1.bWasPressed()) robot.alliance = Robot.Alliance.RED;
                if (gamepad2.backWasPressed()) robot.zeroTurret();
                editGains(robot);
                robot.updateSensors();
                showTelemetry(robot);
                telemetry.update();
                idle();
            }

            while (opModeIsActive()) {
                boolean zero = gamepad2.backWasPressed();
                if (!gamepad2.right_bumper && zero) robot.zeroTurret();
                editGains(robot);
                robot.updateSensors();
                double driveScale = gamepad1.left_bumper ? 0.25 : 0;
                robot.drive(-gamepad1.left_stick_y * driveScale,
                        gamepad1.left_stick_x * driveScale, gamepad1.right_stick_x * driveScale);

                if (!gamepad2.right_bumper) {
                    robot.aimTurret(false, 0);
                } else if (Math.abs(gamepad2.right_stick_x) > 0.1) {
                    robot.aimTurret(false, gamepad2.right_stick_x * 0.12);
                } else {
                    robot.aimTurret(true, 0);
                    if (robot.targetFresh && "TRACK".equals(robot.turretAimState())) {
                        double error = robot.turretAimErrorDegrees();
                        if (Double.isFinite(error)) {
                            samples++;
                            squaredError += error * error;
                            peakError = Math.max(peakError, Math.abs(error));
                            minTx = Math.min(minTx, robot.targetTx);
                            maxTx = Math.max(maxTx, robot.targetTx);
                        }
                    }
                }
                showTelemetry(robot);
                telemetry.update();
                idle();
            }
        } finally {
            robot.stop();
        }
    }

    private void editGains(Robot robot) {
        // Release the motion-enabling bumper before changing the controller.
        boolean up = gamepad2.dpadUpWasPressed();
        boolean down = gamepad2.dpadDownWasPressed();
        boolean left = gamepad2.dpadLeftWasPressed();
        boolean right = gamepad2.dpadRightWasPressed();
        boolean quiet = gamepad2.xWasPressed();
        boolean defaults = gamepad2.yWasPressed();
        boolean resetStats = gamepad2.aWasPressed();
        if (resetStats) clearStats();
        if (gamepad2.right_bumper) return;
        if (up) selected = (selected + NAMES.length - 1) % NAMES.length;
        if (down) selected = (selected + 1) % NAMES.length;
        if (quiet) robot.turretGains.quietStart();
        if (defaults) robot.turretGains.reset();
        if (left || right) {
            double value = value(robot.turretGains) + (right ? 1 : -1) * STEPS[selected];
            value = Range.clip(value, selected == 4 ? 0.25 : 0, MAXIMUMS[selected]);
            switch (selected) {
                case 0: robot.turretGains.p = value; break;
                case 1: robot.turretGains.d = value; break;
                case 2: robot.turretGains.kS = value; break;
                case 3: robot.turretGains.kV = value; break;
                case 4: robot.turretGains.deadbandDeg = value; break;
                default: throw new IllegalStateException("Unknown gain");
            }
        }
        if (left || right || quiet || defaults) {
            robot.resetTurretAim();
            clearStats();
        }
    }

    private double value(Robot.TurretGains gains) {
        switch (selected) {
            case 0: return gains.p;
            case 1: return gains.d;
            case 2: return gains.kS;
            case 3: return gains.kV;
            case 4: return gains.deadbandDeg;
            default: throw new IllegalStateException("Unknown gain");
        }
    }

    private void clearStats() {
        samples = 0;
        squaredError = peakError = 0;
        minTx = Double.POSITIVE_INFINITY;
        maxTx = Double.NEGATIVE_INFINITY;
    }

    private void showTelemetry(Robot robot) {
        telemetry.addLine("TURRET TUNING - gains last for this OpMode only; I stays zero");
        telemetry.addData("Alliance", "%s (gamepad 1 X=BLUE / B=RED in INIT)", robot.alliance);
        telemetry.addLine("G2: hold RB=aim; RB+right stick=manual jog; release RB=stop turret");
        telemetry.addLine("G2 bumper released: Back=forward zero; up/down=select; left/right=adjust");
        telemetry.addLine("G2: X=quiet starting gains; Y=competition defaults; A=reset measurements");
        telemetry.addLine("G1: hold LB to enable 25% driving/rotation with the sticks");
        telemetry.addData("Editing", "%s = %.6f (step %.6f)",
                NAMES[selected], value(robot.turretGains), STEPS[selected]);
        telemetry.addData("Aim State", robot.turretAimState());
        telemetry.addData("Target TX", robot.targetFresh ? robot.targetTx : "unavailable");
        telemetry.addData("Measurements", "n=%d RMS=%.2f deg peak=%.2f deg TX range=%.2f to %.2f",
                samples, samples == 0 ? 0 : Math.sqrt(squaredError / samples), peakError,
                samples == 0 ? 0 : minTx, samples == 0 ? 0 : maxTx);
        robot.addVisionTelemetry(telemetry);
    }
}
