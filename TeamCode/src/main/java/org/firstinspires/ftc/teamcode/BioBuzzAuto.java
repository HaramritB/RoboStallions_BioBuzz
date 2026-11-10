package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Starter Auto: spin up, drive to the shooting spot, shoot the preloads into our HIVE, then LEAVE.
 * The turret aims itself with the Limelight, so the drivetrain never has to turn.
 *
 * Before the match: point the turret straight forward (the encoder zeroes when Auto inits).
 * During INIT (gamepad1): X = blue, B = red, dpad up / down = start delay.
 *
 * The distances below are placeholders. Measure them from your start spot on the field.
 */
@Autonomous(name = "BioBuzz Auto", group = "Competition")
public class BioBuzzAuto extends LinearOpMode {

    static final double DRIVE_POWER = 0.5;
    static final double DRIVE_TIMEOUT_S = 4;
    static final double TO_SHOOT_FORWARD_IN = 24;      // tune: start spot to shooting spot
    static final double LEAVE_STRAFE_IN = 24;          // tune: sideways after shooting, + is right on red
    static final int PRELOAD_BALLS = 3;                // check the game manual's preload limit
    static final double SHOOT_TIMEOUT_S = 8;
    static final double CENTER_TURRET_TIMEOUT_S = 1.5;

    private Robot robot;
    private String step = "init";

    @Override
    public void runOpMode() {
        robot = new Robot();
        robot.init(hardwareMap, true);
        double startDelayS = 0;

        while (opModeInInit()) {
            if (gamepad1.xWasPressed()) robot.alliance = Robot.Alliance.BLUE;
            if (gamepad1.bWasPressed()) robot.alliance = Robot.Alliance.RED;
            if (gamepad1.dpadUpWasPressed()) startDelayS += 1;
            if (gamepad1.dpadDownWasPressed()) startDelayS = Math.max(0, startDelayS - 1);
            Robot.lastAlliance = robot.alliance;

            robot.updateSensors();            telemetry.addData("Alliance", "%s   (X = blue, B = red)", robot.alliance);
            telemetry.addData("Start delay", "%.0f s   (dpad up / down)", startDelayS);
            telemetry.addData("HIVE tag", robot.targetVisible ? "ID " + robot.targetId : "not visible");
            telemetry.addData("Ball at gate", robot.ballColor);
            robot.addGateTelemetry(telemetry);
            telemetry.update();
        }

        robot.shooterOn = true;              // spin up while we wait and drive

        step = "start delay";
        runFor(startDelayS, false, false, 0);

        step = "drive to shooting spot";
        driveInches(TO_SHOOT_FORWARD_IN, 0, false);

        step = "shoot preloads";
        int shotsBefore = robot.shotsFired;
        ElapsedTime shootTimer = new ElapsedTime();
        while (opModeIsActive()
                && robot.shotsFired - shotsBefore < PRELOAD_BALLS
                && shootTimer.seconds() < SHOOT_TIMEOUT_S) {
            updateMechanisms(true, false, Robot.INTAKE_POWER);
        }
        robot.shooterOn = false;

        step = "leave";
        double strafe = robot.alliance == Robot.Alliance.RED ? LEAVE_STRAFE_IN : -LEAVE_STRAFE_IN;
        driveInches(0, strafe, true);

        // Finish with the turret centered so TeleOp keeps a good zero.
        step = "center turret";
        ElapsedTime centerTimer = new ElapsedTime();
        while (opModeIsActive()
                && Math.abs(robot.turretDegrees()) > Robot.TURRET_DEADBAND_DEG
                && centerTimer.seconds() < CENTER_TURRET_TIMEOUT_S) {
            updateMechanisms(false, true, 0);
        }

        robot.stop();
    }

    /**
     * Drives a set distance on the encoders while the turret and shooter keep running.
     * + forward is away from the driver wall at the start, + strafe is to the robot's right.
     */
    private void driveInches(double forwardIn, double strafeIn, boolean centerTurret) {
        int f = (int) Math.round(forwardIn * Robot.DRIVE_TICKS_PER_INCH);
        int s = (int) Math.round(strafeIn * Robot.DRIVE_TICKS_PER_INCH * Robot.STRAFE_MULTIPLIER);
        DcMotorEx[] motors = {robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight};
        int[] deltas = {f + s, f - s, f - s, f + s};

        for (int i = 0; i < motors.length; i++) {
            motors[i].setTargetPosition(motors[i].getCurrentPosition() + deltas[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motors[i].setPower(DRIVE_POWER);
        }

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < DRIVE_TIMEOUT_S
                && robot.frontLeft.isBusy() && robot.frontRight.isBusy()
                && robot.backLeft.isBusy() && robot.backRight.isBusy()) {
            updateMechanisms(false, centerTurret, 0);
        }

        for (DcMotorEx m : motors) {
            m.setPower(0);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    /** Waits while the turret and shooter keep running. */
    private void runFor(double seconds, boolean wantShoot, boolean centerTurret, double intakePower) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < seconds) {
            updateMechanisms(wantShoot, centerTurret, intakePower);
        }
    }

    /** One pass of everything except the drivetrain. Call it in every Auto loop. */
    private void updateMechanisms(boolean wantShoot, boolean centerTurret, double intakePower) {
        robot.updateSensors();
        if (centerTurret) robot.turretToAngle(0);
        else robot.aimTurret(true, 0);
        robot.updateShooter();
        robot.updateFeeder(wantShoot, intakePower);

        telemetry.addData("Step", step);
        telemetry.addData("HIVE tag", robot.targetVisible
                ? String.format("ID %d   tx %.1f deg", robot.targetId, robot.targetTx)
                : "not visible, gate locked");
        telemetry.addData("Shooter", "%.0f / %.0f rpm", robot.shooterRpm(), robot.targetRpm);
        telemetry.addData("Ball", robot.ballColor);
        telemetry.addData("Shots", robot.shotsFired);
        telemetry.update();
    }
}
