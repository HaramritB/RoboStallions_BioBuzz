package org.firstinspires.ftc.teamcode.v2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FlowerLever;
import org.firstinspires.ftc.teamcode.Robot;

/**
 * V2 competition TeleOp with the original one-person gamepad 1 controls:
 *  - RT/LT intake in/out, A shooter toggle, X gate toggle, Y auto-aim toggle, B center
 *  - right stick X steers the chassis; held D-pad left/right jogs the turret
 *  - D-pad up/down trims 10 RPM (LB = 1 RPM), RB toggles flower, LB enables slow drive
 *  - left stick click freezes/unfreezes table RPM for calibration; right stick click logs a point
 *  - Back zeros turret during INIT; after START it only clears RPM trim
 *  - turret keeps pointing at the remembered field target while driving (FieldAimV2)
 *  - aims at the upward HIVE CELL only; RPM from a filtered distance; voltage-compensated F
 *  - rumble on gamepad 1 when ready to shoot; loop time on telemetry
 */
public abstract class BioBuzzTeleOpV2 extends LinearOpMode {

    static final double SLOW_MODE_SCALE = 0.4;
    static final double STICK_DEADBAND = 0.1;
    static final double TURRET_JOG_POWER = 0.15;
    static final double TELEMETRY_PERIOD_MS = 100;
    // Edit these in Robot.java (V2_TELEOP_SHOOTER_P/F): Flywheel Tuning starts from the same F.
    static final double SHOOTER_P = Robot.V2_TELEOP_SHOOTER_P;
    static final double SHOOTER_REFERENCE_F = Robot.V2_TELEOP_SHOOTER_F;
    static final double SHOOTER_READY_TOLERANCE_RPM = 10.0;

    protected abstract Robot.Alliance teleOpAlliance();

    @Override
    public final void runOpMode() {
        Gamepad operator = gamepad1;
        RobotV2 v2 = new RobotV2();
        v2.initTeleOp(hardwareMap);
        v2.setShooterTune(SHOOTER_P, SHOOTER_REFERENCE_F);
        v2.atSpeedToleranceRpm = SHOOTER_READY_TOLERANCE_RPM;
        Robot robot = v2.robot;
        Servo flower = hardwareMap.get(Servo.class, "flower");
        FlowerLever.command(flower, false); // Flower OFF as soon as INIT is pressed.
        v2.setAlliance(teleOpAlliance());
        Robot.lastAlliance = teleOpAlliance();
        telemetry.setMsTransmissionInterval(100);

        ShotSequencerV2 sequencer = new ShotSequencerV2();
        ShooterCalibrationV2 calibration = new ShooterCalibrationV2();
        ShotLoggerV2 logger = new ShotLoggerV2();
        boolean autoAim = true;
        boolean manualGateOpen = false;
        boolean flowerOn = false;
        boolean wasReady = false;
        double lastTelemetryMs = Double.NEGATIVE_INFINITY;

        // ================================================================ INIT
        while (opModeInInit()) {
            v2.beginLoop();
            v2.readPinpoint();
            v2.readVision();
            v2.readBall();
            if (operator.backWasPressed()) {
                robot.zeroTurret();
                v2.aim.clearTarget();
            }
            if (flower instanceof PwmControl && !((PwmControl) flower).isPwmEnabled()) {
                FlowerLever.command(flower, false);
            }
            if (RobotV2.nowMs() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
                lastTelemetryMs = RobotV2.nowMs();
                telemetry.addLine("===== BIOBUZZ V2 INIT =====");
                telemetry.addData("Alliance", "%s (fixed by selected TeleOp)", robot.alliance);
                telemetry.addData("Controls", "original one-person layout (gamepad 1)");
                telemetry.addData("Turret zero", "point forward, gamepad 1 BACK (INIT only)");
                v2.addTelemetry(telemetry);
                robot.addGateTelemetry(telemetry);
                telemetry.update();
            }
        }

        // ================================================================ MAIN
        try {
            if (opModeIsActive()) FlowerLever.command(flower, false);
            while (opModeIsActive()) {
                double now = RobotV2.nowMs();
                v2.beginLoop();
                v2.readPinpoint();
                v2.readVision();
                v2.readBall();
                v2.updateShotDistance();

                // ---------------- flower lever (gamepad 1 RB)
                if (operator.rightBumperWasPressed()) {
                    flowerOn = !flowerOn;
                    FlowerLever.command(flower, flowerOn);
                }
                if (flower instanceof PwmControl && !((PwmControl) flower).isPwmEnabled()) {
                    FlowerLever.command(flower, flowerOn);
                }

                // ---------------- driver 1: drive + intake, unchanged
                double scale = gamepad1.left_bumper ? SLOW_MODE_SCALE : 1.0;
                robot.drive(-gamepad1.left_stick_y * scale, gamepad1.left_stick_x * scale,
                        gamepad1.right_stick_x * scale);
                double driverIntake = intakePower(operator);

                // RT/LT belong exclusively to intake in the original one-person layout.
                if (operator.aWasPressed()) robot.shooterOn = !robot.shooterOn;
                boolean trimUp = operator.dpadUpWasPressed();
                boolean trimDown = operator.dpadDownWasPressed();
                boolean resetTrim = operator.backWasPressed();
                boolean toggleCalibration = operator.leftStickButtonWasPressed();
                if (toggleCalibration) calibration.toggle(Robot.rpmForDistance(v2.shotDistanceM));
                if (resetTrim) calibration.resetTrim();
                else calibration.adjust(trimUp, trimDown, operator.left_bumper);
                if (trimUp != trimDown || resetTrim || toggleCalibration) sequencer.resetSpeedReadiness();

                // ---------------- turret (right stick remains chassis steering)
                if (operator.yWasPressed()) autoAim = !autoAim;
                double manual = manualTurretPower(operator);
                boolean manualActive = Math.abs(manual) > STICK_DEADBAND;
                v2.driveTurret(autoAim && !manualActive, operator.b, manualActive ? manual : 0);

                // ---------------- shooter speed
                v2.refreshVoltage();
                v2.runShooter(robot.shooterOn, calibration.trimRpm, calibration.baseRpmOverride());
                if (operator.rightStickButtonWasPressed()) {
                    if (!calibration.enabled()) {
                        logger.lastMessage = "Click LEFT STICK to enter calibration before logging";
                    } else if (!v2.cameraUsable || !Double.isFinite(v2.hive.distanceM)) {
                        logger.lastMessage = "Point not logged: need a visible HIVE tag with distance";
                    } else {
                        logger.log(v2);
                    }
                }

                // ---------------- gate + feed
                if (operator.xWasPressed()) manualGateOpen = !manualGateOpen;
                sequencer.update(now, false, false, v2.atSpeed(), v2.nearSpeed(), v2.aimed(),
                        robot.gateCalibrated(), manualGateOpen, driverIntake);
                if (robot.gateCalibrated()) {
                    robot.gateOpen = sequencer.gateOpen;
                    robot.gateController.commandSaved(sequencer.gateOpen);
                } else {
                    robot.gateOpen = false;
                }
                robot.intake.setPower(sequencer.intakePower);
                robot.transmission.setPower(sequencer.transmissionPower);

                boolean readyNow = robot.shooterOn && sequencer.ready && robot.gateCalibrated();
                if (readyNow && !wasReady) operator.rumble(200);
                wasReady = readyNow;

                // ---------------- telemetry (throttled so it does not slow the loop)
                if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
                    lastTelemetryMs = now;
                    telemetry.addData("READY", readyNow ? "YES - shoot" : !robot.shooterOn ? "shooter off (A)"
                            : !robot.gateCalibrated() ? "no: gate calibration"
                            : !sequencer.speedStable ? "no: speed stability (+/-10 RPM for 150 ms)" : "no: turret aim");
                    telemetry.addData("Shot", "%s; gate %s; auto-aim %s", sequencer.phase,
                            sequencer.gateOpen ? "OPEN" : "CLOSED", autoAim ? "ON" : "OFF");
                    telemetry.addData("Speed stable", "%s (+/-10 RPM for %.0f ms); manual X/RT feed",
                            sequencer.speedStable ? "YES" : "NO", ShotSequencerV2.READY_STABLE_MS);
                    telemetry.addData("Calibration", calibration.enabled()
                            ? "ON: fixed table RPM (left stick click exits)" : "OFF: following distance table");
                    telemetry.addData("RPM trim controls", "Up/Down +/-10; LB+Up/Down +/-1; BACK clears");
                    telemetry.addData("Shot table log", logger.lastMessage);
                    v2.addTelemetry(telemetry);
                    telemetry.addData("Pollen", robot.ballPresent ? "present (%.1f cm)" : "none (%.1f cm)",
                            robot.ballDistanceCm);
                    telemetry.addData("Flower", "%s (%.4f)", flowerOn ? "ON" : "OFF",
                            flowerOn ? FlowerLever.DEPLOYED_POSITION : FlowerLever.RETRACTED_POSITION);
                    telemetry.update();
                }
            }
        } finally {
            robot.stop();
        }
    }

    static double intakePower(Gamepad pad) {
        return pad.right_trigger > 0.2 ? -Robot.INTAKE_POWER
                : pad.left_trigger > 0.2 ? Robot.INTAKE_POWER : 0;
    }

    static double manualTurretPower(Gamepad pad) {
        int direction = (pad.dpad_right ? 1 : 0) - (pad.dpad_left ? 1 : 0);
        return direction * TURRET_JOG_POWER;
    }
}
