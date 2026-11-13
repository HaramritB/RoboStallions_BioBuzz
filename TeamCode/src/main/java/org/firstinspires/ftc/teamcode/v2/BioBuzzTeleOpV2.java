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
 *  - D-pad up/down trims RPM, RB toggles the flower, LB enables slow drive
 *  - Back zeros the turret during INIT only
 *  - turret keeps pointing at the remembered field target while driving (FieldAimV2)
 *  - aims at the upward HIVE CELL only; RPM from a filtered distance; voltage-compensated F
 *  - rumble on gamepad 1 when ready to shoot; loop time on telemetry
 */
public abstract class BioBuzzTeleOpV2 extends LinearOpMode {

    static final double SLOW_MODE_SCALE = 0.4;
    static final double STICK_DEADBAND = 0.1;
    static final double TURRET_JOG_POWER = 0.15;
    static final double TELEMETRY_PERIOD_MS = 100;

    protected abstract Robot.Alliance teleOpAlliance();

    @Override
    public final void runOpMode() {
        Gamepad operator = gamepad1;
        RobotV2 v2 = new RobotV2();
        v2.initTeleOp(hardwareMap);
        Robot robot = v2.robot;
        Servo flower = hardwareMap.get(Servo.class, "flower");
        FlowerLever.command(flower, false); // Flower OFF as soon as INIT is pressed.
        v2.setAlliance(teleOpAlliance());
        Robot.lastAlliance = teleOpAlliance();
        telemetry.setMsTransmissionInterval(100);

        ShotSequencerV2 sequencer = new ShotSequencerV2();
        boolean autoAim = true;
        boolean manualGateOpen = false;
        boolean flowerOn = false;
        boolean wasReady = false;
        boolean handoffChecked = false;
        String handoff = "waiting for Pinpoint";
        double rpmTrim = 0;
        double lastTelemetryMs = Double.NEGATIVE_INFINITY;

        // ================================================================ INIT
        while (opModeInInit()) {
            v2.beginLoop();
            v2.readPinpoint();
            v2.readVision();
            v2.readBall();
            if (!handoffChecked && v2.poseValid) {
                handoffChecked = true;
                handoff = applyHandoff(v2);
            }
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
                telemetry.addData("Auto handoff", handoff);
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
                if (operator.dpadUpWasPressed()) rpmTrim += Robot.RPM_TRIM_STEP;
                if (operator.dpadDownWasPressed()) rpmTrim -= Robot.RPM_TRIM_STEP;

                // ---------------- turret (right stick remains chassis steering)
                if (operator.yWasPressed()) autoAim = !autoAim;
                double manual = manualTurretPower(operator);
                boolean manualActive = Math.abs(manual) > STICK_DEADBAND;
                v2.driveTurret(autoAim && !manualActive, operator.b, manualActive ? manual : 0);

                // ---------------- shooter speed
                v2.refreshVoltage();
                v2.updateShotDistance();
                v2.runShooter(robot.shooterOn, rpmTrim, Double.NaN);

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

                boolean readyNow = robot.shooterOn && sequencer.ready;
                if (readyNow && !wasReady) operator.rumble(200);
                wasReady = readyNow;

                // ---------------- telemetry (throttled so it does not slow the loop)
                if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
                    lastTelemetryMs = now;
                    telemetry.addData("READY", readyNow ? "YES - shoot" : robot.shooterOn
                            ? "no: " + (sequencer.waitingFor.isEmpty() ? (v2.atSpeed() ? "aim" : "speed")
                            : sequencer.waitingFor) : "shooter off (A)");
                    telemetry.addData("Shot", "%s; gate %s; auto-aim %s", sequencer.phase,
                            sequencer.gateOpen ? "OPEN" : "CLOSED", autoAim ? "ON" : "OFF");
                    v2.addTelemetry(telemetry);
                    telemetry.addData("Pollen", robot.ballPresent ? "present (%.1f cm)" : "none (%.1f cm)",
                            robot.ballDistanceCm);
                    telemetry.addData("Flower", flowerOn ? "ON" : "OFF");
                    telemetry.addData("Auto handoff", handoff);
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

    /** Pre-aim from Auto's last target when the Pinpoint frame is provably the same. */
    private static String applyHandoff(RobotV2 v2) {
        if (!PoseHandoffV2.matches(v2.xIn, v2.yIn, v2.headingRad, v2.robot.alliance)) {
            return "none (no V2 Auto this session, robot moved, or Pinpoint reset)";
        }
        if (PoseHandoffV2.hasTarget()) {
            v2.aim.setGuess(PoseHandoffV2.targetX(), PoseHandoffV2.targetY());
            return PoseHandoffV2.targetConfirmed() ? "OK: pre-aiming at Auto's HIVE target"
                    : "OK: pre-aiming at Auto's guess";
        }
        v2.setHiveGuessForAlliance();
        return "OK: pre-aiming at default HIVE guess";
    }
}
