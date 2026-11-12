
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

/** Shared competition controls for the separately registered Blue and Red TeleOps. */
public abstract class BioBuzzTeleOp extends LinearOpMode {

    static final double SLOW_MODE_SCALE = 0.4;
    static final double STICK_DEADBAND = 0.1;
    static final double FLOWER_ON = FlowerLever.DEPLOYED_POSITION;
    static final double FLOWER_OFF = FlowerLever.RETRACTED_POSITION;

    protected abstract Robot.Alliance teleOpAlliance();

    @Override
    public final void runOpMode() {

        Robot robot = new Robot();
        robot.init(hardwareMap, false);
        Servo flower = hardwareMap.get(Servo.class, "flower");
        boolean flowerOn = false;
        commandFlower(flower, false); // Flower OFF as soon as INIT is pressed.
        // Override any alliance inherited from Auto or a previously run OpMode.
        robot.alliance = teleOpAlliance();
        Robot.lastAlliance = robot.alliance;

        boolean autoAim = true;

        double minBurstRpm = Double.NaN;

        // =========================================
        // INIT
        // =========================================

        while (opModeInInit()) {

            if (gamepad2.backWasPressed()) {
                robot.zeroTurret();
            }

            robot.updateSensors();
            // Keep the INIT OFF command powered if the hub dropped the servo PWM.
            if (flower instanceof PwmControl && !((PwmControl) flower).isPwmEnabled()) {
                commandFlower(flower, false);
            }

            telemetry.addLine("===== BIOBUZZ INIT =====");

            telemetry.addData("Alliance", "%s (fixed by selected TeleOp)", robot.alliance);

            telemetry.addData(
                    "HIVE Tag",
                    robot.targetVisible
                            ? "ID " + robot.targetId
                            : "NOT VISIBLE"
            );

            telemetry.addData(
                    "Turret Angle",
                    "%.2f deg",
                    robot.turretDegrees()
            );

            telemetry.addData(
                    "Shooter Target",
                    "%.0f RPM",
                    robot.distanceBasedShooterRpm()
            );
            addShooterDistanceTelemetry(robot);

            telemetry.addData(
                    "PIDF",
                    "P=%.0f I=%.0f D=%.0f F=%.2f",
                    Robot.SHOOTER_P,
                    Robot.SHOOTER_I,
                    Robot.SHOOTER_D,
                    Robot.SHOOTER_F
            );

            robot.addVisionTelemetry(telemetry);
            robot.addGateTelemetry(telemetry);
            addPollenLedTelemetry(robot);
            telemetry.addData("Flower", "OFF from INIT (%.2f); gamepad 2 RB toggles ON/OFF", FLOWER_OFF);

            telemetry.update();
        }

        // =========================================
        // MAIN TELEOP
        // =========================================

        try {

            if (opModeIsActive()) commandFlower(flower, false);

            while (opModeIsActive()) {

                robot.updateSensors();

                // Controller 2 right bumper toggles the flower lever once per press.
                if (gamepad2.rightBumperWasPressed()) {
                    flowerOn = !flowerOn;
                    commandFlower(flower, flowerOn);
                }
                // Servo PWM normally holds between commands; restore it if necessary.
                if (flower instanceof PwmControl && !((PwmControl) flower).isPwmEnabled()) {
                    commandFlower(flower, flowerOn);
                }

                // =====================================
                // DRIVER 1: DRIVING
                // =====================================

                double scale = gamepad1.left_bumper
                        ? SLOW_MODE_SCALE
                        : 1.0;

                robot.drive(
                        -gamepad1.left_stick_y * scale,
                        gamepad1.left_stick_x * scale,
                        gamepad1.right_stick_x * scale
                );

                // =====================================
                // DRIVER 1: INTAKE
                // =====================================

                double intakePower = 0;

                if (gamepad1.right_trigger > 0.2) {

                    intakePower = -Robot.INTAKE_POWER;

                } else if (gamepad1.left_trigger > 0.2) {

                    intakePower = Robot.INTAKE_POWER;
                }

                // =====================================
                // DRIVER 2: SHOOTER
                // =====================================

                boolean wantShoot =
                        gamepad2.right_trigger > 0.5;

                // A toggles shooter ON/OFF
                if (gamepad2.aWasPressed()) {
                    robot.shooterOn = !robot.shooterOn;
                }

                // Right trigger switches shooter ON
                if (wantShoot) {
                    robot.shooterOn = true;
                }

                // RPM trim
                if (gamepad2.dpadUpWasPressed()) {
                    robot.rpmTrim += Robot.RPM_TRIM_STEP;
                }

                if (gamepad2.dpadDownWasPressed()) {
                    robot.rpmTrim -= Robot.RPM_TRIM_STEP;
                }

                // =====================================
                // DRIVER 2: TURRET
                // =====================================

                if (gamepad2.yWasPressed()) {
                    autoAim = !autoAim;
                }

                // Only re-zero while pointing forward
                if (gamepad2.backWasPressed()) {
                    robot.zeroTurret();
                }

                double manualTurret =
                        gamepad2.right_stick_x;

                if (gamepad2.b) {

                    robot.turretToAngle(0);

                } else if (
                        Math.abs(manualTurret) > STICK_DEADBAND
                ) {

                    robot.aimTurret(false, manualTurret);

                } else {

                    robot.aimTurret(autoAim, 0);
                }

                // =====================================
                // UPDATE SHOOTER
                // =====================================

                robot.updateShooter(robot.distanceBasedShooterRpm());

                // =====================================
                // GATE / FEEDER
                // =====================================

                boolean toggleGate =
                        gamepad2.xWasPressed();

                boolean gateWasOpen = robot.gateOpen;

                robot.updateManualFeeder(
                        toggleGate,
                        intakePower
                );
                robot.addGateTelemetry(telemetry);

                // =====================================
                // BURST RPM TRACKING
                // =====================================

                double currentRpm =
                        robot.shooterRpm();

                if (!gateWasOpen && robot.gateOpen) {
                    minBurstRpm = currentRpm;
                }

                if (robot.gateOpen) {

                    if (Double.isNaN(minBurstRpm)) {

                        minBurstRpm = currentRpm;

                    } else {

                        minBurstRpm = Math.min(
                                minBurstRpm,
                                currentRpm
                        );
                    }
                }

                // =====================================
                // TURRET TELEMETRY
                // =====================================

                telemetry.addLine(
                        "========== TURRET =========="
                );

                telemetry.addData(
                        "Auto Aim",
                        autoAim ? "ENABLED" : "DISABLED"
                );

                telemetry.addData(
                        "Tag Visible",
                        robot.targetVisible
                );

                telemetry.addData(
                        "Tag Fresh",
                        robot.targetFresh
                );

                telemetry.addData(
                        "Tag ID",
                        robot.targetId
                );

                if (robot.targetFresh) {
                    telemetry.addData("Target TX", "%.2f deg", robot.targetTx);
                } else {
                    telemetry.addData("Target TX", "unavailable (no matching tag)");
                }

                telemetry.addData(
                        "Aim State",
                        robot.turretAimState()
                );

                telemetry.addData(
                        "Turret Angle",
                        "%.2f deg",
                        robot.turretDegrees()
                );

                telemetry.addData(
                        "Turret Power",
                        "%.3f",
                        robot.turret.getPower()
                );

                telemetry.addData(
                        "Manual Stick",
                        "%.3f",
                        manualTurret
                );

                // Show latest Limelight debug information
                robot.addVisionTelemetry(telemetry);

                // =====================================
                // SHOOTER TELEMETRY
                // =====================================

                double targetRpm =
                        robot.targetRpm + robot.rpmTrim;

                double rpmError =
                        targetRpm - currentRpm;

                telemetry.addLine(
                        "========== SHOOTER =========="
                );

                telemetry.addData(
                        "Shooter",
                        robot.shooterOn ? "ON" : "OFF"
                );

                telemetry.addData(
                        "Target RPM",
                        "%.0f",
                        targetRpm
                );

                telemetry.addData(
                        "Current RPM",
                        "%.0f",
                        currentRpm
                );

                telemetry.addData(
                        "RPM Error",
                        "%.0f",
                        rpmError
                );

                telemetry.addData(
                        "RPM Trim",
                        "%+.0f",
                        robot.rpmTrim
                );
                addShooterDistanceTelemetry(robot);

                telemetry.addData(
                        "Shooter Ready",
                        robot.shooterAtSpeed()
                                ? "AT SPEED"
                                : "NOT AT SPEED"
                );

                telemetry.addData(
                        "PIDF",
                        "P=%.0f I=%.0f D=%.0f F=%.2f",
                        Robot.SHOOTER_P,
                        Robot.SHOOTER_I,
                        Robot.SHOOTER_D,
                        Robot.SHOOTER_F
                );

                // =====================================
                // BURST TELEMETRY
                // =====================================

                telemetry.addLine(
                        "========== BURST RPM =========="
                );

                telemetry.addData(
                        "Gate",
                        robot.gateOpen ? "OPEN" : "CLOSED"
                );

                telemetry.addData(
                        "Minimum Burst RPM",
                        Double.isNaN(minBurstRpm)
                                ? "NO BURST YET"
                                : String.format(
                                "%.0f RPM",
                                minBurstRpm
                        )
                );

                telemetry.addData(
                        "Maximum Observed RPM Drop",
                        Double.isNaN(minBurstRpm)
                                ? "NO DATA"
                                : String.format(
                                "%.0f RPM",
                                Math.max(
                                        0,
                                        targetRpm - minBurstRpm
                                )
                        )
                );

                telemetry.addData(
                        "Current Velocity",
                        "%.1f ticks/sec",
                        robot.shooter.getVelocity()
                );

                // =====================================
                // OTHER TELEMETRY
                // =====================================

                telemetry.addLine(
                        "========== OTHER =========="
                );

                telemetry.addData(
                        "Alliance",
                        robot.alliance
                );

                telemetry.addData(
                        "Ball",
                        "%s hue %.0f distance %.1f cm",
                        robot.ballColor,
                        robot.ballHue,
                        robot.ballDistanceCm
                );
                addPollenLedTelemetry(robot);
                telemetry.addData("Flower", "%s (%.2f); gamepad 2 RB toggles",
                        flowerOn ? "ON" : "OFF", flowerOn ? FLOWER_ON : FLOWER_OFF);

                telemetry.addData(
                        "Gate",
                        robot.gateOpen
                                ? "OPEN"
                                : "CLOSED"
                );

                telemetry.addData(
                        "Shots",
                        robot.shotsFired
                );

                telemetry.update();
            }

        } finally {

            robot.stop();
        }
    }

    private void addPollenLedTelemetry(Robot robot) {
        telemetry.addData("Pollen LED", robot.led == null ? "LED missing: configure led"
                : robot.colorSensor == null ? "RED: sensor missing (colorsensor)"
                : robot.ballPresent ? "GREEN: pollen detected" : "RED: no pollen detected");
    }

    private void commandFlower(Servo flower, boolean on) {
        FlowerLever.command(flower, on);
    }

    private void addShooterDistanceTelemetry(Robot robot) {
        boolean hasDistance = robot.targetFresh && Double.isFinite(robot.targetDistanceMeters)
                && robot.targetDistanceMeters > 0;
        telemetry.addData("Shooter RPM source", hasDistance
                ? String.format(java.util.Locale.US, "Tag %d: %.2f m", robot.targetDistanceTagId,
                        robot.targetDistanceMeters)
                : "No fresh tag distance: default 3150 RPM");
    }
}
