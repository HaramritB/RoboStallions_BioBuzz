package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

/**
 * Manual calibration only: bypasses Robot.java's software turret limits. No vision, shooter,
 * drive, intake or servo control. Release the right bumper to stop. Do not drive into hard stops.
 *
 * Gamepad 1:
 *   Back (bumper released)       zero encoder while turret points straight forward; clears marks
 *   Hold right bumper + stick X  slow manual movement, maximum 12% power
 *   A (bumper released)          record minimum encoder position
 *   B (bumper released)          record maximum encoder position
 *   Y (bumper released)          clear marks/observed range without changing encoder zero
 *
 * Copy the measured endpoints into Robot.TURRET_MEASURED_MIN_TICKS/MAX_TICKS. Nothing changes them
 * automatically. Degrees assume Robot.TURRET_TICKS_PER_DEG is calibrated for the actual gearing.
 */
@TeleOp(name = "Turret Limits (calibration)", group = "Test")
public class TurretLimitsTest extends LinearOpMode {
    static final double MAX_JOG_POWER = 0.12;
    static final double STICK_DEADBAND = 0.10;
    static final double LIMIT_CLEARANCE_DEG = Robot.TURRET_LIMIT_CLEARANCE_DEG;
    // Include both physical endpoints, not just the narrower software range.
    static final double MAX_OBSERVED_TRAVEL_DEG =
            Math.max(360.0, (Robot.TURRET_MEASURED_MAX_TICKS - Robot.TURRET_MEASURED_MIN_TICKS)
                    / Robot.TURRET_TICKS_PER_DEG + 2 * LIMIT_CLEARANCE_DEG);

    private DcMotorEx turret;
    private Integer minMark, maxMark;
    private int minSeen, maxSeen;
    private boolean zeroSet;
    private String command = "STOPPED";

    @Override
    public void runOpMode() {
        // Match Robot.init(): only this motor is mapped, and all automatic tracking is absent.
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setPower(0);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clearMarks();
        telemetry.setMsTransmissionInterval(100);

        try {
            while (opModeInInit()) {
                handleMarks();
                showTelemetry(false);
                idle();
            }
            while (opModeIsActive()) {
                handleMarks();
                int ticks = turret.getCurrentPosition();
                minSeen = Math.min(minSeen, ticks);
                maxSeen = Math.max(maxSeen, ticks);

                double stick = gamepad1.right_stick_x;
                double power = 0;
                command = "STOPPED - release bumper to record endpoints";
                if (!zeroSet) {
                    command = "ZERO REQUIRED: point straight forward, release bumper, press Back";
                } else if (gamepad1.right_bumper && Math.abs(stick) > STICK_DEADBAND) {
                    // Remove the deadband so speed increases smoothly from zero.
                    double scaled = (Math.abs(stick) - STICK_DEADBAND) / (1 - STICK_DEADBAND);
                    power = Math.copySign(Range.clip(scaled, 0, 1) * MAX_JOG_POWER, stick);
                    command = power > 0 ? "JOG + ENCODER" : "JOG - ENCODER";
                    // Allow movement back inside the observed span, but not farther outward once
                    // the configured calibration travel cap is reached.
                    double maxTicks = MAX_OBSERVED_TRAVEL_DEG * Robot.TURRET_TICKS_PER_DEG;
                    if (maxSeen - (double) minSeen >= maxTicks
                            && ((power > 0 && ticks >= maxSeen) || (power < 0 && ticks <= minSeen))) {
                        power = 0;
                        command = "CALIBRATION TRAVEL CAP - outward motion blocked";
                    }
                }
                turret.setPower(power);
                showTelemetry(true);
                idle();
            }
        } finally {
            turret.setPower(0);
        }
    }

    private void handleMarks() {
        // All these actions require the motion-enabling bumper to be released.
        boolean zero = gamepad1.backWasPressed();
        boolean recordMin = gamepad1.aWasPressed();
        boolean recordMax = gamepad1.bWasPressed();
        boolean clear = gamepad1.yWasPressed();
        if (gamepad1.right_bumper) return;
        turret.setPower(0);
        if (zero) {
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            zeroSet = true;
            clearMarks();
            RobotLog.ii("TurretLimits", "Encoder zero set at physical forward position");
        } else if (clear) {
            clearMarks();
        }
        int ticks = turret.getCurrentPosition();
        if (zeroSet && recordMin) {
            minMark = ticks;
            RobotLog.ii("TurretLimits", "MIN recorded: %d ticks, %.2f deg", ticks, degrees(ticks));
        }
        if (zeroSet && recordMax) {
            maxMark = ticks;
            RobotLog.ii("TurretLimits", "MAX recorded: %d ticks, %.2f deg", ticks, degrees(ticks));
        }
    }

    private void clearMarks() {
        minMark = null;
        maxMark = null;
        minSeen = maxSeen = turret.getCurrentPosition();
    }

    private double degrees(int ticks) {
        return ticks / Robot.TURRET_TICKS_PER_DEG;
    }

    private void showTelemetry(boolean active) {
        int ticks = turret.getCurrentPosition();
        telemetry.addLine("TURRET CALIBRATION - software limits BYPASSED in this OpMode only");
        telemetry.addData("State", active ? command : "INIT: no movement; point forward and press Back");
        telemetry.addData("Forward zero set", zeroSet ? "YES" : "NO - Back required before motion");
        telemetry.addData("Raw encoder", "%d ticks", ticks);
        telemetry.addData("Gears", "%dT motor / %dT turret (%.1f motor turns per turret turn)",
                Robot.TURRET_MOTOR_GEAR_TEETH, Robot.TURRET_GEAR_TEETH,
                Robot.TURRET_MOTOR_REVS_PER_TURRET_REV);
        telemetry.addData("Motor output turns", "%.3f", ticks / Robot.TURRET_MOTOR_TICKS_PER_REV);
        telemetry.addData("Ticks per turret turn", "%.3f", Robot.TURRET_TICKS_PER_TURRET_REV);
        telemetry.addData("Turret angle", "%.2f deg", degrees(ticks));
        telemetry.addData("Power", "%+.3f (max %.2f)", turret.getPower(), MAX_JOG_POWER);
        telemetry.addData("Ticks per degree", "%.5f - verify gear ratio", Robot.TURRET_TICKS_PER_DEG);
        telemetry.addData("Recorded MIN (A)", minMark == null ? "not recorded"
                : String.format("%d ticks / %.2f deg", minMark, degrees(minMark)));
        telemetry.addData("Recorded MAX (B)", maxMark == null ? "not recorded"
                : String.format("%d ticks / %.2f deg", maxMark, degrees(maxMark)));
        telemetry.addData("Observed travel", "%.2f to %.2f deg (span %.2f)",
                degrees(minSeen), degrees(maxSeen), degrees(maxSeen - minSeen));
        telemetry.addData("Current Robot limits", "%.1f to %.1f deg (not applied here)",
                Robot.TURRET_MIN_DEG, Robot.TURRET_MAX_DEG);
        telemetry.addData("Current tick limits", "%d to %d (not applied here)",
                Robot.TURRET_MIN_TICKS, Robot.TURRET_MAX_TICKS);
        telemetry.addData("Calibration travel cap", "%.2f calculated deg", MAX_OBSERVED_TRAVEL_DEG);
        if (minMark != null && maxMark != null) {
            double min = degrees(minMark), max = degrees(maxMark);
            int clearanceTicks = (int) Math.ceil(LIMIT_CLEARANCE_DEG * Robot.TURRET_TICKS_PER_DEG);
            int suggestedMinTicks = minMark + clearanceTicks;
            int suggestedMaxTicks = maxMark - clearanceTicks;
            double suggestedMin = degrees(suggestedMinTicks);
            double suggestedMax = degrees(suggestedMaxTicks);
            if (min < 0 && max > 0 && max - min <= MAX_OBSERVED_TRAVEL_DEG
                    && suggestedMin <= 0 && suggestedMax >= 0
                    && suggestedMax - suggestedMin > 2 * (Robot.TURRET_RETURN_MARGIN_DEG
                            + Robot.TURRET_RETURN_TOLERANCE_DEG)) {
                telemetry.addData("Copy TURRET_MEASURED_MIN_TICKS", minMark);
                telemetry.addData("Copy TURRET_MEASURED_MAX_TICKS", maxMark);
                telemetry.addData("Suggested software limits", "%d / %d ticks",
                        suggestedMinTicks, suggestedMaxTicks);
                telemetry.addData("Clearance", "%.1f deg inside each recorded endpoint", LIMIT_CLEARANCE_DEG);
            } else {
                telemetry.addLine("Check marks: MIN negative, MAX positive; span within displayed calibration cap");
            }
        }
        telemetry.addLine("Hold right bumper + right stick X to jog; release bumper to stop.");
        telemetry.addLine("Back=forward zero; A=MIN; B=MAX; Y=clear marks (bumper released).");
        telemetry.addLine("Stop BEFORE hard stops/cable tension. Record values before leaving this OpMode.");
        telemetry.update();
    }
}
