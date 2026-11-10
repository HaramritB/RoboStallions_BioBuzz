package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

/** Positions the flower lever only; never commands a startup angle. */
@TeleOp(name = "Flower Calibration", group = "Test")
public class FlowerCalibration extends LinearOpMode {
    public static final String PREFERENCES = "biobuzz_flower_calibration_v1";
    private static final double NORMAL_STEP = 0.05;
    private static final double FINE_STEP = 0.005;
    private Servo flower;
    private PwmControl pwm;
    private SharedPreferences preferences;
    private double command = Double.NaN;
    private double markedOn = Double.NaN;
    private double markedOff = Double.NaN;
    private double savedOn = Double.NaN;
    private double savedOff = Double.NaN;
    private String status = "PWM OFF. Free the linkage before the first reference command.";

    @Override
    public void runOpMode() {
        flower = hardwareMap.get(Servo.class, "flower");
        if (!(flower instanceof PwmControl)) {
            telemetry.addLine("Configure flower as a Servo on a REV Hub for per-port PWM control.");
            telemetry.addLine("No position was commanded.");
            telemetry.update();
            waitForStart();
            return;
        }
        pwm = (PwmControl) flower;
        release();
        preferences = hardwareMap.appContext.getSharedPreferences(PREFERENCES, Context.MODE_PRIVATE);
        savedOn = preferences.getFloat("on", Float.NaN);
        savedOff = preferences.getFloat("off", Float.NaN);
        telemetry.setMsTransmissionInterval(100);

        try {
            while (opModeInInit()) {
                show();
                telemetry.update();
                idle();
            }
            while (opModeIsActive()) {
                boolean a = gamepad1.aWasPressed();
                boolean b = gamepad1.b;
                boolean x = gamepad1.xWasPressed();
                boolean y = gamepad1.yWasPressed();
                boolean left = gamepad1.dpadLeftWasPressed();
                boolean right = gamepad1.dpadRightWasPressed();
                boolean on = gamepad1.dpadUpWasPressed();
                boolean off = gamepad1.dpadDownWasPressed();

                if (b) {
                    release();
                    status = "PWM OFF. Command reference again before jogging or recalling.";
                } else if (a && gamepad1.left_bumper && !gamepad1.right_bumper) {
                    move(0.5);
                    status = "Reference 0.5000 commanded. Attach lever with clearance to move.";
                } else if (gamepad1.right_bumper && (left != right)) {
                    if (Double.isFinite(command)) {
                        double step = gamepad1.left_bumper ? FINE_STEP : NORMAL_STEP;
                        move(command + (right ? step : -step));
                        status = "Jogged one step. B releases PWM.";
                    } else {
                        status = "No powered reference. Free linkage, then LB+A.";
                    }
                } else if (gamepad1.right_bumper && (on != off)) {
                    if (Double.isFinite(command) && validPair(savedOn, savedOff)) {
                        move(on ? savedOn : savedOff);
                        status = on ? "Recalled saved ON." : "Recalled saved OFF.";
                    } else {
                        status = "Recall needs a powered reference and a saved ON/OFF pair.";
                    }
                } else if (x != y) {
                    if (Double.isFinite(command)) {
                        if (x) markedOn = command;
                        else markedOff = command;
                        status = x ? "ON marked; mark OFF then A to save both."
                                : "OFF marked; mark ON then A to save both.";
                    } else {
                        status = "No powered reference to mark.";
                    }
                } else if (a && !gamepad1.left_bumper && !gamepad1.right_bumper) {
                    if (!validPair(markedOn, markedOff)) {
                        status = "Mark two distinct positions with X (ON) and Y (OFF) first.";
                    } else if (preferences.edit().putFloat("on", (float) markedOn)
                            .putFloat("off", (float) markedOff).commit()) {
                        savedOn = markedOn;
                        savedOff = markedOff;
                        status = "ON and OFF saved on this Robot Controller.";
                    } else {
                        status = "SAVE FAILED. Record the marked values and retry A.";
                    }
                }
                holdPosition();
                show();
                telemetry.update();
                idle();
            }
        } finally {
            release();
        }
    }

    private void move(double position) {
        command = Math.max(0, Math.min(1, position));
        flower.setPosition(command);
        pwm.setPwmEnable();
    }

    private void release() {
        pwm.setPwmDisable();
        command = Double.NaN;
    }

    private void holdPosition() {
        // Servo PWM normally persists between commands. Restore it if it was disabled
        // while a hold is requested; releasing invalidates command so B stays released.
        if (Double.isFinite(command) && !pwm.isPwmEnabled()) {
            flower.setPosition(command);
            pwm.setPwmEnable();
        }
    }

    static boolean validPair(double on, double off) {
        return Double.isFinite(on) && Double.isFinite(off)
                && on >= 0 && on <= 1 && off >= 0 && off <= 1
                && Math.abs(on - off) >= 0.0001;
    }

    private void show() {
        telemetry.addLine("FLOWER CALIBRATION - GAMEPAD 1");
        telemetry.addData("Status", status);
        telemetry.addLine("LB+A: command 0.5000 reference, with linkage freed first.");
        telemetry.addLine("RB + D-pad left/right: -/+0.05 per press; add LB for 0.005.");
        telemetry.addLine("X: mark ON; Y: mark OFF; A: save BOTH; B: release PWM.");
        telemetry.addLine("RB + D-pad up/down: recall saved ON/OFF after establishing reference.");
        telemetry.addData("Current command", Double.isFinite(command)
                ? String.format(java.util.Locale.US, "%.4f", command) : "unknown / PWM off");
        telemetry.addData("Marked (not yet saved)", "ON=%.4f OFF=%.4f", markedOn, markedOff);
        telemetry.addData("Saved endpoints", validPair(savedOn, savedOff)
                ? String.format(java.util.Locale.US, "ON=%.4f OFF=%.4f", savedOn, savedOff) : "none");
        telemetry.addData("Flower hardware", "port %d; %s", flower.getPortNumber(), flower.getConnectionInfo());
        telemetry.addData("Position hold", Double.isFinite(command) ? "ACTIVE until B or STOP" : "RELEASED");
        telemetry.addData("PWM signal", pwm.isPwmEnabled() ? "ENABLED" : "DISABLED");
        telemetry.addLine("PWM status does not measure servo supply voltage or current.");
        PwmControl.PwmRange range = pwm.getPwmRange();
        telemetry.addData("Configured PWM range", "%.0f to %.0f us", range.usPulseLower, range.usPulseUpper);
        telemetry.addLine("Command values are not measured shaft angles. STOP releases PWM.");
    }
}
