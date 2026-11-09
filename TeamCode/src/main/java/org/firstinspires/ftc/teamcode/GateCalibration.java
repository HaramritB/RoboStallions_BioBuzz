package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

/** Only the gate is mapped. No old endpoint or guessed startup position is commanded. */
@TeleOp(name = "Gate Calibration", group = "Test")
public class GateCalibration extends LinearOpMode {
    private String status = "PWM OFF. Free linkage/mounting before establishing a reference.";
    private Servo servo;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "torqueservo1");
        Robot.GateController gate = new Robot.GateController(servo,
                hardwareMap.appContext.getSharedPreferences(Robot.GATE_PREFERENCES, Context.MODE_PRIVATE));
        if (!gate.canRelease()) {
            telemetry.addLine("Gate calibration requires per-servo PWM control (REV Hub ServoImplEx).");
            telemetry.addLine("No position was commanded. Check the servo port/configuration.");
            telemetry.update();
            waitForStart();
            return;
        }
        gate.release();
        telemetry.setMsTransmissionInterval(100);
        try {
            while (opModeInInit()) {
                show(gate);
                telemetry.update();
                idle();
            }
            while (opModeIsActive()) {
                boolean a = gamepad1.aWasPressed();
                boolean b = gamepad1.bWasPressed();
                boolean x = gamepad1.xWasPressed();
                boolean y = gamepad1.yWasPressed();
                boolean left = gamepad1.dpadLeftWasPressed();
                boolean right = gamepad1.dpadRightWasPressed();
                boolean clear = gamepad1.backWasPressed();

                // PWM release has priority over every other button in this loop.
                if (b) {
                    gate.release();
                    status = "PWM OFF. Reference invalidated; free linkage before re-referencing.";
                } else if (clear && gamepad1.left_bumper) {
                    status = gate.forgetCalibration() ? "Saved endpoints cleared; PWM OFF."
                            : "Storage clear FAILED; PWM OFF. Do not use saved settings until cleared.";
                } else if (a && gamepad1.left_bumper && !gamepad1.right_bumper) {
                    // This is intentionally a separate, explicit action, never a startup default.
                    gate.establishMidpoint();
                    status = "REFERENCE=0.5000. Attach linkage in a clear, unstrained position.";
                } else {
                    if (gamepad1.right_bumper && (left || right)) {
                        status = gate.jogCalibration(right ? 1 : -1, gamepad1.left_bumper)
                                ? "Jogged one step. Stop short of contact; B releases PWM."
                                : "No reference. Free linkage, then LB+A establishes 0.5000.";
                    }
                    if (x) status = gate.markOpen() ? "OPEN marked (not saved yet)." : "No powered reference to mark.";
                    if (y) status = gate.markClosed() ? "CLOSED marked (not saved yet)." : "No powered reference to mark.";
                    if (a) status = gate.saveMarks()
                            ? "Both endpoints SAVED. Normal TeleOp/Auto will use them."
                            : "Not saved: mark two distinct endpoints, or check storage.";
                }
                show(gate);
                telemetry.update();
                idle();
            }
        } finally {
            gate.release();
        }
    }

    private void show(Robot.GateController gate) {
        telemetry.addLine("GATE CALIBRATION - GAMEPAD 1 ONLY");
        telemetry.addData("Status", status);
        telemetry.addLine("Unknown starting angle: temporarily free linkage/mounting before LB+A.");
        telemetry.addLine("LB+A: command 0.5000 reference ONLY WITH LINKAGE FREED.");
        telemetry.addLine("RB + D-pad left/right: -/+0.01 per press; add LB for 0.0005 fine step.");
        telemetry.addLine("X=mark OPEN; Y=mark CLOSED; A=save BOTH; B=release gate PWM now.");
        telemetry.addLine("LB+Back=clear saved endpoints. STOP also releases gate PWM.");
        telemetry.addData("Current command", Double.isFinite(gate.command)
                ? String.format(java.util.Locale.US, "%.4f", gate.command) : "unknown / PWM off");
        PwmControl pwm = (PwmControl) servo;
        telemetry.addData("Hub PWM output", pwm.isPwmEnabled() ? "ENABLED" : "DISABLED");
        telemetry.addData("Gate hardware", "torqueservo1; port %d; %s",
                servo.getPortNumber(), servo.getConnectionInfo());
        PwmControl.PwmRange range = pwm.getPwmRange();
        telemetry.addData("Configured PWM range", "%.0f to %.0f us", range.usPulseLower, range.usPulseUpper);
        telemetry.addData("Gamepad 1 input", "RB=%s LB=%s Dpad left=%s right=%s",
                gamepad1.right_bumper, gamepad1.left_bumper, gamepad1.dpad_left, gamepad1.dpad_right);
        if (Double.isFinite(gate.command)) {
            telemetry.addData("SDK command echo (not shaft angle)", "%.4f", servo.getPosition());
        }
        telemetry.addData("Pending endpoints", "OPEN=%.4f CLOSED=%.4f", gate.markedOpen, gate.markedClosed);
        telemetry.addData("Active endpoints", "OPEN=%.4f CLOSED=%.4f (%s)",
                gate.openPosition, gate.closedPosition, gate.calibrated()
                        ? gate.usingCodeDefaults ? "code defaults" : "saved calibration" : "disabled");
        telemetry.addLine("Commands are not measured shaft angles. Keep clearance from the frame.");
    }
}
