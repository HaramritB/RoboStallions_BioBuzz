
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

@TeleOp(
        name = "Flywheel Tuning + Burst Logger",
        group = "Testing"
)
public class FlywheelTuning extends OpMode {

    // =========================================
    // CONFIGURATION
    // =========================================

    private static final double TICKS_PER_REV = 28.0;

    private static final double HIGH_RPM = 4000.0;
    private static final double LOW_RPM =
            900.0 * 60.0 / TICKS_PER_REV;

    private static final double GATE_CLOSED = 0.0;
    private static final double GATE_OPEN = 0.14;

    private static final double FEED_POWER = 1.0;

    private static final long POST_FEED_WINDOW_NS =
            1_000_000_000L;

    // =========================================
    // HARDWARE
    // =========================================

    private DcMotorEx shooter;
    private DcMotorEx transmission;
    private Servo gate;

    // =========================================
    // PIDF
    // =========================================

    private double targetRpm = HIGH_RPM;

    private double P = 180.0;
    private double F = 13.83;

    private final double[] steps = {
            10.0,
            1.0,
            0.1,
            0.01,
            0.001,
            0.0001
    };

    private int stepIndex = 1;

    // =========================================
    // FEEDER
    // =========================================

    private boolean gateOpen = false;
    private boolean feederOn = false;
    private boolean manualMarker = false;

    private boolean previousGateOpen = false;
    private boolean previousFeederOn = false;
    private boolean previousMarker = false;
    private boolean previousBurstActive = false;

    // =========================================
    // LOGGING
    // =========================================

    private boolean recording = false;

    private BufferedWriter logWriter;

    private String savedPath = "No recording yet";
    private String loggingError = "";

    private long recordingStartNs;
    private long postFeedUntilNs = Long.MIN_VALUE;

    private int sampleCount = 0;
    private int burstStarts = 0;

    private double startingRpm = Double.NaN;
    private double baselineBeforeBurstRpm = Double.NaN;

    private double lowestOverallRpm = Double.NaN;
    private double lowestDuringFeedRpm = Double.NaN;
    private double lowestBurstAndPostRpm = Double.NaN;

    private double endingRpm = Double.NaN;
    private double lastRpm = Double.NaN;

    private double recordingDurationSeconds = 0.0;

    // =========================================
    // INIT
    // =========================================

    @Override
    public void init() {

        shooter = hardwareMap.get(
                DcMotorEx.class,
                "shooter"
        );

        transmission = hardwareMap.get(
                DcMotorEx.class,
                "transmission"
        );

        gate = hardwareMap.get(
                Servo.class,
                "torqueservo1"
        );

        shooter.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        shooter.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        shooter.setPower(0);

        applyPIDF();

        transmission.setDirection(
                DcMotorSimple.Direction.REVERSE
        );

        transmission.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        transmission.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        transmission.setPower(0);

        telemetry.setMsTransmissionInterval(150);

        telemetry.addLine(
                "Flywheel Tuning Ready"
        );

        telemetry.addLine(
                "4000 RPM | P=180 | F=13.83"
        );

        telemetry.addLine(
                "A = Start/Stop Recording"
        );

        telemetry.update();
    }

    @Override
    public void start() {

        gateOpen = false;

        gate.setPosition(GATE_CLOSED);
        transmission.setPower(0);
    }

    // =========================================
    // PIDF
    // =========================================

    private void applyPIDF() {

        shooter.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(
                        P,
                        0,
                        0,
                        F
                )
        );
    }

    // =========================================
    // SENSOR HELPERS
    // =========================================

    private double readRpm() {

        return shooter.getVelocity()
                * 60.0 / TICKS_PER_REV;
    }

    private double readBatteryVoltage() {

        double lowest =
                Double.POSITIVE_INFINITY;

        for (VoltageSensor sensor :
                hardwareMap.voltageSensor) {

            double voltage = sensor.getVoltage();

            if (voltage > 0
                    && Double.isFinite(voltage)) {

                lowest = Math.min(
                        lowest,
                        voltage
                );
            }
        }

        return Double.isInfinite(lowest)
                ? Double.NaN
                : lowest;
    }

    private static double minOrFirst(
            double oldValue,
            double newValue
    ) {

        return Double.isNaN(oldValue)
                ? newValue
                : Math.min(oldValue, newValue);
    }

    private static String displayRpm(double rpm) {

        if (Double.isNaN(rpm)) {
            return "--";
        }

        return String.format(
                Locale.US,
                "%.0f",
                rpm
        );
    }

    // =========================================
    // START RECORDING
    // =========================================

    private void startRecording(
            double currentRpm,
            long nowNs
    ) {

        if (recording) return;

        try {

            File folder = new File(
                    AppUtil.FIRST_FOLDER,
                    "java/src/Datalogs"
            );

            if (!folder.exists()
                    && !folder.mkdirs()) {

                throw new IOException(
                        "Could not create Datalogs folder"
                );
            }

            File file = new File(
                    folder,
                    "shooter_burst_"
                            + System.currentTimeMillis()
                            + ".csv"
            );

            logWriter = new BufferedWriter(
                    new FileWriter(file)
            );

            logWriter.write(
                    "time_s,"
                            + "target_rpm,"
                            + "actual_rpm,"
                            + "error_rpm,"
                            + "target_ticks_s,"
                            + "actual_ticks_s,"
                            + "battery_v,"
                            + "P,"
                            + "F,"
                            + "gate_open,"
                            + "feeder_motor_on,"
                            + "manual_burst_marker,"
                            + "burst_active,"
                            + "event\n"
            );

            savedPath = file.getAbsolutePath();

            loggingError = "";
            recordingStartNs = nowNs;

            recordingDurationSeconds = 0;
            postFeedUntilNs = Long.MIN_VALUE;

            sampleCount = 0;
            burstStarts = 0;

            startingRpm = currentRpm;
            baselineBeforeBurstRpm = currentRpm;

            lowestOverallRpm = currentRpm;
            lowestDuringFeedRpm = Double.NaN;
            lowestBurstAndPostRpm = Double.NaN;

            endingRpm = Double.NaN;
            lastRpm = currentRpm;

            previousBurstActive = false;

            recording = true;

        } catch (IOException error) {

            loggingError = error.toString();

            recording = false;
            closeLog();
        }
    }

    // =========================================
    // STOP RECORDING
    // =========================================

    private void finishRecording(
            double currentRpm,
            long nowNs
    ) {

        if (!recording) return;

        endingRpm = currentRpm;

        recordingDurationSeconds =
                (nowNs - recordingStartNs) / 1e9;

        recording = false;

        closeLog();
    }

    private void closeLog() {

        if (logWriter == null) return;

        try {

            logWriter.flush();
            logWriter.close();

        } catch (IOException error) {

            loggingError = error.toString();

        } finally {

            logWriter = null;
        }
    }

    // =========================================
    // SAVE SAMPLE
    // =========================================

    private void recordSample(
            long nowNs,
            double actualTicks,
            double voltage,
            boolean burstActive,
            String event
    ) {

        if (!recording || logWriter == null) {
            return;
        }

        double actualRpm =
                actualTicks * 60.0 / TICKS_PER_REV;

        double elapsedSeconds =
                (nowNs - recordingStartNs) / 1e9;

        boolean justStartedBurst =
                burstActive && !previousBurstActive;

        boolean justStoppedBurst =
                !burstActive && previousBurstActive;

        if (justStartedBurst) {

            burstStarts++;

            if (burstStarts == 1) {
                baselineBeforeBurstRpm = lastRpm;
            }

            postFeedUntilNs = Long.MIN_VALUE;
        }

        if (justStoppedBurst) {

            postFeedUntilNs =
                    nowNs + POST_FEED_WINDOW_NS;
        }

        if (burstStarts == 0 && !burstActive) {

            baselineBeforeBurstRpm = actualRpm;
        }

        lowestOverallRpm = minOrFirst(
                lowestOverallRpm,
                actualRpm
        );

        if (burstActive) {

            lowestDuringFeedRpm = minOrFirst(
                    lowestDuringFeedRpm,
                    actualRpm
            );
        }

        if (burstActive || nowNs <= postFeedUntilNs) {

            lowestBurstAndPostRpm = minOrFirst(
                    lowestBurstAndPostRpm,
                    actualRpm
            );
        }

        try {

            logWriter.write(
                    String.format(
                            Locale.US,

                            "%.4f,%.2f,%.2f,%.2f,"
                                    + "%.2f,%.2f,%.3f,"
                                    + "%.4f,%.4f,"
                                    + "%d,%d,%d,%d,%s\n",

                            elapsedSeconds,

                            targetRpm,
                            actualRpm,
                            targetRpm - actualRpm,

                            targetRpm * TICKS_PER_REV / 60.0,
                            actualTicks,

                            voltage,

                            P,
                            F,

                            gateOpen ? 1 : 0,
                            feederOn ? 1 : 0,
                            manualMarker ? 1 : 0,
                            burstActive ? 1 : 0,

                            event
                    )
            );

            sampleCount++;

            lastRpm = actualRpm;

            recordingDurationSeconds = elapsedSeconds;

        } catch (IOException error) {

            loggingError = error.toString();

            finishRecording(
                    actualRpm,
                    nowNs
            );
        }
    }

    // =========================================
    // MAIN LOOP
    // =========================================

    @Override
    public void loop() {

        // Step size
        if (gamepad1.bWasPressed()) {

            stepIndex =
                    (stepIndex + 1) % steps.length;
        }

        // Toggle target velocity
        if (gamepad1.yWasPressed()) {

            targetRpm =
                    targetRpm == HIGH_RPM
                            ? LOW_RPM
                            : HIGH_RPM;
        }

        // =====================================
        // LIVE PIDF TUNING
        // =====================================

        double step = steps[stepIndex];

        boolean gainsChanged = false;

        if (gamepad1.dpadUpWasPressed()) {

            P += step;
            gainsChanged = true;
        }

        if (gamepad1.dpadDownWasPressed()) {

            P = Math.max(0, P - step);
            gainsChanged = true;
        }

        if (gamepad1.dpadRightWasPressed()) {

            F += step;
            gainsChanged = true;
        }

        if (gamepad1.dpadLeftWasPressed()) {

            F = Math.max(0, F - step);
            gainsChanged = true;
        }

        if (gainsChanged) {
            applyPIDF();
        }

        // =====================================
        // MOTOR COMMANDS
        // =====================================

        shooter.setVelocity(
                targetRpm * TICKS_PER_REV / 60.0
        );

        // Toggle gate
        if (gamepad1.xWasPressed()) {
            gateOpen = !gateOpen;
        }

        gate.setPosition(
                gateOpen ? GATE_OPEN : GATE_CLOSED
        );

        // Right bumper runs feeder
        feederOn = gamepad1.right_bumper;

        transmission.setPower(
                feederOn ? FEED_POWER : 0
        );

        // Left bumper marks externally fed burst
        manualMarker = gamepad1.left_bumper;

        boolean burstActive =
                feederOn || manualMarker;

        // =====================================
        // MEASURE
        // =====================================

        long nowNs = System.nanoTime();

        double ticks = shooter.getVelocity();

        double currentRpm =
                ticks * 60.0 / TICKS_PER_REV;

        double voltage = readBatteryVoltage();

        // =====================================
        // FIXED RECORDING TOGGLE
        // =====================================

        // Read the A-button press exactly once.
        boolean aPressed = gamepad1.aWasPressed();

        boolean stopRequested =
                aPressed && recording;

        boolean startedRecording = false;

        if (aPressed && !recording) {

            startRecording(
                    currentRpm,
                    nowNs
            );

            startedRecording = recording;
        }

        // =====================================
        // EVENT DETECTION
        // =====================================

        StringBuilder event = new StringBuilder();

        if (startedRecording) {
            event.append("record_start");
        }

        if (gateOpen != previousGateOpen) {

            if (event.length() > 0) {
                event.append('|');
            }

            event.append(
                    gateOpen ? "gate_open" : "gate_close"
            );
        }

        if (feederOn != previousFeederOn) {

            if (event.length() > 0) {
                event.append('|');
            }

            event.append(
                    feederOn ? "feeder_on" : "feeder_off"
            );
        }

        if (manualMarker != previousMarker) {

            if (event.length() > 0) {
                event.append('|');
            }

            event.append(
                    manualMarker ? "marker_on" : "marker_off"
            );
        }

        if (stopRequested) {

            if (event.length() > 0) {
                event.append('|');
            }

            event.append("record_stop");
        }

        // =====================================
        // RECORD DATA
        // =====================================

        if (recording) {

            recordSample(
                    nowNs,
                    ticks,
                    voltage,
                    burstActive,
                    event.toString()
            );
        }

        if (stopRequested) {

            finishRecording(
                    currentRpm,
                    nowNs
            );
        }

        previousGateOpen = gateOpen;
        previousFeederOn = feederOn;
        previousMarker = manualMarker;
        previousBurstActive = burstActive;

        // =====================================
        // TELEMETRY
        // =====================================

        telemetry.addLine(
                "========== FLYWHEEL =========="
        );

        telemetry.addData(
                "Target RPM",
                "%.0f",
                targetRpm
        );

        telemetry.addData(
                "Actual RPM",
                "%.0f",
                currentRpm
        );

        telemetry.addData(
                "RPM Error",
                "%.0f",
                targetRpm - currentRpm
        );

        telemetry.addData(
                "Battery",
                "%.2f V",
                voltage
        );

        telemetry.addData(
                "P / F",
                "%.3f / %.4f",
                P,
                F
        );

        telemetry.addData(
                "Step Size",
                "%.4f",
                step
        );

        telemetry.addLine(
                "========== BURST LOG =========="
        );

        telemetry.addData(
                "Recording",
                recording ? "YES" : "NO"
        );

        telemetry.addData(
                "Gate",
                gateOpen ? "OPEN" : "CLOSED"
        );

        telemetry.addData(
                "Feeder",
                feederOn ? "ON" : "OFF"
        );

        telemetry.addData(
                "Manual Marker",
                manualMarker ? "ON" : "OFF"
        );

        telemetry.addData(
                "Samples / Bursts",
                "%d / %d",
                sampleCount,
                burstStarts
        );

        telemetry.addData(
                "Recorded Duration",
                "%.2f s",
                recordingDurationSeconds
        );

        telemetry.addData(
                "Starting RPM",
                displayRpm(startingRpm)
        );

        telemetry.addData(
                "Before First Burst",
                displayRpm(baselineBeforeBurstRpm)
        );

        telemetry.addData(
                "Min While Feeding",
                displayRpm(lowestDuringFeedRpm)
        );

        telemetry.addData(
                "Min Feed + 1s After",
                displayRpm(lowestBurstAndPostRpm)
        );

        if (!Double.isNaN(lowestBurstAndPostRpm)
                && !Double.isNaN(baselineBeforeBurstRpm)) {

            double drop = Math.max(
                    0,
                    baselineBeforeBurstRpm
                            - lowestBurstAndPostRpm
            );

            double dropPercent =
                    baselineBeforeBurstRpm > 0
                            ? 100.0 * drop
                            / baselineBeforeBurstRpm
                            : 0;

            telemetry.addData(
                    "Burst RPM Drop",
                    "%.0f RPM (%.1f%%)",
                    drop,
                    dropPercent
            );
        }

        telemetry.addData(
                "End RPM",
                displayRpm(endingRpm)
        );

        if (!recording
                && sampleCount > 0
                && burstStarts == 0) {

            telemetry.addLine(
                    "NO BURST MARKED: use RB or LB"
            );
        }

        telemetry.addData(
                "CSV Path",
                savedPath
        );

        if (!loggingError.isEmpty()) {

            telemetry.addData(
                    "Logging Error",
                    loggingError
            );
        }

        telemetry.addLine(
                "========== CONTROLS =========="
        );

        telemetry.addLine(
                "A = Start/Stop Recording"
        );

        telemetry.addLine(
                "X = Toggle Gate"
        );

        telemetry.addLine(
                "RB = Feed Motor"
        );

        telemetry.addLine(
                "LB = Mark External Burst"
        );

        telemetry.addLine(
                "Y = Toggle Speed"
        );

        telemetry.addLine(
                "B = Step Size"
        );

        telemetry.addLine(
                "D-pad Up/Down = P"
        );

        telemetry.addLine(
                "D-pad Left/Right = F"
        );

        telemetry.update();
    }

    // =========================================
    // STOP
    // =========================================

    @Override
    public void stop() {

        if (recording) {

            finishRecording(
                    readRpm(),
                    System.nanoTime()
            );
        }

        shooter.setPower(0);
        transmission.setPower(0);

        gateOpen = false;
        gate.setPosition(GATE_CLOSED);
    }
}
