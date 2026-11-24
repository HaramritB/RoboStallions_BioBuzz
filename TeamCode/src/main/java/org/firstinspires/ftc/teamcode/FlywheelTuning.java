
package org.firstinspires.ftc.teamcode;

import android.content.Context;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Locale;

/**
 * Shooter PIDF tuner and burst logger. See TeamCode/SHOOTER_TUNING.md for the test procedure.
 *
 * Logs go to /sdcard/FIRST/java/src/Datalogs/ on the Control Hub:
 *  - shooter_burst_<time>.csv  every loop sample while recording (A)
 *  - shooter_volleys.csv       one summary row per volley, written even when not recording
 */
@TeleOp(
        name = "Flywheel Tuning + Burst Logger",
        group = "Testing"
)
public class FlywheelTuning extends OpMode {

    // =========================================
    // CONFIGURATION
    // =========================================

    private static final double TICKS_PER_REV = Robot.SHOOTER_TICKS_PER_REV;
    /** Every distinct table speed, plus 3300 for a little headroom above the farthest shot. */
    static final double[] TEST_RPMS = testRpms();

    private static double[] testRpms() {
        double[] calibrated = Robot.shooterCalibrationRpms();
        double[] speeds = Arrays.copyOf(calibrated, calibrated.length + 1);
        speeds[calibrated.length] = 3300.0;
        return speeds;
    }
    /** Right stick click cycles these: 90 (previous V2 trial), 150, 180 (V2 TeleOp now, old baseline). */
    static final double[] P_PRESETS = {90.0, 150.0, 180.0};
    private static final long VOLTAGE_REFRESH_NS = 500_000_000L;
    private static final long TELEMETRY_PERIOD_NS = 150_000_000L;
    private static final long GATE_OPEN_WAIT_NS = 250_000_000L;
    private static final long GATE_RELIEF_NS = 100_000_000L;
    private static final double GATE_RELIEF_POWER = 0.30;
    static final String DATALOG_SUBFOLDER = "java/src/Datalogs";
    static final String VOLLEY_FILE = "shooter_volleys.csv";
    private static final int SUMMARY_LINES = 4;

    // =========================================
    // HARDWARE
    // =========================================

    private DcMotorEx shooter;
    private DcMotorEx intake;
    private DcMotorEx transmission;
    private Servo gate;
    private Robot.GateController gateController;
    private List<LynxModule> hubs = new ArrayList<>();

    // =========================================
    // PIDF
    // =========================================

    private int speedIndex;
    private double targetRpm = TEST_RPMS[0];
    private boolean shooterOn;
    private double P = 0.0;
    /** Reference F, normalized to Robot.SHOOTER_F_REFERENCE_VOLTS, not the applied hub F. */
    private double F = Robot.V2_TELEOP_SHOOTER_F;
    private double appliedF = Double.NaN;
    private double batteryVolts = Double.NaN;
    private long lastVoltageUpdateNs = Long.MIN_VALUE;
    private int presetIndex = -1;

    private final double[] steps = {
            10.0,
            1.0,
            0.1,
            0.01,
            0.001,
            0.0001
    };

    private int stepIndex = 2; // 0.1 for feedforward tuning first.

    // =========================================
    // FEEDER
    // =========================================

    private boolean gateOpen = false;
    private boolean feederOn = false;
    private boolean manualMarker = false;
    private boolean feederRequested;
    private double intakePower;
    private double transmissionPower;
    private long gateOpenedNs;

    private boolean previousGateOpen = false;
    private boolean previousFeederOn = false;
    private boolean previousMarker = false;

    // =========================================
    // ANALYSIS AND LOGGING
    // =========================================

    private final BurstAnalyzer analyzer = new BurstAnalyzer();
    private final String[] summaries = new String[SUMMARY_LINES];
    private int summaryCount;

    private boolean recording = false;
    private BufferedWriter logWriter;
    private String savedPath = "No recording yet";
    private String volleyFilePath = "written after the first volley";
    private String loggingError = "";
    private long recordingStartNs;
    private int sampleCount = 0;
    private double recordingDurationSeconds = 0.0;

    // Loop timing and velocity resolution.
    private long lastLoopNs = Long.MIN_VALUE;
    private double loopMs = Double.NaN;
    private double loopMsAverage = Double.NaN;
    private double loopMsMax;
    private double loopWindowMax;
    private long loopWindowStartNs;
    private long velocityStepTicks;
    private int velocityReadings;
    private boolean velocityNonInteger;
    private long lastTelemetryNs = Long.MIN_VALUE;

    // =========================================
    // INIT
    // =========================================

    @Override
    public void init() {

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            // One bulk read per loop instead of one transaction per encoder/velocity call.
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            hub.clearBulkCache();
        }

        shooter = hardwareMap.get(
                DcMotorEx.class,
                "shooter"
        );

        transmission = hardwareMap.get(
                DcMotorEx.class,
                "transmission"
        );

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE); // Matches Robot (reversed 2026-11-23).
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0);

        gate = hardwareMap.get(
                Servo.class,
                "torqueservo1"
        );
        gateController = new Robot.GateController(gate,
                hardwareMap.appContext.getSharedPreferences(Robot.GATE_PREFERENCES, Context.MODE_PRIVATE));
        if (!gateController.calibrated()) gateController.release();

        shooter.setDirection(
                DcMotorSimple.Direction.REVERSE // Matches Robot (reversed 2026-11-23).
        );

        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        shooter.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        shooter.setPower(0);

        long now = nanoTime();
        batteryVolts = readBatteryVoltage();
        lastVoltageUpdateNs = now;
        applyPIDF();

        transmission.setDirection(
                DcMotorSimple.Direction.FORWARD // Matches Robot (reversed 2026-11-23).
        );

        transmission.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        transmission.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );

        transmission.setPower(0);

        // Telemetry is rebuilt every 150 ms instead of every loop, so logging runs faster.
        telemetry.setAutoClear(false);
        telemetry.setMsTransmissionInterval(150);
        telemetry.addLine("Flywheel Tuning Ready");
        telemetry.addData("Test RPMs", Arrays.toString(TEST_RPMS));
        telemetry.addLine("P starts at 0 | I=D=0");
        telemetry.addData("Start reference F", "%.2f (V2 TeleOp) at %.1f V", F, Robot.SHOOTER_F_REFERENCE_VOLTS);
        telemetry.addData("Battery", "%.2f V", batteryVolts);
        telemetry.addLine("START, then click LEFT STICK to run the flywheel; A records only");
        telemetry.update();
    }

    @Override
    public void start() {

        gateOpen = false;
        shooterOn = false;
        shooter.setPower(0);
        gateController.commandSaved(false);
        transmission.setPower(0);
        intake.setPower(0);
    }

    // =========================================
    // PIDF
    // =========================================

    /** Re-reads battery voltage every 500 ms (it is a separate hub transaction, not bulk data). */
    private void refreshVoltage(long nowNs, boolean gainsChanged) {
        if (lastVoltageUpdateNs == Long.MIN_VALUE || nowNs - lastVoltageUpdateNs >= VOLTAGE_REFRESH_NS) {
            lastVoltageUpdateNs = nowNs;
            double volts = readBatteryVoltage();
            if (Double.isFinite(volts)) batteryVolts = volts;
            double nextF = Robot.compensatedShooterF(F, batteryVolts);
            if (gainsChanged || !Double.isFinite(appliedF) || Math.abs(nextF - appliedF) > 0.01 * Math.abs(appliedF)) {
                applyPIDF();
                return;
            }
        }
        if (gainsChanged) applyPIDF();
    }

    private void applyPIDF() {
        appliedF = Robot.compensatedShooterF(F, batteryVolts);
        shooter.setVelocityPIDFCoefficients(P, 0, 0, appliedF);
    }

    // =========================================
    // SENSOR HELPERS
    // =========================================

    private double readBatteryVoltage() {

        if (hardwareMap == null) return Double.NaN;

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

    /** Tracks the smallest step the hub's velocity reading can show (often 20 ticks/s on REV hubs). */
    private void trackVelocityResolution(double ticks) {
        if (!shooterOn || Math.abs(ticks) < 100) return;
        double rounded = Math.rint(ticks);
        if (Math.abs(ticks - rounded) > 1e-6) {
            velocityNonInteger = true;
            return;
        }
        velocityStepTicks = gcd(velocityStepTicks, Math.abs((long) rounded));
        velocityReadings++;
    }

    static long gcd(long a, long b) {
        while (b != 0) {
            long t = a % b;
            a = b;
            b = t;
        }
        return Math.abs(a);
    }

    private void trackLoopTime(long nowNs) {
        if (lastLoopNs != Long.MIN_VALUE) {
            loopMs = (nowNs - lastLoopNs) / 1e6;
            loopMsAverage = Double.isFinite(loopMsAverage) ? 0.9 * loopMsAverage + 0.1 * loopMs : loopMs;
            loopWindowMax = Math.max(loopWindowMax, loopMs);
            if (nowNs - loopWindowStartNs >= 2_000_000_000L) {
                loopMsMax = loopWindowMax;
                loopWindowMax = 0;
                loopWindowStartNs = nowNs;
            }
        }
        lastLoopNs = nowNs;
    }

    private static String displayRpm(double rpm) {
        return Double.isFinite(rpm) ? String.format(Locale.US, "%.0f", rpm) : "--";
    }

    // =========================================
    // FILES
    // =========================================

    /** /sdcard/FIRST/java/src/Datalogs on the Control Hub. Pull it with adb or Device Explorer. */
    protected File datalogFolder() {
        return new File(AppUtil.FIRST_FOLDER, DATALOG_SUBFOLDER);
    }

    private File ensureFolder() throws IOException {
        File folder = datalogFolder();
        if (!folder.exists() && !folder.mkdirs()) {
            throw new IOException("Could not create " + folder.getAbsolutePath());
        }
        return folder;
    }

    private void startRecording(long nowNs) {

        if (recording) return;

        try {
            File file = new File(ensureFolder(), "shooter_burst_" + System.currentTimeMillis() + ".csv");
            logWriter = new BufferedWriter(new FileWriter(file));
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
                            + "reference_v,"
                            + "applied_f,"
                            + "shooter_on,"
                            + "intake_power,"
                            + "transmission_power,"
                            + "loop_ms,"
                            + "volley,"
                            + "dip_count,"
                            + "velocity_step_ticks,"
                            + "event\n"
            );
            savedPath = file.getAbsolutePath();
            loggingError = "";
            recordingStartNs = nowNs;
            recordingDurationSeconds = 0;
            sampleCount = 0;
            recording = true;
        } catch (IOException | RuntimeException error) {
            loggingError = error.toString();
            recording = false;
            closeLog();
        }
    }

    private void finishRecording(long nowNs) {

        if (!recording) return;
        recordingDurationSeconds = (nowNs - recordingStartNs) / 1e9;
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

    private void recordSample(long nowNs, double actualTicks, boolean burstActive, String event) {

        if (!recording || logWriter == null) {
            return;
        }

        double actualRpm = actualTicks * 60.0 / TICKS_PER_REV;
        double elapsedSeconds = (nowNs - recordingStartNs) / 1e9;

        try {
            logWriter.write(
                    String.format(
                            Locale.US,
                            "%.4f,%.2f,%.2f,%.2f,"
                                    + "%.2f,%.2f,%.3f,"
                                    + "%.4f,%.4f,"
                                    + "%d,%d,%d,%d,%.3f,%.4f,%d,%.3f,%.3f,"
                                    + "%.2f,%d,%d,%d,%s\n",
                            elapsedSeconds,
                            targetRpm,
                            actualRpm,
                            targetRpm - actualRpm,
                            targetRpm * TICKS_PER_REV / 60.0,
                            actualTicks,
                            batteryVolts,
                            P,
                            F,
                            gateOpen ? 1 : 0,
                            feederOn ? 1 : 0,
                            manualMarker ? 1 : 0,
                            burstActive ? 1 : 0,
                            Robot.SHOOTER_F_REFERENCE_VOLTS,
                            appliedF,
                            shooterOn ? 1 : 0,
                            intakePower,
                            transmissionPower,
                            Double.isFinite(loopMs) ? loopMs : 0,
                            analyzer.volley,
                            analyzer.inVolley ? analyzer.dips : 0,
                            velocityStepTicks,
                            event
                    )
            );
            sampleCount++;
            recordingDurationSeconds = elapsedSeconds;
        } catch (IOException error) {
            loggingError = error.toString();
            finishRecording(nowNs);
        }
    }

    /** One row per volley in shooter_volleys.csv, so P trials can be compared without the raw logs. */
    private void writeVolleySummary() {
        try {
            File file = new File(ensureFolder(), VOLLEY_FILE);
            boolean header = !file.exists();
            try (FileWriter out = new FileWriter(file, true)) {
                if (header) {
                    out.write("time_ms,volley,P,reference_f,applied_f,battery_v,target_rpm,baseline_rpm,"
                            + "launch_1,launch_2,launch_3,launch_4,dips,first_minus_last_launch,"
                            + "min_rpm,drop_rpm,recovery_ms,sample_file\n");
                }
                StringBuilder launches = new StringBuilder();
                for (int i = 0; i < 4; i++) {
                    launches.append(i < Math.min(analyzer.dips, BurstAnalyzer.MAX_DIPS)
                            ? String.format(Locale.US, "%.0f", analyzer.launchRpm[i]) : "").append(',');
                }
                out.write(String.format(Locale.US, "%d,%d,%.1f,%.4f,%.4f,%.2f,%.0f,%.0f,%s%d,%s,%.0f,%.0f,%s,%s%n",
                        System.currentTimeMillis(), analyzer.volley, P, F, appliedF, batteryVolts,
                        analyzer.targetRpm, analyzer.baselineRpm, launches, analyzer.dips,
                        Double.isFinite(analyzer.launchSpread()) ? String.format(Locale.US, "%.0f", analyzer.launchSpread()) : "",
                        analyzer.minRpm, analyzer.drop(),
                        Double.isFinite(analyzer.recoveryMs) ? String.format(Locale.US, "%.0f", analyzer.recoveryMs) : "",
                        recording ? new File(savedPath).getName() : ""));
            }
            volleyFilePath = file.getAbsolutePath();
        } catch (IOException | RuntimeException error) {
            loggingError = "Volley summary: " + error;
        }
    }

    private void rememberSummary(String line) {
        for (int i = SUMMARY_LINES - 1; i > 0; i--) summaries[i] = summaries[i - 1];
        summaries[0] = line;
        summaryCount = Math.min(summaryCount + 1, SUMMARY_LINES);
    }

    // =========================================
    // MAIN LOOP
    // =========================================

    @Override
    public void loop() {

        for (LynxModule hub : hubs) hub.clearBulkCache();
        long nowNs = nanoTime();
        trackLoopTime(nowNs);

        // Measure first, once per loop, from fresh bulk data.
        double ticks = shooter.getVelocity();
        double currentRpm = ticks * 60.0 / TICKS_PER_REV;
        trackVelocityResolution(ticks);

        // Step size
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % steps.length;
        }

        // Cycle the test speeds.
        if (gamepad1.yWasPressed()) {
            speedIndex = (speedIndex + 1) % TEST_RPMS.length;
            targetRpm = TEST_RPMS[speedIndex];
        }

        // =====================================
        // LIVE PIDF TUNING
        // =====================================

        double step = steps[stepIndex];
        boolean gainsChanged = false;

        if (gamepad1.backWasPressed()) {
            P = 0;
            presetIndex = -1;
            gainsChanged = true;
        }

        if (gamepad1.rightStickButtonWasPressed()) {
            presetIndex = (presetIndex + 1) % P_PRESETS.length;
            P = P_PRESETS[presetIndex];
            gainsChanged = true;
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += step;
            presetIndex = -1;
            gainsChanged = true;
        }

        if (gamepad1.dpadDownWasPressed()) {
            P = Math.max(0, P - step);
            presetIndex = -1;
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

        refreshVoltage(nowNs, gainsChanged);

        // =====================================
        // MOTOR COMMANDS
        // =====================================

        if (gamepad1.leftStickButtonWasPressed()) shooterOn = !shooterOn;
        if (shooterOn) shooter.setVelocity(targetRpm * TICKS_PER_REV / 60.0);
        else shooter.setPower(0);

        if (gamepad1.xWasPressed()) {
            gateOpen = gateController.calibrated() && !gateOpen;
            if (gateOpen) gateOpenedNs = nowNs;
        }
        gateController.commandSaved(gateOpen);

        // RT/LT load or eject. RB feeds a volley using the same inward powers as V2.
        double loadPower = gamepad1.right_trigger > 0.2 ? -Robot.INTAKE_POWER
                : gamepad1.left_trigger > 0.2 ? Robot.INTAKE_POWER : 0;
        feederRequested = gamepad1.right_bumper && gateController.calibrated() && shooterOn && gateOpen;
        long sinceOpenNs = nowNs - gateOpenedNs;
        feederOn = feederRequested && sinceOpenNs >= GATE_OPEN_WAIT_NS;
        intakePower = feederOn ? -Robot.INTAKE_POWER : loadPower;
        transmissionPower = transmissionPower(gateController.calibrated(), gateOpen,
                sinceOpenNs, feederOn, loadPower);
        intake.setPower(intakePower);
        transmission.setPower(transmissionPower);

        // Left bumper marks an externally fed burst.
        manualMarker = gamepad1.left_bumper;
        boolean burstActive = feederOn || manualMarker;

        // =====================================
        // VOLLEY ANALYSIS (always on)
        // =====================================

        String analysisEvent = analyzer.sample(nowNs / 1e6, currentRpm, targetRpm, burstActive);
        if (analysisEvent.contains("volley_end")) {
            rememberSummary(analyzer.summary(P, F));
            writeVolleySummary();
        }

        // =====================================
        // RECORDING (A toggles; independent of the flywheel)
        // =====================================

        boolean aPressed = gamepad1.aWasPressed();
        boolean stopRequested = aPressed && recording;
        boolean startedRecording = false;
        if (aPressed && !recording) {
            startRecording(nowNs);
            startedRecording = recording;
        }

        StringBuilder event = new StringBuilder();
        if (startedRecording) appendEvent(event, "record_start");
        if (gateOpen != previousGateOpen) appendEvent(event, gateOpen ? "gate_open" : "gate_close");
        if (feederOn != previousFeederOn) appendEvent(event, feederOn ? "feeder_on" : "feeder_off");
        if (manualMarker != previousMarker) appendEvent(event, manualMarker ? "marker_on" : "marker_off");
        if (!analysisEvent.isEmpty()) appendEvent(event, analysisEvent);
        if (stopRequested) appendEvent(event, "record_stop");

        if (recording) recordSample(nowNs, ticks, burstActive, event.toString());
        if (stopRequested) finishRecording(nowNs);

        previousGateOpen = gateOpen;
        previousFeederOn = feederOn;
        previousMarker = manualMarker;

        if (lastTelemetryNs == Long.MIN_VALUE || nowNs - lastTelemetryNs >= TELEMETRY_PERIOD_NS) {
            lastTelemetryNs = nowNs;
            publishTelemetry(currentRpm, step);
        }
    }

    private static void appendEvent(StringBuilder event, String text) {
        if (event.length() > 0) event.append('|');
        event.append(text);
    }

    private void publishTelemetry(double currentRpm, double step) {
        telemetry.clear();
        telemetry.addLine("========== FLYWHEEL ==========");
        telemetry.addData("Flywheel", shooterOn ? "RUNNING" : "OFF - click left stick");
        telemetry.addData("Target / actual / error", "%.0f / %.0f / %+.0f RPM",
                targetRpm, currentRpm, currentRpm - targetRpm);
        telemetry.addData("Battery", "%.2f V", batteryVolts);
        telemetry.addData("P / reference F", "%.3f / %.4f  (step %s)", P, F,
                String.format(Locale.US, step >= 1 ? "%.0f" : "%.4f", step));
        telemetry.addData("P preset", presetIndex >= 0 ? String.format(Locale.US, "%.0f (right stick: 90/150/180)",
                P_PRESETS[presetIndex]) : "custom (right stick: 90/150/180)");
        telemetry.addData("Applied F", "%.4f (reference %.1f V)", appliedF, Robot.SHOOTER_F_REFERENCE_VOLTS);
        telemetry.addData("Save to Robot.java", "V2_TELEOP_SHOOTER_P=%.1f; V2_TELEOP_SHOOTER_F=%.4f", P, F);
        telemetry.addData("Loop", "%.1f ms avg / %.1f ms max", loopMsAverage, loopMsMax);
        telemetry.addData("Velocity steps", velocityNonInteger ? "fine (non-integer readings)"
                : velocityReadings < 20 ? "measuring (run the flywheel)"
                : String.format(Locale.US, "%d ticks/s = %.1f RPM (%d readings)", velocityStepTicks,
                velocityStepTicks * 60.0 / TICKS_PER_REV, velocityReadings));

        telemetry.addLine("========== VOLLEYS ==========");
        if (analyzer.inVolley) {
            telemetry.addData("Volley " + analyzer.volley, "in progress: %d dips, launch %s, min %s",
                    analyzer.dips, analyzer.launchList(), displayRpm(analyzer.minRpm));
        }
        if (summaryCount == 0) telemetry.addLine("No volley yet: X opens gate, hold RB to feed");
        for (int i = 0; i < summaryCount; i++) telemetry.addLine(summaries[i]);
        telemetry.addData("Gate / feeder", "%s / %s", gateOpen ? "OPEN" : "CLOSED",
                feederOn ? "ON (inward)" : feederRequested ? "waiting for gate travel"
                        : gamepad1.right_bumper ? "blocked: need flywheel ON + gate OPEN" : "OFF");

        telemetry.addLine("========== FILES ==========");
        telemetry.addData("Recording", recording
                ? String.format(Locale.US, "YES %.1f s, %d samples", recordingDurationSeconds, sampleCount) : "NO (A)");
        telemetry.addData("Sample CSV", savedPath);
        telemetry.addData("Volley CSV", volleyFilePath);
        if (!loggingError.isEmpty()) telemetry.addData("Logging error", loggingError);

        telemetry.addLine("LEFT STICK flywheel | Y speed | B step | D-pad P/F | R-STICK P preset");
        telemetry.addLine("A record | X gate | RB feed | LB mark | RT/LT load/eject | BACK P=0");
        telemetry.addData("Gate calibration", gateController.calibrated()
                ? gateController.usingCodeDefaults ? "code defaults" : "saved endpoints"
                : "REQUIRED - run Gate Calibration; gate/feed disabled");
    }

    /** Matches V2's manual gate relief and inward feed convention. */
    static double transmissionPower(boolean calibrated, boolean open, long sinceOpenNs,
                                    boolean feed, double loadPower) {
        if (!calibrated) return 0;
        if (open && sinceOpenNs < GATE_RELIEF_NS) return GATE_RELIEF_POWER;
        if (open && sinceOpenNs < GATE_OPEN_WAIT_NS) return 0;
        if (feed || loadPower < 0) return -Robot.TRANSMISSION_INDEX_POWER;
        if (loadPower > 0) return Robot.TRANSMISSION_INDEX_POWER;
        return 0;
    }

    // =========================================
    // STOP
    // =========================================

    @Override
    public void stop() {

        if (recording) finishRecording(nanoTime());

        shooter.setPower(0);
        transmission.setPower(0);
        intake.setPower(0);

        gateOpen = false;
        gateController.commandSaved(false);
    }

    protected long nanoTime() {
        return System.nanoTime();
    }
}
