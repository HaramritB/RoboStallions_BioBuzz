package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

/** Guarded Pedro route tests and optional reviewed mechanism actions. See TeamCode/AUTONOMOUS.md. */
@Autonomous(name = "BioBuzz Pedro Auto", group = "Development")
public class BioBuzzAuto extends OpMode {
    private BioBuzzAutoPaths paths = new BioBuzzAutoPaths();
    private final List<DcMotorEx> driveMotors = new ArrayList<>();
    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private BioBuzzAutoRoutine routine;
    private BioBuzzAutoMechanisms mechanisms = new BioBuzzAutoMechanisms();
    private boolean mechanismsInitialized;
    private boolean mechanismInitFailed;
    private int selectedPaths = BioBuzzAutoPaths.PATH_COUNT;
    private boolean fullRoute = true;
    private Robot.Alliance alliance = Robot.lastAlliance;
    private Pose currentPose;
    private boolean started;
    private boolean safeStopRequested;
    private boolean localizationReady;
    private long autoStart;
    private String diagnostic = "Not initialized";
    private String cleanupFailure = "";
    private String failureReason = "";

    @Override
    public void init() {
        // Also supports repeated INIT on the same instance without stale command state.
        if (routine != null || follower != null || mechanismsInitialized) shutdown();
        Scheduler.reset();
        follower = null;
        pinpoint = null;
        routine = null;
        mechanisms = new BioBuzzAutoMechanisms();
        mechanismsInitialized = false;
        mechanismInitFailed = false;
        selectedPaths = BioBuzzAutoPaths.PATH_COUNT;
        fullRoute = true;
        paths = new BioBuzzAutoPaths();
        alliance = Robot.lastAlliance;
        driveMotors.clear();
        currentPose = null;
        started = false;
        safeStopRequested = false;
        localizationReady = false;
        autoStart = 0;
        cleanupFailure = "";
        failureReason = "";
        telemetry.setMsTransmissionInterval(BioBuzzAutoConfig.TELEMETRY_INTERVAL_MS);
        diagnostic = BioBuzzAutoConfig.blockers(selectedPaths, fullRoute);
        // Initialize the tested hardware without commanding movement, even if the FLOWER
        // offset is missing. This also permits controlled first-1..3-path selection in INIT.
        if (!BioBuzzAutoConfig.blockers(1, false).isEmpty()) {
            publishTelemetry();
            return;
        }
        try {
            follower = createFollower();
            follower.setPose(paths.startPose);
            diagnostic = "Waiting for Pinpoint READY and a valid starting pose";
        } catch (RuntimeException error) {
            fail("INIT: " + describe(error));
        }
        publishTelemetry();
    }

    protected Follower createFollower() {
        for (String name : new String[]{Constants.drivetrainConfig.frontLeftName.get(),
                Constants.drivetrainConfig.frontRightName.get(),
                Constants.drivetrainConfig.backLeftName.get(),
                Constants.drivetrainConfig.backRightName.get()}) {
            DcMotorEx motor = hardwareMap.get(DcMotorEx.class, name);
            driveMotors.add(motor);
            motor.setPower(0);
            // Mecanum 3.0.2 doesn't set this mode. Do not silently change encoder modes.
            if (motor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER) {
                throw new IllegalStateException(name + " must already be RUN_WITHOUT_ENCODER");
            }
        }
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        Follower configured = Constants.create(hardwareMap);
        // Reuse the factory's exact drivetrain, localizer, algorithm and hold setting.
        // Validate each fresh localizer sample INSIDE update(), before motor output.
        Follower result = new Follower(new CheckedLocalizer(configured.localizer,
                () -> pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY),
                configured.drivetrain, configured.algorithm());
        result.holdEnd.set(configured.holdEnd.get());
        if (!result.holdEnd.get()) {
            throw new IllegalStateException("Validated waypoint completion requires holdEnd=true");
        }
        return result;
    }

    private void configurePaths() {
        paths = selectedPaths>=4 ? new BioBuzzAutoPaths(BioBuzzAutoConfig.INTAKE_TIP_FORWARD_IN,
                BioBuzzAutoConfig.INTAKE_TIP_LEFT_IN, true) : new BioBuzzAutoPaths(0, 0, true);
    }

    @Override
    public void init_loop() {
        if (gamepad1.yWasPressed()) { fullRoute = !fullRoute; selectedPaths = fullRoute ? BioBuzzAutoPaths.PATH_COUNT : 1; }
        if (gamepad1.dpadUpWasPressed()) { fullRoute = false; selectedPaths = Math.min(BioBuzzAutoPaths.PATH_COUNT, selectedPaths + 1); }
        if (gamepad1.dpadDownWasPressed()) { fullRoute = false; selectedPaths = Math.max(1, selectedPaths - 1); }
        if (gamepad1.aWasPressed()) alliance = Robot.Alliance.BLUE;
        if (gamepad1.bWasPressed()) alliance = Robot.Alliance.RED;
        String blockers = BioBuzzAutoConfig.blockers(selectedPaths, fullRoute);
        if (!blockers.isEmpty()) { diagnostic = blockers; localizationReady = false; }
        if (follower != null && !safeStopRequested && blockers.isEmpty()) {
            try {
                configurePaths();
                // Localizer only: no follower controller or scheduler runs before START.
                refreshLocalization();
                localizationReady = BioBuzzAutoConfig.atPose(currentPose, paths.startPose) && BioBuzzAutoConfig.settled(follower.velocity());
                prepareMechanisms();
                mechanisms.observe();
                diagnostic = localizationReady ? "READY - " + (fullRoute && BioBuzzAutoConfig.hasMechanismActions() ? "mapped actions" : "movement only")
                        : "Place robot at the configured start pose; heading must match";
            } catch (RuntimeException error) {
                localizationReady = false;
                currentPose = null;
                diagnostic = "Not ready: " + describe(error);
            }
        }
        publishTelemetry();
    }

    @Override
    public void start() {
        if (started || safeStopRequested) return;
        started = true;
        autoStart = System.nanoTime();
        try {
            if (!BioBuzzAutoConfig.blockers(selectedPaths, fullRoute).isEmpty() || follower == null) {
                throw new IllegalStateException("Configuration blocked: " + BioBuzzAutoConfig.blockers(selectedPaths, fullRoute));
            }
            configurePaths();
            refreshLocalization();
            if (!BioBuzzAutoConfig.atPose(currentPose, paths.startPose)
                    || !BioBuzzAutoConfig.settled(follower.velocity())) {
                throw new IllegalStateException("Robot must be stationary at the configured start pose");
            }
            localizationReady = true;
            Robot.lastAlliance = alliance;
            prepareMechanisms();
            mechanisms.observe();
            BioBuzzAutoConfig.Plan plan = BioBuzzAutoConfig.plan(fullRoute);
            routine = new BioBuzzAutoRoutine(follower, paths, selectedPaths, mechanisms, plan);
            routine.start();
            diagnostic = "Running " + (fullRoute ? "full route" : "controlled route test") + ": " + selectedPaths + " paths";
        } catch (RuntimeException error) {
            fail("START: " + describe(error));
        }
        publishTelemetry();
    }

    @Override
    public void loop() {
        if (started && !safeStopRequested && routine != null && !routine.finished()) {
            try {
                routine.update();
                currentPose = follower.pose();
                if (routine.failed()) {
                    fail(routine.failure());
                } else if (routine.finished()) {
                    diagnostic = "COMPLETE - selected paths/actions finished";
                    shutdown();
                }
            } catch (RuntimeException error) {
                currentPose = null;
                localizationReady = false;
                fail("LOOP: " + describe(error));
            }
        }
        publishTelemetry();
    }

    @Override
    public void stop() {
        if (diagnostic.startsWith("Running")) diagnostic = "FTC Stop requested";
        shutdown();
    }

    protected BioBuzzAutoMechanisms createMechanisms() {
        return BioBuzzAutoMechanisms.initialize(hardwareMap, alliance,
                () -> -Math.toDegrees(follower.pose().heading()),
                () -> -Math.toDegrees(follower.velocity().omega));
    }

    private void prepareMechanisms() {
        if (fullRoute && BioBuzzAutoConfig.hasMechanismActions() && !mechanismsInitialized) {
            if (mechanismInitFailed) throw new IllegalStateException("Mechanism initialization failed; re-INIT after correcting hardware");
            try {
                mechanisms = createMechanisms();
                mechanismsInitialized = true;
            } catch (RuntimeException error) {
                mechanismInitFailed = true;
                throw error;
            }
        }
        if (mechanismsInitialized) mechanisms.setAlliance(alliance);
    }

    private void refreshLocalization() {
        follower.localizer.update();
        currentPose = follower.pose();
    }

    private void fail(String reason) {
        diagnostic = reason;
        if (failureReason.isEmpty()) failureReason = reason;
        cleanup(() -> { if (routine != null) routine.fail(reason); });
        shutdown();
    }

    private void shutdown() {
        safeStopRequested = true;
        // Each cleanup step is independent: one failed device write must not skip the others.
        cleanup(() -> { if (routine != null) routine.stop(); });
        cleanup(() -> mechanisms.stop());
        cleanup(() -> { if (routine != null) Scheduler.cancel(routine.command()); });
        cleanup(Scheduler::reset); // reset alone doesn't run Ivy end callbacks
        cleanup(() -> { if (follower != null) follower.stop(); });
        cleanup(() -> { if (follower != null) follower.drivetrain.stop(); });
        for (DcMotorEx motor : driveMotors) cleanup(() -> motor.setPower(0));
        // Never call Robot.init()/drive()/stop(): Pedro is the sole drivetrain owner.
    }

    private void cleanup(Runnable action) {
        try {
            action.run();
        } catch (RuntimeException error) {
            cleanupFailure += describe(error) + "; ";
        }
    }

    private double elapsedSeconds() {
        return autoStart == 0 ? 0 : (System.nanoTime() - autoStart) / 1e9;
    }

    private static String describe(RuntimeException error) {
        return error.getClass().getSimpleName() + ": " + error.getMessage();
    }

    private void publishTelemetry() {
        telemetry.addData("Configuration", BioBuzzAutoConfig.blockers(selectedPaths, fullRoute).isEmpty()
                ? "Review gates confirmed" : BioBuzzAutoConfig.blockers(selectedPaths, fullRoute));
        telemetry.addData("Status", diagnostic);
        telemetry.addData("Mode", fullRoute ? "FULL ROUTE" : "CONTROLLED TEST (movement only)");
        telemetry.addData("Selected paths", "%d/%d (D-pad +/-; Y toggles full)", selectedPaths, BioBuzzAutoPaths.PATH_COUNT);
        telemetry.addData("Alliance for mechanisms", "%s (A blue, B red); route is not mirrored", alliance);
        telemetry.addData("Mechanisms", mechanisms.state());
        telemetry.addData("Mechanism diagnostics", mechanisms.diagnostics());
        telemetry.addData("Intake policy", mechanisms.intakePolicy());
        telemetry.addData("Scoring plan", "Shoot 4 preloads; intake 4 during path 1; shoot @2; intake 4 FLOWER @4; shoot @5");
        telemetry.addData("Requested shots (not inventory)", "%d preload / %d staged / %d FLOWER",
                BioBuzzAutoConfig.PRELOAD_BALL_COUNT, BioBuzzAutoConfig.STAGED_BALL_COUNT, BioBuzzAutoConfig.FLOWER_BALL_COUNT);
        telemetry.addData("Mechanism failure", mechanisms.failure().isEmpty() ? "none" : mechanisms.failure());
        telemetry.addData("Route source SHA", BioBuzzAutoPaths.SOURCE_SHA256);
        telemetry.addData("FLOWER target", "Visualizer point 4 is intake tip; measured forward reach=%s in", BioBuzzAutoConfig.INTAKE_TIP_FORWARD_IN);
        if (Double.isFinite(BioBuzzAutoConfig.INTAKE_TIP_FORWARD_IN) && selectedPaths>=4) {
            Pose center=paths.end(4);
            telemetry.addData("FLOWER robot center (in)", "%.3f, %.3f", center.x(),center.y());
        }
        telemetry.addData("Stage", routine == null ? "BLOCKED" : routine.stage());
        telemetry.addData("Route segment", routine == null ? "none" : routine.segment());
        telemetry.addData("Routine finished", routine != null && routine.finished());
        telemetry.addData("Elapsed (s)", "%.1f", elapsedSeconds());
        telemetry.addData("Safe stop requested", safeStopRequested);
        telemetry.addData("Localization ready", localizationReady);
        telemetry.addData("Pinpoint", pinpoint == null ? "not initialized" : pinpoint.getDeviceStatus());
        telemetry.addData("Follower mode", follower == null ? "unavailable" : follower.mode());
        telemetry.addData("Following", follower != null && follower.following());
        telemetry.addData("Failure", failureReason.isEmpty() ? "none" : failureReason);
        telemetry.addData("Cleanup errors", cleanupFailure.isEmpty() ? "none" : cleanupFailure);
        telemetry.addData("Configured start (in, deg)", "%.1f, %.1f, %.1f", paths.startPose.x(),
                paths.startPose.y(), Math.toDegrees(paths.startPose.heading()));
        if (BioBuzzAutoConfig.finite(currentPose)) {
            telemetry.addData("X (in)", "%.2f", currentPose.x());
            telemetry.addData("Y (in)", "%.2f", currentPose.y());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(currentPose.heading()));
            Pose target = routine == null ? paths.startPose : routine.target();
            telemetry.addData("Target (in, deg)", "%.1f, %.1f, %.1f", target.x(), target.y(),
                    Math.toDegrees(target.heading()));
            telemetry.addData("Error X (in)", "%.2f", target.x() - currentPose.x());
            telemetry.addData("Error Y (in)", "%.2f", target.y() - currentPose.y());
            telemetry.addData("Heading error (deg)", "%.1f",
                    BioBuzzAutoConfig.headingErrorDegrees(target, currentPose));
            telemetry.addData("Distance to target (in)", "%.2f", BioBuzzAutoConfig.distance(target, currentPose));
        } else {
            telemetry.addData("Pose / errors", "unavailable");
        }
        telemetry.update();
    }

    /** Reject bad samples before Follower.update() can use them to compute drivetrain powers. */
    static final class CheckedLocalizer implements Localizer {
        private final Localizer delegate;
        private final BooleanSupplier ready;

        CheckedLocalizer(Localizer delegate, BooleanSupplier ready) {
            this.delegate = delegate;
            this.ready = ready;
        }

        @Override public void update() {
            delegate.update();
            if (!ready.getAsBoolean()) throw new IllegalStateException("Pinpoint is not READY");
            if (!BioBuzzAutoConfig.finite(delegate.pose()) || !BioBuzzAutoConfig.finite(delegate.velocity())) {
                throw new IllegalStateException("Nonfinite or missing localization sample");
            }
        }
        @Override public MotionState state() { return delegate.state(); }
        @Override public void setPose(Pose pose) { delegate.setPose(pose); }
        @Override public void reset() { delegate.reset(); }
    }
}
