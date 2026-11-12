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

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

/** Runs the fixed five-path route using only the drivetrain and Pinpoint. */
@Autonomous(name = "BioBuzz Pedro Auto", group = "Development")
public class BioBuzzAuto extends OpMode {
    private BioBuzzAutoPaths paths = new BioBuzzAutoPaths();
    private final List<DcMotorEx> driveMotors = new ArrayList<>();
    private Follower follower;
    private GoBildaPinpointDriver pinpoint;
    private BioBuzzAutoRoutine routine;
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
        if (routine != null || follower != null || !driveMotors.isEmpty()) shutdown();
        Scheduler.reset();
        follower = null;
        pinpoint = null;
        routine = null;
        paths = new BioBuzzAutoPaths();
        driveMotors.clear();
        currentPose = null;
        started = false;
        safeStopRequested = false;
        localizationReady = false;
        autoStart = 0;
        cleanupFailure = "";
        failureReason = "";
        telemetry.setMsTransmissionInterval(BioBuzzAutoConfig.TELEMETRY_INTERVAL_MS);
        try {
            configurePaths();
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
        String blocker = BioBuzzAutoConfig.blockers(BioBuzzAutoPaths.PATH_COUNT, false);
        if (!blocker.isEmpty()) throw new IllegalStateException(blocker);
        paths = new BioBuzzAutoPaths(BioBuzzAutoConfig.INTAKE_TIP_FORWARD_IN,
                BioBuzzAutoConfig.INTAKE_TIP_LEFT_IN, true);
    }

    @Override
    public void init_loop() {
        if (follower != null && !safeStopRequested) {
            try {
                // Localizer only: no follower controller or scheduler runs before START.
                refreshLocalization();
                localizationReady = BioBuzzAutoConfig.atPose(currentPose, paths.startPose) && BioBuzzAutoConfig.settled(follower.velocity());
                diagnostic = localizationReady ? "READY - fixed route, drive only"
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
        autoStart = nanoTime();
        try {
            if (follower == null) throw new IllegalStateException("Drivetrain not initialized");
            configurePaths();
            refreshLocalization();
            if (!BioBuzzAutoConfig.atPose(currentPose, paths.startPose)
                    || !BioBuzzAutoConfig.settled(follower.velocity())) {
                throw new IllegalStateException("Robot must be stationary at the configured start pose");
            }
            localizationReady = true;
            routine = BioBuzzAutoRoutine.driveOnly(follower, paths, this::nanoTime);
            routine.start();
            diagnostic = "Running fixed five-path route";
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
                    diagnostic = "COMPLETE - all five paths finished";
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

    protected long nanoTime() {
        return System.nanoTime();
    }

    private double elapsedSeconds() {
        return autoStart == 0 ? 0 : (nanoTime() - autoStart) / 1e9;
    }

    private static String describe(RuntimeException error) {
        return error.getClass().getSimpleName() + ": " + error.getMessage();
    }

    private void publishTelemetry() {
        telemetry.addData("Status", diagnostic);
        telemetry.addData("Mode", "FIXED ROUTE (drive only)");
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
