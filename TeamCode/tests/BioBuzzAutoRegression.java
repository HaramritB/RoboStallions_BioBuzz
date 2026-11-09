package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.algorithm.Foresight;
import org.firstinspires.ftc.teamcode.Constants;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import java.util.Collections;
import java.util.Map;

/** Exercises real Ivy sequencing and Pedro paths; no hardware or Android runtime is used. */
public final class BioBuzzAutoRegression {
    private static int checks;
    private static final PoseFactory P = PoseFactory.degrees();

    private static void check(boolean ok, String message) {
        checks++;
        if (!ok) throw new AssertionError(message);
    }

    private static void rejects(Runnable action, String message) {
        try {
            action.run();
        } catch (IllegalArgumentException expected) {
            check(true, message);
            return;
        }
        throw new AssertionError(message);
    }

    private static final class Navigation implements Localizer {
        Pose pose = P.of(24, 24, 0);
        Velocity velocity = Velocity.zero();
        @Override public void setPose(Pose value) { pose = value; }
        @Override public MotionState state() { return MotionState.ofVelocity(pose, velocity); }
        @Override public void update() { }
        @Override public void reset() { }
    }

    private static final class Drive implements Drivetrain {
        int writes;
        int stops;
        @Override public void drive(DrivePowers powers, boolean manual) { writes++; }
        @Override public void stop() { stops++; }
        @Override public void stop(boolean brake) { stop(); }
        @Override public double maxScaling(DrivePowers current, DrivePowers delta) { return 1; }
        @Override public Map<String, Object> debug() { return Collections.emptyMap(); }
        @Override public double interpolateVelocity(double x, double y, double theta) { return 1; }
    }

    private static final class Tracking extends Follower {
        int pathsStarted;
        Tracking(Navigation navigation, Drive drive) {
            super(navigation, drive, new Foresight(Constants.foresightConfig));
        }
        @Override public void follow(Path path) {
            pathsStarted++;
            super.follow(path);
        }
    }

    private static final class Fixture {
        final BioBuzzAutoPaths paths = new BioBuzzAutoPaths();
        final Navigation navigation = new Navigation();
        final Drive drive = new Drive();
        final Tracking follower = new Tracking(navigation, drive);
        final long[] now = {1_000_000_000L};
        final BioBuzzAutoRoutine routine = new BioBuzzAutoRoutine(follower, paths, () -> now[0]);
        Fixture() { Scheduler.reset(); }
        void start() { Scheduler.schedule(routine.command()); }
        void arrive(Pose pose) { navigation.pose = pose; follower.hold(pose); }
        void ticks() { for (int i = 0; i < 8; i++) Scheduler.execute(); }
    }

    public static void main(String[] args) {
        BioBuzzAutoPaths paths = new BioBuzzAutoPaths();
        check(BioBuzzAuto.class.isAnnotationPresent(Disabled.class), "Physical OpMode must be disabled");
        check(!BioBuzzAutoConfig.blockers().isEmpty(), "Configuration must remain blocked");
        check(paths.startToScore() != paths.startToScore(), "Path factories must return fresh paths");
        check(BioBuzzAutoConfig.atPose(paths.startToScore().get(0), paths.startPose), "Path A start");
        check(BioBuzzAutoConfig.atPose(paths.startToScore().get(1), paths.scorePose), "Path A end");
        check(paths.startToScore().get(1).x() == paths.scoreToPark().get(0).x()
                && paths.startToScore().get(1).y() == paths.scoreToPark().get(0).y()
                && paths.startToScore().get(1).heading() == paths.scoreToPark().get(0).heading(),
                "Shared endpoint must match exactly");
        check(Math.abs(Math.toDegrees(paths.startToScore().get(0.5).heading()) - 45) < 1e-9,
                "Path A must interpolate heading");
        check(Math.abs(Math.toDegrees(paths.scoreToPark().get(0.5).heading()) - 90) < 1e-9,
                "Path B must hold 90 degree heading");
        Path curved = BioBuzzAutoPaths.bezier(paths.startPose, paths.scorePose, false, P.of(36, 60, 0));
        check(BioBuzzAutoConfig.atPose(curved.get(1), paths.scorePose), "Curve endpoint and heading");
        rejects(() -> BioBuzzAutoPaths.straight(paths.startPose, paths.scorePose, true),
                "Reject conflicting constant heading");
        rejects(() -> BioBuzzAutoPaths.straight(paths.startPose, paths.startPose, false), "Reject zero length");
        rejects(() -> BioBuzzAutoPaths.straight(P.of(Double.NaN, 0, 0), paths.scorePose, false),
                "Reject invalid coordinates");
        check(Math.abs(BioBuzzAutoConfig.headingErrorDegrees(P.of(0, 0, 1), P.of(0, 0, 359)) - 2) < 1e-8,
                "Heading errors must wrap");

        Fixture normal = new Fixture();
        check(normal.follower.pathsStarted == 0, "Construction must not start driving");
        normal.start();
        normal.ticks();
        check(normal.follower.pathsStarted == 1, "Do not start park while first path tracks");
        normal.arrive(paths.scorePose);
        normal.navigation.velocity = new Velocity(8, 0, 0);
        normal.ticks();
        check(normal.follower.pathsStarted == 1, "Arrival must wait until settled");
        normal.navigation.velocity = Velocity.zero();
        normal.ticks();
        check(normal.follower.pathsStarted == 2, "Start park only after verified arrival");
        normal.arrive(paths.parkPose);
        normal.ticks();
        check(normal.routine.stage() == BioBuzzAutoRoutine.Stage.COMPLETE, "Routine must complete");
        check(normal.follower.idle() && normal.drive.stops > 0, "Completion must stop drivetrain");
        check(!Scheduler.isScheduled(normal.routine.command()), "Completion must release scheduler state");

        Fixture timeout = new Fixture();
        timeout.start();
        timeout.now[0] += 11_000_000_000L;
        timeout.ticks();
        check(timeout.routine.failed() && timeout.routine.failure().contains("timeout"), "Latch segment timeout");
        check(timeout.follower.pathsStarted == 1 && timeout.drive.stops > 0, "Timeout must not start park");

        Fixture missed = new Fixture();
        missed.start();
        missed.arrive(P.of(10, 10, 0));
        missed.ticks();
        check(missed.follower.pathsStarted == 1, "Geometric path end is insufficient for arrival");
        missed.now[0] += 11_000_000_000L;
        missed.ticks();
        check(missed.routine.failed(), "Unreached waypoint must fail");

        Fixture interrupted = new Fixture();
        interrupted.start();
        interrupted.follower.stop();
        interrupted.ticks();
        check(interrupted.routine.failed() && interrupted.follower.pathsStarted == 1,
                "Unexpected follower interruption must block park");

        Fixture cancelled = new Fixture();
        cancelled.start();
        Scheduler.cancel(cancelled.routine.command());
        cancelled.ticks();
        check(cancelled.routine.failed() && cancelled.drive.stops > 0, "Ivy cancellation must stop motors");
        check(cancelled.follower.pathsStarted == 1, "Cancellation must not start unstarted children");

        Fixture stopped = new Fixture();
        stopped.start();
        stopped.routine.stop();
        Scheduler.cancel(stopped.routine.command());
        stopped.ticks();
        check(stopped.routine.stage() == BioBuzzAutoRoutine.Stage.STOPPED, "FTC stop differs from failure");
        check(stopped.follower.idle() && stopped.follower.pathsStarted == 1, "Stop must block subsequent paths");

        Fixture badStart = new Fixture();
        badStart.navigation.pose = P.of(50, 50, 0);
        badStart.start();
        badStart.ticks();
        check(badStart.routine.failed() && badStart.follower.pathsStarted == 0, "Invalid start must not drive");

        Navigation broken = new Navigation();
        Drive protectedDrive = new Drive();
        boolean[] ready = {false};
        Follower protectedFollower = new Follower(new BioBuzzAuto.CheckedLocalizer(broken, () -> ready[0]),
                protectedDrive, new Foresight(Constants.foresightConfig));
        protectedFollower.follow(paths.startToScore());
        assertUpdateBlocked(protectedFollower, protectedDrive, "Not-ready localizer");
        ready[0] = true;
        broken.pose = P.of(Double.NaN, 24, 0);
        assertUpdateBlocked(protectedFollower, protectedDrive, "Invalid fresh pose");
        broken.pose = paths.startPose;
        broken.velocity = new Velocity(0, Double.POSITIVE_INFINITY, 0);
        assertUpdateBlocked(protectedFollower, protectedDrive, "Invalid fresh velocity");
        Scheduler.reset();
        check(!Scheduler.isScheduled(stopped.routine.command()), "Reset must clear stale state");
        System.out.println("Passed " + checks + " autonomous regression checks.");
    }

    private static void assertUpdateBlocked(Follower follower, Drive drive, String message) {
        int writes = drive.writes;
        try {
            follower.update();
            throw new AssertionError(message + " must reject update");
        } catch (IllegalStateException expected) {
            check(drive.writes == writes, message + " must be rejected BEFORE drivetrain output");
        }
    }
}
