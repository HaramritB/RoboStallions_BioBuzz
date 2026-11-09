package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.function.LongSupplier;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/** One fresh routine per INIT. Failures latch and prevent subsequent paths from starting. */
public final class BioBuzzAutoRoutine {
    public enum Stage { READY, TO_SCORE, TO_PARK, COMPLETE, FAILED, STOPPED }

    private final Follower follower;
    private final LongSupplier clock;
    private final Command command;
    private Stage stage = Stage.READY;
    private Pose target;
    private String failure = "";
    private String segment = "none";
    private long segmentStart;

    public BioBuzzAutoRoutine(Follower follower, BioBuzzAutoPaths paths) {
        this(follower, paths, System::nanoTime);
    }

    // A monotonic clock can be injected for hardware-free timeout tests.
    BioBuzzAutoRoutine(Follower follower, BioBuzzAutoPaths paths, LongSupplier clock) {
        this.follower = follower;
        this.clock = clock;
        target = paths.startPose;
        command = sequential(
                movement(paths.startToScore(), paths.startPose, paths.scorePose, Stage.TO_SCORE),
                // Future verified, nonblocking scoring command belongs HERE.
                // Require its actuators, check success, and call fail() on timeout before park.
                movement(paths.scoreToPark(), paths.scorePose, paths.parkPose, Stage.TO_PARK),
                instant(() -> {
                    if (!failed() && stage != Stage.STOPPED) {
                        stopDrive();
                        stage = Stage.COMPLETE;
                    }
                })
        );
    }

    private Command movement(Path path, Pose start, Pose end, Stage movementStage) {
        final boolean[] started = {false};
        // Ivy 1.1.1 checks atParametricEnd by default. Replace that check: geometry alone
        // does not prove the robot arrived, stopped, or avoided an interruption.
        return follow(follower, path)
                .requiring(follower)
                .setStart(() -> {
                    if (failed() || stage == Stage.STOPPED) return;
                    if (!BioBuzzAutoConfig.atPose(follower.pose(), start)) {
                        fail("Invalid start pose for " + movementStage);
                        return;
                    }
                    started[0] = true;
                    stage = movementStage;
                    segment = movementStage.name();
                    target = end;
                    segmentStart = clock.getAsLong();
                    follower.follow(path);
                })
                .setDone(() -> {
                    if (!started[0] || failed() || stage == Stage.STOPPED) return true;
                    if (!BioBuzzAutoConfig.finite(follower.pose())
                            || !BioBuzzAutoConfig.finite(follower.velocity())) {
                        fail("Invalid localization during " + segment);
                        return true;
                    }
                    if (segmentSeconds() >= BioBuzzAutoConfig.SEGMENT_TIMEOUT_S) {
                        fail("Waypoint timeout: " + segment);
                        return true;
                    }
                    if (follower.following()) {
                        if (follower.currentPath() != path) {
                            fail("Path replaced unexpectedly: " + segment);
                            return true;
                        }
                        return false;
                    }
                    if (!follower.holding()) {
                        fail("Path interrupted: " + segment);
                        return true;
                    }
                    return BioBuzzAutoConfig.atPose(follower.pose(), end)
                            && BioBuzzAutoConfig.settled(follower.velocity());
                })
                .setEnd(reason -> {
                    // Sequential calls end() on unstarted children during cancellation.
                    if (started[0] && reason != EndCondition.NATURALLY
                            && stage != Stage.STOPPED && !finished()) {
                        fail("Ivy command interrupted: " + segment + " (" + reason + ")");
                    }
                });
    }

    public void fail(String reason) {
        if (stage == Stage.COMPLETE || stage == Stage.STOPPED) return;
        if (failure.isEmpty()) failure = reason;
        stage = Stage.FAILED;
        stopDrive();
    }

    public void stop() {
        if (!finished()) stage = Stage.STOPPED;
        stopDrive();
    }

    private void stopDrive() {
        // Pedro 3.0.2 stop() clears navigation state only. Stop motor output explicitly.
        try {
            follower.stop();
        } finally {
            follower.drivetrain.stop();
        }
    }

    public Command command() { return command; }
    public Stage stage() { return stage; }
    public String segment() { return segment; }
    public Pose target() { return target; }
    public String failure() { return failure; }
    public boolean failed() { return stage == Stage.FAILED; }
    public boolean finished() { return stage == Stage.COMPLETE || stage == Stage.FAILED; }
    public double segmentSeconds() {
        return stage == Stage.READY ? 0 : (clock.getAsLong() - segmentStart) / 1e9;
    }
}
