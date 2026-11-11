package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.function.LongSupplier;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/** Follows the five supplied paths with Pedro and Pinpoint. */
@Autonomous(name = "AutoPath", group = "Autonomous")
public final class AutoPath extends LinearOpMode {
    private static final double SPEED_FRACTION = 0.70;
    private static final double FLOWER_ARRIVAL_IN = 1.0;
    private static final double FLOWER_PROGRESS_IN = 0.25;
    private static final long FLOWER_STALL_NS = 4_000_000_000L;
    private static final PoseFactory POSES = PoseFactory.degrees();
    private static final Pose START = POSES.of(57.6323, 9.0657, 90);
    private static final Pose POINT_1 = POSES.of(9.5657, 9.2715, 179.7546);
    private static final Pose POINT_2 = POSES.of(32.2934, 127.6559, -84.7123);
    private static final Pose PATH_3_START = POSES.of(32.2934, 127.6559, 0);
    private static final Pose POINT_3 = POSES.of(46.9373, 125.3042, 90);
    private static final Pose POINT_4 = POSES.of(46.7883, 140.2305, 90.572);
    private static final Pose POINT_5 = POSES.of(38.0943, 127.3583, -66.9637);

    @Override
    public void runOpMode() {
        Scheduler.reset();
        Follower follower = Constants.create(hardwareMap);
        Command route = route(follower, System::nanoTime);

        try {
            follower.setPose(START);
            waitForStart();
            if (!opModeIsActive()) return;

            Scheduler.schedule(route);
            while (opModeIsActive() && Scheduler.isScheduled(route)) {
                follower.update();
                Scheduler.execute();

                Pose pose = follower.pose();
                telemetry.addData("X (in)", "%.2f", pose.x());
                telemetry.addData("Y (in)", "%.2f", pose.y());
                telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
                if (follower.currentPath() != null) {
                    telemetry.addData("Distance remaining (in)", "%.2f", follower.distanceToEndpoint());
                }
                telemetry.update();
                idle();
            }
        } finally {
            try {
                Scheduler.cancel(route);
            } finally {
                Scheduler.reset();
                follower.stop();
                follower.drivetrain.stop();
            }
        }
    }

    static Command route(Follower follower, LongSupplier clock) {
        FlowerApproach flower = new FlowerApproach();
        Path approach = atRouteSpeed(path4());
        return sequential(
                follow(follower, atRouteSpeed(path1())),
                follow(follower, atRouteSpeed(path2())),
                follow(follower, atRouteSpeed(path3())),
                Command.build().setStart(() -> {
                    follower.follow(approach);
                    flower.start(follower.pose(), clock.getAsLong());
                }).setDone(() -> flower.finished(follower, clock.getAsLong())),
                lazy(() -> follow(follower, atRouteSpeed(path5(flower.departure)))));
    }

    private static Path atRouteSpeed(Path path) {
        return path.with(Constants.foresightConfig.maxPathSpeed.at(SPEED_FRACTION));
    }

    private static final class FlowerApproach {
        private Pose departure;
        private double closestDistance;
        private long lastProgress;

        void start(Pose pose, long now) {
            departure = POINT_4;
            closestDistance = pose.distance(POINT_4);
            lastProgress = now;
        }

        boolean finished(Follower follower, long now) {
            Pose pose = follower.pose();
            double distance = pose.distance(POINT_4);
            if (!Double.isFinite(distance) || !Double.isFinite(pose.heading())) {
                throw new IllegalStateException("Invalid Pinpoint pose on FLOWER approach");
            }
            if (distance <= FLOWER_ARRIVAL_IN) return true;
            if (closestDistance - distance >= FLOWER_PROGRESS_IN) {
                closestDistance = distance;
                lastProgress = now;
            }
            if (now - lastProgress < FLOWER_STALL_NS) return false;

            // Rebase the exit so Pedro can back away without chasing the blocked endpoint.
            departure = pose;
            follower.stop();
            follower.drivetrain.stop();
            return true;
        }
    }

    static Path path1() {
        return line(START, POINT_1).tangent();
    }

    static Path path2() {
        return curve(POINT_1, POSES.of(40.923, 34.4143, 0), POINT_2).reverseTangent();
    }

    static Path path3() {
        return curve(PATH_3_START, POSES.of(40.2631, 108.1664, 0), POINT_3)
                .linear(PATH_3_START, POINT_3);
    }

    static Path path4() {
        return line(POINT_3, POINT_4).tangent();
    }

    static Path path5() {
        return path5(POINT_4);
    }

    private static Path path5(Pose departure) {
        return curve(departure, POSES.of(45.4429, 110.0765, 0), POINT_5).reverseTangent();
    }
}
