package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/** Follows the five supplied paths with Pedro and Pinpoint. */
@Autonomous(name = "AutoPath", group = "Autonomous")
public final class AutoPath extends LinearOpMode {
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
        Command route = sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()));

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
        return curve(POINT_4, POSES.of(45.4429, 110.0765, 0), POINT_5).reverseTangent();
    }
}
