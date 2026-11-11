package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.autonomous.BioBuzzAutoConfig;

import java.util.Collections;
import java.util.Map;

/** Checks the direct visualizer route against its exported geometry and heading modes. */
public final class AutoPathRegression {
    public static void main(String[] args) {
        assertRegistered(AutoPath.class, "AutoPath");
        assertRegistered(org.firstinspires.ftc.teamcode.autonomous.BioBuzzAuto.class, "BioBuzz Pedro Auto");
        if (!BioBuzzAuto.class.isAnnotationPresent(Disabled.class)) {
            throw new AssertionError("Legacy encoder autonomous must remain disabled");
        }
        Path[] paths = {AutoPath.path1(), AutoPath.path2(), AutoPath.path3(), AutoPath.path4(), AutoPath.path5()};
        double[][] xy = {{57.6323, 9.0657}, {9.5657, 9.2715}, {32.2934, 127.6559},
                {46.9373, 125.3042}, {46.7883, 140.2305}, {38.0943, 127.3583}};
        for (int i = 0; i < paths.length; i++) {
            assertPoint(paths[i].get(0), xy[i][0], xy[i][1]);
            assertPoint(paths[i].get(1), xy[i + 1][0], xy[i + 1][1]);
        }
        assertHeading(paths[0].heading(0.5), Math.atan2(9.2715 - 9.0657, 9.5657 - 57.6323));
        assertHeading(paths[1].heading(0), Math.atan2(34.4143 - 9.2715, 34.923 - 9.5657) + Math.PI);
        checkCenterFrameClearance(paths[1]);
        assertHeading(paths[2].heading(0), 0);
        assertHeading(paths[2].heading(1), Math.PI / 2);
        assertHeading(paths[3].heading(0.5), Math.atan2(140.2305 - 125.3042, 46.7883 - 46.9373));
        assertHeading(paths[4].heading(1), Math.atan2(127.3583 - 110.0765, 38.0943 - 45.4429) + Math.PI);
        checkFlowerExit(false);
        checkFlowerExit(true);
        System.out.println("Passed direct route geometry, headings, speed, path-1 pause, FLOWER recovery, and registration checks.");
    }

    private static void checkCenterFrameClearance(Path path) {
        double halfLength = BioBuzzAutoConfig.ROBOT_LENGTH_IN / 2;
        double halfWidth = BioBuzzAutoConfig.ROBOT_WIDTH_IN / 2;
        double clearance = Double.POSITIVE_INFINITY;
        for (int i = 0; i <= 1000; i++) {
            Pose pose = path.get(i / 1000.0);
            double cos = Math.abs(Math.cos(pose.heading()));
            double sin = Math.abs(Math.sin(pose.heading()));
            double extentY = sin * halfLength + cos * halfWidth;
            // The red HIVE drawing spans Y=50..91.4 and begins at X=46.
            if (pose.y() + extentY >= 50 && pose.y() - extentY <= 91.4) {
                double extentX = cos * halfLength + sin * halfWidth;
                clearance = Math.min(clearance, 46 - pose.x() - extentX);
            }
        }
        if (!Double.isFinite(clearance) || clearance < 4.3) {
            throw new AssertionError("Path 2 nominal center-frame clearance fell below 4.3 inches: " + clearance);
        }
    }

    private static void checkFlowerExit(boolean stalled) {
        PoseFactory poses = PoseFactory.degrees();
        Navigation navigation = new Navigation();
        Drive drive = new Drive();
        Tracking follower = new Tracking(navigation, drive);
        long[] now = {0};
        double originalSpeed = Constants.foresightConfig.maxPathSpeed.get();
        Command route = AutoPath.route(follower, () -> now[0]);
        Scheduler.reset();
        try {
            Scheduler.schedule(route);
            assertStep(follower, 1);
            now[0] += 2_000_000_000L;
            Scheduler.execute();
            assertStep(follower, 1); // Travel time alone must not trigger the pause.
            navigation.setPose(stalled
                    ? poses.of(12.0657, 9.2715, 179.7546)
                    : follower.currentPath().endPose());
            follower.atEnd = !stalled;
            int stopsBeforePause = drive.stops;
            Scheduler.execute();
            if (!follower.idle() || drive.stops <= stopsBeforePause) {
                throw new AssertionError("Path-1 pause must start with the drivetrain stopped");
            }
            now[0] += 999_000_000L;
            Scheduler.execute();
            if (follower.pathsStarted != 1) throw new AssertionError("Path 2 must wait one full second");
            now[0] += 1_000_000L;
            Scheduler.execute();

            for (int step = 2; step <= 3; step++) {
                assertStep(follower, step);
                navigation.setPose(follower.currentPath().endPose());
                follower.atEnd = true;
                Scheduler.execute();
            }
            assertStep(follower, 4);
            if (stalled) {
                navigation.setPose(poses.of(46.88, 130.0, 90));
                follower.atEnd = true; // Path projection alone cannot establish arrival at FLOWER.
                Scheduler.execute();
                now[0] += 3_900_000_000L;
                Scheduler.execute();
                assertStep(follower, 4);

                navigation.setPose(poses.of(46.88, 131.0, 90));
                Scheduler.execute(); // Renew the window after genuine approach progress.
                now[0] += 3_999_000_000L;
                Scheduler.execute();
                assertStep(follower, 4);
                int stopsBeforeEscape = drive.stops;
                now[0] += 1_000_000L;
                Scheduler.execute();
                assertStep(follower, 5);
                if (drive.stops <= stopsBeforeEscape) throw new AssertionError("Stall must stop pushing");
                assertPoint(follower.currentPath().get(0), 46.88, 131.0);
            } else {
                navigation.setPose(poses.of(46.7883, 140.2305, 90.572));
                Scheduler.execute();
                assertStep(follower, 5);
                assertPoint(follower.currentPath().get(0), 46.7883, 140.2305);
            }
            assertPoint(follower.currentPath().get(1), 38.0943, 127.3583);
            follower.atEnd = true;
            Scheduler.execute();
            if (Scheduler.isScheduled(route)) throw new AssertionError("Path 5 must finish the route");
        } finally {
            Scheduler.cancel(route);
            follower.stop();
            Scheduler.reset();
        }
        if (Constants.foresightConfig.maxPathSpeed.get() != originalSpeed) {
            throw new AssertionError("AutoPath speed must stay scoped to its paths");
        }
    }

    private static void assertStep(Tracking follower, int step) {
        if (follower.pathsStarted != step || Constants.foresightConfig.maxPathSpeed.get() != 0.70) {
            throw new AssertionError("Expected path " + step + " at 70% speed, saw path " + follower.pathsStarted);
        }
    }

    private static final class Navigation implements Localizer {
        private Pose pose = AutoPath.path1().get(0);

        @Override public MotionState state() { return MotionState.ofVelocity(pose, Velocity.zero()); }
        @Override public void setPose(Pose pose) { this.pose = pose; }
        @Override public void update() { }
        @Override public void reset() { }
    }

    private static final class Drive implements Drivetrain {
        private int stops;

        @Override public void drive(DrivePowers powers, boolean manual) { }
        @Override public void stop() { stops++; }
        @Override public void stop(boolean brake) { stop(); }
        @Override public double maxScaling(DrivePowers a, DrivePowers b) { return 1; }
        @Override public double interpolateVelocity(double x, double y, double turn) { return 1; }
        @Override public Map<String, Object> debug() { return Collections.emptyMap(); }
    }

    private static final class Tracking extends Follower {
        private int pathsStarted;
        private boolean atEnd;

        Tracking(Navigation navigation, Drive drive) {
            super(navigation, drive, new Foresight(Constants.foresightConfig));
        }

        @Override public boolean atParametricEnd() { return atEnd; }

        @Override
        public void follow(Path path) {
            super.follow(path);
            pathsStarted++;
            atEnd = false;
        }
    }

    private static void assertRegistered(Class<?> type, String name) {
        Autonomous opMode = type.getAnnotation(Autonomous.class);
        if (type.isAnnotationPresent(Disabled.class) || opMode == null || !opMode.name().equals(name)) {
            throw new AssertionError("Expected Driver Station entry " + name);
        }
    }

    private static void assertPoint(Pose pose, double x, double y) {
        if (Math.hypot(pose.x() - x, pose.y() - y) > 1e-6) {
            throw new AssertionError("Unexpected route endpoint " + pose);
        }
    }

    private static void assertHeading(double actual, double expected) {
        double delta = actual - expected;
        if (Math.abs(Math.atan2(Math.sin(delta), Math.cos(delta))) > 1e-6) {
            throw new AssertionError("Unexpected route heading " + Math.toDegrees(actual));
        }
    }
}
