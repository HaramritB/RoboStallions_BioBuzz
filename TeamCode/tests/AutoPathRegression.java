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

import java.util.Collections;
import java.util.Map;

/** Checks the direct visualizer route against its exported geometry and heading modes. */
public final class AutoPathRegression {
    public static void main(String[] args) {
        assertRegistered(AutoPath.class, "AutoPath");
        assertRegistered(AutoPathDriveOnly.class, "AutoPath (Drive Only)");
        assertRegistered(org.firstinspires.ftc.teamcode.autonomous.BioBuzzAuto.class, "BioBuzz Pedro Auto");
        assertRegistered(BioBuzzAuto.class, "BioBuzz Auto"); // Replaced by the teammate's Pedro route.
        Path[] paths = {AutoPath.path1(), AutoPath.path2(), AutoPath.path3(), AutoPath.path4(),
                AutoPath.path5(), AutoPath.path6(), AutoPath.path7(), AutoPath.path8()};
        double[][] xy = {{56, 8}, {8.2575, 16.1386}, {7.994, 0}, {8.2939, 15.6271},
                {26.4473, 15.5231}, {56.5176, 111.2149}, {46.5038, 110.9216},
                {46.5109, 139.7571}, {56.4722, 111.3911}};
        for (int i = 0; i < paths.length; i++) {
            assertPoint(paths[i].get(0), xy[i][0], xy[i][1]);
            assertPoint(paths[i].get(1), xy[i + 1][0], xy[i + 1][1]);
        }
        assertHeading(paths[0].heading(.5), Math.toRadians(270));
        assertHeading(paths[1].heading(.5), Math.atan2(0 - 16.1386, 7.994 - 8.2575));
        assertHeading(paths[2].heading(.5), Math.atan2(15.6271 - 0, 8.2939 - 7.994) + Math.PI);
        assertHeading(paths[3].heading(.5), -Math.PI / 2);
        assertHeading(paths[4].heading(0), -Math.PI / 2);
        assertHeading(paths[4].heading(1), Math.PI / 2);
        for (int i = 5; i < paths.length; i++) assertHeading(paths[i].heading(.5), Math.PI / 2);
        assertPoint(paths[0].get(.5), (56 + 2 * 37.8335 + 8.2575) / 4, (8 + 2 * 44.6525 + 16.1386) / 4);
        assertPoint(paths[4].get(.5), (26.4473 + 2 * 5.9643 + 56.5176) / 4,
                (15.5231 + 2 * 105.7087 + 111.2149) / 4);
        assertPoint(paths[7].get(.5), (46.5109 + 2 * 43.558 + 56.4722) / 4,
                (139.7571 + 2 * 106.1367 + 111.3911) / 4);
        checkFlowerExit(false);
        checkFlowerExit(true);
        checkAutoTurretSearch();
        System.out.println("Passed eight-path geometry, headings, speed, pauses, FLOWER recovery, and registration checks.");
    }

    /** AutoPath's turret sweeps whenever the tag is lost, even though the heading is known. */
    private static void checkAutoTurretSearch() {
        double[] now = {0};
        Robot.TurretAimController aim = new Robot.TurretAimController(
                Robot.TURRET_MIN_DEG, Robot.TURRET_MAX_DEG, () -> now[0], new Robot.TurretGains());
        double held = aim.update(0, false, Double.NaN, 0, true, 90, 0, 0, 0);
        if (held != 0) throw new AssertionError("Default (TeleOp) turret holds its start direction, no sweep");
        Robot.TurretAimController auto = new Robot.TurretAimController(
                Robot.TURRET_MIN_DEG, Robot.TURRET_MAX_DEG, () -> now[0], new Robot.TurretGains());
        auto.searchWhenTagLost = true;
        if (Math.abs(auto.update(0, false, Double.NaN, 0, true, 90, 0, 0, 0)) < Robot.TURRET_SEARCH_POWER - 1e-9) {
            throw new AssertionError("Autonomous turret must sweep for the HIVE before the first tag");
        }
        now[0] = 100;
        if (auto.update(20, true, 10, 1, true, 90, 0, 0, 0) <= 0) {
            throw new AssertionError("A seen tag to the right must be tracked");
        }
        now[0] = 300; // Brief dropout: keep reacquiring the last tag.
        if (auto.update(25, false, Double.NaN, 1, true, 90, 0, 0, 0) <= 0) {
            throw new AssertionError("A brief tag dropout keeps tracking toward the last tag");
        }
        now[0] = 5000; // Tag gone (e.g. robot drove to the 2nd shooting spot): sweep to find it again.
        double after = auto.update(30, false, Double.NaN, 1, true, 90, 0, 0, 0);
        if (Math.abs(Math.abs(after) - Robot.TURRET_SEARCH_POWER) > 1e-9) {
            throw new AssertionError("Once the tag is lost, AutoPath's turret sweeps instead of holding a stale bearing");
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
            tickRoute();
            assertStep(follower, 1);
            arrive(follower, navigation);
            assertStep(follower, 2);
            // No early cutoff: 2.5 in short of point 2 the robot keeps driving.
            navigation.setPose(poses.of(8.0348, 2.4997, -90.9351));
            follower.atEnd = false;
            tickRoute();
            if (follower.idle() || follower.pathsStarted != 2) {
                throw new AssertionError("Path 2 must drive all the way to point 2, not stop 3 in early");
            }
            if (stalled) {
                // Safety: balls block the robot short of point 2 and it stops making progress.
                navigation.setPose(poses.of(8.1313, 8.41, -90.9351));
                follower.atEnd = false;
                tickRoute();
                now[0] += 999_000_000L;
                tickRoute();
                if (follower.idle()) throw new AssertionError("Keep pressing into the wall until the stall window ends");
                now[0] += 1_000_000L;
            } else {
                navigation.setPose(follower.currentPath().endPose());
                follower.atEnd = true;
            }
            int stopsBeforePause = drive.stops;
            tickRoute();
            if (!follower.idle() || drive.stops <= stopsBeforePause) {
                throw new AssertionError("Wall pause must stop the drivetrain");
            }
            now[0] += 999_000_000L;
            tickRoute();
            if (follower.pathsStarted != 2) throw new AssertionError("Path 3 must wait one full second");
            now[0] += 1_000_000L;
            tickRoute();

            for (int step = 3; step <= 5; step++) {
                assertStep(follower, step);
                arrive(follower, navigation);
            }
            if (!follower.idle()) throw new AssertionError("Path 5 must end at a stopped shooting spot");
            now[0] += 3_999_000_000L;
            tickRoute();
            if (follower.pathsStarted != 5) throw new AssertionError("Path 6 must wait four full seconds");
            now[0] += 1_000_000L;
            tickRoute();
            assertStep(follower, 6);
            arrive(follower, navigation);
            assertStep(follower, 7);

            if (stalled) {
                navigation.setPose(poses.of(46.51, 130.0, 90));
                follower.atEnd = true; // Path projection alone cannot establish arrival at FLOWER.
                tickRoute();
                now[0] += 3_900_000_000L;
                tickRoute();
                assertStep(follower, 7);
                navigation.setPose(poses.of(46.51, 131.0, 90));
                tickRoute();
                now[0] += 3_999_000_000L;
                tickRoute();
                assertStep(follower, 7);
                int stopsBeforeEscape = drive.stops;
                now[0] += 1_000_000L;
                tickRoute();
                assertStep(follower, 8);
                if (drive.stops <= stopsBeforeEscape) throw new AssertionError("Stall must stop pushing");
                assertPoint(follower.currentPath().get(0), 46.51, 131.0);
            } else {
                arrive(follower, navigation);
                assertStep(follower, 8);
                assertPoint(follower.currentPath().get(0), 46.5109, 139.7571);
            }
            assertPoint(follower.currentPath().get(1), 56.4722, 111.3911);
            assertHeading(follower.currentPath().heading(.5), Math.PI / 2);
            arrive(follower, navigation);
            if (Scheduler.isScheduled(route)) throw new AssertionError("Path 8 must finish the route");
        } finally {
            Scheduler.cancel(route);
            follower.stop();
            Scheduler.reset();
        }
        if (Constants.foresightConfig.maxPathSpeed.get() != originalSpeed) {
            throw new AssertionError("AutoPath speed must stay scoped to its paths");
        }
    }

    private static void arrive(Tracking follower, Navigation navigation) {
        navigation.setPose(follower.currentPath().endPose());
        follower.atEnd = true;
        tickRoute();
    }

    private static void tickRoute() {
        // A child group can finish one iteration before its parent advances.
        for (int i = 0; i < 3; i++) Scheduler.execute();
    }

    private static void assertStep(Tracking follower, int step) {
        // Only path 2 (into the wall balls) and path 7 (FLOWER intake) run slower than the route speed.
        double expected = step == 2 || step == 7
                ? org.firstinspires.ftc.teamcode.autonomous.BioBuzzAutoConfig.PICKUP_SPEED_FRACTION : AutoPath.SPEED_FRACTION;
        if (follower.pathsStarted != step || Constants.foresightConfig.maxPathSpeed.get() != expected) {
            throw new AssertionError("Expected path " + step + " at " + expected + " speed, saw path "
                    + follower.pathsStarted + " at " + Constants.foresightConfig.maxPathSpeed.get());
        }
    }

    static final class Navigation implements Localizer {
        private Pose pose = AutoPath.path1().get(0);

        @Override public MotionState state() { return MotionState.ofVelocity(pose, Velocity.zero()); }
        @Override public void setPose(Pose pose) { this.pose = pose; }
        @Override public void update() { }
        @Override public void reset() { }
    }

    static final class Drive implements Drivetrain {
        int stops;

        @Override public void drive(DrivePowers powers, boolean manual) { }
        @Override public void stop() { stops++; }
        @Override public void stop(boolean brake) { stop(); }
        @Override public double maxScaling(DrivePowers a, DrivePowers b) { return 1; }
        @Override public double interpolateVelocity(double x, double y, double turn) { return 1; }
        @Override public Map<String, Object> debug() { return Collections.emptyMap(); }
    }

    static final class Tracking extends Follower {
        int pathsStarted;
        boolean atEnd;

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
