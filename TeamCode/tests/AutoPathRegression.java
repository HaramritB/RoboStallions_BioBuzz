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
        double[][] xy = {{54.6728, 423.17683 / 25.4 / 2}, {7.4387, 15.6441}, {7.3833, 0.0881}, {7.2845, 15.5174},
                {26.4473, 15.5231}, {59.3283, 124.8302}, {43.2211, 110.6879},
                {42.0231, 139.2817}, {15.7092, 122.0158}};
        for (int i = 0; i < paths.length; i++) {
            assertPoint(paths[i].get(0), xy[i][0], xy[i][1]);
            assertPoint(paths[i].get(1), xy[i + 1][0], xy[i + 1][1]);
        }
        assertHeading(paths[0].heading(.5), Math.toRadians(270));
        assertHeading(paths[1].heading(.5), Math.atan2(0.0881 - 15.6441, 7.3833 - 7.4387));
        assertHeading(paths[2].heading(.5), Math.atan2(15.5174 - 0.0881, 7.2845 - 7.3833) + Math.PI);
        assertHeading(paths[3].heading(.5), -Math.PI / 2);
        assertHeading(paths[4].heading(0), -Math.PI / 2);
        assertHeading(paths[4].heading(1), Math.PI / 2);
        for (int i = 5; i < paths.length; i++) assertHeading(paths[i].heading(.5), Math.PI / 2);
        assertPoint(paths[0].get(.5), (54.6728 + 2 * 37.8335 + 7.4387) / 4, (423.17683 / 25.4 / 2 + 2 * 44.6525 + 15.6441) / 4);
        assertPoint(paths[4].get(.5), (26.4473 + 2 * 16.3139 + 59.3283) / 4,
                (15.5231 + 2 * 96.4996 + 124.8302) / 4);
        checkFlowerExit(false);
        checkFlowerExit(true);
        checkAutoTurretSearch();
        System.out.println("Passed eight-path geometry (park), FLOWER shuffle, headings, speed, pauses, FLOWER recovery, and registration checks.");
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
                now[0] += AutoPath.WALL_STALL_NS - 1_000_000L;
                tickRoute();
                if (follower.idle()) throw new AssertionError("Keep pressing into the wall until the stall window ends");
                now[0] += 1_000_000L;
            } else {
                navigation.setPose(follower.currentPath().endPose());
                follower.atEnd = true;
            }
            int stopsBeforePause = drive.stops;
            if (AutoPath.WALL_STALL_NS != 700_000_000L || AutoPath.WALL_PAUSE_NS != 700_000_000L
                    || AutoPath.SPEED_FRACTION != 0.90) {
                throw new AssertionError("Wall stall and pause stay 0.7 s; route speed is 90%");
            }
            tickRoute();
            if (!follower.idle() || drive.stops <= stopsBeforePause || follower.pathsStarted != 2) {
                throw new AssertionError("No wall shuffle: path 2's end goes straight to the stopped wall pause");
            }
            now[0] += AutoPath.WALL_PAUSE_NS - 1_000_000L;
            tickRoute();
            if (follower.pathsStarted != 2) throw new AssertionError("Path 3 must wait the full wall pause");
            now[0] += 1_000_000L;
            tickRoute();

            for (int step = 3; step <= 5; step++) {
                assertStep(follower, step);
                arrive(follower, navigation);
            }
            if (!follower.idle()) throw new AssertionError("Path 5 must end at a stopped shooting spot");
            if (AutoPath.FLOWER_STALL_NS != 1_000_000_000L || AutoPath.FLOWER_RPM != 3180 || AutoPath.PARK_LEAVE_BY_S != 27.5) {
                throw new AssertionError("FLOWER stall 1 s; FLOWER volley at 3180 RPM; park leaves at 27.5 s");
            }
            now[0] += 3_999_000_000L;
            tickRoute();
            if (follower.pathsStarted != 5) throw new AssertionError("Path 6 must wait four full seconds");
            now[0] += 1_000_000L;
            tickRoute();
            assertStep(follower, 6);
            arrive(follower, navigation);
            assertStep(follower, 7);

            if (stalled) {
                navigation.setPose(poses.of(42.10, 130.0, 90));
                follower.atEnd = true; // Path projection alone cannot establish arrival at FLOWER.
                tickRoute();
                now[0] += AutoPath.FLOWER_STALL_NS - 100_000_000L;
                tickRoute();
                assertStep(follower, 7);
                navigation.setPose(poses.of(42.10, 131.0, 90));
                tickRoute();
                now[0] += AutoPath.FLOWER_STALL_NS - 1_000_000L;
                tickRoute();
                assertStep(follower, 7);
                int stopsBeforeEscape = drive.stops;
                now[0] += 1_000_000L;
                tickRoute();
                if (drive.stops <= stopsBeforeEscape) throw new AssertionError("Stall must stop pushing");
            } else {
                arrive(follower, navigation);
            }
            Pose atFlower = navigation.state().pose();
            tickRoute();
            // FLOWER shuffle: one short pickup-speed follow from where path 7 stopped.
            double pickupSpeed = org.firstinspires.ftc.teamcode.autonomous.BioBuzzAutoConfig.PICKUP_SPEED_FRACTION;
            if (follower.pathsStarted != 8 || follower.idle() || Constants.foresightConfig.maxPathSpeed.get() != pickupSpeed) {
                throw new AssertionError("The FLOWER shuffle starts at pickup speed right after path 7");
            }
            Path shuffle = follower.currentPath();
            assertPoint(shuffle.get(0), atFlower.x(), atFlower.y());
            double h = atFlower.heading();
            assertPoint(shuffle.endPose(), atFlower.x() + AutoPath.FLOWER_SHUFFLE_PUSH_IN * Math.cos(h),
                    atFlower.y() + AutoPath.FLOWER_SHUFFLE_PUSH_IN * Math.sin(h));
            assertHeading(shuffle.heading(.5), h);
            // Right first, then left (robot-relative; robot right = (sin h, -cos h)).
            double firstSide = -Math.sin(h) * (shuffle.get(0.1).x() - atFlower.x()) + Math.cos(h) * (shuffle.get(0.1).y() - atFlower.y());
            if (firstSide >= 0) throw new AssertionError("The FLOWER shuffle must move RIGHT first, then left");
            double maxLeft = 0, maxRight = 0, maxBack = 0;
            for (int i = 0; i <= 400; i++) {
                Pose q = shuffle.get(i / 400.0);
                double dx = q.x() - atFlower.x(), dy = q.y() - atFlower.y();
                double left = -Math.sin(h) * dx + Math.cos(h) * dy;
                maxLeft = Math.max(maxLeft, left); maxRight = Math.max(maxRight, -left);
                maxBack = Math.max(maxBack, -(Math.cos(h) * dx + Math.sin(h) * dy));
            }
            if (Math.abs(maxLeft - AutoPath.FLOWER_SHUFFLE_LEFT_IN) > 0.05 || Math.abs(maxRight - AutoPath.FLOWER_SHUFFLE_RIGHT_IN) > 0.05
                    || Math.abs(maxBack - AutoPath.FLOWER_SHUFFLE_BACK_IN) > 0.05 || maxLeft > 2 || maxRight > 2 || maxBack > 2) {
                throw new AssertionError("The FLOWER shuffle is slight: " + maxLeft + " / " + maxRight + " / " + maxBack);
            }
            // Pressed into the FLOWER: no movement, so the shuffle ends after its stall window.
            now[0] += AutoPath.FLOWER_SHUFFLE_STALL_NS - 1_000_000L;
            tickRoute();
            if (follower.pathsStarted != 8 || follower.idle()) throw new AssertionError("Keep shuffling until the stall window ends");
            now[0] += 1_000_000L;
            tickRoute();
            assertStep(follower, 8);
            assertPoint(follower.currentPath().get(0), atFlower.x(), atFlower.y());
            assertPoint(follower.currentPath().get(1), 15.7092, 122.0158);
            assertHeading(follower.currentPath().heading(.5), Math.PI / 2);
            arrive(follower, navigation);
            if (!follower.idle() || !Scheduler.isScheduled(route)) {
                throw new AssertionError("Drive-only parks with path 8 from the FLOWER, then holds still");
            }
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

    /** step = route path number 1-8 (one follow per path, plus the FLOWER shuffle between 7 and 8). */
    private static void assertStep(Tracking follower, int step) {
        // Only path 2 (into the wall balls) and path 7 (FLOWER intake) run slower than the route speed.
        double expected = step == 2 || step == 7
                ? org.firstinspires.ftc.teamcode.autonomous.BioBuzzAutoConfig.PICKUP_SPEED_FRACTION : AutoPath.SPEED_FRACTION;
        int started = step >= 8 ? step + 1 : step; // The FLOWER shuffle is one extra follow before path 8.
        if (follower.pathsStarted != started || Constants.foresightConfig.maxPathSpeed.get() != expected) {
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
