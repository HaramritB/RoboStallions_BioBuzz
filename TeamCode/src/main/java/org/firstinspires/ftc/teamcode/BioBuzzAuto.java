package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

// Replace the old org.firstinspires.ftc.teamcode.BioBuzzAuto.java with this file.
// Compiled against the project's FTC SDK 12.0.0 and PedroPathing 3.0.2.
// Drive-only legacy route with no mechanisms or alliance choice: kept out of the Competition group
// so it cannot be picked by mistake at a match. AutoPath / AutoPath V2 are the scoring autos.
@Autonomous(name = "BioBuzz Auto", group = "Development")
public final class BioBuzzAuto extends LinearOpMode {
    private static final PoseFactory P = PoseFactory.degrees();

    private static final Pose START = P.of(57.6323, 9.0657, 90);
    private static final Pose PATH_1_ENTRY = P.of(57.6323, 9.0657, 179.7546);
    private static final Pose POINT_1 = P.of(9.5657, 9.2715, 179.7546);
    private static final Pose POINT_2 = P.of(32.2934, 127.6559, -84.7123);
    private static final Pose PATH_3_ENTRY = P.of(32.2934, 127.6559, 0);
    private static final Pose POINT_3 = P.of(46.9373, 125.3042, 90);
    private static final Pose POINT_5 = P.of(38.0943, 127.3583, -66.9637);

    // Exported point 4 is the intake tip. Pedro tracks the chassis center.
    private static final double INTAKE_REACH_IN = 213.642 / 25.4;
    private static final double FLOWER_HEADING = Math.toRadians(90.572);
    private static final Pose POINT_4 = P.of(
            46.7883 - INTAKE_REACH_IN * Math.cos(FLOWER_HEADING),
            140.2305 - INTAKE_REACH_IN * Math.sin(FLOWER_HEADING),
            90.572);

    private final ElapsedTime runTime = new ElapsedTime();
    private Follower follower;

    @Override
    public void runOpMode() {
        try {
            GoBildaPinpointDriver pinpoint =
                    hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            Follower configured = Constants.create(hardwareMap);
            follower = new Follower(
                    checkedOdometry(configured.localizer, pinpoint),
                    configured.drivetrain,
                    configured.algorithm());
            follower.holdEnd.set(true);

            // Place the chassis at START before INIT and keep it still.
            boolean seeded = false;
            while (opModeInInit()) {
                if (pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY
                        && !seeded) {
                    follower.setPose(START);
                    seeded = true;
                }
                telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
                telemetry.addData("Start", "(57.6323, 9.0657) in, facing +Y (90 deg)");
                telemetry.update();
                idle();
                // Refresh device status while waiting for Pinpoint to become ready.
                configured.localizer.update();
            }

            if (!opModeIsActive()) return;
            if (!seeded) throw new IllegalStateException("Pinpoint was not ready during INIT");
            follower.localizer.update();
            if (!settledAt(START)) throw new IllegalStateException("Re-INIT at the starting pose");
            runTime.reset();

            // Turn away from the wall, then return to the original straight path-1 entry.
            double turnY = Math.hypot(423.17683, 448.7) / (2 * 25.4) + 2.25;
            Pose inward = P.of(START.x(), turnY, 90);
            Pose aligned = P.of(START.x(), turnY, 179.7546);

            if (!follow("Move inward", line(START, inward).constant(START), inward, false)) return;
            if (!turn("Align for path 1", aligned)) return;
            if (!follow("Return to entry", line(aligned, PATH_1_ENTRY).constant(aligned),
                    PATH_1_ENTRY, false)) return;
            if (!follow("Path 1", line(PATH_1_ENTRY, POINT_1).constant(POINT_1),
                    POINT_1, false)) return;

            // Retain the drive-only build's linear headings on paths 2 and 5.
            if (!follow("Path 2", curve(POINT_1, P.of(40.923, 34.4143, 0), POINT_2)
                    .linear(POINT_1, POINT_2), POINT_2, true)) return;
            if (!turn("Turn before path 3", PATH_3_ENTRY)) return;
            if (!follow("Path 3", curve(PATH_3_ENTRY, P.of(40.2631, 108.1664, 0), POINT_3)
                    .linear(PATH_3_ENTRY, POINT_3), POINT_3, true)) return;
            if (!follow("Path 4", line(POINT_3, POINT_4).linear(POINT_3, POINT_4),
                    POINT_4, false)) return;
            if (!follow("Path 5", curve(POINT_4, P.of(45.4429, 110.0765, 0), POINT_5)
                    .linear(POINT_4, POINT_5), POINT_5, false)) return;

            telemetry.addData("Status", "Complete");
            telemetry.update();
        } catch (RuntimeException error) {
            telemetry.addData("Auto stopped", error.getMessage());
            telemetry.update();
        } finally {
            if (follower != null) {
                try {
                    follower.stop();
                } finally {
                    follower.drivetrain.stop();
                }
            }
        }
    }

    private boolean follow(String name, Path path, Pose target, boolean transit) {
        if (!opModeIsActive()) return false;
        follower.follow(path.with(
                Constants.foresightConfig.maxVelocityConstraint.at(transit ? 36.0 : 18.0),
                Constants.foresightConfig.maxPathSpeed.at(transit ? 0.60 : 0.35),
                Constants.foresightConfig.pathSkip.at(false)));
        return awaitArrival(name, target, 15);
    }

    private boolean turn(String name, Pose target) {
        if (!opModeIsActive()) return false;
        follower.hold(target);
        return awaitArrival(name, target, 5);
    }

    private boolean awaitArrival(String name, Pose target, double timeout) {
        ElapsedTime legTime = new ElapsedTime();
        double stableSince = -1;

        while (opModeIsActive() && runTime.seconds() < 27 && legTime.seconds() < timeout) {
            follower.update();
            Pose pose = follower.pose();
            telemetry.addData("Path", name);
            telemetry.addData("Pinpoint pose", "%.2f, %.2f in, %.1f deg",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            telemetry.update();

            if (follower.holding() && settledAt(target)) {
                double now = legTime.seconds();
                if (stableSince < 0) stableSince = now;
                if (now - stableSince >= 0.20) return true;
            } else {
                stableSince = -1;
            }
            idle();
        }

        if (opModeIsActive()) throw new IllegalStateException("Timeout on " + name);
        return false;
    }

    private boolean settledAt(Pose target) {
        Pose pose = follower.pose();
        Velocity velocity = follower.velocity();
        double headingError = target.heading() - pose.heading();
        double wrappedError = Math.atan2(Math.sin(headingError), Math.cos(headingError));

        return Math.hypot(target.x() - pose.x(), target.y() - pose.y()) <= 2.0
                && Math.abs(Math.toDegrees(wrappedError)) <= 3.0
                && Math.hypot(velocity.vx, velocity.vy) <= 2.0
                && Math.abs(Math.toDegrees(velocity.omega)) <= 8.0;
    }

    private static Localizer checkedOdometry(Localizer odometry, GoBildaPinpointDriver pinpoint) {
        return new Localizer() {
            @Override public MotionState state() { return odometry.state(); }
            @Override public void setPose(Pose pose) { odometry.setPose(pose); }
            @Override public void reset() { odometry.reset(); }

            @Override
            public void update() {
                odometry.update();
                if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
                    throw new IllegalStateException("Pinpoint " + pinpoint.getDeviceStatus());
                }

                Pose pose = odometry.pose();
                Velocity velocity = odometry.velocity();
                if (!Double.isFinite(pose.x())
                        || !Double.isFinite(pose.y())
                        || !Double.isFinite(pose.heading())
                        || !Double.isFinite(velocity.vx)
                        || !Double.isFinite(velocity.vy)
                        || !Double.isFinite(velocity.omega)) {
                    throw new IllegalStateException("Invalid Pinpoint sample");
                }
            }
        };
    }
}