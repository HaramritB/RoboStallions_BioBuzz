package org.firstinspires.ftc.teamcode.v2;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.FlowerLever;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.autonomous.BioBuzzAutoConfig;
import org.firstinspires.ftc.teamcode.autonomous.BioBuzzAutoMechanisms;

import java.util.function.LongSupplier;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/**
 * V2 of AutoPath: the same eight-path route, speeds and mechanism commands, plus
 *  - RED: the blue geometry mirrored by FieldMirrorV2 (not yet driven on a field!)
 *  - turret pre-aims at the HIVE from odometry instead of sweeping at every shooting spot
 *  - no fixed four-second stop at the second spot; bursts can end early once all balls leave
 *  - V2 aiming/shooter (upward CELL, filtered distance, voltage-compensated F)
 *  - pose + HIVE target handed to the V2 TeleOp
 * Separate Blue and Red OpModes, so the alliance cannot be left on the wrong setting.
 */
public abstract class AutoPathV2 extends LinearOpMode {
    private static final double SPEED_FRACTION = 0.70;
    private static final long WALL_STALL_NS = 1_000_000_000L;
    private static final long WALL_PAUSE_NS = 1_000_000_000L;
    /** Original: 4 s minimum stop at the second spot even when the volley finished early. */
    private static final long SECOND_SHOT_MIN_PAUSE_NS = 0L;
    private static final double FLOWER_ARRIVAL_IN = 1.0;
    private static final double APPROACH_PROGRESS_IN = 0.25;
    private static final long FLOWER_STALL_NS = 4_000_000_000L;
    private static final double TELEMETRY_PERIOD_MS = 100;
    private static final PoseFactory POSES = PoseFactory.degrees();

    protected abstract Robot.Alliance alliance();

    /** The original AutoPath geometry (BLUE), mirrored for RED. */
    static final class Geometry {
        final Pose start, p1, p2, p3, p4, p5, p6, p7, p8;
        final Pose c1, c5, c8;
        private final Robot.Alliance alliance;

        Geometry(Robot.Alliance alliance) {
            this.alliance = alliance;
            start = pose(56, 8, 270);
            p1 = pose(8.2575, 16.1386, 270);
            p2 = pose(7.994, 0, -90.9351);
            p3 = pose(8.2939, 15.6271, -91.0991);
            p4 = pose(26.4473, 15.5231, -90);
            p5 = pose(56.5176, 111.2149, 90);
            p6 = pose(46.5038, 110.9216, 90);
            p7 = pose(46.5109, 139.7571, 90);
            p8 = pose(56.4722, 111.3911, 90);
            c1 = pose(37.8335, 44.6525, 0);
            c5 = pose(5.9643, 105.7087, 0);
            c8 = pose(43.558, 106.1367, 0);
        }

        private Pose pose(double x, double y, double headingDeg) {
            return POSES.of(FieldMirrorV2.x(x, y, alliance), FieldMirrorV2.y(x, y, alliance),
                    FieldMirrorV2.headingDeg(headingDeg, alliance));
        }

        Path path1() { return curve(start, c1, p1).linear(start, p1); }
        Path path2() { return line(p1, p2).tangent(); }
        Path path3() { return line(p2, p3).reverseTangent(); }
        Path path4() { return line(p3, p4).constant(p4); }
        Path path5() { return curve(p4, c5, p5).linear(p4, p5); }
        Path path6() { return line(p5, p6).constant(p6); }
        Path path7() { return line(p6, p7).constant(p7); }
        Path path8(Pose departure) { return curve(departure, c8, p8).constant(p8); }
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        Robot.Alliance alliance = alliance();
        Geometry g = new Geometry(alliance);
        Follower follower = Constants.create(hardwareMap);
        RobotV2 v2 = new RobotV2();
        BioBuzzAutoMechanisms mechanisms = new BioBuzzAutoMechanisms();
        FlowerLever flowerLever = null;
        Command route = null;
        RuntimeException primaryFailure = null;
        boolean mechanismsReady = false;
        double lastTelemetryMs = Double.NEGATIVE_INFINITY;
        double lastHandoffMs = Double.NEGATIVE_INFINITY;

        try {
            follower.setPose(g.start);
            flowerLever = new FlowerLever(hardwareMap.get(Servo.class, "flower"));
            flowerLever.setDeployed(false); // Flower OFF as soon as INIT is pressed.
            v2.initMechanisms(hardwareMap);
            requireHardware(v2);
            v2.setAlliance(alliance);
            Robot.lastAlliance = alliance;
            v2.setHiveGuessForAlliance();
            mechanisms = new BioBuzzAutoMechanisms(new AutoHardwareV2(v2, System::nanoTime));
            mechanismsReady = true;
            mechanisms.setFaultTolerant(true);
            mechanisms.searchForTagWhenLost(true); // Only sweeps if the pre-aim guess shows no tag.
            mechanisms.manualIntake(false); // Motors idle; no INIT gate position command.

            while (opModeInInit()) {
                v2.beginLoop();
                if (gamepad1.backWasPressed()) {
                    mechanisms.zeroTurret();
                    v2.aim.clearTarget();
                }
                follower.localizer.update(); // Read only; never run the drive controller in INIT.
                feedPose(follower, v2);
                mechanisms.observe();
                flowerLever.refresh(); // Keep the INIT OFF command powered.
                if (RobotV2.nowMs() - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
                    lastTelemetryMs = RobotV2.nowMs();
                    publish(follower, mechanisms, v2, alliance);
                }
                idle();
            }
            waitForStart();
            if (!opModeIsActive()) return;

            mechanisms.keepShooterReady(true);
            mechanisms.keepIntakeRunning(true);
            mechanisms.observe();
            flowerLever.setDeployed(false);
            route = route(follower, System::nanoTime, mechanisms, flowerLever, g);
            Scheduler.schedule(route);
            while (opModeIsActive() && Scheduler.isScheduled(route)) {
                v2.beginLoop();
                follower.update();
                feedPose(follower, v2);
                mechanisms.update();
                Scheduler.execute();
                flowerLever.refresh();
                double now = RobotV2.nowMs();
                if (now - lastHandoffMs >= 250) {
                    lastHandoffMs = now;
                    PoseHandoffV2.save(v2.xIn, v2.yIn, v2.headingRad, v2.aim, alliance);
                }
                if (now - lastTelemetryMs >= TELEMETRY_PERIOD_MS) {
                    lastTelemetryMs = now;
                    publish(follower, mechanisms, v2, alliance);
                }
                idle();
            }
        } catch (RuntimeException error) {
            primaryFailure = error;
            telemetry.addData("AutoPath V2 failure", "%s: %s", error.getClass().getSimpleName(), error.getMessage());
            telemetry.update();
            requestOpModeStop();
            throw error;
        } finally {
            try {
                Pose last = follower.pose();
                PoseHandoffV2.save(last.x(), last.y(), last.heading(), v2.aim, alliance);
            } catch (RuntimeException ignored) {
                // The handoff is optional; never mask the real failure.
            }
            // A failed hardware check leaves no mechanism adapter to stop the camera.
            if (!mechanismsReady && v2.robot.Limelight != null) {
                try { v2.robot.Limelight.stop(); } catch (RuntimeException ignored) { }
            }
            try { stopAll(follower, mechanisms, route, flowerLever); }
            catch (RuntimeException cleanupError) {
                if (primaryFailure != null) primaryFailure.addSuppressed(cleanupError);
                else throw cleanupError;
            }
        }
    }

    private static void requireHardware(RobotV2 v2) {
        if (v2.robot.ballDistanceSensor == null) throw new IllegalStateException("Ball distance sensor (colorsensor) required");
        if (v2.robot.Limelight == null) throw new IllegalStateException("Limelight required for the shot sequence");
        if (!v2.robot.gateCalibrated()) throw new IllegalStateException("Run Gate Calibration: saved gate endpoints are invalid");
    }

    private static void feedPose(Follower follower, RobotV2 v2) {
        Pose pose = follower.pose();
        double omega = follower.velocity() != null ? follower.velocity().omega : 0;
        v2.setPose(pose.x(), pose.y(), pose.heading(), omega, true);
    }

    private void publish(Follower follower, BioBuzzAutoMechanisms mechanisms, RobotV2 v2, Robot.Alliance alliance) {
        telemetry.addData("Mode", "AutoPath V2 %s%s", alliance,
                alliance == Robot.Alliance.RED ? " (mirrored; verify on a field first)" : "");
        telemetry.addData("Turret zero", "INIT: point turret forward, then gamepad 1 BACK");
        telemetry.addData("Mechanisms", mechanisms.state());
        telemetry.addData("Shooter / aiming / gate", mechanisms.diagnostics());
        telemetry.addData("Mechanism failure", mechanisms.failure().isEmpty() ? "none" : mechanisms.failure());
        telemetry.addData("Mechanism warning", mechanisms.warning().isEmpty() ? "none" : mechanisms.warning());
        v2.addTelemetry(telemetry);
        if (follower.currentPath() != null) {
            telemetry.addData("Distance remaining (in)", "%.2f", follower.distanceToEndpoint());
        }
        telemetry.update();
    }

    static Command route(Follower follower, LongSupplier clock, BioBuzzAutoMechanisms mechanisms,
                         FlowerLever flowerLever, Geometry g) {
        Approach wall = new Approach(g.p2, 0, WALL_STALL_NS);
        Approach flower = new Approach(g.p7, FLOWER_ARRIVAL_IN, FLOWER_STALL_NS);
        Command wallPickup = sequential(
                follow(follower, atRouteSpeed(g.path1())),
                approach(follower, clock, atPickupSpeed(g.path2()), wall, true));
        Command toSecondShot = sequential(
                follow(follower, atRouteSpeed(g.path3())),
                follow(follower, atRouteSpeed(g.path4())),
                follow(follower, atRouteSpeed(g.path5())));
        Command toFlower = sequential(
                follow(follower, atRouteSpeed(g.path6())),
                approach(follower, clock, atPickupSpeed(g.path7()), flower, false));
        Command exit = lazy(() -> follow(follower, atRouteSpeed(g.path8(flower.departure))));
        Command collectAtFlower = sequential(
                intakeAlong(mechanisms, toFlower, follower),
                parallel(pauseDrive(follower, clock, 0), mechanisms.command(
                        BioBuzzAutoMechanisms.Action.intakeFor(BioBuzzAutoConfig.FLOWER_INTAKE_SECONDS,
                                BioBuzzAutoConfig.COLLECTION_TIMEOUT_S), clock, AutoPathV2::mechanismFailure)));
        Command volley2 = stoppedShot(follower, clock, mechanisms, BioBuzzAutoConfig.STAGED_BALL_COUNT,
                SECOND_SHOT_MIN_PAUSE_NS);
        // Flower deploys on arrival at the second spot and stays deployed (as the original).
        if (flowerLever != null) volley2 = parallel(instant(() -> flowerLever.setDeployed(true)), volley2);
        return sequential(
                stoppedShot(follower, clock, mechanisms, BioBuzzAutoConfig.PRELOAD_BALL_COUNT, 0),
                intakeAlong(mechanisms, wallPickup, follower),
                parallel(pauseDrive(follower, clock, WALL_PAUSE_NS), mechanisms.command(
                        BioBuzzAutoMechanisms.Action.intakeFor(WALL_PAUSE_NS / 1e9,
                                BioBuzzAutoConfig.COLLECTION_TIMEOUT_S), clock, AutoPathV2::mechanismFailure)),
                toSecondShot,
                volley2,
                collectAtFlower,
                exit,
                stoppedShot(follower, clock, mechanisms, BioBuzzAutoConfig.FLOWER_BALL_COUNT, 0));
    }

    private static Command stoppedShot(Follower follower, LongSupplier clock, BioBuzzAutoMechanisms mechanisms,
                                       int count, long minimumPauseNs) {
        return parallel(pauseDrive(follower, clock, minimumPauseNs), mechanisms.command(
                BioBuzzAutoMechanisms.Action.shootFor(count, BioBuzzAutoConfig.SHOOT_BURST_SECONDS,
                        BioBuzzAutoConfig.SHOOT_SPINUP_TIMEOUT_S),
                clock, AutoPathV2::mechanismFailure));
    }

    private static Command intakeAlong(BioBuzzAutoMechanisms mechanisms, Command movement, Follower follower) {
        boolean[] active = {false};
        return Command.build().requiring(follower, mechanisms)
                .setStart(() -> { active[0] = true; mechanisms.manualIntake(true); movement.start(); })
                .setExecute(movement::execute).setDone(movement::done)
                .setEnd(reason -> {
                    if (!active[0]) return;
                    active[0] = false;
                    try { movement.end(reason); }
                    finally { mechanisms.manualIntake(false); }
                });
    }

    private static void mechanismFailure(String reason) {
        throw new IllegalStateException(reason);
    }

    static void stopAll(Follower follower, BioBuzzAutoMechanisms mechanisms, Command route, FlowerLever flower) {
        try { mechanisms.stop(); }
        finally {
            try { if (route != null) Scheduler.cancel(route); }
            finally {
                Scheduler.reset();
                try { if (flower != null) flower.stop(); }
                finally { stopDrive(follower); }
            }
        }
    }

    private static Path atRouteSpeed(Path path) {
        return path.with(Constants.foresightConfig.maxPathSpeed.at(SPEED_FRACTION));
    }

    private static Path atPickupSpeed(Path path) {
        return path.with(Constants.foresightConfig.maxPathSpeed.at(BioBuzzAutoConfig.PICKUP_SPEED_FRACTION),
                Constants.foresightConfig.maxVelocityConstraint.at(BioBuzzAutoConfig.PICKUP_MAX_VELOCITY_IN_S));
    }

    private static Command pauseDrive(Follower follower, LongSupplier clock, long durationNs) {
        long[] started = {0};
        return Command.build().setStart(() -> {
            stopDrive(follower);
            started[0] = clock.getAsLong();
        }).setDone(() -> clock.getAsLong() - started[0] >= durationNs);
    }

    private static void stopDrive(Follower follower) {
        try {
            follower.stop();
        } finally {
            follower.drivetrain.stop();
        }
    }

    private static Command approach(Follower follower, LongSupplier clock, Path path, Approach watch,
                                    boolean pedroEndCounts) {
        return Command.build().setStart(() -> {
            follower.follow(path);
            watch.start(follower.pose(), clock.getAsLong());
        }).setDone(() -> (pedroEndCounts && (follower.atParametricEnd() || follower.holding()))
                || watch.finished(follower, clock.getAsLong()));
    }

    private static final class Approach {
        private final Pose target;
        private final double arrivalIn;
        private final long stallNs;
        private Pose departure;
        private double closestDistance;
        private long lastProgress;

        Approach(Pose target, double arrivalIn, long stallNs) {
            this.target = target;
            this.arrivalIn = arrivalIn;
            this.stallNs = stallNs;
            departure = target;
        }

        void start(Pose pose, long now) {
            departure = target;
            closestDistance = pose.distance(target);
            lastProgress = now;
        }

        boolean finished(Follower follower, long now) {
            Pose pose = follower.pose();
            double distance = pose.distance(target);
            if (!Double.isFinite(distance) || !Double.isFinite(pose.heading())) {
                throw new IllegalStateException("Invalid Pinpoint pose approaching " + target);
            }
            if (distance <= arrivalIn) return true;
            if (closestDistance - distance >= APPROACH_PROGRESS_IN) {
                closestDistance = distance;
                lastProgress = now;
            }
            if (now - lastProgress < stallNs) return false;
            departure = pose;
            stopDrive(follower);
            return true;
        }
    }
}
