package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
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

/** BioBuzz scoring using the field-tested AutoPath route and shared TeleOp mechanisms. */
@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends LinearOpMode {
    private static final double SPEED_FRACTION = 0.70;
    private static final double WALL_ARRIVAL_IN = 3.0;
    // Path 2 aims ~1.4 in past the wall; if balls stop the robot farther out, continue after this.
    private static final long WALL_STALL_NS = 1_000_000_000L;
    private static final long WALL_PAUSE_NS = 1_000_000_000L;
    private static final long SECOND_SHOT_PAUSE_NS = 4_000_000_000L;
    private static final double FLOWER_ARRIVAL_IN = 1.0;
    private static final double APPROACH_PROGRESS_IN = 0.25;
    private static final long FLOWER_STALL_NS = 4_000_000_000L;
    private static final PoseFactory POSES = PoseFactory.degrees();
    private static final Pose START = POSES.of(56, 8, 270);
    private static final Pose POINT_1 = POSES.of(8.2575, 16.1386, 270);
    private static final Pose POINT_2 = POSES.of(8.2853, 7.0319, -89.8249);
    private static final Pose POINT_3 = POSES.of(8.2939, 15.6271, -90.0571);
    private static final Pose POINT_4 = POSES.of(26.4473, 15.5231, -90);
    private static final Pose POINT_5 = POSES.of(56.5176, 111.2149, 90);
    private static final Pose POINT_6 = POSES.of(46.5038, 110.9216, 90);
    private static final Pose POINT_7 = POSES.of(46.5109, 139.7571, 90);
    private static final Pose POINT_8 = POSES.of(56.4722, 111.3911, 90);

    @Override
    public void runOpMode() {
        Scheduler.reset();
        Follower follower = Constants.create(hardwareMap);
        BioBuzzAutoMechanisms mechanisms = new BioBuzzAutoMechanisms();
        FlowerLever flowerLever = null;
        Command route = null;
        Robot.Alliance alliance = Robot.lastAlliance;
        RuntimeException primaryFailure = null;

        try {
            follower.setPose(START);
            if (useMechanisms()) {
                flowerLever = new FlowerLever(hardwareMap.get(Servo.class, "flower"));
                mechanisms = BioBuzzAutoMechanisms.initialize(hardwareMap, alliance,
                        () -> -Math.toDegrees(follower.pose().heading()),
                        () -> -Math.toDegrees(follower.velocity().omega));
                // A sensor glitch or slow volley is reported and skipped; it never ends the run.
                mechanisms.setFaultTolerant(true);
                mechanisms.manualIntake(false); // Motors idle; no INIT gate position command.
            }
            while (opModeInInit()) {
                if (useMechanisms()) {
                    if (gamepad1.aWasPressed()) alliance = Robot.Alliance.BLUE;
                    if (gamepad1.bWasPressed()) alliance = Robot.Alliance.RED;
                    if (gamepad1.backWasPressed()) mechanisms.zeroTurret();
                    mechanisms.setAlliance(alliance);
                }
                follower.localizer.update(); // Read only; never run the drive controller in INIT.
                mechanisms.observe();
                publish(follower, mechanisms, alliance);
                idle();
            }
            waitForStart();
            if (!opModeIsActive()) return;

            if (useMechanisms()) {
                Robot.lastAlliance = alliance;
                // Like TeleOp: flywheel at calibrated RPM and turret tracking for the whole run,
                // so every volley starts warm and aimed instead of from zero after stopping.
                mechanisms.keepShooterReady(true);
            }
            mechanisms.observe();
            if (flowerLever != null) flowerLever.setDeployed(false);
            route = useMechanisms() ? route(follower, System::nanoTime, mechanisms, flowerLever)
                    : route(follower, System::nanoTime);
            Scheduler.schedule(route);
            while (opModeIsActive() && Scheduler.isScheduled(route)) {
                follower.update();
                mechanisms.update(); // Use the chassis sample from Pedro; no second Pinpoint update.
                Scheduler.execute();
                if (flowerLever != null) flowerLever.refresh(); // Re-enable PWM if the hub dropped it.
                publish(follower, mechanisms, alliance);
                idle();
            }
        } catch (RuntimeException error) {
            primaryFailure = error;
            telemetry.addData("AutoPath failure", "%s: %s", error.getClass().getSimpleName(), error.getMessage());
            telemetry.update();
            requestOpModeStop();
            throw error; // Let Driver Station show the original cause, not a cleanup exception.
        } finally {
            try { stopAll(follower, mechanisms, route, flowerLever); }
            catch (RuntimeException cleanupError) {
                if (primaryFailure != null) primaryFailure.addSuppressed(cleanupError);
                else throw cleanupError;
            }
        }
    }

    protected boolean useMechanisms() { return true; }

    private void publish(Follower follower, BioBuzzAutoMechanisms mechanisms, Robot.Alliance alliance) {
        telemetry.addData("Mode", useMechanisms() ? "BioBuzz FTC 2026: shoot + intake" : "Drive only");
        if (useMechanisms()) {
            telemetry.addData("Alliance", "%s (INIT: A blue / B red)", alliance);
            telemetry.addData("Turret zero", "INIT: point turret forward, then BACK to zero");
            telemetry.addData("Mechanisms", mechanisms.state());
            telemetry.addData("Shooter / aiming / gate", mechanisms.diagnostics());
            telemetry.addData("Feed RPM: start / target / minimum", mechanisms.feedRpmTelemetry());
            telemetry.addData("Mechanism failure", mechanisms.failure().isEmpty() ? "none" : mechanisms.failure());
            telemetry.addData("Mechanism warning", mechanisms.warning().isEmpty() ? "none" : mechanisms.warning());
        }
        Pose pose = follower.pose();
        telemetry.addData("X (in)", "%.2f", pose.x());
        telemetry.addData("Y (in)", "%.2f", pose.y());
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(pose.heading()));
        if (follower.currentPath() != null) telemetry.addData("Distance remaining (in)", "%.2f", follower.distanceToEndpoint());
        telemetry.update();
    }

    static Command route(Follower follower, LongSupplier clock) {
        return route(follower, clock, null);
    }

    static Command route(Follower follower, LongSupplier clock, BioBuzzAutoMechanisms mechanisms) {
        return route(follower, clock, mechanisms, null);
    }

    static Command route(Follower follower, LongSupplier clock, BioBuzzAutoMechanisms mechanisms, FlowerLever flowerLever) {
        // Every segment runs at 70% except the two that drive into balls: path 2 (into the wall
        // balls) and path 7 (FLOWER intake). Drive-only uses the same speeds so it rehearses the run.
        Approach wall = new Approach(POINT_2, WALL_ARRIVAL_IN, WALL_STALL_NS);
        Approach flower = new Approach(POINT_7, FLOWER_ARRIVAL_IN, FLOWER_STALL_NS);
        Command wallPickup = sequential(
                follow(follower, atRouteSpeed(path1())),
                approach(follower, clock, atPickupSpeed(path2()), wall, true));
        Command toSecondShot = sequential(
                follow(follower, atRouteSpeed(path3())),
                follow(follower, atRouteSpeed(path4())),
                follow(follower, atRouteSpeed(path5())));
        Command toFlower = sequential(
                follow(follower, atRouteSpeed(path6())),
                approach(follower, clock, atPickupSpeed(path7()), flower, false));
        Command exit = lazy(() -> follow(follower, atRouteSpeed(path8(flower.departure))));
        if (mechanisms == null) return sequential(
                wallPickup, pauseDrive(follower, clock, WALL_PAUSE_NS),
                toSecondShot, pauseDrive(follower, clock, SECOND_SHOT_PAUSE_NS),
                toFlower, exit);
        // Intake starts when volley 2 ends and runs through the FLOWER approach and pickup window.
        Command collectAtFlower = sequential(
                intakeAlong(follower, mechanisms, toFlower),
                // Stop pushing during pickup, including a recovered approach stall.
                parallel(pauseDrive(follower, clock, 0), mechanisms.command(
                        BioBuzzAutoMechanisms.Action.intakeFor(BioBuzzAutoConfig.FLOWER_INTAKE_SECONDS,
                                BioBuzzAutoConfig.COLLECTION_TIMEOUT_S), clock, AutoPath::mechanismFailure)));
        Command volley2 = stoppedShot(follower, clock, mechanisms, BioBuzzAutoConfig.STAGED_BALL_COUNT, SECOND_SHOT_PAUSE_NS);
        // Flower stays retracted for the wall pickup, deploys on arrival at the 2nd shooting spot,
        // and stays deployed for the rest of the route. Only STOP/cleanup retracts it.
        if (flowerLever != null) volley2 = parallel(instant(() -> flowerLever.setDeployed(true)), volley2);
        return sequential(
                stoppedShot(follower, clock, mechanisms, BioBuzzAutoConfig.PRELOAD_BALL_COUNT, 0),
                intakeAlong(follower, mechanisms, wallPickup),
                // Keep intaking through the one-second stop: arrival is declared 3 in early, and the
                // last wall ball is still being pulled in when the chassis stops.
                parallel(pauseDrive(follower, clock, WALL_PAUSE_NS), mechanisms.command(
                        BioBuzzAutoMechanisms.Action.intakeFor(WALL_PAUSE_NS / 1e9,
                                BioBuzzAutoConfig.COLLECTION_TIMEOUT_S), clock, AutoPath::mechanismFailure)),
                toSecondShot,
                volley2, // Flower deploys here and stays on.
                collectAtFlower,
                exit,
                stoppedShot(follower, clock, mechanisms, BioBuzzAutoConfig.FLOWER_BALL_COUNT, 0));
    }

    private static Command stoppedShot(Follower follower, LongSupplier clock, BioBuzzAutoMechanisms mechanisms,
                                       int count, long minimumPauseNs) {
        // Parallel waits for both children: retain the tested four-second stop even if shots finish early.
        return parallel(pauseDrive(follower, clock, minimumPauseNs), mechanisms.command(
                BioBuzzAutoMechanisms.Action.shootFor(count, BioBuzzAutoConfig.SHOOT_BURST_SECONDS,
                        BioBuzzAutoConfig.SHOOT_SPINUP_TIMEOUT_S),
                clock, AutoPath::mechanismFailure));
    }

    private static Command intakeAlong(Follower follower, BioBuzzAutoMechanisms mechanisms, Command movement) {
        boolean[] active = {false};
        return Command.build().requiring(follower, mechanisms)
                .setStart(() -> { active[0] = true; mechanisms.manualIntake(true); movement.start(); })
                .setExecute(movement::execute).setDone(movement::done)
                .setEnd(reason -> {
                    if (!active[0]) return; // Ivy may end children that have not started.
                    active[0] = false;
                    try { movement.end(reason); }
                    finally { mechanisms.manualIntake(false); }
                });
    }

    private static void mechanismFailure(String reason) {
        // Abort the composition immediately; never let a failed volley start the next path.
        throw new IllegalStateException(reason);
    }

    static void stopAll(Follower follower, BioBuzzAutoMechanisms mechanisms, Command route) {
        stopAll(follower, mechanisms, route, null);
    }

    static void stopAll(Follower follower, BioBuzzAutoMechanisms mechanisms, Command route, FlowerLever flower) {
        // Mark mechanisms closed before Ivy runs interruption callbacks. A normal STOP or
        // cleanup after a sensor/aim failure is not a second failed shooting command.
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

    /**
     * Follows a segment that drives into balls. It finishes on Pedro's end signal (when allowed),
     * within the arrival radius, or after the stall window passes with less than 0.25 in of new
     * progress, so balls or a field element blocking the robot can never hang the route.
     */
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
            this.target = target; this.arrivalIn = arrivalIn; this.stallNs = stallNs;
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

            // Rebase the exit so Pedro can back away without chasing the blocked endpoint.
            departure = pose;
            stopDrive(follower);
            return true;
        }
    }

    static Path path1() {
        return curve(START, POSES.of(30.9563, 74.1493, 0), POINT_1).linear(START, POINT_1);
    }

    static Path path2() {
        return line(POINT_1, POINT_2).tangent();
    }

    static Path path3() {
        return line(POINT_2, POINT_3).reverseTangent();
    }

    static Path path4() {
        return line(POINT_3, POINT_4).constant(POINT_4);
    }

    static Path path5() {
        return curve(POINT_4, POSES.of(12.3536, 107.7844, 0), POINT_5).linear(POINT_4, POINT_5);
    }

    static Path path6() {
        return line(POINT_5, POINT_6).constant(POINT_6);
    }

    static Path path7() {
        return line(POINT_6, POINT_7).constant(POINT_7);
    }

    static Path path8() {
        return path8(POINT_7);
    }

    private static Path path8(Pose departure) {
        return curve(departure, POSES.of(43.558, 106.1367, 0), POINT_8).constant(POINT_8);
    }
}
