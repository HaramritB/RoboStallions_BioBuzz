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
    private static final double PATH_1_ARRIVAL_IN = 3.0;
    private static final long PATH_1_PAUSE_NS = 1_000_000_000L;
    private static final long PATH_2_PAUSE_NS = 4_000_000_000L;
    private static final double FLOWER_ARRIVAL_IN = 1.0;
    private static final double FLOWER_PROGRESS_IN = 0.25;
    private static final long FLOWER_STALL_NS = 4_000_000_000L;
    private static final PoseFactory POSES = PoseFactory.degrees();
    private static final Pose START = POSES.of(57.6323, 9.0657, 90);
    private static final Pose POINT_1 = POSES.of(9.5657, 9.2715, 179.7546);
    private static final Pose POINT_2 = POSES.of(32.4082, 122.9979, -88.7957);
    private static final Pose POINT_3 = POSES.of(46.9373, 125.3042, 90);
    private static final Pose POINT_4 = POSES.of(46.7883, 140.2305, 90.572);
    private static final Pose POINT_5 = POSES.of(38.0943, 127.3583, -66.9637);

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
        FlowerApproach flower = new FlowerApproach();
        Path transit = path2();
        Path approach = atRouteSpeed(path4());
        Command pickup = follow(follower, atRouteSpeed(path1())).setDone(() ->
                        follower.atParametricEnd() || follower.holding()
                                || follower.pose().distance(POINT_1) <= PATH_1_ARRIVAL_IN);
        Command flowerPickup = Command.build().setStart(() -> {
            follower.follow(approach);
            flower.start(follower.pose(), clock.getAsLong());
        }).setDone(() -> flower.finished(follower, clock.getAsLong()));
        Command exit = lazy(() -> follow(follower, atRouteSpeed(path5(flower.departure))));
        if (mechanisms == null) return sequential(
                pickup,
                pauseDrive(follower, clock, PATH_1_PAUSE_NS),
                follow(follower, atRouteSpeed(transit)),
                pauseDrive(follower, clock, PATH_2_PAUSE_NS),
                follow(follower, atRouteSpeed(path3(transit.endPose()))),
                flowerPickup, exit);
        // The second intake cycle starts the moment volley 2 ends: intake running through path 3,
        // the FLOWER approach, and the stopped pickup window.
        Command collectAtFlower = sequential(
                intakeAlong(follower, mechanisms, sequential(
                        follow(follower, atRouteSpeed(path3(transit.endPose()))), flowerPickup)),
                // Stop pushing during pickup, including a recovered approach stall.
                parallel(pauseDrive(follower, clock, 0), mechanisms.command(
                        BioBuzzAutoMechanisms.Action.intakeFor(BioBuzzAutoConfig.FLOWER_INTAKE_SECONDS,
                                BioBuzzAutoConfig.COLLECTION_TIMEOUT_S), clock, AutoPath::mechanismFailure)));
        Command volley2 = stoppedShot(follower, clock, mechanisms, BioBuzzAutoConfig.STAGED_BALL_COUNT, PATH_2_PAUSE_NS);
        // Flower stays retracted for the wall pickup, deploys on arrival at the 2nd shooting spot,
        // and stays deployed for the rest of the route. Only STOP/cleanup retracts it.
        if (flowerLever != null) volley2 = parallel(instant(() -> flowerLever.setDeployed(true)), volley2);
        // Same line and arrival test as the drive-only route, but slow enough for the intake to
        // grab each wall ball instead of batting it away at 70% route speed.
        Command wallPickup = follow(follower, atPickupSpeed(path1())).setDone(() ->
                follower.atParametricEnd() || follower.holding()
                        || follower.pose().distance(POINT_1) <= PATH_1_ARRIVAL_IN);
        return sequential(
                stoppedShot(follower, clock, mechanisms, BioBuzzAutoConfig.PRELOAD_BALL_COUNT, 0),
                intakeAlong(follower, mechanisms, wallPickup),
                // Keep intaking through the one-second stop: arrival is declared 3 in early, and the
                // last wall ball is still being pulled in when the chassis stops.
                parallel(pauseDrive(follower, clock, PATH_1_PAUSE_NS), mechanisms.command(
                        BioBuzzAutoMechanisms.Action.intakeFor(PATH_1_PAUSE_NS / 1e9,
                                BioBuzzAutoConfig.COLLECTION_TIMEOUT_S), clock, AutoPath::mechanismFailure)),
                follow(follower, atRouteSpeed(transit)),
                volley2, // Flower deploys here and stays on.
                collectAtFlower, // Path 3 + path 4 + FLOWER pickup.
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
            stopDrive(follower);
            return true;
        }
    }

    static Path path1() {
        return line(START, POINT_1).tangent();
    }

    static Path path2() {
        // Trim the wider curve before FLOWER so the chassis stops about 8 inches clear.
        return curve(POINT_1, POSES.of(34.2837, 33.7804, 0), POINT_2).reverseTangent();
    }

    static Path path3(Pose start) {
        return curve(start, POSES.of(40.2631, 108.1664, 0), POINT_3).linear(start, POINT_3);
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
