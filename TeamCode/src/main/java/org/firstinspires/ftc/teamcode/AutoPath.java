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
import static com.pedropathing.api.Paths.path;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/** BioBuzz scoring using the field-tested AutoPath route and shared TeleOp mechanisms. */
@Autonomous(name = "AutoPath", group = "Autonomous")
public class AutoPath extends LinearOpMode {
    // Route segments (all but the two ball pickups). Was 1.0, slowed to 0.70 on 2026-11-10,
    // raised to 0.85 on 2026-11-12 and 0.90 on 2026-11-14. Pickups stay at PICKUP_SPEED_FRACTION.
    static final double SPEED_FRACTION = 0.90;
    // Path 2 drives all the way to point 2 (Pedro's end of path); there is no early-arrival radius.
    // Safety only: if balls or the wall stop the robot first, continue after this long without progress.
    // Time at the wall balls (stall wait, intake pause) cut 30% on 2026-11-13 via WALL_TIME_SCALE.
    // The wall shuffle was removed on 2026-11-14 to save ~1.5 s for the FLOWER.
    static final double WALL_TIME_SCALE = 0.7;
    static final long WALL_STALL_NS = Math.round(1_000_000_000L * WALL_TIME_SCALE); // was 1 s
    static final long WALL_PAUSE_NS = Math.round(1_000_000_000L * WALL_TIME_SCALE); // was 1 s
    /**
     * Feed window (gate open, balls pushed) for volleys 1 and 3. Run logs of 2026-11-13/14: the
     * ball sensor at the gate last sees a ball 0.3-0.6 s into the window, so 1.25 s keeps 2x margin.
     */
    static final double SHORT_FEED_S = 1.25;
    /** Volley 2 keeps the full window, stops at least 4 s, and stays still this long after feeding. */
    static final double VOLLEY2_FEED_S = BioBuzzAutoConfig.SHOOT_BURST_SECONDS;
    static final long VOLLEY2_MIN_STOP_NS = 4_000_000_000L;
    static final long VOLLEY2_HOLD_AFTER_NS = 500_000_000L;
    /** Drive-only stand-in for volley 2. */
    private static final long SECOND_SHOT_PAUSE_NS = VOLLEY2_MIN_STOP_NS;
    private static final double FLOWER_ARRIVAL_IN = 1.0;
    private static final double APPROACH_PROGRESS_IN = 0.25;
    /**
     * The FLOWER physically stops the robot near y = 127.5 (logs), short of POINT_7, so path 7 ends by
     * stalling: after 1 s with no progress the drive stops and the robot stays there, still shooting.
     */
    static final long FLOWER_STALL_NS = 1_000_000_000L;
    /**
     * From the start of volley 2 to the end of the auto the shooting sequence stays on (gate open,
     * intake + transmission feeding). Path 6 keeps volley 2's RPM; from the start of path 7 the
     * flywheel runs at this fixed RPM (not the distance table) for the FLOWER (volley 3).
     */
    static final double FLOWER_RPM = 3180;
    /**
     * Park (path 8, 2026-11-14): stay at the FLOWER shooting until this many seconds after START, then
     * drive the ~35 in park curve (~1.9 s at 90%, from the old path-8 log) so part of the robot is in
     * the zone before 30 s. If the robot is not at the FLOWER yet by then, it parks immediately.
     */
    static final double PARK_LEAVE_BY_S = 27.5;
    // FLOWER shuffle (2026-11-14): once path 7 stops at the FLOWER, shift slightly right, left, back
    // off and push 1 in past the stop point (heading held, pickup speed, still shooting), so the
    // intake picks up every ball there. Robot-relative offsets from where the robot actually stopped.
    static final double FLOWER_SHUFFLE_LEFT_IN = 1.5;
    static final double FLOWER_SHUFFLE_RIGHT_IN = 1.5;
    static final double FLOWER_SHUFFLE_BACK_IN = 1.5;
    static final double FLOWER_SHUFFLE_PUSH_IN = 1.0;
    // Ends at Pedro's end of path, after this long without 0.25 in of movement, or at the time cap.
    static final long FLOWER_SHUFFLE_STALL_NS = 350_000_000L;
    static final long FLOWER_SHUFFLE_MAX_NS = 1_750_000_000L;
    private static final PoseFactory POSES = PoseFactory.degrees();
    private static final Pose START = POSES.of(54.6728, 8.1648, 270);
    private static final Pose POINT_1 = POSES.of(7.4387, 15.6441, 270);
    private static final Pose POINT_2 = POSES.of(7.3833, 0.0881, -90.2042);
    private static final Pose POINT_3 = POSES.of(7.2845, 15.5174, -89.6334);
    private static final Pose POINT_4 = POSES.of(26.4473, 15.5231, -90);
    private static final Pose POINT_5 = POSES.of(59.3283, 124.8302, 90);
    private static final Pose POINT_6 = POSES.of(43.2211, 110.6879, 90);
    private static final Pose POINT_7 = POSES.of(42.0231, 139.2817, 90);
    private static final Pose POINT_8 = POSES.of(15.7092, 122.0158, 90); // Park

    @Override
    public void runOpMode() {
        Scheduler.reset();
        Follower follower = Constants.create(hardwareMap);
        BioBuzzAutoMechanisms mechanisms = new BioBuzzAutoMechanisms();
        FlowerLever flowerLever = null;
        Command route = null;
        Robot.Alliance alliance = Robot.lastAlliance;
        RuntimeException primaryFailure = null;
        AutoRunLog runLog = null;

        try {
            follower.setPose(START);
            if (useMechanisms()) {
                flowerLever = new FlowerLever(hardwareMap.get(Servo.class, "flower"));
                flowerLever.setDeployed(false); // Flower OFF as soon as INIT is pressed.
                // Pose lets the turret remember where the HIVE is after shot 1 and pre-aim there
                // while driving, instead of sweeping from scratch at each shooting spot.
                mechanisms = BioBuzzAutoMechanisms.initialize(hardwareMap, alliance,
                        () -> -Math.toDegrees(follower.pose().heading()),
                        () -> -Math.toDegrees(follower.velocity().omega),
                        () -> follower.pose().x(),
                        () -> follower.pose().y());
                // A sensor glitch or slow volley is reported and skipped; it never ends the run.
                mechanisms.setFaultTolerant(true);
                // Sweep for the HIVE whenever the tag is lost: at START it is behind the robot, and at
                // the 2nd/3rd spots it is ~160 degrees from where it was first seen.
                mechanisms.searchForTagWhenLost(true);
                mechanisms.manualIntake(false); // Motors idle; robot init already closed the gate.
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
                if (flowerLever != null) flowerLever.refresh(); // Keep the INIT OFF command powered.
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
                // Intake roller runs inward from START to the end of the route; the transmission runs only
                // while collecting (wall, FLOWER) and feeding volleys.
                mechanisms.keepIntakeRunning(true);
            }
            mechanisms.observe();
            if (flowerLever != null) flowerLever.setDeployed(false);
            route = useMechanisms() ? route(follower, System::nanoTime, mechanisms, flowerLever)
                    : route(follower, System::nanoTime);
            Scheduler.schedule(route);
            if (useMechanisms()) runLog = AutoRunLog.open("alliance,x_in,y_in,heading_deg,turn_rate_cw_dps,path_busy,"
                    + BioBuzzAutoMechanisms.LOG_HEADER);
            if (runLog != null) lastLogName = runLog.fileName;
            while (opModeIsActive() && Scheduler.isScheduled(route)) {
                follower.update();
                mechanisms.update(); // Use the chassis sample from Pedro; no second Pinpoint update.
                Scheduler.execute();
                if (runLog != null) {
                    Pose p = follower.pose();
                    runLog.row(System.nanoTime(), String.format(java.util.Locale.US, "%s,%.2f,%.2f,%.1f,%.1f,%d,%s",
                            alliance, p.x(), p.y(), Math.toDegrees(p.heading()),
                            -Math.toDegrees(follower.velocity().omega), follower.isBusy() ? 1 : 0,
                            mechanisms.logRow()));
                }
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
            if (runLog != null) runLog.close();
            try { stopAll(follower, mechanisms, route, flowerLever); }
            catch (RuntimeException cleanupError) {
                if (primaryFailure != null) primaryFailure.addSuppressed(cleanupError);
                else throw cleanupError;
            }
        }
    }

    private String lastLogName = "starts at START (/sdcard/FIRST/java/src/Datalogs/autopath_*.csv)";

    protected boolean useMechanisms() { return true; }

    private void publish(Follower follower, BioBuzzAutoMechanisms mechanisms, Robot.Alliance alliance) {
        telemetry.addData("Mode", useMechanisms() ? "BioBuzz FTC 2026: shoot + intake" : "Drive only");
        if (useMechanisms()) {
            telemetry.addData("Alliance", "%s (INIT: A blue / B red)", alliance);
            telemetry.addData("Turret zero", "INIT: point turret straight BACKWARD (%.0f deg), then BACK to zero",
                    BioBuzzAutoConfig.TURRET_ZERO_DIRECTION_DEG);
            telemetry.addData("Mechanisms", mechanisms.state());
            telemetry.addData("Shooter / aiming / gate", mechanisms.diagnostics());
            telemetry.addData("Feed RPM: start / target / minimum", mechanisms.feedRpmTelemetry());
            telemetry.addData("Mechanism failure", mechanisms.failure().isEmpty() ? "none" : mechanisms.failure());
            telemetry.addData("Mechanism warning", mechanisms.warning().isEmpty() ? "none" : mechanisms.warning());
            telemetry.addData("Run log", lastLogName);
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
        // Every segment runs at SPEED_FRACTION (90%) except the two that drive into balls: path 2 (into the wall
        // balls) and path 7 (FLOWER intake). Drive-only uses the same speeds so it rehearses the run.
        Approach wall = new Approach(POINT_2, 0, WALL_STALL_NS);
        Approach flower = new Approach(POINT_7, FLOWER_ARRIVAL_IN, FLOWER_STALL_NS);
        Command wallPickup = sequential(
                follow(follower, atRouteSpeed(path1())),
                approach(follower, clock, atPickupSpeed(path2()), wall, true));
        Command toSecondShot = sequential(
                follow(follower, atRouteSpeed(path3())),
                follow(follower, atRouteSpeed(path4())),
                follow(follower, atRouteSpeed(path5())));
        if (mechanisms == null) return sequential(
                wallPickup, pauseDrive(follower, clock, WALL_PAUSE_NS),
                toSecondShot, pauseDrive(follower, clock, SECOND_SHOT_PAUSE_NS),
                follow(follower, atRouteSpeed(path6())),
                approach(follower, clock, atPickupSpeed(path7()), flower, false),
                flowerShuffle(follower, clock),
                park(follower, clock)); // Drive-only parks right after the FLOWER shuffle.
        // Volley 2 accuracy: stopped at least 4 s, full feed window, and still for 0.5 s after feeding.
        // Continuous feeding is armed as volley 2 starts, so when its feed window ends the gate never
        // closes: gate open, intake + transmission feeding and turret tracking stay on to the end.
        Command volley2 = parallel(instant(mechanisms::continueFeedingAfterShot),
                stoppedShot(follower, clock, mechanisms, BioBuzzAutoConfig.STAGED_BALL_COUNT,
                        VOLLEY2_FEED_S, VOLLEY2_MIN_STOP_NS, VOLLEY2_HOLD_AFTER_NS));
        // Path 6 still shoots at volley 2's RPM; the drop to FLOWER_RPM starts with path 7. The flower
        // lever stays OFF through path 6 and deploys right before path 7; cleanup retracts it.
        Command toFlower = sequential(
                follow(follower, atRouteSpeed(path6())),
                instant(() -> {
                    mechanisms.continuousFeedRpm(FLOWER_RPM);
                    if (flowerLever != null) flowerLever.setDeployed(true);
                }),
                approach(follower, clock, atPickupSpeed(path7()), flower, false),
                flowerShuffle(follower, clock)); // Slight shuffle at the FLOWER, still shooting.
        long[] routeStart = {0};
        return sequential(
                parallel(instant(() -> routeStart[0] = clock.getAsLong()), // START time for PARK_LEAVE_BY_S
                        stoppedShot(follower, clock, mechanisms, BioBuzzAutoConfig.PRELOAD_BALL_COUNT, SHORT_FEED_S, 0, 0)),
                intakeAlong(follower, mechanisms, wallPickup),
                // Keep intaking through the 0.7 s stop: the last wall ball is still being pulled
                // in when the chassis stops against the wall.
                parallel(pauseDrive(follower, clock, WALL_PAUSE_NS), mechanisms.command(
                        BioBuzzAutoMechanisms.Action.intakeFor(WALL_PAUSE_NS / 1e9,
                                BioBuzzAutoConfig.COLLECTION_TIMEOUT_S), clock, AutoPath::mechanismFailure)),
                toSecondShot,
                volley2,
                // Shooting the whole way; flower ON right before path 7. Volley 3: stay stopped at the
                // FLOWER, still feeding at FLOWER_RPM, until PARK_LEAVE_BY_S.
                endBy(follower, mechanisms, sequential(toFlower, pauseDrive(follower, clock, Long.MAX_VALUE)),
                        clock, routeStart, PARK_LEAVE_BY_S),
                park(follower, clock)); // Still feeding at FLOWER_RPM while parking.
    }

    /** Slight side-to-side, then back-and-forth moves from wherever path 7 stopped at the FLOWER. */
    private static Command flowerShuffle(Follower follower, LongSupplier clock) {
        long[] started = {0}, lastMove = {0};
        Pose[] anchor = {null};
        return Command.build().setStart(() -> {
            Pose from = follower.pose();
            follower.follow(atPickupSpeed(flowerShufflePath(from)));
            started[0] = lastMove[0] = clock.getAsLong();
            anchor[0] = from;
        }).setDone(() -> {
            long now = clock.getAsLong();
            Pose pose = follower.pose();
            if (!Double.isFinite(pose.x()) || !Double.isFinite(pose.y())) {
                throw new IllegalStateException("Invalid Pinpoint pose during the FLOWER shuffle");
            }
            if (pose.distance(anchor[0]) >= APPROACH_PROGRESS_IN) { anchor[0] = pose; lastMove[0] = now; }
            boolean done = follower.atParametricEnd() || now - lastMove[0] >= FLOWER_SHUFFLE_STALL_NS
                    || now - started[0] >= FLOWER_SHUFFLE_MAX_NS;
            if (done) stopDrive(follower);
            return done;
        });
    }

    /** Right, then left, back to center, back off, then push FLOWER_SHUFFLE_PUSH_IN past the start, holding heading. */
    static Path flowerShufflePath(Pose from) {
        double h = from.heading();
        double fx = Math.cos(h), fy = Math.sin(h); // forward: into the FLOWER
        double lx = -fy, ly = fx;                  // robot left
        Pose left = shifted(from, lx * FLOWER_SHUFFLE_LEFT_IN, ly * FLOWER_SHUFFLE_LEFT_IN);
        Pose right = shifted(from, -lx * FLOWER_SHUFFLE_RIGHT_IN, -ly * FLOWER_SHUFFLE_RIGHT_IN);
        Pose back = shifted(from, -fx * FLOWER_SHUFFLE_BACK_IN, -fy * FLOWER_SHUFFLE_BACK_IN);
        Pose push = shifted(from, fx * FLOWER_SHUFFLE_PUSH_IN, fy * FLOWER_SHUFFLE_PUSH_IN);
        return path(line(from, right).constant(from), line(right, left).constant(from),
                line(left, from).constant(from), line(from, back).constant(from),
                line(back, push).constant(from));
    }

    private static Pose shifted(Pose pose, double dx, double dy) {
        return POSES.of(pose.x() + dx, pose.y() + dy, Math.toDegrees(pose.heading()));
    }

    /** Path 8 from wherever the robot is, at route speed, then hold still until the auto ends. */
    private static Command park(Follower follower, LongSupplier clock) {
        return sequential(lazy(() -> follow(follower, atRouteSpeed(path8(follower.pose())))),
                pauseDrive(follower, clock, Long.MAX_VALUE));
    }

    /** Runs {@code inner} but cuts it short once {@code leaveByS} seconds have passed since START. */
    private static Command endBy(Follower follower, BioBuzzAutoMechanisms mechanisms, Command inner,
                                 LongSupplier clock, long[] routeStart, double leaveByS) {
        boolean[] active = {false};
        long leaveByNs = Math.round(leaveByS * 1e9);
        return Command.build().requiring(follower, mechanisms)
                .setStart(() -> { active[0] = true; inner.start(); })
                .setExecute(inner::execute)
                .setDone(() -> inner.done() || clock.getAsLong() - routeStart[0] >= leaveByNs)
                .setEnd(reason -> {
                    if (!active[0]) return;
                    active[0] = false;
                    inner.end(reason);
                });
    }

    /**
     * Stopped volley: the drive stays stopped for at least {@code minimumStopNs} and for
     * {@code holdAfterNs} after the feed window ends, whichever is later.
     */
    private static Command stoppedShot(Follower follower, LongSupplier clock, BioBuzzAutoMechanisms mechanisms,
                                       int count, double feedSeconds, long minimumStopNs, long holdAfterNs) {
        Command shoot = mechanisms.command(
                BioBuzzAutoMechanisms.Action.shootFor(count, feedSeconds, BioBuzzAutoConfig.SHOOT_SPINUP_TIMEOUT_S),
                clock, AutoPath::mechanismFailure);
        if (holdAfterNs > 0) shoot = sequential(shoot, pauseDrive(follower, clock, holdAfterNs));
        return parallel(pauseDrive(follower, clock, minimumStopNs), shoot);
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
        private double closestDistance;
        private long lastProgress;

        Approach(Pose target, double arrivalIn, long stallNs) {
            this.target = target; this.arrivalIn = arrivalIn; this.stallNs = stallNs;
        }

        void start(Pose pose, long now) {
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

            // Stop pushing: the pickup and the final volley happen right here.
            stopDrive(follower);
            return true;
        }
    }

    static Path path1() {
        return curve(START, POSES.of(37.8335, 44.6525, 0), POINT_1).linear(START, POINT_1);
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
        return curve(POINT_4, POSES.of(16.3139, 96.4996, 0), POINT_5).linear(POINT_4, POINT_5);
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

    /** Park curve, rebased to start where the robot actually is (the FLOWER stops it near y = 127.5). */
    static Path path8(Pose from) {
        return curve(from, POSES.of(46.9474, 110.7735, 0), POINT_8).constant(POINT_8);
    }
}
