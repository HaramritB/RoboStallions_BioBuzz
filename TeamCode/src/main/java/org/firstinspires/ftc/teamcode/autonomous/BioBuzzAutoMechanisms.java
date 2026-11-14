package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.Arrays;
import java.util.function.Consumer;
import java.util.function.LongSupplier;
import java.util.function.DoubleSupplier;

/** Mechanism-only boundary. Movement tests stay NONE and never initialize hardware. */
public final class BioBuzzAutoMechanisms {
    public enum Kind { NONE, SHOOT, COLLECT, CENTER_TURRET }
    public static final class Action {
        public final Kind kind;
        public final int count; // Planned balls; strict shoot() verifies counts, shootFor() times a feed attempt.
        public final double timeoutSeconds;
        public final double durationSeconds; // Timed intake or shooting feed window; zero for counted shooting/path intake.
        private Action(Kind kind, int count, double timeoutSeconds, double durationSeconds) {
            if (kind != Kind.NONE && (!Double.isFinite(timeoutSeconds) || timeoutSeconds <= 0)) {
                throw new IllegalArgumentException("Mechanism action needs a positive finite deadline");
            }
            if (kind == Kind.SHOOT && count < 1) throw new IllegalArgumentException("Shot count must be positive");
            if (!Double.isFinite(durationSeconds) || durationSeconds<0
                    || (kind==Kind.COLLECT && durationSeconds>=timeoutSeconds)) {
                throw new IllegalArgumentException("Intake duration must be finite and shorter than its timeout");
            }
            this.kind=kind; this.count=count; this.timeoutSeconds=timeoutSeconds; this.durationSeconds=durationSeconds;
        }
        public static Action none() { return new Action(Kind.NONE,0,0,0); }
        public static Action shoot(int balls, double timeout) { return new Action(Kind.SHOOT,balls,timeout,0); }
        public static Action shootFor(int plannedBalls, double seconds, double spinupTimeout) {
            if (!Double.isFinite(seconds) || seconds<=0) throw new IllegalArgumentException("Shooting burst needs a positive finite duration");
            return new Action(Kind.SHOOT,plannedBalls,spinupTimeout,seconds);
        }
        public static Action intakeDuringPath(double timeout) { return new Action(Kind.COLLECT,0,timeout,0); }
        public static Action intakeFor(double seconds, double timeout) {
            if (seconds<=0) throw new IllegalArgumentException("Stationary intake needs a positive duration");
            return new Action(Kind.COLLECT,0,timeout,seconds);
        }
        public static Action centerTurret(double timeout) { return new Action(Kind.CENTER_TURRET,0,timeout,0); }
    }

    /** Sampled sensors, not inferred completion based on elapsed feed time alone. */
    public static final class Observation {
        public final boolean ballPresent;
        public final int shotsFired;
        public final double turretDegrees;
        public final boolean sensorsReady;
        public Observation(boolean ballPresent, int shotsFired, double turretDegrees) {
            this(ballPresent,shotsFired,turretDegrees,true);
        }
        public Observation(boolean ballPresent, int shotsFired, double turretDegrees, boolean sensorsReady) {
            this.ballPresent=ballPresent; this.shotsFired=shotsFired; this.turretDegrees=turretDegrees;
            this.sensorsReady=sensorsReady;
        }
    }
    public interface Hardware {
        default String diagnostics() { return "No controller diagnostics"; }
        /** True only once the gate has finished opening and both feed motors are running. */
        default boolean shootingBurstActive() { return false; }
        /**
         * Optional: the adapter has seen every planned ball leave and may end a timed burst before
         * its full window. The original adapter never does, so its timing is unchanged.
         */
        default boolean burstFinishedEarly(int plannedBalls) { return false; }
        Observation observe();
        void control(Kind request);
        void stopOutputs();
        /** Close the gate and stop intake/transmission; a ready flywheel may keep spinning. */
        default void stopFeed() { stopOutputs(); }
        /** Keep the flywheel at calibrated RPM and the turret tracking while no feed is requested. */
        default void setShooterIdle(boolean spinning) { }
        /** Run the intake inward for the whole run, not only during collect commands. */
        default void setIntakeAlways(boolean running) { }
        /** Sweep the turret for the HIVE whenever the tag has been lost, instead of holding a bearing. */
        default void setSearchWhenTagLost(boolean search) { }
        void close();
    }

    private final Hardware hardware;
    private Kind request=Kind.NONE;
    private Observation observation;
    private boolean closed;
    private boolean commandActive;
    private boolean faultTolerant;
    private boolean shooterReady;
    private boolean intakeAlways;
    private LongSupplier activeClock;
    private long activeStart;
    private double activeTimeout;
    private double activeIntakeDuration;
    private double activeShotDuration;
    private int activeShotCount;
    private long burstStarted=-1;
    private boolean burstCompleted;
    private long shotLimit;
    private int skippedVolleys, ignoredSensorSamples;
    private String failure="";
    private String warning="";

    public BioBuzzAutoMechanisms() { this(null); }
    public BioBuzzAutoMechanisms(Hardware hardware) { this.hardware=hardware; }

    public static BioBuzzAutoMechanisms initialize(HardwareMap map, Robot.Alliance alliance) {
        RobotHardware hardware=new RobotHardware();
        try {
            hardware.robot.initMechanismsOnly(map, false); // Never re-zero turret without a verified physical reference.
            hardware.robot.alliance=alliance;
            if (hardware.robot.ballDistanceSensor == null) throw new IllegalStateException("Ball distance sensor required by the existing shooting/feeder interface");
            if (hardware.robot.colorSensor == null) throw new IllegalStateException("Color sensor required by Robot.updateSensors ball detection");
            if (hardware.robot.Limelight == null) throw new IllegalStateException("Limelight required for the existing shot sequence");
            if (!hardware.robot.gateCalibrated()) throw new IllegalStateException("Run Gate Calibration: saved gate endpoints are invalid");
            return new BioBuzzAutoMechanisms(hardware);
        } catch (RuntimeException error) {
            try { hardware.close(); } catch (RuntimeException cleanup) { error.addSuppressed(cleanup); }
            throw error;
        }
    }

    public static BioBuzzAutoMechanisms initialize(HardwareMap map, Robot.Alliance alliance,
                                                  DoubleSupplier headingCw, DoubleSupplier turnRateCw) {
        BioBuzzAutoMechanisms result=initialize(map,alliance);
        RobotHardware port=(RobotHardware)result.hardware;
        port.headingCw=headingCw; port.turnRateCw=turnRateCw;
        return result;
    }

    /**
     * Also passes the Pedro position (inches). The adapter then remembers where the HIVE is from
     * each centered sighting while stopped to shoot, and pre-aims the turret there from odometry
     * whenever the tag is out of view, so it reaches the next shooting spot already pointed at it.
     */
    public static BioBuzzAutoMechanisms initialize(HardwareMap map, Robot.Alliance alliance,
                                                  DoubleSupplier headingCw, DoubleSupplier turnRateCw,
                                                  DoubleSupplier xIn, DoubleSupplier yIn) {
        BioBuzzAutoMechanisms result=initialize(map,alliance,headingCw,turnRateCw);
        RobotHardware port=(RobotHardware)result.hardware;
        port.xIn=xIn; port.yIn=yIn;
        return result;
    }

    /**
     * Match autonomous policy: a bad feeder-sensor sample or a volley that cannot start is
     * reported in {@link #warning()} and the route keeps going. Without this, one I2C glitch or a
     * slow flywheel ended the whole autonomous. Diagnostic OpModes keep the strict default.
     */
    public void setFaultTolerant(boolean tolerant) { faultTolerant=tolerant; }

    /**
     * Keep the flywheel at the calibrated RPM and the turret tracking between commands, as
     * TeleOp does. Each volley then starts warm and already aimed instead of spinning up from
     * zero and sweeping for the tag after the robot stops.
     */
    public void keepShooterReady(boolean ready) {
        if (closed) return;
        shooterReady=ready;
        if (hardware!=null) {
            hardware.setShooterIdle(ready);
            if (!ready && request==Kind.NONE) hardware.stopOutputs();
        }
    }

    /**
     * AutoPath: keep the intake ROLLER running inward from START to the end of the route. The
     * transmission runs only while collecting (and while feeding a volley), not between pickups.
     */
    public void keepIntakeRunning(boolean running) {
        if (closed) return;
        intakeAlways=running;
        if (hardware!=null) hardware.setIntakeAlways(running);
    }

    /**
     * AutoPath: the turret sweeps for the HIVE whenever it has not seen a tag for 400 ms. Main's
     * turret otherwise holds the last field bearing, which points the wrong way at the 2nd and 3rd
     * shooting spots (the HIVE is ~160 degrees away from where it was seen at START).
     */
    public void searchForTagWhenLost(boolean search) {
        if (hardware!=null && !closed) hardware.setSearchWhenTagLost(search);
    }

    /** Stationary operator-held test only; mutually exclusive with any command. */
    public void manualIntake(boolean running) {
        if (closed && !running) return; // An end callback may run after emergency shutdown.
        if (closed || commandActive || !failure.isEmpty()) throw new IllegalStateException("Mechanism port is unavailable or command-owned");
        request=running ? Kind.COLLECT : Kind.NONE;
        if (!running && hardware!=null) hardware.stopFeed();
    }

    /** Reads only. Safe to call in INIT; never runs a controller or commands a gate/motor. */
    public void observe() {
        if (hardware == null || closed) return;
        Observation sample=hardware.observe();
        if (valid(sample)) { observation=sample; return; }
        if (!faultTolerant) {
            observation=sample;
            throw new IllegalStateException("Mechanism sensor sample unavailable/nonfinite: " + hardware.diagnostics());
        }
        // Timed feeding/intake do not depend on this sample; treat the feeder as empty and continue.
        ignoredSensorSamples++;
        warning="Ignored " + ignoredSensorSamples + " invalid feeder sensor sample(s): " + hardware.diagnostics();
        double turret=sample!=null && Double.isFinite(sample.turretDegrees) ? sample.turretDegrees
                : observation!=null ? observation.turretDegrees : 0;
        observation=new Observation(false,sample!=null ? sample.shotsFired : 0,turret,true);
    }

    /** Call exactly once per active FTC loop, after navigation and before Scheduler.execute(). */
    public void update() {
        if (closed) return;
        if (commandActive && activeShotDuration>0 && burstStarted>=0 && !burstCompleted
                && ((activeClock.getAsLong()-burstStarted)/1e9>=activeShotDuration
                    || hardware.burstFinishedEarly(activeShotCount))) {
            burstCompleted=true;
            request=Kind.NONE;
            hardware.stopFeed(); // End feeding before another actuator write, even if no four sensor edges occurred.
        }
        if (commandActive && activeIntakeDuration>0 && (activeClock.getAsLong()-activeStart)/1e9>=activeIntakeDuration
                && request==Kind.COLLECT) {
            request=Kind.NONE;
            hardware.stopFeed(); // End timed intake before another actuator write.
        }
        if (commandActive && deadlineExpired(activeClock.getAsLong())) {
            if (faultTolerant && activeShotDuration>0) {
                skipVolley();
            } else {
                request=Kind.NONE;
                hardware.stopOutputs();
                throw new IllegalStateException(activeShotDuration>0
                        ? "Shooter did not begin feeding before aim/RPM spin-up timeout: " + hardware.diagnostics()
                        : "Mechanism timeout before actuator output");
            }
        }
        observe();
        if (hardware != null && !closed && (request != Kind.NONE || shooterReady || intakeAlways)) {
            if (!validObservation()) throw new IllegalStateException("Mechanism sensor sample unavailable/nonfinite");
            if (request == Kind.SHOOT && activeShotDuration==0 && observation.shotsFired >= shotLimit) {
                // The fourth observed exit ends the burst before another feed motor write.
                request = Kind.NONE;
                hardware.stopFeed();
            } else {
                hardware.control(request);
                if (commandActive && request==Kind.SHOOT && activeShotDuration>0 && burstStarted<0
                        && hardware.shootingBurstActive()) {
                    burstStarted=activeClock.getAsLong();
                }
            }
        }
    }

    private void skipVolley() {
        String detail=hardware.diagnostics();
        burstCompleted=true;
        request=Kind.NONE;
        hardware.stopFeed();
        skippedVolleys++;
        warning=String.format(java.util.Locale.US,"Skipped volley %d: feeding never started within %.1f s; continuing route. %s",
                skippedVolleys,activeTimeout,detail);
    }

    private boolean deadlineExpired(long now) {
        // Timed shooting has a spin-up deadline, then its own complete feed window.
        if (activeShotDuration>0 && (burstStarted>=0 || burstCompleted)) return false;
        return (now-activeStart)/1e9>=activeTimeout;
    }

    private static boolean valid(Observation sample) {
        return sample != null && sample.sensorsReady && Double.isFinite(sample.turretDegrees);
    }

    private boolean validObservation() { return valid(observation); }

    public String intakePolicy() { return "Open-loop: path duration / configured FLOWER window; no stored-ball count"; }

    public Command command(Action action, LongSupplier clock, Consumer<String> onFailure) {
        final boolean[] active={false};
        final boolean[] previousBall={false};
        final int[] clearTransitions={0};
        final int[] baseline={0};
        final long[] began={0};
        final long[] stableSince={-1};
        return Command.build().requiring(this)
                .setStart(() -> {
                    if (hardware == null || closed || !validObservation()) {
                        failure="Mechanism hardware/sensors not initialized";
                        onFailure.accept(failure);
                        return;
                    }
                    if (commandActive) throw new IllegalStateException("Competing mechanism commands");
                    active[0]=true; commandActive=true;
                    began[0]=clock.getAsLong();
                    activeClock=clock; activeStart=began[0]; activeTimeout=action.timeoutSeconds;
                    activeIntakeDuration=action.kind==Kind.COLLECT ? action.durationSeconds : 0;
                    activeShotDuration=action.kind==Kind.SHOOT ? action.durationSeconds : 0;
                    activeShotCount=action.kind==Kind.SHOOT ? action.count : 0;
                    burstStarted=-1; burstCompleted=false;
                    stableSince[0]=-1;
                    clearTransitions[0]=0;
                    baseline[0]=observation.shotsFired;
                    shotLimit=(long) baseline[0]+action.count;
                    previousBall[0]=observation.ballPresent;
                    request=action.kind;
                })
                .setDone(() -> {
                    if (!active[0] || closed || !failure.isEmpty()) return true;
                    long now=clock.getAsLong();
                    if (deadlineExpired(now)) {
                        if (faultTolerant && action.kind==Kind.SHOOT && action.durationSeconds>0) {
                            skipVolley();
                        } else {
                            failure="Mechanism timeout: " + action.kind;
                            onFailure.accept(failure);
                            return true;
                        }
                    }
                    if (!validObservation()) {
                        failure="Invalid mechanism sensor sample";
                        onFailure.accept(failure);
                        return true;
                    }
                    if (previousBall[0] && !observation.ballPresent) clearTransitions[0]++;
                    previousBall[0]=observation.ballPresent;
                    boolean done;
                    switch (action.kind) {
                        case SHOOT:
                            if (action.durationSeconds>0) {
                                done=burstCompleted; // Ball-clear count is advisory for timed continuous feeding.
                                break;
                            }
                            if (observation.shotsFired-baseline[0]>action.count) {
                                failure="Shot counter exceeded requested count";
                                onFailure.accept(failure);
                                return true;
                            }
                            if (observation.shotsFired-baseline[0]>=action.count && request==Kind.SHOOT) {
                                request=Kind.NONE;
                                hardware.stopFeed(); // End the burst at the requested ball count.
                            }
                            done=observation.shotsFired-baseline[0]>=action.count && clearTransitions[0]>=action.count;
                            break;
                        case COLLECT:
                            // This completes an intake attempt, never a measured inventory target.
                            return action.durationSeconds>0 && (now-began[0])/1e9>=action.durationSeconds;
                        case CENTER_TURRET: done=Math.abs(observation.turretDegrees)<=Robot.TURRET_DEADBAND_DEG; break;
                        default: done=true;
                    }
                    if (!done) { stableSince[0]=-1; return false; }
                    if (stableSince[0]<0) {
                        stableSince[0]=now;
                        request=Kind.NONE;
                        hardware.stopFeed(); // No extra feed/collection while verifying the result.
                    }
                    return (now-stableSince[0])/1e9>=BioBuzzAutoConfig.SETTLE_TIME_S;
                })
                .setEnd(reason -> {
                    if (!active[0]) return; // Ivy may end children which have never started.
                    request=Kind.NONE; commandActive=false;
                    try { hardware.stopFeed(); }
                    finally {
                        if (reason != EndCondition.NATURALLY && !closed) {
                            failure="Mechanism command interrupted: " + action.kind;
                            onFailure.accept(failure);
                        }
                    }
                });
    }

    public void stop() {
        closed=true;
        commandActive=false;
        shooterReady=false;
        intakeAlways=false;
        request=Kind.NONE;
        if (hardware != null) hardware.close();
    }
    public void setAlliance(Robot.Alliance alliance) {
        if (hardware instanceof RobotHardware) ((RobotHardware) hardware).robot.alliance = alliance;
    }
    /** Explicit INIT operator action, with the turret pointing at BioBuzzAutoConfig.TURRET_ZERO_DIRECTION_DEG (backward). */
    public void zeroTurret() {
        if (closed || commandActive || request!=Kind.NONE) throw new IllegalStateException("Turret zero requires idle mechanisms");
        if (hardware instanceof RobotHardware) ((RobotHardware)hardware).robot.zeroTurret();
    }
    public String diagnostics() {
        String detail=hardware==null ? "inactive" : hardware.diagnostics();
        if (commandActive && activeShotDuration>0) {
            if (burstStarted<0 && !burstCompleted) {
                detail+=String.format(java.util.Locale.US,"; waiting %.2f s (aim grace %.1f s, force %.1f s, skip %.1f s)",
                        (activeClock.getAsLong()-activeStart)/1e9,BioBuzzAutoConfig.SHOOT_AIM_GRACE_SECONDS,
                        BioBuzzAutoConfig.SHOOT_FORCE_FEED_SECONDS,activeTimeout);
            } else if (burstStarted>=0) {
                detail+=String.format(java.util.Locale.US,"; timed feed %.2f/%.2f s (%s); exits advisory",
                        (activeClock.getAsLong()-burstStarted)/1e9,activeShotDuration,burstCompleted ? "complete" : "feeding");
            }
        }
        return detail;
    }
    public String feedRpmTelemetry() {
        if (!(hardware instanceof RobotHardware)) return "unavailable";
        RobotHardware port=(RobotHardware)hardware;
        return Double.isFinite(port.feedStartRpm) ? String.format(java.util.Locale.US,
                "%.0f / %.0f / %.0f RPM (%s)",port.feedStartRpm,port.feedTargetRpm,port.minBurstRpm,port.feedTrigger)
                : "waiting for stable speed";
    }
    public String state() {
        if (hardware == null) return "DISABLED - no mapped actions";
        if (closed) return "STOPPED";
        String mode=(shooterReady ? ", flywheel ready, turret tracking" : "") + (intakeAlways ? ", intake always on" : "");
        return mode.isEmpty() ? request.name() : request.name() + " (" + mode.substring(2) + ")";
    }
    /** CSV columns for {@link #logRow()}. */
    public static final String LOG_HEADER="request,shot_phase,gate_open,turret_deg,turret_vel_dps,turret_state,"
            + "turret_req_power,turret_applied_power,turret_limit,target_fresh,target_visible,target_id,tx_deg,"
            + "vision_err_deg,dist_m,frame_age_ms,aimed,still_s,hive_fixes,hive_x,hive_y,pre_aim_deg,shots_fired,"
            + "intake_power,intake_vel_tps,transmission_power,transmission_vel_tps,ball_present,feed_trigger,tags";

    /** One CSV row of turret/vision/shot state for AutoPath's run log (empty without real hardware). */
    public String logRow() {
        String head=request.name();
        if (!(hardware instanceof RobotHardware)) return head;
        return head+","+((RobotHardware)hardware).logRow();
    }

    public String failure() { return failure; }
    /** Non-fatal problems the fault-tolerant route continued through. Empty when none. */
    public String warning() { return warning; }

    /** Reuses existing Robot controllers. This adapter never calls Robot.drive(), init() or stop(). */
    static final class RobotHardware implements Hardware {
        private enum Phase { IDLE, SPINUP, GATE_OPENING, FEEDING }
        /**
         * The one inward direction (TeleOp gamepad 1 RIGHT trigger) for BOTH collecting and
         * feeding shots: intake -1, transmission -0.6. Only the gate differs (closed = collect and
         * stack balls against it, open = shoot). Collecting must never run the opposite way.
         */
        static final double INWARD_INTAKE_POWER = -Robot.INTAKE_POWER;
        static final double INWARD_TRANSMISSION_POWER = -Robot.TRANSMISSION_INDEX_POWER;

        final Robot robot;
        private final LongSupplier clock;
        DoubleSupplier headingCw, turnRateCw;
        /** Pedro position in inches; null disables the HIVE memory and pre-aim (old sweep behavior). */
        DoubleSupplier xIn, yIn;
        double hiveX=Double.NaN, hiveY=Double.NaN;
        int hiveFixes;
        double preAimDeg=Double.NaN;
        /** Last feed powers this adapter commanded (logged; reading them back costs a hub transaction). */
        private double loggedIntakePower, loggedTransmissionPower;
        private double lastPoseX=Double.NaN, lastPoseY=Double.NaN;
        private long lastPoseAt=-1, stillSince=-1;
        private Phase phase=Phase.IDLE;
        private boolean idleSpin;
        private boolean intakeAlways;
        private boolean gateCommandOpen;
        private long gateMovedAt=-1;
        private boolean previousBurstBall;
        private long gateOpenedAt=-1;
        private double burstBaseRpm=Double.NaN;
        private long shootingBegan=-1, speedStableSince=-1, lastSpeedSample=-1;
        private double stableTargetRpm=Double.NaN;
        private int stableSpeedSamples;
        private double feedStartRpm=Double.NaN, feedTargetRpm=Double.NaN, minBurstRpm=Double.NaN;
        private String feedTrigger="";
        private int aimLossFeedLoops;
        private final double[] distanceSamples=new double[Math.max(1,BioBuzzAutoConfig.DISTANCE_FILTER_SAMPLES)];
        private int distanceCount, distanceNext;
        private long lastDistanceAt=-1;
        private String detail="Not sampled";
        private boolean used;

        RobotHardware() { this(new Robot()); }
        RobotHardware(Robot robot) { this(robot,System::nanoTime); }
        RobotHardware(Robot robot, LongSupplier clock) { this.robot=robot; this.clock=clock; }

        @Override public String diagnostics() {
            double stableSeconds=speedStableSince<0 ? 0 : (clock.getAsLong()-speedStableSince)/1e9;
            String readiness=String.format(java.util.Locale.US,"; %s; speed stable %.2f/%.2f s (%d samples, +/-%.0f RPM)",
                    phase,stableSeconds,BioBuzzAutoConfig.SHOOT_READY_STABLE_SECONDS,stableSpeedSamples,
                    BioBuzzAutoConfig.SHOOT_READY_RPM_TOLERANCE);
            String burst=Double.isFinite(feedStartRpm) ? String.format(java.util.Locale.US,
                    "; feed-start RPM %.0f/%.0f (%s); min burst RPM %.0f; aim lost %d feed loops",
                    feedStartRpm,feedTargetRpm,feedTrigger,minBurstRpm,aimLossFeedLoops) : "";
            return detail+readiness+burst;
        }
        @Override public boolean shootingBurstActive() { return phase==Phase.FEEDING; }

        private static double motorVelocity(com.qualcomm.robotcore.hardware.DcMotorEx motor) {
            // Served from the hub's bulk cache (AUTO mode), so this adds no hardware transaction.
            try { return motor==null ? Double.NaN : motor.getVelocity(); } catch (RuntimeException e) { return Double.NaN; }
        }

        String logRow() {
            long now=clock.getAsLong();
            double still=stillSince<0 ? 0 : (now-stillSince)/1e9;
            String tags=robot.visionTagDetails();
            return String.format(java.util.Locale.US,
                    "%s,%d,%.2f,%.1f,%s,%.3f,%.3f,%s,%d,%d,%d,%.2f,%.2f,%.3f,%.0f,%d,%.2f,%d,%.1f,%.1f,%.1f,%d,"
                            + "%.2f,%.0f,%.2f,%.0f,%d,%s,\"%s\"",
                    phase,robot.gateOpen ? 1 : 0,robot.turretDegrees(),robot.turretVelocityDegPerSec(),
                    robot.turretAimState(),robot.turretRequestedPower(),robot.turretAppliedPower(),robot.turretPowerLimit(),
                    robot.targetFresh ? 1 : 0,robot.targetVisible ? 1 : 0,robot.targetId,robot.targetTx,
                    robot.visionErrorDegrees(),robot.targetDistanceMeters,robot.targetFrameAgeMs(),
                    robot.aimedAtTarget() ? 1 : 0,still,hiveFixes,hiveX,hiveY,preAimDeg,robot.shotsFired,
                    loggedIntakePower,motorVelocity(robot.intake),
                    loggedTransmissionPower,motorVelocity(robot.transmission),
                    robot.ballPresent ? 1 : 0,feedTrigger.isEmpty() ? "-" : feedTrigger.replace(',',';'),
                    tags==null ? "" : tags.replace('"','\''));
        }
        @Override public void setShooterIdle(boolean spinning) { idleSpin=spinning; }
        @Override public void setIntakeAlways(boolean running) { intakeAlways=running; }
        @Override public void setSearchWhenTagLost(boolean search) { robot.setTurretSearchWhenTagLost(search); }

        @Override public Observation observe() {
            if (headingCw==null) robot.updateSensors();
            else robot.updateMechanismSensors(headingCw.getAsDouble(),turnRateCw.getAsDouble());
            // Count actual feeder presence->clear transitions during the open burst, not
            // timed gate pulses. Observe before control so the fourth exit stops immediately.
            if (phase==Phase.FEEDING) {
                if (previousBurstBall && !robot.ballPresent) robot.shotsFired++;
                previousBurstBall=robot.ballPresent;
            }
            detail=String.format(java.util.Locale.US,"RPM %.0f/%.0f; target fresh=%s aimed=%s tx=%.2f distance=%.3f m (filtered %.3f m); gate calibrated=%s; ball=%s distance=%.2f cm; exits=%d",
                    robot.shooterRpm(),robot.targetRpm+robot.rpmTrim,robot.targetFresh,robot.aimedAtTarget(),
                    robot.targetTx,robot.targetDistanceMeters,filteredDistance(clock.getAsLong()),robot.gateCalibrated(),
                    robot.ballPresent,robot.ballDistanceCm,robot.shotsFired);
            if (xIn!=null) {
                detail+=hiveFixes==0 ? "; HIVE not located yet (turret sweeps)"
                        : String.format(java.util.Locale.US,"; HIVE at (%.0f, %.0f) from %d sightings; pre-aim %.0f deg; turret %s",
                        hiveX,hiveY,hiveFixes,preAimDeg,robot.turretAimState());
            }
            return new Observation(robot.ballPresent,robot.shotsFired,robot.turretDegrees(),
                    Double.isFinite(robot.ballDistanceCm) && robot.ballDistanceCm > 0);
        }

        @Override public void control(Kind request) {
            used=true;
            long now=clock.getAsLong();
            boolean shooting=request==Kind.SHOOT;
            boolean spin=shooting || idleSpin;
            updateHivePreAim(shooting);
            if (request==Kind.CENTER_TURRET) robot.turretToAngle(0);
            else if (spin) {
                robot.aimTurret(true,0); // TeleOp keeps auto-aim on all match.
                // Never sweep for a tag while the gate is open: hold the turret where it is.
                if (shooting && phase!=Phase.IDLE && phase!=Phase.SPINUP
                        && robot.turretAimState().startsWith("SEARCH")) robot.turret.setPower(0);
            }
            else robot.turret.setPower(0);

            // Same measured distance table and tuned PIDF as TeleOp, from a median-filtered distance.
            boolean validDistance=robot.targetFresh && Double.isFinite(robot.targetDistanceMeters)
                    && robot.targetDistanceMeters>0;
            if (validDistance) recordDistance(robot.targetDistanceMeters,now);
            boolean latched=(phase==Phase.GATE_OPENING || phase==Phase.FEEDING) && Double.isFinite(burstBaseRpm);
            double baseRpm=latched ? burstBaseRpm : Robot.rpmForDistance(filteredDistance(now));
            robot.shooterOn=spin;
            robot.updateShooter(baseRpm);
            double measuredRpm=robot.shooterRpm();
            double targetRpm=robot.targetRpm+robot.rpmTrim;

            if (shooting) {
                if (phase==Phase.IDLE) {
                    phase=Phase.SPINUP;
                    shootingBegan=now;
                    resetSpeedQualification();
                    feedStartRpm=feedTargetRpm=minBurstRpm=Double.NaN;
                    feedTrigger=""; aimLossFeedLoops=0;
                }
                boolean aimed=validDistance && robot.aimedAtTarget();
                if (phase==Phase.SPINUP) {
                    double waited=(now-shootingBegan)/1e9;
                    // Unaimed fallback only when there is nothing to turn to (sweeping, no camera); a
                    // turret still turning toward a known or visible HIVE keeps the gate closed
                    // until the force-feed time.
                    String turretState=robot.turretAimState();
                    boolean turretTravelling=turretState!=null && (turretState.startsWith("PRE-AIM")
                            || turretState.startsWith("UNWIND") || turretState.startsWith("TRACK")
                            || turretState.startsWith("REACQUIRE") || turretState.startsWith("LIMIT RECOVERY"));
                    boolean aimGraceOver=waited>=BioBuzzAutoConfig.SHOOT_AIM_GRACE_SECONDS
                            && (!turretTravelling || waited>=BioBuzzAutoConfig.SHOOT_FORCE_FEED_SECONDS);
                    // A single target crossing during acceleration is not a settled flywheel.
                    boolean stable=speedQualified(now,robot.gateCalibrated() && (aimed || aimGraceOver),
                            measuredRpm,targetRpm);
                    boolean forced=!stable && robot.gateCalibrated()
                            && waited>=BioBuzzAutoConfig.SHOOT_FORCE_FEED_SECONDS
                            && Double.isFinite(measuredRpm) && Double.isFinite(targetRpm) && targetRpm>0
                            && measuredRpm>=BioBuzzAutoConfig.SHOOT_FORCE_MIN_RPM_FRACTION*targetRpm;
                    if (stable || forced) {
                        // Stationary burst: do not chase noisy distance frames while feeding.
                        phase=Phase.GATE_OPENING;
                        gateOpenedAt=now;
                        burstBaseRpm=baseRpm;
                        feedTrigger=forced ? String.format(java.util.Locale.US,"forced at %.1f s",waited)
                                : aimed ? "aimed" : "no aimed tag: calibrated fallback";
                        feedStartRpm=measuredRpm; feedTargetRpm=targetRpm; minBurstRpm=measuredRpm;
                    }
                }
                // Wait for the gate to physically open before pushing balls into it.
                if (phase==Phase.GATE_OPENING
                        && (now-gateOpenedAt)/1e9>=BioBuzzAutoConfig.GATE_OPEN_WAIT_SECONDS) {
                    phase=Phase.FEEDING;
                    previousBurstBall=robot.ballPresent;
                }
                if (phase==Phase.FEEDING && !aimed) aimLossFeedLoops++; // Reported; never aborts a short burst.
                if (phase!=Phase.SPINUP && Double.isFinite(measuredRpm)) {
                    minBurstRpm=Double.isFinite(minBurstRpm) ? Math.min(minBurstRpm,measuredRpm) : measuredRpm;
                }
                // Gate CLOSED while spinning up, OPEN while it travels, then both RT-direction motors.
                // With the intake always on, the roller keeps running, but the transmission stops
                // packing balls into the closed gate, then briefly backs them off while it opens.
                boolean feeding=phase==Phase.FEEDING;
                boolean relieving=intakeAlways && phase==Phase.GATE_OPENING
                        && (now-gateOpenedAt)/1e9<BioBuzzAutoConfig.GATE_RELIEF_SECONDS;
                driveFeeder(now,phase!=Phase.SPINUP,
                        feeding || intakeAlways ? INWARD_INTAKE_POWER : 0,
                        feeding ? INWARD_TRANSMISSION_POWER
                                : relieving ? -Math.signum(INWARD_TRANSMISSION_POWER)*BioBuzzAutoConfig.GATE_RELIEF_TRANSMISSION_POWER : 0);
            } else {
                resetShot();
                // Collect in the same direction as the shot, gate closed: balls stack against the gate.
                // The always-on intake keeps only the roller running between pickups; the
                // transmission runs only while actually collecting (user request 2026-11-13).
                boolean collecting=request==Kind.COLLECT;
                boolean roller=collecting || (intakeAlways && request==Kind.NONE);
                driveFeeder(now,false,roller ? INWARD_INTAKE_POWER : 0,
                        collecting ? INWARD_TRANSMISSION_POWER : 0);
            }
            robot.updateLed();
        }

        /**
         * Learns the HIVE's field position from every sighting taken while the chassis is still
         * (any request, not only while shooting, and the tag need not be centered: the turret
         * angle at capture time plus the tag offset gives the bearing), and hands the turret the
         * angle odometry predicts for it. Without a pose or a sighting yet the prediction is NaN
         * and the turret sweeps exactly as before.
         */
        private void updateHivePreAim(boolean stoppedToShoot) {
            if (xIn==null || yIn==null || headingCw==null) return;
            long now=clock.getAsLong();
            double x=xIn.getAsDouble(), y=yIn.getAsDouble(), heading=headingCw.getAsDouble();
            double turn=turnRateCw!=null ? turnRateCw.getAsDouble() : 0;
            boolean slow=false;
            if (lastPoseAt>=0 && now>lastPoseAt && Double.isFinite(lastPoseX) && Double.isFinite(lastPoseY)) {
                double speed=Math.hypot(x-lastPoseX,y-lastPoseY)/((now-lastPoseAt)/1e9);
                slow=speed<=BioBuzzAutoConfig.HIVE_FIX_MAX_SPEED_IN_S
                        && (!Double.isFinite(turn) || Math.abs(turn)<=BioBuzzAutoConfig.HIVE_FIX_MAX_TURN_DEG_S);
            } else if (lastPoseAt>=0 && now==lastPoseAt) {
                slow=stillSince>=0; // Same timestamp: keep the previous verdict.
            }
            if (!slow) stillSince=-1;
            else if (stillSince<0) stillSince=now;
            lastPoseX=x; lastPoseY=y; lastPoseAt=now;
            boolean still=stillSince>=0 && (now-stillSince)/1e9>=BioBuzzAutoConfig.HIVE_FIX_STILL_SECONDS;

            double turretSpeed=robot.turretVelocityDegPerSec();
            if (!Double.isFinite(turretSpeed)) turretSpeed=0;
            double frameAge=robot.targetFrameAgeMs();
            if (!Double.isFinite(frameAge) || frameAge<0) frameAge=0;
            if (still && robot.targetFresh && Math.abs(turretSpeed)<=BioBuzzAutoConfig.HIVE_FIX_MAX_TURRET_DEG_S
                    && Double.isFinite(robot.targetDistanceMeters) && robot.targetDistanceMeters>0) {
                // Where the turret pointed when the camera captured the frame.
                double captureTurret=robot.turretDegrees()-turretSpeed*frameAge/1000.0
                        +BioBuzzAutoConfig.TURRET_ZERO_DIRECTION_DEG; // Encoder -> angle from robot front.
                double[] fix=Robot.hiveFieldPoint(x,y,heading,captureTurret,
                        robot.visionErrorDegrees(),robot.targetDistanceMeters/0.0254);
                if (Double.isFinite(fix[0]) && Double.isFinite(fix[1])) {
                    hiveFixes++;
                    double weight=1.0/Math.min(hiveFixes,Math.max(1,BioBuzzAutoConfig.HIVE_FIX_AVERAGE_SAMPLES));
                    hiveX=Double.isFinite(hiveX) ? hiveX+(fix[0]-hiveX)*weight : fix[0];
                    hiveY=Double.isFinite(hiveY) ? hiveY+(fix[1]-hiveY)*weight : fix[1];
                }
            }
            // Angle from robot front -> encoder angle; the controller picks the reachable equivalent.
            preAimDeg=Robot.turretDegreesToward(x,y,heading,hiveX,hiveY)-BioBuzzAutoConfig.TURRET_ZERO_DIRECTION_DEG;
            // Sweep fallback only while stopped to shoot; while driving keep following the prediction.
            robot.setTurretPreAim(preAimDeg,stoppedToShoot);
        }

        /** Calibrated gate plus both feed motors; the transmission never pushes balls into a moving gate. */
        private void driveFeeder(long now, boolean open, double intakePower, double transmissionPower) {
            boolean calibrated=robot.gateCalibrated();
            open&=calibrated;
            if (open!=gateCommandOpen) { gateCommandOpen=open; gateMovedAt=now; }
            boolean gateMoving=gateMovedAt>=0 && (now-gateMovedAt)/1e9<BioBuzzAutoConfig.GATE_OPEN_WAIT_SECONDS;
            robot.gateOpen=open;
            if (calibrated) robot.gateController.commandSaved(open);
            robot.intake.setPower(intakePower);
            loggedIntakePower=intakePower;
            // While the gate moves, never push balls INTO it; backing them off is allowed.
            boolean intoGate=transmissionPower!=0 && Math.signum(transmissionPower)==Math.signum(INWARD_TRANSMISSION_POWER);
            loggedTransmissionPower=calibrated && !(gateMoving && intoGate) ? transmissionPower : 0;
            robot.transmission.setPower(loggedTransmissionPower);
        }

        private void closeGate(RuntimeException[] first) {
            if (gateCommandOpen) gateMovedAt=clock.getAsLong();
            gateCommandOpen=false;
            robot.gateOpen=false;
            // No INIT gate write. Commands require separately verified gate hardware/sequence.
            attempt(() -> { if (used && robot.gateController!=null) robot.gateController.commandSaved(false); },first);
        }

        private void recordDistance(double meters, long now) {
            distanceSamples[distanceNext]=meters;
            distanceNext=(distanceNext+1)%distanceSamples.length;
            distanceCount=Math.min(distanceCount+1,distanceSamples.length);
            lastDistanceAt=now;
        }

        /** Median of recent fresh tag distances, or NaN (table default RPM) when none is recent. */
        private double filteredDistance(long now) {
            if (distanceCount==0 || lastDistanceAt<0
                    || (now-lastDistanceAt)/1e9>BioBuzzAutoConfig.DISTANCE_HOLD_SECONDS) return Double.NaN;
            double[] sorted=Arrays.copyOf(distanceSamples,distanceCount);
            Arrays.sort(sorted);
            int middle=distanceCount/2;
            return distanceCount%2==1 ? sorted[middle] : (sorted[middle-1]+sorted[middle])/2;
        }

        private boolean speedQualified(long now, boolean targetOk, double measuredRpm, double targetRpm) {
            if (!targetOk || !Double.isFinite(measuredRpm) || !Double.isFinite(targetRpm)
                    || measuredRpm<=0 || targetRpm<=0
                    || Math.abs(measuredRpm-targetRpm)>BioBuzzAutoConfig.SHOOT_READY_RPM_TOLERANCE) {
                resetSpeedQualification();
                return false;
            }
            if (speedStableSince<0
                    || (lastSpeedSample>=0 && (now-lastSpeedSample)/1e9>BioBuzzAutoConfig.SHOOT_READY_MAX_SAMPLE_GAP_SECONDS)
                    || Math.abs(targetRpm-stableTargetRpm)>BioBuzzAutoConfig.SHOOT_READY_TARGET_CHANGE_RPM) {
                speedStableSince=now; stableTargetRpm=targetRpm; stableSpeedSamples=0;
            }
            if (lastSpeedSample!=now || stableSpeedSamples==0) stableSpeedSamples++;
            lastSpeedSample=now;
            return shootingBegan>=0 && (now-shootingBegan)/1e9>=BioBuzzAutoConfig.SHOOT_MIN_SPINUP_SECONDS
                    && (now-speedStableSince)/1e9>=BioBuzzAutoConfig.SHOOT_READY_STABLE_SECONDS
                    && stableSpeedSamples>=BioBuzzAutoConfig.SHOOT_READY_MIN_SAMPLES;
        }

        private void resetSpeedQualification() {
            speedStableSince=lastSpeedSample=-1;
            stableTargetRpm=Double.NaN; stableSpeedSamples=0;
        }

        private void resetShot() {
            phase=Phase.IDLE;
            previousBurstBall=false;
            gateOpenedAt=-1; shootingBegan=-1;
            burstBaseRpm=Double.NaN;
            resetSpeedQualification();
        }

        @Override public void stopFeed() {
            if (!idleSpin && !intakeAlways) { stopOutputs(); return; }
            resetShot();
            RuntimeException[] first={null};
            if (!idleSpin) {
                robot.shooterOn=false;
                attempt(() -> { if (robot.turret!=null) robot.turret.setPower(0); },first);
                attempt(() -> { if (robot.shooter!=null) robot.shooter.setPower(0); },first);
            }
            // An always-on intake keeps spinning; the transmission waits for the gate to close.
            loggedIntakePower=intakeAlways ? INWARD_INTAKE_POWER : 0; loggedTransmissionPower=0;
            attempt(() -> { if (robot.intake!=null) robot.intake.setPower(intakeAlways ? INWARD_INTAKE_POWER : 0); },first);
            attempt(() -> { if (robot.transmission!=null) robot.transmission.setPower(0); },first);
            closeGate(first);
            if (first[0]!=null) throw first[0];
        }

        @Override public void stopOutputs() {
            robot.shooterOn=false;
            resetShot();
            distanceCount=distanceNext=0; lastDistanceAt=-1;
            RuntimeException[] first={null};
            attempt(() -> { if (robot.turret!=null) robot.turret.setPower(0); },first);
            attempt(() -> { if (robot.shooter!=null) robot.shooter.setPower(0); },first);
            loggedIntakePower=0; loggedTransmissionPower=0;
            attempt(() -> { if (robot.intake!=null) robot.intake.setPower(0); },first);
            attempt(() -> { if (robot.transmission!=null) robot.transmission.setPower(0); },first);
            closeGate(first);
            if (first[0]!=null) throw first[0];
        }
        @Override public void close() {
            idleSpin=false; intakeAlways=false;
            try { stopOutputs(); }
            finally { if (robot.Limelight!=null) robot.Limelight.stop(); }
        }
        private static void attempt(Runnable action, RuntimeException[] first) {
            try { action.run(); }
            catch (RuntimeException error) { if (first[0]==null) first[0]=error; else first[0].addSuppressed(error); }
        }
    }
}
