package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.function.Consumer;
import java.util.function.LongSupplier;
import java.util.function.DoubleSupplier;

/** Mechanism-only boundary. Movement tests stay NONE and never initialize hardware. */
public final class BioBuzzAutoMechanisms {
    public enum Kind { NONE, SHOOT, COLLECT, CENTER_TURRET }
    public static final class Action {
        public final Kind kind;
        public final int count; // Requested shot count only. Intake never counts stored balls.
        public final double timeoutSeconds;
        public final double durationSeconds; // Zero for path-owned intake.
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
        Observation observe();
        void control(Kind request);
        void stopOutputs();
        void close();
    }

    private final Hardware hardware;
    private Kind request=Kind.NONE;
    private Observation observation;
    private boolean closed;
    private boolean commandActive;
    private LongSupplier activeClock;
    private long activeStart;
    private double activeTimeout;
    private double activeIntakeDuration;
    private long shotLimit;
    private String failure="";

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

    /** Stationary operator-held test only; mutually exclusive with any command. */
    public void manualIntake(boolean running) {
        if (closed || commandActive || !failure.isEmpty()) throw new IllegalStateException("Mechanism port is unavailable or command-owned");
        request=running ? Kind.COLLECT : Kind.NONE;
        if (!running && hardware!=null) hardware.stopOutputs();
    }

    /** Reads only. Safe to call in INIT; never runs a controller or commands a gate/motor. */
    public void observe() {
        if (hardware != null && !closed) {
            observation=hardware.observe();
            if (!validObservation()) throw new IllegalStateException("Mechanism sensor sample unavailable/nonfinite");
        }
    }
    /** Call exactly once per active FTC loop, after navigation and before Scheduler.execute(). */
    public void update() {
        if (closed) return;
        if (commandActive && activeIntakeDuration>0 && (activeClock.getAsLong()-activeStart)/1e9>=activeIntakeDuration
                && request==Kind.COLLECT) {
            request=Kind.NONE;
            hardware.stopOutputs(); // End timed intake before another actuator write.
        }
        if (commandActive && (activeClock.getAsLong()-activeStart)/1e9>=activeTimeout) {
            request=Kind.NONE;
            hardware.stopOutputs();
            throw new IllegalStateException("Mechanism timeout before actuator output");
        }
        observe();
        if (hardware != null && !closed && request != Kind.NONE) {
            if (!validObservation()) throw new IllegalStateException("Mechanism sensor sample unavailable/nonfinite");
            if (request == Kind.SHOOT && observation.shotsFired >= shotLimit) {
                // The fourth observed exit ends the burst before another feed motor write.
                request = Kind.NONE;
                hardware.stopOutputs();
            } else {
                hardware.control(request);
            }
        }
    }

    private boolean validObservation() {
        return observation != null && observation.sensorsReady && Double.isFinite(observation.turretDegrees);
    }

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
                    if ((now-began[0])/1e9 >= action.timeoutSeconds) {
                        failure="Mechanism timeout: " + action.kind;
                        onFailure.accept(failure);
                        return true;
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
                            if (observation.shotsFired-baseline[0]>action.count) {
                                failure="Shot counter exceeded requested count";
                                onFailure.accept(failure);
                                return true;
                            }
                            if (observation.shotsFired-baseline[0]>=action.count && request==Kind.SHOOT) {
                                request=Kind.NONE;
                                hardware.stopOutputs(); // End the burst at the requested ball count.
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
                        hardware.stopOutputs(); // No extra feed/collection while verifying the result.
                    }
                    return (now-stableSince[0])/1e9>=BioBuzzAutoConfig.SETTLE_TIME_S;
                })
                .setEnd(reason -> {
                    if (!active[0]) return; // Ivy may end children which have never started.
                    request=Kind.NONE; commandActive=false;
                    try { hardware.stopOutputs(); }
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
        request=Kind.NONE;
        if (hardware != null) hardware.close();
    }
    public void setAlliance(Robot.Alliance alliance) {
        if (hardware instanceof RobotHardware) ((RobotHardware) hardware).robot.alliance = alliance;
    }
    /** Explicit INIT operator action, with the turret physically pointing forward. */
    public void zeroTurret() {
        if (closed || commandActive || request!=Kind.NONE) throw new IllegalStateException("Turret zero requires idle mechanisms");
        if (hardware instanceof RobotHardware) ((RobotHardware)hardware).robot.zeroTurret();
    }
    public String diagnostics() { return hardware==null ? "inactive" : hardware.diagnostics(); }
    public String state() { return hardware == null ? "DISABLED - no mapped actions" : closed ? "STOPPED" : request.name(); }
    public String failure() { return failure; }

    /** Reuses existing Robot controllers. This adapter never calls Robot.drive(), init() or stop(). */
    static final class RobotHardware implements Hardware {
        final Robot robot;
        DoubleSupplier headingCw, turnRateCw;
        private boolean shootingRequested;
        private boolean burstFeeding;
        private boolean previousBurstBall;
        private String detail="Not sampled";
        @Override public String diagnostics() { return detail; }
        private boolean used;
        RobotHardware() { this(new Robot()); }
        RobotHardware(Robot robot) { this.robot=robot; }
        @Override public Observation observe() {
            if (headingCw==null) robot.updateSensors();
            else robot.updateMechanismSensors(headingCw.getAsDouble(),turnRateCw.getAsDouble());
            // Count actual feeder presence->clear transitions during the open burst, not
            // timed gate pulses. Observe before control so the fourth exit stops immediately.
            if (burstFeeding) {
                if (previousBurstBall && !robot.ballPresent) robot.shotsFired++;
                previousBurstBall=robot.ballPresent;
            }
            detail=String.format(java.util.Locale.US,"RPM %.0f/%.0f; target fresh=%s aimed=%s; gate calibrated=%s; ball=%s; exits=%d; burst=%s",
                    robot.shooterRpm(),robot.targetRpm+robot.rpmTrim,robot.targetFresh,robot.aimedAtTarget(),
                    robot.gateCalibrated(),robot.ballPresent,robot.shotsFired,burstFeeding);
            return new Observation(robot.ballPresent,robot.shotsFired,robot.turretDegrees(),
                    Double.isFinite(robot.ballDistanceCm) && robot.ballDistanceCm > 0);
        }
        @Override public void control(Kind request) {
            used=true;
            if (request==Kind.SHOOT && !shootingRequested) {
                shootingRequested=true;
                robot.gateOpen=false;
                robot.updateManualFeeder(false,0); // Establish CLOSED once before spin-up.
            }
            robot.shooterOn=request==Kind.SHOOT;
            if (request==Kind.SHOOT) robot.aimTurret(true,0);
            else if (request==Kind.CENTER_TURRET) robot.turretToAngle(0);
            else robot.turret.setPower(0);
            // Same measured distance interpolation and tuned PIDF as working TeleOp.
            robot.updateShooter(robot.distanceBasedShooterRpm());
            boolean targetOk=request==Kind.SHOOT && robot.targetFresh && robot.aimedAtTarget()
                    && Double.isFinite(robot.targetDistanceMeters) && robot.targetDistanceMeters>0;
            if (request==Kind.SHOOT) {
                if (burstFeeding && !targetOk) {
                    stopOutputs();
                    throw new IllegalStateException("Aim/distance lost during shooting burst");
                }
                // Start on the first at-speed sample; do not add a delay or re-check RPM
                // between balls. Keep the gate open through normal flywheel speed dips.
                if (!burstFeeding && targetOk && robot.readyToShoot()) {
                    burstFeeding=true;
                    previousBurstBall=robot.ballPresent;
                }
                robot.gateOpen=burstFeeding;
                // Open the calibrated gate and start both RT-direction motors together.
                robot.updateManualFeeder(false,burstFeeding ? -Robot.INTAKE_POWER : 0);
            } else {
                shootingRequested=burstFeeding=previousBurstBall=false;
                robot.gateOpen=false;
                // Same forward intake, gate-closed indexing and ball sensor behavior as TeleOp.
                robot.updateManualFeeder(false,request==Kind.COLLECT ? Robot.INTAKE_POWER : 0);
            }
            robot.updateLed();
        }
        @Override public void stopOutputs() {
            robot.shooterOn=false;
            robot.gateOpen=false;
            shootingRequested=burstFeeding=previousBurstBall=false;
            RuntimeException[] first={null};
            attempt(() -> { if (robot.turret!=null) robot.turret.setPower(0); },first);
            attempt(() -> { if (robot.shooter!=null) robot.shooter.setPower(0); },first);
            attempt(() -> { if (robot.intake!=null) robot.intake.setPower(0); },first);
            attempt(() -> { if (robot.transmission!=null) robot.transmission.setPower(0); },first);
            // No INIT gate write. Commands require separately verified gate hardware/sequence.
            attempt(() -> { if (used && robot.gateController!=null) robot.gateController.commandSaved(false); },first);
            if (first[0]!=null) throw first[0];
        }
        @Override public void close() {
            try { stopOutputs(); }
            finally { if (robot.Limelight!=null) robot.Limelight.stop(); }
        }
        private static void attempt(Runnable action, RuntimeException[] first) {
            try { action.run(); }
            catch (RuntimeException error) { if (first[0]==null) first[0]=error; else first[0].addSuppressed(error); }
        }
    }
}
