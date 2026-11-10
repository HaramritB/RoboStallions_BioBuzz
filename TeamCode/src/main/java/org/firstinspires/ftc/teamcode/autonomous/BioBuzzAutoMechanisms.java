package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.function.Consumer;
import java.util.function.LongSupplier;

/** Mechanism-only boundary. Movement tests stay NONE and never initialize hardware. */
public final class BioBuzzAutoMechanisms {
    public enum Kind { NONE, SHOOT, COLLECT, CENTER_TURRET }
    public static final class Action {
        public final Kind kind;
        public final int count;
        public final double timeoutSeconds;
        private Action(Kind kind, int count, double timeoutSeconds) {
            if (kind != Kind.NONE && (!Double.isFinite(timeoutSeconds) || timeoutSeconds <= 0)) {
                throw new IllegalArgumentException("Mechanism action needs a positive finite deadline");
            }
            if ((kind == Kind.SHOOT || kind == Kind.COLLECT) && count < 1) throw new IllegalArgumentException("Ball count must be positive");
            this.kind=kind; this.count=count; this.timeoutSeconds=timeoutSeconds;
        }
        public static Action none() { return new Action(Kind.NONE,0,0); }
        public static Action shoot(int balls, double timeout) { return new Action(Kind.SHOOT,balls,timeout); }
        public static Action collectUntilBall(double timeout) { return new Action(Kind.COLLECT,1,timeout); }
        public static Action collect(int balls, double timeout) { return new Action(Kind.COLLECT,balls,timeout); }
        public static Action centerTurret(double timeout) { return new Action(Kind.CENTER_TURRET,0,timeout); }
    }

    /** Sampled sensors, not inferred completion based on elapsed feed time alone. */
    public static final class Observation {
        public final boolean ballPresent;
        public final int shotsFired;
        public final double turretDegrees;
        public final boolean sensorsReady;
        /** Monotonic count of physically verified new stored balls; -1 means unsupported. */
        public final int collectedBalls;
        public Observation(boolean ballPresent, int shotsFired, double turretDegrees) {
            this(ballPresent,shotsFired,turretDegrees,true);
        }
        public Observation(boolean ballPresent, int shotsFired, double turretDegrees, boolean sensorsReady) {
            this(ballPresent,shotsFired,turretDegrees,sensorsReady,-1);
        }
        public Observation(boolean ballPresent, int shotsFired, double turretDegrees, boolean sensorsReady, int collectedBalls) {
            this.ballPresent=ballPresent; this.shotsFired=shotsFired; this.turretDegrees=turretDegrees;
            this.sensorsReady=sensorsReady;
            this.collectedBalls=collectedBalls;
        }
    }
    public interface Hardware {
        default boolean supportsCountedCollection() { return false; }
        Observation observe();
        void control(Kind request);
        void stopOutputs();
        void close();
    }

    private final Hardware hardware;
    private Kind request=Kind.NONE;
    private Observation observation;
    private boolean closed;
    private String failure="";

    public BioBuzzAutoMechanisms() { this(null); }
    public BioBuzzAutoMechanisms(Hardware hardware) { this.hardware=hardware; }

    public static BioBuzzAutoMechanisms initialize(HardwareMap map, Robot.Alliance alliance) {
        RobotHardware hardware=new RobotHardware();
        try {
            hardware.robot.initMechanismsOnly(map, false); // Never re-zero turret without a verified physical reference.
            hardware.robot.alliance=alliance;
            if (hardware.robot.ballDistanceSensor == null) throw new IllegalStateException("Ball distance sensor required for verified collection/shooting");
            if (hardware.robot.colorSensor == null) throw new IllegalStateException("Color sensor required by Robot.updateSensors ball detection");
            if (hardware.robot.Limelight == null) throw new IllegalStateException("Limelight required for the existing shot sequence");
            if (!hardware.supportsCountedCollection()) throw new IllegalStateException(
                    "Robot has no verified four-ball inventory/indexing API; full scoring hardware initialization blocked");
            return new BioBuzzAutoMechanisms(hardware);
        } catch (RuntimeException error) {
            try { hardware.close(); } catch (RuntimeException cleanup) { error.addSuppressed(cleanup); }
            throw error;
        }
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
        observe();
        if (hardware != null && !closed && request != Kind.NONE) {
            if (!validObservation()) throw new IllegalStateException("Mechanism sensor sample unavailable/nonfinite");
            hardware.control(request);
        }
    }

    private boolean validObservation() {
        return observation != null && observation.sensorsReady && Double.isFinite(observation.turretDegrees);
    }

    public String inventoryStatus() {
        return hardware != null && hardware.supportsCountedCollection()
                ? "Verified pickup counter: " + (observation==null ? "unavailable" : observation.collectedBalls)
                : "UNAVAILABLE - feeder presence does not count four stored balls";
    }

    public Command command(Action action, LongSupplier clock, Consumer<String> onFailure) {
        final boolean[] active={false};
        final boolean[] previousBall={false};
        final int[] clearTransitions={0};
        final int[] baseline={0};
        final int[] collectionBaseline={-1};
        final int[] previousCollection={-1};
        final long[] began={0};
        final long[] stableSince={-1};
        return Command.build().requiring(this)
                .setStart(() -> {
                    if (hardware == null || closed || !validObservation()) {
                        failure="Mechanism hardware/sensors not initialized";
                        onFailure.accept(failure);
                        return;
                    }
                    if (action.kind==Kind.COLLECT && action.count>1
                            && (!hardware.supportsCountedCollection() || observation.collectedBalls<0)) {
                        failure="Four-ball collection requires a verified inventory/indexing controller";
                        onFailure.accept(failure);
                        return;
                    }
                    if (action.kind==Kind.COLLECT && action.count==1 && observation.ballPresent) {
                        failure="Collection started with an occupied feeder; inventory is unknown";
                        onFailure.accept(failure);
                        return;
                    }
                    active[0]=true;
                    began[0]=clock.getAsLong();
                    stableSince[0]=-1;
                    clearTransitions[0]=0;
                    baseline[0]=observation.shotsFired;
                    collectionBaseline[0]=observation.collectedBalls;
                    previousCollection[0]=observation.collectedBalls;
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
                                hardware.stopOutputs(); // Never feed a fifth while waiting for clear verification.
                            }
                            done=observation.shotsFired-baseline[0]>=action.count && clearTransitions[0]>=action.count;
                            break;
                        case COLLECT:
                            if (action.count>1 && observation.collectedBalls<previousCollection[0]) {
                                failure="Pickup counter reset during collection";
                                onFailure.accept(failure);
                                return true;
                            }
                            previousCollection[0]=observation.collectedBalls;
                            if (action.count>1 && observation.collectedBalls-collectionBaseline[0]>action.count) {
                                failure="Pickup counter exceeded requested count";
                                onFailure.accept(failure);
                                return true;
                            }
                            done=action.count==1 ? observation.ballPresent
                                    : observation.collectedBalls-collectionBaseline[0]>=action.count;
                            break;
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
                    request=Kind.NONE;
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
        request=Kind.NONE;
        if (hardware != null) hardware.close();
    }
    public void setAlliance(Robot.Alliance alliance) {
        if (hardware instanceof RobotHardware) ((RobotHardware) hardware).robot.alliance = alliance;
    }
    public String state() { return hardware == null ? "DISABLED - no mapped actions" : closed ? "STOPPED" : request.name(); }
    public String failure() { return failure; }

    /** Reuses existing Robot controllers. This adapter never calls Robot.drive(), init() or stop(). */
    static final class RobotHardware implements Hardware {
        final Robot robot=new Robot();
        private boolean used;
        @Override public Observation observe() {
            robot.updateSensors();
            return new Observation(robot.ballPresent,robot.shotsFired,robot.turretDegrees(),
                    Double.isFinite(robot.ballDistanceCm) && robot.ballDistanceCm > 0);
        }
        @Override public void control(Kind request) {
            used=true;
            robot.shooterOn=request==Kind.SHOOT;
            if (request==Kind.SHOOT) robot.aimTurret(true,0);
            else if (request==Kind.CENTER_TURRET) robot.turretToAngle(0);
            else robot.turret.setPower(0);
            robot.updateShooter();
            // A stale target cannot keep a gate open through the old feeder's timed branch.
            // Existing updateFeeder() checks flywheel speed before OPENING. Keep its timed
            // feed branch running through the expected RPM dip after a ball enters the wheel.
            boolean shoot=request==Kind.SHOOT && robot.targetFresh && robot.aimedAtTarget();
            if (!shoot) robot.gateOpen=false;
            robot.updateFeeder(shoot,(request==Kind.COLLECT || shoot) ? Robot.INTAKE_POWER : 0);
            robot.updateLed();
        }
        @Override public void stopOutputs() {
            robot.shooterOn=false;
            robot.gateOpen=false;
            RuntimeException[] first={null};
            attempt(() -> { if (robot.turret!=null) robot.turret.setPower(0); },first);
            attempt(() -> { if (robot.shooter!=null) robot.shooter.setPower(0); },first);
            attempt(() -> { if (robot.intake!=null) robot.intake.setPower(0); },first);
            attempt(() -> { if (robot.transmission!=null) robot.transmission.setPower(0); },first);
            // No INIT gate write. Commands require separately verified gate hardware/sequence.
            attempt(() -> { if (used && robot.gate!=null) robot.gate.setPosition(Robot.GATE_CLOSED); },first);
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
