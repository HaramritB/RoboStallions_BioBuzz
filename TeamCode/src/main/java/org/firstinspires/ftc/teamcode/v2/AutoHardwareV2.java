package org.firstinspires.ftc.teamcode.v2;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.autonomous.BioBuzzAutoConfig;
import org.firstinspires.ftc.teamcode.autonomous.BioBuzzAutoMechanisms;

import java.util.Locale;
import java.util.function.LongSupplier;

/**
 * V2 mechanism adapter for BioBuzzAutoMechanisms (whose tested command, timeout and fault-tolerance
 * logic is reused unchanged). Same shooting sequence as the original RobotHardware adapter, with:
 *  - field-anchored aim from the Pedro pose, pre-aimed at a HIVE guess, so the turret is already
 *    on target when the robot stops (the original swept for the tag at every shooting spot)
 *  - upward-CELL targeting, filtered/odometry shot distance, voltage-compensated F
 *  - shorter readiness waits (the turret no longer has to find the tag after stopping)
 *  - a timed burst ends early once every planned ball has been seen leaving
 */
final class AutoHardwareV2 implements BioBuzzAutoMechanisms.Hardware {
    /** Original: SHOOT_MIN_SPINUP_SECONDS = 0.30. */
    public static double MIN_STOPPED_S = 0.20;
    /** Original: SHOOT_READY_STABLE_SECONDS = 0.30. */
    public static double READY_STABLE_S = 0.15;
    public static boolean EARLY_BURST_END = true;
    /** Feeder sensor must read empty this long after the last counted ball. */
    public static double EARLY_END_CLEAR_S = 0.25;
    public static double EARLY_END_MIN_FEED_S = 0.50;

    private enum Phase { IDLE, SPINUP, GATE_OPENING, FEEDING }

    static final double INWARD_INTAKE_POWER = -Robot.INTAKE_POWER;
    static final double INWARD_TRANSMISSION_POWER = -Robot.TRANSMISSION_INDEX_POWER;

    final RobotV2 v2;
    final Robot robot;
    private final LongSupplier clock;
    private Phase phase = Phase.IDLE;
    private boolean idleSpin;
    private boolean intakeAlways;
    private boolean gateCommandOpen;
    private long gateMovedAt = -1;
    private boolean previousBurstBall;
    private long gateOpenedAt = -1;
    private long feedingStartedAt = -1;
    private long lastBallSeenAt = -1;
    private int burstShotBaseline;
    private double burstBaseRpm = Double.NaN;
    private long shootingBegan = -1, speedStableSince = -1, lastSpeedSample = -1;
    private double stableTargetRpm = Double.NaN;
    private int stableSpeedSamples;
    private double feedStartRpm = Double.NaN, feedTargetRpm = Double.NaN, minBurstRpm = Double.NaN;
    private String feedTrigger = "";
    private int aimLossFeedLoops;
    private String earlyEnd = "";
    private String detail = "Not sampled";
    private boolean used;

    AutoHardwareV2(RobotV2 v2, LongSupplier clock) {
        this.v2 = v2;
        this.robot = v2.robot;
        this.clock = clock;
    }

    @Override public String diagnostics() {
        double stableSeconds = speedStableSince < 0 ? 0 : (clock.getAsLong() - speedStableSince) / 1e9;
        String readiness = String.format(Locale.US, "; %s; speed stable %.2f/%.2f s (+/-%.0f RPM)",
                phase, stableSeconds, READY_STABLE_S, BioBuzzAutoConfig.SHOOT_READY_RPM_TOLERANCE);
        String burst = Double.isFinite(feedStartRpm) ? String.format(Locale.US,
                "; feed-start RPM %.0f/%.0f (%s); min burst RPM %.0f; aim lost %d loops%s",
                feedStartRpm, feedTargetRpm, feedTrigger, minBurstRpm, aimLossFeedLoops, earlyEnd) : "";
        return detail + readiness + burst;
    }

    @Override public boolean shootingBurstActive() { return phase == Phase.FEEDING; }
    @Override public void setShooterIdle(boolean spinning) { idleSpin = spinning; }
    @Override public void setIntakeAlways(boolean running) { intakeAlways = running; }
    @Override public void setSearchWhenTagLost(boolean search) { v2.aim.searchIfLost = search; }

    @Override public boolean burstFinishedEarly(int plannedBalls) {
        if (!EARLY_BURST_END || phase != Phase.FEEDING || plannedBalls <= 0 || feedingStartedAt < 0) return false;
        long now = clock.getAsLong();
        boolean allLeft = robot.shotsFired - burstShotBaseline >= plannedBalls;
        boolean fedLongEnough = (now - feedingStartedAt) / 1e9 >= EARLY_END_MIN_FEED_S;
        boolean empty = !robot.ballPresent && lastBallSeenAt >= 0
                && (now - lastBallSeenAt) / 1e9 >= EARLY_END_CLEAR_S;
        if (allLeft && fedLongEnough && empty) {
            earlyEnd = String.format(Locale.US, "; burst ended early after %.2f s", (now - feedingStartedAt) / 1e9);
            return true;
        }
        return false;
    }

    @Override public BioBuzzAutoMechanisms.Observation observe() {
        v2.readVision();
        v2.readBall();
        long now = clock.getAsLong();
        if (robot.ballPresent) lastBallSeenAt = now;
        // Count real presence -> clear transitions at the feeder sensor during the open burst.
        if (phase == Phase.FEEDING) {
            if (previousBurstBall && !robot.ballPresent) robot.shotsFired++;
            previousBurstBall = robot.ballPresent;
        }
        detail = String.format(Locale.US,
                "RPM %.0f/%.0f; cell %s fresh=%s aimed=%s aim=%s; distance %.2f m (%s); gate cal=%s; ball=%s %.1f cm; exits=%d",
                robot.shooterRpm(), v2.commandedRpm(), HiveTargetV2.label(v2.hive.cell, robot.alliance),
                v2.frameFresh, v2.aimed(), v2.aim.state, v2.shotDistanceM, v2.distanceFilter.source,
                robot.gateCalibrated(), robot.ballPresent, robot.ballDistanceCm, robot.shotsFired);
        return new BioBuzzAutoMechanisms.Observation(robot.ballPresent, robot.shotsFired, robot.turretDegrees(),
                Double.isFinite(robot.ballDistanceCm) && robot.ballDistanceCm > 0);
    }

    @Override public void control(BioBuzzAutoMechanisms.Kind request) {
        used = true;
        long now = clock.getAsLong();
        boolean shooting = request == BioBuzzAutoMechanisms.Kind.SHOOT;
        boolean spin = shooting || idleSpin;

        // Never sweep while the gate is open; field tracking itself continues.
        v2.aim.suppressSearch = shooting && phase != Phase.IDLE && phase != Phase.SPINUP;
        if (request == BioBuzzAutoMechanisms.Kind.CENTER_TURRET) v2.driveTurret(false, true, 0);
        else v2.driveTurret(spin, false, 0);

        v2.refreshVoltage();
        v2.updateShotDistance();
        boolean latched = (phase == Phase.GATE_OPENING || phase == Phase.FEEDING) && Double.isFinite(burstBaseRpm);
        v2.runShooter(spin, 0, latched ? burstBaseRpm : Double.NaN);
        double measuredRpm = robot.shooterRpm();
        double targetRpm = v2.commandedRpm();

        if (shooting) {
            if (phase == Phase.IDLE) {
                phase = Phase.SPINUP;
                shootingBegan = now;
                resetSpeedQualification();
                feedStartRpm = feedTargetRpm = minBurstRpm = Double.NaN;
                feedTrigger = "";
                earlyEnd = "";
                aimLossFeedLoops = 0;
            }
            boolean aimed = v2.aimed();
            if (phase == Phase.SPINUP) {
                double waited = (now - shootingBegan) / 1e9;
                boolean aimGraceOver = waited >= BioBuzzAutoConfig.SHOOT_AIM_GRACE_SECONDS;
                boolean stable = speedQualified(now, robot.gateCalibrated() && (aimed || aimGraceOver),
                        measuredRpm, targetRpm);
                boolean forced = !stable && robot.gateCalibrated()
                        && waited >= BioBuzzAutoConfig.SHOOT_FORCE_FEED_SECONDS
                        && Double.isFinite(measuredRpm) && Double.isFinite(targetRpm) && targetRpm > 0
                        && measuredRpm >= BioBuzzAutoConfig.SHOOT_FORCE_MIN_RPM_FRACTION * targetRpm;
                if (stable || forced) {
                    phase = Phase.GATE_OPENING;
                    gateOpenedAt = now;
                    burstBaseRpm = v2.baseRpm;
                    feedTrigger = forced ? String.format(Locale.US, "forced at %.1f s", waited)
                            : aimed ? "aimed" : "no aimed tag: table fallback";
                    feedStartRpm = measuredRpm;
                    feedTargetRpm = targetRpm;
                    minBurstRpm = measuredRpm;
                }
            }
            if (phase == Phase.GATE_OPENING
                    && (now - gateOpenedAt) / 1e9 >= BioBuzzAutoConfig.GATE_OPEN_WAIT_SECONDS) {
                phase = Phase.FEEDING;
                feedingStartedAt = now;
                burstShotBaseline = robot.shotsFired;
                previousBurstBall = robot.ballPresent;
            }
            if (phase == Phase.FEEDING && !aimed) aimLossFeedLoops++;
            if (phase != Phase.SPINUP && Double.isFinite(measuredRpm)) {
                minBurstRpm = Double.isFinite(minBurstRpm) ? Math.min(minBurstRpm, measuredRpm) : measuredRpm;
            }
            boolean feeding = phase == Phase.FEEDING;
            boolean relieving = intakeAlways && phase == Phase.GATE_OPENING
                    && (now - gateOpenedAt) / 1e9 < BioBuzzAutoConfig.GATE_RELIEF_SECONDS;
            driveFeeder(now, phase != Phase.SPINUP,
                    feeding || intakeAlways ? INWARD_INTAKE_POWER : 0,
                    feeding ? INWARD_TRANSMISSION_POWER
                            : relieving ? -Math.signum(INWARD_TRANSMISSION_POWER)
                            * BioBuzzAutoConfig.GATE_RELIEF_TRANSMISSION_POWER : 0);
        } else {
            resetShot();
            boolean collecting = request == BioBuzzAutoMechanisms.Kind.COLLECT
                    || (intakeAlways && request == BioBuzzAutoMechanisms.Kind.NONE);
            driveFeeder(now, false, collecting ? INWARD_INTAKE_POWER : 0,
                    collecting ? INWARD_TRANSMISSION_POWER : 0);
        }
    }

    /** Calibrated gate plus both feed motors; the transmission never pushes balls into a moving gate. */
    private void driveFeeder(long now, boolean open, double intakePower, double transmissionPower) {
        boolean calibrated = robot.gateCalibrated();
        open &= calibrated;
        if (open != gateCommandOpen) {
            gateCommandOpen = open;
            gateMovedAt = now;
        }
        boolean gateMoving = gateMovedAt >= 0
                && (now - gateMovedAt) / 1e9 < BioBuzzAutoConfig.GATE_OPEN_WAIT_SECONDS;
        robot.gateOpen = open;
        if (calibrated) robot.gateController.commandSaved(open);
        robot.intake.setPower(intakePower);
        boolean intoGate = transmissionPower != 0
                && Math.signum(transmissionPower) == Math.signum(INWARD_TRANSMISSION_POWER);
        robot.transmission.setPower(calibrated && !(gateMoving && intoGate) ? transmissionPower : 0);
    }

    private boolean speedQualified(long now, boolean targetOk, double measuredRpm, double targetRpm) {
        if (!targetOk || !Double.isFinite(measuredRpm) || !Double.isFinite(targetRpm)
                || measuredRpm <= 0 || targetRpm <= 0
                || Math.abs(measuredRpm - targetRpm) > BioBuzzAutoConfig.SHOOT_READY_RPM_TOLERANCE) {
            resetSpeedQualification();
            return false;
        }
        if (speedStableSince < 0
                || (lastSpeedSample >= 0 && (now - lastSpeedSample) / 1e9 > BioBuzzAutoConfig.SHOOT_READY_MAX_SAMPLE_GAP_SECONDS)
                || Math.abs(targetRpm - stableTargetRpm) > BioBuzzAutoConfig.SHOOT_READY_TARGET_CHANGE_RPM) {
            speedStableSince = now;
            stableTargetRpm = targetRpm;
            stableSpeedSamples = 0;
        }
        if (lastSpeedSample != now || stableSpeedSamples == 0) stableSpeedSamples++;
        lastSpeedSample = now;
        return shootingBegan >= 0 && (now - shootingBegan) / 1e9 >= MIN_STOPPED_S
                && (now - speedStableSince) / 1e9 >= READY_STABLE_S
                && stableSpeedSamples >= BioBuzzAutoConfig.SHOOT_READY_MIN_SAMPLES;
    }

    private void resetSpeedQualification() {
        speedStableSince = lastSpeedSample = -1;
        stableTargetRpm = Double.NaN;
        stableSpeedSamples = 0;
    }

    private void resetShot() {
        phase = Phase.IDLE;
        previousBurstBall = false;
        gateOpenedAt = feedingStartedAt = -1;
        shootingBegan = -1;
        burstBaseRpm = Double.NaN;
        v2.aim.suppressSearch = false;
        resetSpeedQualification();
    }

    private void closeGate(RuntimeException[] first) {
        if (gateCommandOpen) gateMovedAt = clock.getAsLong();
        gateCommandOpen = false;
        robot.gateOpen = false;
        attempt(() -> { if (used && robot.gateController != null) robot.gateController.commandSaved(false); }, first);
    }

    @Override public void stopFeed() {
        if (!idleSpin && !intakeAlways) { stopOutputs(); return; }
        resetShot();
        RuntimeException[] first = {null};
        if (!idleSpin) {
            robot.shooterOn = false;
            attempt(() -> { if (robot.turret != null) robot.turret.setPower(0); }, first);
            attempt(() -> { if (robot.shooter != null) robot.shooter.setPower(0); }, first);
        }
        attempt(() -> { if (robot.intake != null) robot.intake.setPower(intakeAlways ? INWARD_INTAKE_POWER : 0); }, first);
        attempt(() -> { if (robot.transmission != null) robot.transmission.setPower(0); }, first);
        closeGate(first);
        if (first[0] != null) throw first[0];
    }

    @Override public void stopOutputs() {
        robot.shooterOn = false;
        resetShot();
        RuntimeException[] first = {null};
        attempt(() -> { if (robot.turret != null) robot.turret.setPower(0); }, first);
        attempt(() -> { if (robot.shooter != null) robot.shooter.setPower(0); }, first);
        attempt(() -> { if (robot.intake != null) robot.intake.setPower(0); }, first);
        attempt(() -> { if (robot.transmission != null) robot.transmission.setPower(0); }, first);
        closeGate(first);
        if (first[0] != null) throw first[0];
    }

    @Override public void close() {
        idleSpin = false;
        intakeAlways = false;
        try { stopOutputs(); }
        finally { if (robot.Limelight != null) robot.Limelight.stop(); }
    }

    private static void attempt(Runnable action, RuntimeException[] first) {
        try { action.run(); }
        catch (RuntimeException error) {
            if (first[0] == null) first[0] = error;
            else first[0].addSuppressed(error);
        }
    }
}
