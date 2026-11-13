package org.firstinspires.ftc.teamcode.v2;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.function.DoubleSupplier;

/**
 * V2 turret aim: remembers WHERE the target is on the field, not just which way it was.
 *
 * Every fresh camera frame places the target as a field point: the robot pose and turret angle at
 * the frame's capture time, plus the camera bearing and range. Between frames (or when the tag is
 * lost) the turret keeps pointing at that point from odometry, so driving sideways or spinning
 * keeps it on target. The original controller only corrected for chassis rotation.
 *
 * Other differences from Robot.TurretAimController:
 *  - The 0.22 tracking cap applies to the P/D correction only. Feedforward for chassis spin and
 *    sideways driving is added on top (total limited to TURRET_MAX_POWER), so fast turns no
 *    longer saturate the turret at ~177 deg/s.
 *  - Before the first sighting the turret can pre-aim at a guessed HIVE position (Auto), and only
 *    falls back to a sweep if that guess shows nothing.
 *  - Pure logic: no hardware calls, injectable clock, so it runs in hardware-free tests.
 *
 * Conventions match Robot: turret angles are degrees, clockwise-positive from forward, unwrapped
 * across the full cable-limited travel. Chassis pose is field inches with heading CCW radians.
 */
public final class FieldAimV2 {

    public static double TRACK_FEEDBACK_MAX_POWER = Robot.TURRET_TRACK_MAX_POWER;
    public static double MAX_POWER = Robot.TURRET_MAX_POWER;
    public static double RETURN_MAX_POWER = Robot.TURRET_RETURN_MAX_POWER;
    public static double SEARCH_POWER = Robot.TURRET_SEARCH_POWER;
    public static double RETURN_MARGIN_DEG = Robot.TURRET_RETURN_MARGIN_DEG;
    public static double ROTATION_DEADBAND_DEG_S = Robot.TURRET_ROTATION_DEADBAND_DEG_S;
    /** Turret rotation axis relative to the chassis center, robot frame (inches). Measure and set. */
    public static double TURRET_OFFSET_FORWARD_IN = 0.0;
    public static double TURRET_OFFSET_LEFT_IN = 0.0;
    /** Range used when a frame has no usable distance and no earlier target exists. */
    public static double DEFAULT_RANGE_IN = 70.0;
    public static double MIN_RANGE_IN = 12.0;
    /** Without odometry, a camera goal is held this long before searching (as the original). */
    public static double CAMERA_REACQUIRE_MS = Robot.TURRET_REACQUIRE_MS;
    /** Auto: if the guessed HIVE shows no tag for this long, sweep instead. */
    public static double GUESS_SEARCH_AFTER_MS = 1500;
    public static double AIM_TOLERANCE_DEG = Robot.AIM_TOLERANCE_DEG;
    /** "Aimed" needs a camera confirmation of the target point at most this old. */
    public static double AIM_MAX_OBSERVATION_AGE_MS = 1000;
    public static double STALE_MS = Robot.LIMELIGHT_STALE_MS;
    public static double VELOCITY_WINDOW_MS = 60;
    /** Smooth small vision changes in field coordinates; odometry compensation stays immediate. */
    public static double VISION_SMOOTHING_MS = 100;
    /** Larger changes may be a new CELL after a HIVE tip: acquire those without smoothing... */
    public static double VISION_SNAP_DEG = 3.0;
    /**
     * ...but only once this many consecutive frames agree, so one bad frame (e.g. two tags of a
     * cluster blinking out) cannot twitch the turret. Costs about one frame (~30 ms) on a real jump.
     */
    public static int VISION_SNAP_CONFIRM_FRAMES = 2;
    /** Ramp friction compensation near the goal instead of applying a full kS kick. */
    public static double STATIC_RAMP_DEG = 3.0;

    /** Everything the controller needs for one loop. Callers fill the fields each loop. */
    public static final class Inputs {
        public double turretDeg = Double.NaN;
        public double turretVelDegS;
        public double minDeg = Robot.TURRET_MIN_DEG;
        public double maxDeg = Robot.TURRET_MAX_DEG;

        public boolean poseValid;
        public double xIn, yIn, headingRad, omegaRadS;

        public boolean cameraUsable;
        /** True only for a frame not seen before. */
        public boolean observationFresh;
        /** Turret-frame offset to the target in degrees (positive = turn clockwise). */
        public double errorDeg = Double.NaN;
        public double rangeIn = Double.NaN;
        public double frameTimestamp = Double.NaN;
        public double frameAgeMs = Double.NaN;
    }

    private static final int HISTORY = 64;
    private final double[] histT = new double[HISTORY];
    private final double[] histTurret = new double[HISTORY];
    private final double[] histX = new double[HISTORY];
    private final double[] histY = new double[HISTORY];
    private final double[] histH = new double[HISTORY];
    private final boolean[] histPose = new boolean[HISTORY];
    private int histCount, histNext;

    private final Robot.TurretGains gains;
    private final DoubleSupplier clockMs;

    /** Auto: sweep for the HIVE when nothing has confirmed a target yet. */
    public boolean searchIfLost;
    /** Auto: hold still instead of sweeping (e.g. while the gate is open). */
    public boolean suppressSearch;
    /** Operator trim added to the aim goal, degrees clockwise. Applies immediately. */
    public double aimTrimDeg;

    // Target memory.
    private boolean targetValid;
    private boolean targetConfirmed;
    private boolean targetIsGuess;
    private double targetX = Double.NaN, targetY = Double.NaN;
    private double guessX = Double.NaN, guessY = Double.NaN;
    private double guessAimStartMs = Double.NaN;
    private double lastObservationMs = Double.NEGATIVE_INFINITY;
    private double lastFrameTimestamp = Double.NaN;
    private double cameraGoalDeg = Double.NaN;
    private double cameraGoalMs = Double.NEGATIVE_INFINITY;
    private int pendingJumpFrames;
    private double pendingJumpX = Double.NaN, pendingJumpY = Double.NaN;
    private int searchDirection = 1;
    private boolean settled;
    private boolean returning;

    // Telemetry.
    public String state = "IDLE";
    public double goalDeg = Double.NaN;
    public double desiredRateDegS;
    public double pTerm, dTerm, velocityTerm, staticTerm;
    public double chassisVxInS, chassisVyInS;

    public FieldAimV2(Robot.TurretGains gains) {
        this(gains, () -> System.nanoTime() / 1e6);
    }

    public FieldAimV2(Robot.TurretGains gains, DoubleSupplier clockMs) {
        this.gains = gains;
        this.clockMs = clockMs;
    }

    // ------------------------------------------------------------------ target memory

    /** A field position to pre-aim at before the camera has confirmed anything. */
    public void setGuess(double x, double y) {
        guessX = x;
        guessY = y;
        if (!targetConfirmed && Double.isFinite(x) && Double.isFinite(y)) {
            targetX = x;
            targetY = y;
            targetValid = true;
            targetIsGuess = true;
            guessAimStartMs = Double.NaN;
        }
    }

    /** Forget every target (alliance change, re-zero, or a deliberate reset). */
    public void clearTarget() {
        targetValid = targetConfirmed = targetIsGuess = false;
        targetX = targetY = Double.NaN;
        guessAimStartMs = Double.NaN;
        lastObservationMs = Double.NEGATIVE_INFINITY;
        lastFrameTimestamp = Double.NaN;
        cameraGoalDeg = Double.NaN;
        cameraGoalMs = Double.NEGATIVE_INFINITY;
        pendingJumpFrames = 0;
        settled = returning = false;
        if (Double.isFinite(guessX) && Double.isFinite(guessY)) setGuess(guessX, guessY);
    }

    public boolean hasTarget() { return targetValid; }
    public boolean targetConfirmed() { return targetConfirmed; }
    public boolean targetIsGuess() { return targetIsGuess; }
    public double targetX() { return targetX; }
    public double targetY() { return targetY; }
    public boolean returning() { return returning; }

    public double observationAgeMs() {
        return clockMs.getAsDouble() - lastObservationMs;
    }

    /** Straight-line distance from the turret to the remembered target, or NaN. */
    public double rangeToTargetIn(double x, double y, double headingRad) {
        if (!targetValid || !Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(headingRad)) {
            return Double.NaN;
        }
        double tx = turretX(x, y, headingRad), ty = turretY(x, y, headingRad);
        return Math.hypot(targetX - tx, targetY - ty);
    }

    /** True when the turret is on a camera-confirmed target, within tolerance and not unwinding. */
    public boolean aimed(double turretDeg) {
        return targetConfirmed && !returning && Double.isFinite(goalDeg) && Double.isFinite(turretDeg)
                && observationAgeMs() < AIM_MAX_OBSERVATION_AGE_MS
                && Math.abs(goalDeg - turretDeg) < AIM_TOLERANCE_DEG
                && !state.startsWith("SEARCH") && !state.startsWith("LIMIT");
    }

    // ------------------------------------------------------------------ main update

    public double update(Inputs in) {
        pTerm = dTerm = velocityTerm = staticTerm = 0;
        desiredRateDegS = 0;
        double now = clockMs.getAsDouble();

        boolean poseOk = in.poseValid && Double.isFinite(in.xIn) && Double.isFinite(in.yIn)
                && Double.isFinite(in.headingRad);
        if (!Double.isFinite(in.turretDeg)) {
            state = "INVALID READING";
            goalDeg = Double.NaN;
            return 0;
        }
        record(now, in.turretDeg, poseOk, in.xIn, in.yIn, in.headingRad);
        updateChassisVelocity(now, poseOk, in);

        // 1) A new camera frame moves the target point.
        if (in.cameraUsable && in.observationFresh && Double.isFinite(in.errorDeg)
                && Double.isFinite(in.frameTimestamp) && Double.isFinite(in.frameAgeMs)
                && in.frameAgeMs >= 0 && in.frameAgeMs < STALE_MS
                && in.frameTimestamp != lastFrameTimestamp) {
            lastFrameTimestamp = in.frameTimestamp;
            observe(now, in);
        }

        double turret = in.turretDeg;
        double omegaDegS = Double.isFinite(in.omegaRadS) ? Math.toDegrees(in.omegaRadS) : 0;

        // 2) An encoder reading outside the soft limits gets a short inward recovery.
        returning = false;
        if (turret < in.minDeg || turret > in.maxDeg) {
            double goal = turret < in.minDeg ? in.minDeg + RETURN_MARGIN_DEG : in.maxDeg - RETURN_MARGIN_DEG;
            state = turret < in.minDeg ? "LIMIT RECOVERY RIGHT" : "LIMIT RECOVERY LEFT";
            goalDeg = goal;
            returning = true;
            settled = false;
            return limited(turret, in, feedbackOnly(goal - turret, RETURN_MAX_POWER));
        }

        // 3) Choose what to aim at.
        double desired;
        double desiredRate;
        String label;
        if (poseOk) {
            if (!targetValid) {
                if (searchIfLost) return search(turret, in, "SEARCH");
                // TeleOp before any sighting: hold today's field direction (like the original).
                placeTarget(in.xIn, in.yIn, in.headingRad, turret, DEFAULT_RANGE_IN);
                targetIsGuess = false;
                targetConfirmed = false;
            }
            if (targetIsGuess) {
                if (Double.isNaN(guessAimStartMs)) guessAimStartMs = now;
                if (searchIfLost && now - guessAimStartMs > GUESS_SEARCH_AFTER_MS) {
                    return search(turret, in, "SEARCH (guess empty)");
                }
            }
            double tx = turretX(in.xIn, in.yIn, in.headingRad), ty = turretY(in.xIn, in.yIn, in.headingRad);
            double relX = targetX - tx, relY = targetY - ty;
            double r2 = relX * relX + relY * relY;
            if (r2 < MIN_RANGE_IN * MIN_RANGE_IN) {
                state = "TOO CLOSE: HOLD";
                goalDeg = turret;
                settled = false;
                return 0;
            }
            double bearing = Math.atan2(relY, relX);
            desired = Math.toDegrees(in.headingRad - bearing);
            // Rate of the bearing caused by our own translation (the target is fixed on the field).
            double bearingRate = (relY * chassisVxInS - relX * chassisVyInS) / r2;
            desiredRate = omegaDegS - Math.toDegrees(bearingRate);
            boolean freshVision = now - lastObservationMs < CAMERA_REACQUIRE_MS;
            label = targetIsGuess ? "PRE-AIM GUESS"
                    : !targetConfirmed ? "FIELD HOLD"
                    : freshVision ? "TRACK" : "FIELD TRACK (odometry)";
        } else {
            // No odometry: camera-only, like the original without heading.
            if (now - cameraGoalMs < CAMERA_REACQUIRE_MS && Double.isFinite(cameraGoalDeg)) {
                desired = cameraGoalDeg;
                desiredRate = 0;
                label = "CAMERA ONLY";
            } else {
                return search(turret, in, "SEARCH (no odometry)");
            }
        }

        if (Double.isFinite(aimTrimDeg)) desired += aimTrimDeg;
        double goal = nearestReachableAngle(desired, turret, in.minDeg, in.maxDeg);
        if (!Double.isFinite(goal)) {
            double nearest = desired + 360.0 * Math.rint((turret - desired) / 360.0);
            goal = Math.max(in.minDeg, Math.min(in.maxDeg, nearest));
            label = "FIELD HOLD LIMIT";
            desiredRate = 0;
        }
        goalDeg = goal;
        desiredRateDegS = desiredRate;

        double error = goal - turret;
        // A full turn may be needed to unwind near a cable limit. Never wrap the encoder.
        returning = Math.abs(error) > 180;
        state = returning ? (error > 0 ? "UNWIND RIGHT" : "UNWIND LEFT") : label;

        // Encoder velocity quantization must not wake a settled turret by itself.
        boolean moving = Math.abs(desiredRate) >= ROTATION_DEADBAND_DEG_S;
        if (moving || Math.abs(error) >= gains.deadbandDeg + 0.5) settled = false;
        else if (Math.abs(error) < gains.deadbandDeg) settled = true;
        if (settled) return 0;

        return limited(turret, in, trackPower(error, desiredRate, in.turretVelDegS, returning));
    }

    private void observe(double now, Inputs in) {
        double capture = now - in.frameAgeMs;
        int[] pair = bracket(capture);
        double f = fraction(pair, capture);
        double turretAtCapture = lerp(histTurret[pair[0]], histTurret[pair[1]], f);
        double goalAtCapture = turretAtCapture + in.errorDeg;
        double cameraDelta = Math.toDegrees(wrapRad(Math.toRadians(goalAtCapture - cameraGoalDeg)));
        double cameraAlpha = visionAlpha(capture - cameraGoalMs, cameraDelta);
        cameraGoalDeg = Double.isFinite(cameraGoalDeg)
                ? cameraGoalDeg + cameraAlpha * cameraDelta : goalAtCapture;
        cameraGoalMs = capture;
        double observationGap = capture - lastObservationMs;
        lastObservationMs = capture;
        if (Math.abs(in.errorDeg) >= gains.deadbandDeg) searchDirection = in.errorDeg > 0 ? 1 : -1;

        boolean poseAtCapture = histPose[pair[0]] && histPose[pair[1]];
        if (!poseAtCapture) return;
        double x = lerp(histX[pair[0]], histX[pair[1]], f);
        double y = lerp(histY[pair[0]], histY[pair[1]], f);
        double h = histH[pair[0]] + f * wrapRad(histH[pair[1]] - histH[pair[0]]);
        double range = in.rangeIn;
        if (!Double.isFinite(range) || range < MIN_RANGE_IN) {
            range = targetValid && !targetIsGuess
                    ? Math.hypot(targetX - turretX(x, y, h), targetY - turretY(x, y, h))
                    : DEFAULT_RANGE_IN;
        }
        double tx = turretX(x, y, h), ty = turretY(x, y, h);
        double bearing = h - Math.toRadians(goalAtCapture);
        double measuredX = tx + range * Math.cos(bearing);
        double measuredY = ty + range * Math.sin(bearing);
        double innovation = Math.toDegrees(wrapRad(bearing - Math.atan2(targetY - ty, targetX - tx)));
        boolean tracking = targetConfirmed && targetValid && !targetIsGuess;
        if (tracking && Math.abs(innovation) >= VISION_SNAP_DEG && VISION_SNAP_CONFIRM_FRAMES > 1) {
            boolean agrees = pendingJumpFrames > 0 && Math.abs(Math.toDegrees(wrapRad(
                    bearing - Math.atan2(pendingJumpY - ty, pendingJumpX - tx)))) < VISION_SNAP_DEG;
            pendingJumpFrames = agrees ? pendingJumpFrames + 1 : 1;
            pendingJumpX = measuredX;
            pendingJumpY = measuredY;
            if (pendingJumpFrames < VISION_SNAP_CONFIRM_FRAMES) return; // Hold until confirmed.
        }
        pendingJumpFrames = 0;
        double alpha = tracking ? visionAlpha(observationGap, innovation) : 1.0;
        targetX = alpha == 1 ? measuredX : lerp(targetX, measuredX, alpha);
        targetY = alpha == 1 ? measuredY : lerp(targetY, measuredY, alpha);
        targetValid = true;
        targetConfirmed = true;
        targetIsGuess = false;
    }

    private static double visionAlpha(double dtMs, double innovationDeg) {
        if (!Double.isFinite(innovationDeg) || !Double.isFinite(dtMs) || dtMs <= 0
                || dtMs >= CAMERA_REACQUIRE_MS || Math.abs(innovationDeg) >= VISION_SNAP_DEG
                || VISION_SMOOTHING_MS <= 0) return 1.0;
        return dtMs / (VISION_SMOOTHING_MS + dtMs);
    }

    private void placeTarget(double x, double y, double h, double turretGoalDeg, double range) {
        double bearing = h - Math.toRadians(turretGoalDeg);
        targetX = turretX(x, y, h) + range * Math.cos(bearing);
        targetY = turretY(x, y, h) + range * Math.sin(bearing);
        targetValid = true;
    }

    private double search(double turret, Inputs in, String label) {
        settled = false;
        goalDeg = Double.NaN;
        if (suppressSearch) {
            state = "HOLD (search suppressed)";
            return 0;
        }
        if (turret >= in.maxDeg - RETURN_MARGIN_DEG) searchDirection = -1;
        if (turret <= in.minDeg + RETURN_MARGIN_DEG) searchDirection = 1;
        state = label + (searchDirection > 0 ? " RIGHT" : " LEFT");
        return limited(turret, in, searchDirection * SEARCH_POWER);
    }

    private double trackPower(double error, double desiredRate, double turretVel, boolean unwinding) {
        pTerm = Math.abs(error) < gains.deadbandDeg ? 0 : gains.p * error;
        dTerm = unwinding ? 0 : gains.d * (desiredRate - (Double.isFinite(turretVel) ? turretVel : 0));
        double cap = unwinding ? RETURN_MAX_POWER : TRACK_FEEDBACK_MAX_POWER;
        double feedback = clamp(pTerm + dTerm, cap);
        velocityTerm = !unwinding && Math.abs(desiredRate) >= ROTATION_DEADBAND_DEG_S
                ? gains.kV * desiredRate : 0;
        double power = feedback + velocityTerm;
        double staticScale = Math.abs(desiredRate) >= ROTATION_DEADBAND_DEG_S ? 1.0
                : Math.min(1.0, Math.abs(error) / Math.max(STATIC_RAMP_DEG, 0.001));
        staticTerm = power == 0 ? 0 : Math.copySign(gains.kS * staticScale, power);
        return clamp(power + staticTerm, MAX_POWER);
    }

    private double feedbackOnly(double error, double cap) {
        pTerm = Math.abs(error) < gains.deadbandDeg ? 0 : gains.p * error;
        double power = clamp(pTerm, cap);
        staticTerm = power == 0 ? 0 : Math.copySign(gains.kS, power);
        return clamp(power + staticTerm, cap);
    }

    private static double limited(double angle, Inputs in, double power) {
        if (!Double.isFinite(power) || (angle >= in.maxDeg && power > 0)
                || (angle <= in.minDeg && power < 0)) return 0;
        return clamp(power, Robot.TURRET_MAX_POWER);
    }

    /** The equivalent of desired (mod 360) inside the limits that is closest to current, or NaN. */
    public static double nearestReachableAngle(double desired, double current, double minDeg, double maxDeg) {
        if (!Double.isFinite(desired)) return Double.NaN;
        double firstTurn = Math.ceil((minDeg - desired) / 360.0);
        double lastTurn = Math.floor((maxDeg - desired) / 360.0);
        if (firstTurn > lastTurn) return Double.NaN;
        double nearestTurn = Math.rint((current - desired) / 360.0);
        nearestTurn = Math.max(firstTurn, Math.min(lastTurn, nearestTurn));
        return desired + nearestTurn * 360.0;
    }

    // ------------------------------------------------------------------ helpers

    private static double turretX(double x, double y, double h) {
        return x + TURRET_OFFSET_FORWARD_IN * Math.cos(h) - TURRET_OFFSET_LEFT_IN * Math.sin(h);
    }

    private static double turretY(double x, double y, double h) {
        return y + TURRET_OFFSET_FORWARD_IN * Math.sin(h) + TURRET_OFFSET_LEFT_IN * Math.cos(h);
    }

    private void record(double t, double turret, boolean pose, double x, double y, double h) {
        histT[histNext] = t;
        histTurret[histNext] = turret;
        histPose[histNext] = pose;
        histX[histNext] = pose ? x : Double.NaN;
        histY[histNext] = pose ? y : Double.NaN;
        histH[histNext] = pose ? h : Double.NaN;
        histNext = (histNext + 1) % HISTORY;
        histCount = Math.min(histCount + 1, HISTORY);
    }

    private int index(int age) {
        // age 0 = newest sample.
        return ((histNext - 1 - age) % HISTORY + HISTORY) % HISTORY;
    }

    /** Indices {older, newer} of the samples around time t (clamped to the history ends). */
    private int[] bracket(double t) {
        int newest = index(0);
        if (histCount <= 1 || t >= histT[newest]) return new int[] {newest, newest};
        for (int age = 1; age < histCount; age++) {
            int older = index(age);
            if (histT[older] <= t) return new int[] {older, index(age - 1)};
        }
        int oldest = index(histCount - 1);
        return new int[] {oldest, oldest};
    }

    private double fraction(int[] pair, double t) {
        double span = histT[pair[1]] - histT[pair[0]];
        if (pair[0] == pair[1] || span <= 0) return 0;
        return Math.max(0, Math.min(1, (t - histT[pair[0]]) / span));
    }

    private void updateChassisVelocity(double now, boolean poseOk, Inputs in) {
        chassisVxInS = chassisVyInS = 0;
        if (!poseOk || histCount < 2) return;
        for (int age = 1; age < histCount; age++) {
            int i = index(age);
            double dt = now - histT[i];
            if (dt >= VELOCITY_WINDOW_MS) {
                if (!histPose[i] || dt <= 0) return;
                chassisVxInS = (in.xIn - histX[i]) / (dt / 1000.0);
                chassisVyInS = (in.yIn - histY[i]) / (dt / 1000.0);
                return;
            }
        }
    }

    private static double lerp(double a, double b, double f) {
        return a + (b - a) * f;
    }

    private static double wrapRad(double a) {
        return Math.atan2(Math.sin(a), Math.cos(a));
    }

    private static double clamp(double v, double limit) {
        return Math.max(-limit, Math.min(limit, v));
    }
}
