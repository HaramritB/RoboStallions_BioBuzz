package org.firstinspires.ftc.teamcode.v2;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * TeleOp gate + feed logic.
 *
 * Hold-to-shoot runs the same sequence Auto already uses on the field: wait until the flywheel has
 * been at speed for a moment and the turret is aimed, briefly back the transmission off so the
 * packed balls stop loading the gate, open the gate, wait for it to travel, then feed inward
 * until the button is released. The original TeleOp had none of this: the operator toggled the
 * gate by hand and driver 1 had to hold intake to push balls through.
 *
 * The manual gate toggle still works when hold-to-shoot is not active, now with the same relief
 * pulse on opening so the gate servo is not fighting a packed stack.
 */
public final class ShotSequencerV2 {
    // Same values as BioBuzzAutoConfig's GATE_OPEN_WAIT / GATE_RELIEF settings.
    public static double GATE_OPEN_WAIT_MS = 250;
    public static double RELIEF_MS = 100;
    public static double RELIEF_TRANSMISSION_POWER = 0.30;
    /** The flywheel must stay inside the tight window this long before the gate opens. */
    public static double READY_STABLE_MS = 150;
    public static double READY_MAX_SAMPLE_GAP_MS = 150;

    public static final double INWARD_INTAKE = -Robot.INTAKE_POWER;
    public static final double INWARD_TRANSMISSION = -Robot.TRANSMISSION_INDEX_POWER;

    public enum Phase { IDLE, SPINUP, GATE_OPENING, FEEDING }

    public Phase phase = Phase.IDLE;
    public boolean gateOpen;
    public double intakePower;
    public double transmissionPower;
    /** Flywheel stable and turret aimed: what the operator rumble signals. */
    public boolean ready;
    public boolean speedStable;
    public String waitingFor = "";

    private double speedOkSinceMs = Double.NaN;
    private double lastSpeedSampleMs = Double.NaN;
    private double gateCommandedOpenMs = Double.NaN;
    private boolean lastGateOpen;

    /**
     * @param wantShoot   operator holds hold-to-shoot
     * @param forceShoot  operator override: skip the aim check (flywheel must still be near speed)
     * @param atSpeed     flywheel inside the tight RPM window this loop
     * @param nearSpeed   flywheel at least the force-shot fraction of target
     * @param aimed       turret on a camera-confirmed target
     * @param manualGateOpen state of the manual gate toggle (used when not holding hold-to-shoot)
     * @param driverIntake driver 1 intake command: negative inward, positive outward
     */
    public void update(double nowMs, boolean wantShoot, boolean forceShoot, boolean atSpeed,
                       boolean nearSpeed, boolean aimed, boolean gateCalibrated,
                       boolean manualGateOpen, double driverIntake) {
        if (Double.isFinite(lastSpeedSampleMs) && (nowMs < lastSpeedSampleMs
                || nowMs - lastSpeedSampleMs > READY_MAX_SAMPLE_GAP_MS)) {
            speedOkSinceMs = Double.NaN;
        }
        lastSpeedSampleMs = nowMs;
        if (atSpeed) {
            if (Double.isNaN(speedOkSinceMs)) speedOkSinceMs = nowMs;
        } else {
            speedOkSinceMs = Double.NaN;
        }
        speedStable = !Double.isNaN(speedOkSinceMs) && nowMs - speedOkSinceMs >= READY_STABLE_MS;
        ready = speedStable && aimed;

        if (!gateCalibrated) {
            // Same as the original: no gate commands and no transmission without calibration.
            phase = Phase.IDLE;
            gateOpen = false;
            intakePower = driverIntake;
            transmissionPower = 0;
            waitingFor = "gate calibration";
            trackGate(nowMs);
            return;
        }

        boolean shooting = wantShoot || forceShoot;
        if (!shooting) {
            phase = Phase.IDLE;
            waitingFor = "";
            gateOpen = manualGateOpen;
            trackGate(nowMs);
            double sinceOpen = gateOpen ? nowMs - gateCommandedOpenMs : Double.POSITIVE_INFINITY;
            intakePower = driverIntake;
            if (gateOpen && sinceOpen < RELIEF_MS) {
                transmissionPower = -Math.signum(INWARD_TRANSMISSION) * RELIEF_TRANSMISSION_POWER;
            } else if (driverIntake < 0) {
                // Never push balls into a gate that is still moving.
                transmissionPower = gateOpen && sinceOpen < GATE_OPEN_WAIT_MS ? 0 : INWARD_TRANSMISSION;
            } else if (driverIntake > 0) {
                transmissionPower = -INWARD_TRANSMISSION;
            } else {
                transmissionPower = 0;
            }
            return;
        }

        if (phase == Phase.IDLE) phase = Phase.SPINUP;
        if (phase == Phase.SPINUP) {
            boolean go = forceShoot ? nearSpeed : speedStable && aimed;
            waitingFor = go ? "" : forceShoot ? "flywheel (forced)"
                    : !speedStable ? "flywheel speed" : "turret aim";
            if (go) phase = Phase.GATE_OPENING;
        }

        gateOpen = phase != Phase.SPINUP;
        trackGate(nowMs);
        double sinceOpen = gateOpen ? nowMs - gateCommandedOpenMs : 0;
        if (phase == Phase.GATE_OPENING && sinceOpen >= GATE_OPEN_WAIT_MS) phase = Phase.FEEDING;

        switch (phase) {
            case SPINUP:
                // Keep collecting while waiting; balls stack against the closed gate.
                intakePower = driverIntake;
                transmissionPower = driverIntake < 0 ? INWARD_TRANSMISSION : 0;
                break;
            case GATE_OPENING:
                intakePower = driverIntake < 0 ? INWARD_INTAKE : 0;
                transmissionPower = sinceOpen < RELIEF_MS
                        ? -Math.signum(INWARD_TRANSMISSION) * RELIEF_TRANSMISSION_POWER : 0;
                break;
            default:
                waitingFor = "";
                intakePower = INWARD_INTAKE;
                transmissionPower = INWARD_TRANSMISSION;
        }
    }

    private void trackGate(double nowMs) {
        if (gateOpen && !lastGateOpen) gateCommandedOpenMs = nowMs;
        lastGateOpen = gateOpen;
    }

    /** Editing a setpoint starts a fresh readiness interval, even for a 1-RPM change. */
    public void resetSpeedReadiness() {
        speedOkSinceMs = Double.NaN;
        lastSpeedSampleMs = Double.NaN;
        speedStable = ready = false;
    }
}
