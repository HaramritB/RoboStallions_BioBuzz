package org.firstinspires.ftc.teamcode.v2;

/** Holds a fixed table RPM while calibrating so camera range noise cannot move the setpoint. */
public final class ShooterCalibrationV2 {
    public static final double TRIM_STEP_RPM = 10.0;
    public static final double FINE_TRIM_STEP_RPM = 1.0;

    private boolean enabled;
    private double heldBaseRpm = Double.NaN;
    public double trimRpm;

    public boolean enabled() { return enabled; }

    /** Preserve trim on entry/exit; only the distance table's contribution is frozen. */
    public void toggle(double automaticBaseRpm) {
        if (enabled) {
            enabled = false;
            heldBaseRpm = Double.NaN;
        } else if (Double.isFinite(automaticBaseRpm) && automaticBaseRpm > 0) {
            enabled = true;
            heldBaseRpm = automaticBaseRpm;
        }
    }

    /** NaN tells RobotV2 to keep following the distance table normally. */
    public double baseRpmOverride() { return enabled ? heldBaseRpm : Double.NaN; }

    public void adjust(boolean up, boolean down, boolean fine) {
        int direction = (up ? 1 : 0) - (down ? 1 : 0);
        trimRpm += direction * (fine ? FINE_TRIM_STEP_RPM : TRIM_STEP_RPM);
    }

    public void resetTrim() { trimRpm = 0; }
}
