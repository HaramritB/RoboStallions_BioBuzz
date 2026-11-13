package org.firstinspires.ftc.teamcode.v2;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Passes Auto's last pose and HIVE target to TeleOp.
 *
 * Pedro writes its pose into the Pinpoint itself, and neither TeleOp resets it, so TeleOp reads
 * the same field coordinates Auto ended in. TeleOp only trusts the handoff when the Pinpoint pose
 * still matches what Auto saved (no power cycle, no robot moved between OpModes, same app run).
 */
public final class PoseHandoffV2 {
    public static double MATCH_DISTANCE_IN = 6.0;
    public static double MATCH_HEADING_DEG = 15.0;
    public static double MAX_AGE_MS = 10 * 60 * 1000;

    private static boolean saved;
    private static double x, y, heading;
    private static double targetX = Double.NaN, targetY = Double.NaN;
    private static boolean targetConfirmed;
    private static Robot.Alliance alliance;
    private static double savedAtMs;

    private PoseHandoffV2() { }

    public static synchronized void save(double poseX, double poseY, double headingRad, FieldAimV2 aim,
                                         Robot.Alliance savedAlliance) {
        if (!Double.isFinite(poseX) || !Double.isFinite(poseY) || !Double.isFinite(headingRad)) return;
        saved = true;
        x = poseX;
        y = poseY;
        heading = headingRad;
        targetConfirmed = aim != null && aim.targetConfirmed();
        targetX = aim != null && aim.hasTarget() ? aim.targetX() : Double.NaN;
        targetY = aim != null && aim.hasTarget() ? aim.targetY() : Double.NaN;
        alliance = savedAlliance;
        savedAtMs = RobotV2.nowMs();
    }

    /** True when the current Pinpoint pose still matches Auto's last saved pose. */
    public static synchronized boolean matches(double poseX, double poseY, double headingRad,
                                               Robot.Alliance currentAlliance) {
        if (!saved || alliance != currentAlliance || RobotV2.nowMs() - savedAtMs > MAX_AGE_MS) return false;
        double dHeading = Math.toDegrees(Math.atan2(Math.sin(headingRad - heading), Math.cos(headingRad - heading)));
        return Math.hypot(poseX - x, poseY - y) <= MATCH_DISTANCE_IN && Math.abs(dHeading) <= MATCH_HEADING_DEG;
    }

    public static synchronized boolean hasTarget() {
        return saved && Double.isFinite(targetX) && Double.isFinite(targetY);
    }

    public static synchronized double targetX() { return targetX; }
    public static synchronized double targetY() { return targetY; }
    public static synchronized boolean targetConfirmed() { return targetConfirmed; }
}
