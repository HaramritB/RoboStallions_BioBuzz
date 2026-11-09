package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

/** Review gates and initial test policy; these are NOT measured robot tuning values. */
public final class BioBuzzAutoConfig {
    // Keep all false until the corresponding on-robot/field checks in AUTONOMOUS.md pass.
    public static final boolean HARDWARE_VALIDATED = false;
    public static final boolean CALIBRATION_VALIDATED = false;
    public static final boolean ROUTE_VALIDATED = false;

    public static final double SEGMENT_TIMEOUT_S = 10.0;
    public static final double AUTO_TIMEOUT_S = 27.0;
    public static final double POSITION_TOLERANCE_IN = 2.0;
    public static final double HEADING_TOLERANCE_DEG = 5.0;
    public static final double SETTLED_SPEED_IN_S = 2.0;
    public static final double SETTLED_TURN_DEG_S = 10.0;
    public static final int TELEMETRY_INTERVAL_MS = 100;

    private BioBuzzAutoConfig() { }

    public static String blockers() {
        String result = "";
        if (!HARDWARE_VALIDATED) result += "Hardware/directions unverified; ";
        if (!CALIBRATION_VALIDATED) result += "Calibration not physically confirmed; ";
        if (!ROUTE_VALIDATED) result += "EXAMPLE route, not field validated; ";
        return result;
    }

    public static boolean finite(Pose pose) {
        return pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y())
                && Double.isFinite(pose.heading());
    }

    public static boolean finite(Velocity velocity) {
        return velocity != null && Double.isFinite(velocity.vx) && Double.isFinite(velocity.vy)
                && Double.isFinite(velocity.omega);
    }

    public static double headingErrorDegrees(Pose target, Pose current) {
        double delta = target.heading() - current.heading();
        return Math.toDegrees(Math.atan2(Math.sin(delta), Math.cos(delta)));
    }

    public static double distance(Pose target, Pose current) {
        return Math.hypot(target.x() - current.x(), target.y() - current.y());
    }

    public static boolean atPose(Pose current, Pose target) {
        return finite(current) && finite(target)
                && distance(target, current) <= POSITION_TOLERANCE_IN
                && Math.abs(headingErrorDegrees(target, current)) <= HEADING_TOLERANCE_DEG;
    }

    public static boolean settled(Velocity velocity) {
        return finite(velocity) && Math.hypot(velocity.vx, velocity.vy) <= SETTLED_SPEED_IN_S
                && Math.abs(Math.toDegrees(velocity.omega)) <= SETTLED_TURN_DEG_S;
    }
}
