package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

/** Review gates and initial test policy. None of these approvals is inferred from an export. */
public final class BioBuzzAutoConfig {
    // The team confirmed drivetrain, localization and mechanisms have already been tested.
    // These are not additional approval gates; runtime hardware/sensor checks remain active.
    // Point 4 was clarified to be the intake tip, not the chassis center.
    public static final double ROBOT_LENGTH_IN = 427.284 / 25.4;
    public static final double ROBOT_WIDTH_IN = 448.7 / 25.4;
    public static double INTAKE_TIP_FORWARD_IN = 213.642 / 25.4; // Team measurement in millimeters, converted to inches.
    public static double INTAKE_TIP_LEFT_IN = 0.0; // Centered intake assumption; adjust if offset left/right.

    public static final int PRELOAD_BALL_COUNT = 4;
    public static final int STAGED_BALL_COUNT = 4;
    public static final int FLOWER_BALL_COUNT = 4;
    public static final double COLLECTION_TIMEOUT_S = 10.0;
    // Initial test window, not a measured time to acquire four balls. Tune on the robot.
    public static final double FLOWER_INTAKE_SECONDS = 2.0;
    public static final double SHOOT_TIMEOUT_S = 7.0;

    /** Commands before driving, alongside each path, and after each verified arrival. */
    public static final class Plan {
        public final BioBuzzAutoMechanisms.Action beforeFirstPath;
        public final BioBuzzAutoMechanisms.Action[] duringPath;
        public final BioBuzzAutoMechanisms.Action[] afterPath;
        public boolean movementOnly() {
            if (beforeFirstPath.kind!=BioBuzzAutoMechanisms.Kind.NONE) return false;
            for (int i=0;i<duringPath.length;i++) {
                if (duringPath[i].kind!=BioBuzzAutoMechanisms.Kind.NONE || afterPath[i].kind!=BioBuzzAutoMechanisms.Kind.NONE) return false;
            }
            return true;
        }
        public Plan(BioBuzzAutoMechanisms.Action before, BioBuzzAutoMechanisms.Action[] during,
                    BioBuzzAutoMechanisms.Action[] after) {
            if (before == null || during.length != BioBuzzAutoPaths.PATH_COUNT
                    || after.length != BioBuzzAutoPaths.PATH_COUNT) throw new IllegalArgumentException("Invalid plan");
            beforeFirstPath=before; duringPath=during.clone(); afterPath=after.clone();
            for (int i=0; i<duringPath.length; i++) {
                if (duringPath[i]==null || afterPath[i]==null) throw new IllegalArgumentException("Null action");
                if (duringPath[i].kind!=BioBuzzAutoMechanisms.Kind.NONE
                        && duringPath[i].kind!=BioBuzzAutoMechanisms.Kind.COLLECT) {
                    throw new IllegalArgumentException("Only intake is permitted during travel");
                }
                if (duringPath[i].kind==BioBuzzAutoMechanisms.Kind.COLLECT && duringPath[i].durationSeconds!=0) {
                    throw new IllegalArgumentException("Travel intake must be owned by path completion");
                }
                if (afterPath[i].kind==BioBuzzAutoMechanisms.Kind.COLLECT && afterPath[i].durationSeconds<=0) {
                    throw new IllegalArgumentException("Stationary intake needs a configured duration");
                }
            }
        }
    }

    public static BioBuzzAutoMechanisms.Action[] actions(boolean fullRoute) {
        if (fullRoute) return scoringPlan().afterPath;
        BioBuzzAutoMechanisms.Action[] result = new BioBuzzAutoMechanisms.Action[BioBuzzAutoPaths.PATH_COUNT];
        for (int i = 0; i < result.length; i++) result[i] = BioBuzzAutoMechanisms.Action.none();
        return result;
    }

    public static Plan scoringPlan() {
        BioBuzzAutoMechanisms.Action[] during=actions(false), after=actions(false);
        during[0]=BioBuzzAutoMechanisms.Action.intakeDuringPath(SEGMENT_TIMEOUT_S);
        after[1]=BioBuzzAutoMechanisms.Action.shoot(STAGED_BALL_COUNT, SHOOT_TIMEOUT_S);
        during[3]=BioBuzzAutoMechanisms.Action.intakeDuringPath(SEGMENT_TIMEOUT_S);
        after[3]=BioBuzzAutoMechanisms.Action.intakeFor(FLOWER_INTAKE_SECONDS, COLLECTION_TIMEOUT_S);
        after[4]=BioBuzzAutoMechanisms.Action.shoot(FLOWER_BALL_COUNT, SHOOT_TIMEOUT_S);
        // No flower lever is mapped or commanded. The user specified intake only.
        return new Plan(BioBuzzAutoMechanisms.Action.shoot(PRELOAD_BALL_COUNT, SHOOT_TIMEOUT_S), during, after);
    }

    public static Plan plan(boolean fullRoute) {
        return fullRoute ? scoringPlan() : new Plan(BioBuzzAutoMechanisms.Action.none(), actions(false), actions(false));
    }

    public static final double SEGMENT_TIMEOUT_S = 15.0;
    public static final double TURN_TIMEOUT_S = 5.0;
    public static final double AUTO_TIMEOUT_S = 27.0;
    public static final double PATH_SPEED_IN_S = 12.0;
    public static final double PATH_SPEED_FRACTION = 0.25;
    public static final double POSITION_TOLERANCE_IN = 2.0;
    // Turn with enough room for the rectangle's diagonal, arrival tolerance, and a small nominal margin.
    public static final double PATH1_TURN_Y_IN = Math.hypot(ROBOT_LENGTH_IN, ROBOT_WIDTH_IN)/2
            + POSITION_TOLERANCE_IN + 0.25;
    public static final double HEADING_TOLERANCE_DEG = 3.0;
    public static final double SETTLED_SPEED_IN_S = 2.0;
    public static final double SETTLED_TURN_DEG_S = 8.0;
    public static final double SETTLE_TIME_S = 0.20;
    public static final int TELEMETRY_INTERVAL_MS = 100;

    private BioBuzzAutoConfig() { }

    public static String blockers(int pathCount, boolean fullRoute) {
        if (pathCount<1 || pathCount>BioBuzzAutoPaths.PATH_COUNT) return "Invalid path count";
        if (fullRoute && pathCount!=BioBuzzAutoPaths.PATH_COUNT) return "Full routine requires all five paths";
        if (pathCount>=4 && (!Double.isFinite(INTAKE_TIP_FORWARD_IN) || INTAKE_TIP_FORWARD_IN<=0
                || !Double.isFinite(INTAKE_TIP_LEFT_IN))) {
            return "Set INTAKE_TIP_FORWARD_IN: FLOWER point 4 is the intake tip, not robot center";
        }
        return "";
    }

    public static boolean hasMechanismActions() {
        return true; // Full route always includes the user's preload shot and two collection/shooting cycles.
    }

    public static boolean finite(Pose pose) {
        return pose != null && Double.isFinite(pose.x()) && Double.isFinite(pose.y()) && Double.isFinite(pose.heading());
    }
    public static boolean finite(Velocity v) {
        return v != null && Double.isFinite(v.vx) && Double.isFinite(v.vy) && Double.isFinite(v.omega);
    }
    public static double headingErrorDegrees(Pose target, Pose current) {
        double delta = target.heading() - current.heading();
        return Math.toDegrees(Math.atan2(Math.sin(delta), Math.cos(delta)));
    }
    public static double distance(Pose target, Pose current) { return Math.hypot(target.x()-current.x(), target.y()-current.y()); }
    public static boolean atPose(Pose current, Pose target) {
        return finite(current) && finite(target) && distance(target,current) <= POSITION_TOLERANCE_IN
                && Math.abs(headingErrorDegrees(target,current)) <= HEADING_TOLERANCE_DEG;
    }
    public static boolean settled(Velocity v) {
        return finite(v) && Math.hypot(v.vx,v.vy) <= SETTLED_SPEED_IN_S
                && Math.abs(Math.toDegrees(v.omega)) <= SETTLED_TURN_DEG_S;
    }
}
