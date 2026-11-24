package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.autonomous.BioBuzzAutoConfig;

/**
 * Field geometry for odometry-only aiming (no Limelight on the robot since 2026-11-23), shared by
 * AutoPath and TeleOp V2.
 *
 * Coordinates are the robot's own Pedro frame: origin at the field corner behind START, inches.
 * Red and blue run the same AutoPath from the same START in their own frame and the field is
 * 180-degree symmetric, so our HIVE is in the same place for both alliances.
 *
 * Official BIOBUZZ manual: the HIVE structure sits in the field center; the red and blue HIVES are
 * 25.5 in apart center-to-center; each HIVE's two CELLS are 18.84 in apart, one facing the audience
 * side and one the opposite side. On the field (2026-11-23) the +x HIVE turned out to be the
 * opponent's (red) one: our HIVE is on the -x side of center.
 */
public final class FieldGeometry {
    private FieldGeometry() { }

    /**
     * Inside size of the field. The AutoPath HIVE estimate (y = 70.7) points to ~141 in, not Pedro's
     * nominal 144. Change it if your field measures differently.
     */
    public static double FIELD_SIZE_IN = 141.0;
    /** Our HIVE center is this far from the field center in x (half the official 25.5 in). */
    public static double HIVE_HALF_SPACING_IN = 25.5 / 2;
    /**
     * Which side of center our HIVE is on in x: -1 (the -x side). A field test on 2026-11-23 with +1
     * aimed at the red HIVE; the old camera-era estimate had the side mirrored.
     */
    public static double OUR_HIVE_SIDE_X = -1;
    /** Each CELL is this far from its HIVE center in y (START side and far side). */
    public static double CELL_HALF_SPACING_IN = 18.84 / 2;
    /** The aim switches CELLS only after the robot is this far past the center line. */
    public static double CELL_SWITCH_HYSTERESIS_IN = 3.0;
    /**
     * Shot distance = straight line to the CELL using this height difference (0 = horizontal range).
     * The RPM table was measured with Limelight distances; re-check it with test shots.
     */
    public static double SHOT_HEIGHT_IN = 0.0;

    /**
     * Turret rotation axis: centered left-right, 145.086 mm from the back of the robot (measured
     * 2026-11-23). The robot is 423.17683 mm long, so the axis is 2.62 in BEHIND the robot center.
     */
    public static final double TURRET_FROM_BACK_IN = 145.086 / 25.4;
    public static double turretForwardOffsetIn() {
        return TURRET_FROM_BACK_IN - BioBuzzAutoConfig.ROBOT_LENGTH_IN / 2;
    }

    public static double centerIn() { return FIELD_SIZE_IN / 2; }

    public static double hiveXIn() { return centerIn() + OUR_HIVE_SIDE_X * HIVE_HALF_SPACING_IN; }

    /** CELL on the START half (y < center) or on the far half. */
    public static double cellYIn(boolean farHalf) {
        return centerIn() + (farHalf ? CELL_HALF_SPACING_IN : -CELL_HALF_SPACING_IN);
    }

    /**
     * Which CELL to aim at for a robot at {@code y}: the one on its half of the field. With
     * {@code chosen} false (first call) it picks by the center line; after that it switches only
     * once the robot is CELL_SWITCH_HYSTERESIS_IN past the line, so it never flickers.
     */
    public static boolean aimFarCell(double y, boolean currentlyFar, boolean chosen) {
        double c = centerIn();
        if (!chosen) return y >= c;
        if (currentlyFar && y < c - CELL_SWITCH_HYSTERESIS_IN) return false;
        if (!currentlyFar && y > c + CELL_SWITCH_HYSTERESIS_IN) return true;
        return currentlyFar;
    }

    /** Shot distance in meters for the RPM table, from the horizontal range in inches. */
    public static double shotDistanceM(double rangeIn) {
        return Double.isFinite(rangeIn) ? Math.hypot(rangeIn, SHOT_HEIGHT_IN) * 0.0254 : Double.NaN;
    }

    /**
     * Park-corner pose for re-localizing: the corner next to the parking zone (x = 0, far wall), the
     * same for both alliances, with the robot pressed square into it. Heading snaps to 90 degrees.
     */
    public static double[] parkCornerPose(double headingRad) {
        double snapped = (Math.PI / 2) * Math.rint(headingRad / (Math.PI / 2));
        boolean lengthAlongY = Math.abs(Math.sin(snapped)) > 0.5;
        double halfX = (lengthAlongY ? BioBuzzAutoConfig.ROBOT_WIDTH_IN : BioBuzzAutoConfig.ROBOT_LENGTH_IN) / 2;
        double halfY = (lengthAlongY ? BioBuzzAutoConfig.ROBOT_LENGTH_IN : BioBuzzAutoConfig.ROBOT_WIDTH_IN) / 2;
        return new double[]{halfX, FIELD_SIZE_IN - halfY, snapped};
    }
}
