package org.firstinspires.ftc.teamcode.v2;

import org.firstinspires.ftc.teamcode.Robot;

/**
 * Converts the BLUE-side AutoPath geometry to the RED side.
 *
 * Default: mirror across the field's center line (x -> 144 - x, heading -> 180 deg - heading),
 * the same convention top DECODE teams used with Pedro coordinates. If the BioBuzz field turns
 * out to be rotationally symmetric instead, set MODE to ROTATE_180. Red has NOT been driven yet.
 */
public final class FieldMirrorV2 {
    public enum Mode { MIRROR_X, ROTATE_180 }

    public static Mode MODE = Mode.MIRROR_X;
    public static final double FIELD_SIZE_IN = 144.0;

    private FieldMirrorV2() { }

    public static double x(double blueX, double blueY, Robot.Alliance alliance) {
        return alliance == Robot.Alliance.BLUE ? blueX : FIELD_SIZE_IN - blueX;
    }

    public static double y(double blueX, double blueY, Robot.Alliance alliance) {
        if (alliance == Robot.Alliance.BLUE || MODE == Mode.MIRROR_X) return blueY;
        return FIELD_SIZE_IN - blueY;
    }

    public static double headingDeg(double blueHeadingDeg, Robot.Alliance alliance) {
        if (alliance == Robot.Alliance.BLUE) return blueHeadingDeg;
        return MODE == Mode.MIRROR_X ? 180.0 - blueHeadingDeg : blueHeadingDeg + 180.0;
    }
}
