package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;

/**
 * EXAMPLE ONLY. Inches; field origin at lower-left on the chosen Pedro field drawing,
 * +X right, +Y up, 0 degrees along +X, positive heading counterclockwise.
 * Robot pose references the localizer's robot center. At the example start the robot's
 * forward axis must point along +X. Alliance/start-wall placement is NOT established.
 */
public final class BioBuzzAutoPaths {
    private static final PoseFactory DEGREES = PoseFactory.degrees();
    public final Pose startPose = DEGREES.of(24, 24, 0);
    public final Pose scorePose = DEGREES.of(48, 48, 90);
    public final Pose parkPose = DEGREES.of(72, 48, 90);

    public Path startToScore() {
        return straight(startPose, scorePose, false);
    }

    public Path scoreToPark() {
        return straight(scorePose, parkPose, true);
    }

    /** Constant heading requires matching endpoint headings; otherwise interpolate linearly. */
    public static Path straight(Pose start, Pose end, boolean constantHeading) {
        validate(start, end, constantHeading);
        return constantHeading ? line(start, end).constant(start) : line(start, end).linear(start, end);
    }

    /** Intermediate poses specify only curve control positions, not heading waypoints. */
    public static Path bezier(Pose start, Pose end, boolean constantHeading, Pose... controls) {
        validate(start, end, constantHeading);
        if (controls == null || controls.length == 0) {
            throw new IllegalArgumentException("Use a straight path when no control points are needed");
        }
        Pose[] points = new Pose[controls.length + 2];
        points[0] = start;
        for (int i = 0; i < controls.length; i++) {
            if (!BioBuzzAutoConfig.finite(controls[i])) {
                throw new IllegalArgumentException("Nonfinite curve control pose");
            }
            points[i + 1] = controls[i];
        }
        points[points.length - 1] = end;
        Path path = curve(points);
        return constantHeading ? path.constant(start) : path.linear(start, end);
    }

    private static void validate(Pose start, Pose end, boolean constantHeading) {
        if (!BioBuzzAutoConfig.finite(start) || !BioBuzzAutoConfig.finite(end)
                || BioBuzzAutoConfig.distance(start, end) < 1e-6) {
            throw new IllegalArgumentException("Path needs finite, distinct endpoints");
        }
        if (constantHeading && Math.abs(BioBuzzAutoConfig.headingErrorDegrees(start, end)) > 1e-6) {
            throw new IllegalArgumentException("Constant heading must match both endpoints");
        }
    }
}
