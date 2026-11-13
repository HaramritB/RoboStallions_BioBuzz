package org.firstinspires.ftc.teamcode.v2;

import java.util.Arrays;

/**
 * Distance used to pick shooter RPM.
 *
 * The original TeleOp used each raw frame's distance and fell back to 3150 RPM the moment a tag
 * blinked out, so the flywheel target jumped between frames. This uses the median of the last
 * few fresh distances, holds it briefly through dropouts, then falls back to the odometry range to
 * the remembered target point, and only then to the default.
 */
public final class ShotDistanceFilterV2 {
    public static int SAMPLES = 5;
    public static double HOLD_MS = 1000;

    public enum Source { VISION, HELD, ODOMETRY, NONE }

    private final double[] samples = new double[Math.max(1, SAMPLES)];
    private int count, next;
    private double lastFreshMs = Double.NEGATIVE_INFINITY;

    public Source source = Source.NONE;
    public double distanceM = Double.NaN;

    /**
     * @param freshDistanceM distance from a NEW frame of the chosen HIVE cell, else NaN
     * @param odometryRangeM range from odometry to a camera-confirmed target point, else NaN
     */
    public double update(double nowMs, double freshDistanceM, double odometryRangeM) {
        if (Double.isFinite(freshDistanceM) && freshDistanceM > 0) {
            samples[next] = freshDistanceM;
            next = (next + 1) % samples.length;
            count = Math.min(count + 1, samples.length);
            lastFreshMs = nowMs;
        }
        if (count > 0 && nowMs - lastFreshMs <= HOLD_MS) {
            double[] sorted = Arrays.copyOf(samples, count);
            Arrays.sort(sorted);
            int mid = count / 2;
            distanceM = count % 2 == 1 ? sorted[mid] : (sorted[mid - 1] + sorted[mid]) / 2;
            source = nowMs == lastFreshMs ? Source.VISION : Source.HELD;
        } else if (Double.isFinite(odometryRangeM) && odometryRangeM > 0) {
            distanceM = odometryRangeM;
            source = Source.ODOMETRY;
        } else {
            distanceM = Double.NaN;
            source = Source.NONE;
        }
        return distanceM;
    }

    public void reset() {
        count = next = 0;
        lastFreshMs = Double.NEGATIVE_INFINITY;
        distanceM = Double.NaN;
        source = Source.NONE;
    }
}
