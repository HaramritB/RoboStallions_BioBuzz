package org.firstinspires.ftc.teamcode.v2;

import org.firstinspires.ftc.teamcode.Robot;

import java.util.List;

/**
 * Picks the HIVE CELL to aim at from one Limelight frame.
 *
 * Each alliance HIVE has two CELLS, one at each end, and each CELL carries its own cluster of
 * four AprilTags (RED 30-33 and 34-37, BLUE 38-41 and 42-45). Only the upward-facing CELL scores,
 * and the two CELLS swap every time the HIVE tips. The original code averaged every alliance
 * tag, so a frame showing both clusters aimed between the CELLS. This picks one cluster.
 *
 * With one cluster visible it is used. With both visible, the cluster that sits higher in the
 * image (by mean ty) is the upward CELL. A small hysteresis keeps the choice from flickering.
 */
public final class HiveTargetV2 {

    /**
     * +1 if a larger Limelight ty means higher on the field (normal camera mounting).
     * Set to -1 if the camera image is upside down. Verify with the "HIVE cell" telemetry:
     * with both clusters in view, it must name the CELL that is physically up.
     */
    public static double UP_CELL_TY_SIGN = 1.0;
    /** The other cluster must look this much higher (degrees of ty) before the choice switches. */
    public static double CELL_SWITCH_TY_MARGIN_DEG = 1.0;

    public enum Cell { NONE, LOW_IDS, HIGH_IDS }

    /** One detected tag. distanceM is NaN when the frame has no pose for it. */
    public static final class Tag {
        public final int id;
        public final double txDeg;
        public final double tyDeg;
        public final double distanceM;

        public Tag(int id, double txDeg, double tyDeg, double distanceM) {
            this.id = id;
            this.txDeg = txDeg;
            this.tyDeg = tyDeg;
            this.distanceM = distanceM;
        }
    }

    // Results of the latest update.
    public Cell cell = Cell.NONE;
    public double txDeg = Double.NaN;
    public double distanceM = Double.NaN;
    public int distanceTagId = -1;
    public int tagsUsed;
    public int lowCount;
    public int highCount;
    public double lowTyDeg = Double.NaN;
    public double highTyDeg = Double.NaN;
    public boolean bothCellsVisible;

    private Cell lastChoice = Cell.NONE;

    public static int firstId(Robot.Alliance alliance) {
        return alliance == Robot.Alliance.RED ? Robot.RED_TAG_MIN : Robot.BLUE_TAG_MIN;
    }

    public static Cell cellOf(int id, Robot.Alliance alliance) {
        int base = firstId(alliance);
        if (id >= base && id <= base + 3) return Cell.LOW_IDS;
        if (id >= base + 4 && id <= base + 7) return Cell.HIGH_IDS;
        return Cell.NONE;
    }

    public static String label(Cell cell, Robot.Alliance alliance) {
        int base = firstId(alliance);
        switch (cell) {
            case LOW_IDS: return "tags " + base + "-" + (base + 3);
            case HIGH_IDS: return "tags " + (base + 4) + "-" + (base + 7);
            default: return "none";
        }
    }

    /** Forget the previous choice, e.g. when the alliance changes. */
    public void reset() {
        lastChoice = Cell.NONE;
        clearResult();
    }

    private void clearResult() {
        cell = Cell.NONE;
        txDeg = Double.NaN;
        distanceM = Double.NaN;
        distanceTagId = -1;
        tagsUsed = lowCount = highCount = 0;
        lowTyDeg = highTyDeg = Double.NaN;
        bothCellsVisible = false;
    }

    /** tags may be null or empty; tags from the other alliance or with a non-finite tx are ignored. */
    public void update(List<Tag> tags, Robot.Alliance alliance) {
        clearResult();
        if (tags == null || tags.isEmpty()) return;

        double lowTy = 0, highTy = 0;
        for (Tag tag : tags) {
            if (tag == null || !Double.isFinite(tag.txDeg)) continue;
            Cell c = cellOf(tag.id, alliance);
            double ty = Double.isFinite(tag.tyDeg) ? tag.tyDeg : 0;
            if (c == Cell.LOW_IDS) { lowCount++; lowTy += ty; }
            else if (c == Cell.HIGH_IDS) { highCount++; highTy += ty; }
        }
        if (lowCount > 0) lowTyDeg = lowTy / lowCount;
        if (highCount > 0) highTyDeg = highTy / highCount;
        bothCellsVisible = lowCount > 0 && highCount > 0;

        Cell choice;
        if (lowCount == 0 && highCount == 0) {
            return;
        } else if (!bothCellsVisible) {
            choice = lowCount > 0 ? Cell.LOW_IDS : Cell.HIGH_IDS;
        } else {
            double lowHeight = UP_CELL_TY_SIGN * lowTyDeg;
            double highHeight = UP_CELL_TY_SIGN * highTyDeg;
            if (lastChoice == Cell.LOW_IDS) {
                choice = highHeight > lowHeight + CELL_SWITCH_TY_MARGIN_DEG ? Cell.HIGH_IDS : Cell.LOW_IDS;
            } else if (lastChoice == Cell.HIGH_IDS) {
                choice = lowHeight > highHeight + CELL_SWITCH_TY_MARGIN_DEG ? Cell.LOW_IDS : Cell.HIGH_IDS;
            } else {
                choice = highHeight > lowHeight ? Cell.HIGH_IDS : Cell.LOW_IDS;
            }
        }

        double txSum = 0;
        for (Tag tag : tags) {
            if (tag == null || !Double.isFinite(tag.txDeg) || cellOf(tag.id, alliance) != choice) continue;
            txSum += tag.txDeg;
            tagsUsed++;
            // Nearest tag of the chosen CELL, matching how the distance table was measured.
            if (Double.isFinite(tag.distanceM) && tag.distanceM > 0
                    && (!Double.isFinite(distanceM) || tag.distanceM < distanceM)) {
                distanceM = tag.distanceM;
                distanceTagId = tag.id;
            }
        }
        cell = choice;
        lastChoice = choice;
        txDeg = txSum / tagsUsed;
    }
}
