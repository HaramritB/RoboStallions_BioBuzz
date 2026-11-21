package org.firstinspires.ftc.teamcode;

import java.util.Locale;

/**
 * Per-volley flywheel metrics for the Flywheel Tuning burst logger. Pure logic, one sample per loop.
 *
 * A volley starts when feeding starts (RB, or the LB marker) and ends once the flywheel is back
 * inside +/-RECOVERY_BAND_RPM for RECOVERY_HOLD_MS after feeding stops (or POST_WINDOW_MS passes).
 *
 * Each ball shows up as a dip: speed falls DIP_START_RPM below the running peak. The peak just
 * before a dip is that ball's LAUNCH RPM, the number to compare between ball 1 and ball 4. If the
 * wheel never rises between two balls, their dips merge, so fewer dips than balls means it had
 * no time to recover between them.
 */
final class BurstAnalyzer {
    static final double RECOVERY_BAND_RPM = 50;
    static final double RECOVERY_HOLD_MS = 150;
    static final double DIP_START_RPM = 60;
    static final double DIP_END_RISE_RPM = 30;
    /** Balls still leaving shortly after RB is released are counted. */
    static final double DIP_TAIL_MS = 300;
    static final double POST_WINDOW_MS = 1500;
    static final double BASELINE_TAU_MS = 100;
    static final int MAX_DIPS = 8;

    // Latest completed (or in-progress) volley.
    int volley;
    boolean inVolley;
    double targetRpm = Double.NaN;
    double baselineRpm = Double.NaN;
    double minRpm = Double.NaN;
    double recoveryMs = Double.NaN;
    int dips;
    final double[] launchRpm = new double[MAX_DIPS];
    final double[] dipMinRpm = new double[MAX_DIPS];
    String lastSummary = "";

    private double idleBaseline = Double.NaN;
    private double lastSampleMs = Double.NaN;
    private boolean lastBurst;
    private double burstEndMs = Double.NaN;
    private double lastDipStartMs = Double.NaN;
    private double inBandSinceMs = Double.NaN;
    private boolean inDip;
    private double runPeak = Double.NaN;
    private double dipMin = Double.NaN;

    /** @return a CSV event tag ("volley_start", "dip_2", "volley_end"...) or "" */
    String sample(double nowMs, double rpm, double target, boolean burstActive) {
        String event = "";
        double dt = Double.isFinite(lastSampleMs) ? Math.max(0, nowMs - lastSampleMs) : 0;
        lastSampleMs = nowMs;
        if (!Double.isFinite(rpm)) {
            lastBurst = burstActive;
            return event;
        }

        if (!inVolley) {
            idleBaseline = Double.isFinite(idleBaseline) && dt > 0
                    ? idleBaseline + (rpm - idleBaseline) * dt / (BASELINE_TAU_MS + dt) : rpm;
        }

        if (burstActive && !lastBurst && !inVolley) {
            inVolley = true;
            volley++;
            targetRpm = target;
            baselineRpm = Double.isFinite(idleBaseline) ? idleBaseline : rpm;
            minRpm = rpm;
            recoveryMs = Double.NaN;
            dips = 0;
            inDip = false;
            runPeak = rpm;
            burstEndMs = lastDipStartMs = inBandSinceMs = Double.NaN;
            event = "volley_start";
        }
        if (burstActive) burstEndMs = Double.NaN;
        else if (lastBurst && inVolley) burstEndMs = nowMs;
        lastBurst = burstActive;
        if (!inVolley) return event;

        minRpm = Math.min(minRpm, rpm);
        boolean countingDips = burstActive || (Double.isFinite(burstEndMs) && nowMs - burstEndMs <= DIP_TAIL_MS);
        if (inDip) {
            dipMin = Math.min(dipMin, rpm);
            if (dips <= MAX_DIPS) dipMinRpm[dips - 1] = dipMin;
            if (rpm >= dipMin + DIP_END_RISE_RPM) {
                inDip = false;
                runPeak = rpm;
            }
        } else {
            runPeak = Math.max(runPeak, rpm);
            if (countingDips && rpm <= runPeak - DIP_START_RPM) {
                inDip = true;
                dipMin = rpm;
                lastDipStartMs = nowMs;
                dips++; // Total count; only the first MAX_DIPS are stored.
                if (dips <= MAX_DIPS) {
                    launchRpm[dips - 1] = runPeak;
                    dipMinRpm[dips - 1] = rpm;
                }
                event = "dip_" + dips;
            }
        }

        if (!burstActive && Double.isFinite(burstEndMs)) {
            boolean inBand = Math.abs(rpm - targetRpm) <= RECOVERY_BAND_RPM;
            if (!inBand) inBandSinceMs = Double.NaN;
            else if (Double.isNaN(inBandSinceMs)) inBandSinceMs = nowMs;
            boolean recovered = !inDip && Double.isFinite(inBandSinceMs) && nowMs - inBandSinceMs >= RECOVERY_HOLD_MS;
            if (recovered || nowMs - burstEndMs >= POST_WINDOW_MS) {
                double from = Double.isFinite(lastDipStartMs) ? lastDipStartMs : burstEndMs;
                recoveryMs = recovered ? Math.max(0, inBandSinceMs - from) : Double.NaN;
                inVolley = false;
                inDip = false;
                event = event.isEmpty() ? "volley_end" : event + "|volley_end";
            }
        }
        return event;
    }

    /** Launch RPM of ball 1 minus the last detected ball: the slowdown across the volley. */
    double launchSpread() {
        return dips >= 2 ? launchRpm[0] - launchRpm[Math.min(dips, MAX_DIPS) - 1] : Double.NaN;
    }

    double drop() {
        return Double.isFinite(baselineRpm) && Double.isFinite(minRpm) ? Math.max(0, baselineRpm - minRpm) : Double.NaN;
    }

    String launchList() {
        if (dips == 0) return "none";
        StringBuilder out = new StringBuilder();
        for (int i = 0; i < Math.min(dips, MAX_DIPS); i++) {
            if (i > 0) out.append('/');
            out.append(Math.round(launchRpm[i]));
        }
        return out.toString();
    }

    /** One line per volley for telemetry. */
    String summary(double p, double referenceF) {
        return String.format(Locale.US, "#%d P=%.0f F=%.2f @%.0f: launch %s (1st-last %s) min %.0f drop %.0f rec %s",
                volley, p, referenceF, targetRpm, launchList(),
                Double.isFinite(launchSpread()) ? String.format(Locale.US, "%.0f", launchSpread()) : "--",
                minRpm, drop(), Double.isFinite(recoveryMs) ? String.format(Locale.US, "%.0f ms", recoveryMs) : "not in 1.5 s");
    }
}
