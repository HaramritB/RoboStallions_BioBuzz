package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;

/**
 * AutoPath run log: one CSV row per loop with pose, turret, vision and shot state, written to
 * /sdcard/FIRST/java/src/Datalogs/autopath_<millis>.csv on the Control Hub (same folder as the
 * flywheel logs). Logging never stops the autonomous: any file error just turns it off.
 */
final class AutoRunLog {
    static final String FOLDER = "java/src/Datalogs";
    private static final long FLUSH_NS = 1_000_000_000L;

    private BufferedWriter out;
    private long startNs = -1, lastFlushNs;
    String fileName = "not logging";

    static AutoRunLog open(String header) {
        AutoRunLog log = new AutoRunLog();
        try {
            File folder = new File(AppUtil.FIRST_FOLDER, FOLDER);
            if (!folder.exists() && !folder.mkdirs()) return log;
            File file = new File(folder, "autopath_" + System.currentTimeMillis() + ".csv");
            log.out = new BufferedWriter(new FileWriter(file));
            log.out.write("t_s," + header);
            log.out.newLine();
            log.fileName = file.getName();
        } catch (Exception | LinkageError ignored) {
            log.close();
            log.fileName = "not logging";
        }
        return log;
    }

    void row(long nowNs, String values) {
        if (out == null) return;
        try {
            if (startNs < 0) { startNs = nowNs; lastFlushNs = nowNs; }
            out.write(String.format(java.util.Locale.US, "%.3f,", (nowNs - startNs) / 1e9));
            out.write(values);
            out.newLine();
            if (nowNs - lastFlushNs >= FLUSH_NS) { out.flush(); lastFlushNs = nowNs; } // Survives STOP.
        } catch (Exception ignored) {
            close();
        }
    }

    void close() {
        if (out == null) return;
        try { out.close(); } catch (Exception ignored) { }
        out = null;
    }
}
