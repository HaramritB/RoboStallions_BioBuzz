package org.firstinspires.ftc.teamcode.v2;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

/**
 * Builds the distance -> RPM table for longer shots.
 *
 * The measured table only covers 1.39-1.62 m and clamps at 3235 RPM beyond it, but the robot
 * usually shoots from farther. At each spot: trim RPM (D-pad) until shots go in, then press the
 * log button. Each press appends one row to /sdcard/FIRST/biobuzz_shot_table.csv. Send the file
 * back and the rows become new table points.
 */
public final class ShotLoggerV2 {
    public static final String FILE_NAME = "biobuzz_shot_table.csv";

    public int rowsWritten;
    public String lastMessage = "Calibration: gamepad 1 RIGHT STICK click logs distance/RPM";

    public void log(RobotV2 v2) {
        File file = new File(AppUtil.FIRST_FOLDER, FILE_NAME);
        boolean header = !file.exists();
        try (FileWriter out = new FileWriter(file, true)) {
            if (header) {
                out.write("time_ms,alliance,distance_m,distance_source,raw_tag_distance_m,cell,base_rpm,"
                        + "trim_rpm,commanded_rpm,measured_rpm,battery_v,aim_trim_deg,pose_x_in,pose_y_in\n");
            }
            out.write(String.format(Locale.US, "%d,%s,%.3f,%s,%.3f,%s,%.0f,%.0f,%.0f,%.0f,%.2f,%.2f,%.1f,%.1f%n",
                    System.currentTimeMillis(), v2.robot.alliance, v2.shotDistanceM, v2.distanceFilter.source,
                    v2.hive.distanceM, v2.hive.cell, v2.baseRpm, v2.robot.rpmTrim, v2.commandedRpm(),
                    v2.robot.shooterRpm(), v2.batteryVolts, v2.aim.aimTrimDeg, v2.xIn, v2.yIn));
            rowsWritten++;
            lastMessage = String.format(Locale.US, "Logged #%d: %.2f m -> %.0f RPM (%s)", rowsWritten,
                    v2.shotDistanceM, v2.commandedRpm(), file.getAbsolutePath());
        } catch (IOException error) {
            lastMessage = "Log failed: " + error.getMessage();
        }
    }
}
