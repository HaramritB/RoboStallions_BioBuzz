package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/** Checks the direct visualizer route against its exported geometry and heading modes. */
public final class AutoPathRegression {
    public static void main(String[] args) {
        assertRegistered(AutoPath.class, "AutoPath");
        assertRegistered(org.firstinspires.ftc.teamcode.autonomous.BioBuzzAuto.class, "BioBuzz Pedro Auto");
        if (!BioBuzzAuto.class.isAnnotationPresent(Disabled.class)) {
            throw new AssertionError("Legacy encoder autonomous must remain disabled");
        }
        Path[] paths = {AutoPath.path1(), AutoPath.path2(), AutoPath.path3(), AutoPath.path4(), AutoPath.path5()};
        double[][] xy = {{57.6323, 9.0657}, {9.5657, 9.2715}, {32.2934, 127.6559},
                {46.9373, 125.3042}, {46.7883, 140.2305}, {38.0943, 127.3583}};
        for (int i = 0; i < paths.length; i++) {
            assertPoint(paths[i].get(0), xy[i][0], xy[i][1]);
            assertPoint(paths[i].get(1), xy[i + 1][0], xy[i + 1][1]);
        }
        assertHeading(paths[0].heading(0.5), Math.atan2(9.2715 - 9.0657, 9.5657 - 57.6323));
        assertHeading(paths[1].heading(0), Math.atan2(34.4143 - 9.2715, 40.923 - 9.5657) + Math.PI);
        assertHeading(paths[2].heading(0), 0);
        assertHeading(paths[2].heading(1), Math.PI / 2);
        assertHeading(paths[3].heading(0.5), Math.atan2(140.2305 - 125.3042, 46.7883 - 46.9373));
        assertHeading(paths[4].heading(1), Math.atan2(127.3583 - 110.0765, 38.0943 - 45.4429) + Math.PI);
        System.out.println("Passed direct route geometry, headings, and autonomous registration checks.");
    }

    private static void assertRegistered(Class<?> type, String name) {
        Autonomous opMode = type.getAnnotation(Autonomous.class);
        if (type.isAnnotationPresent(Disabled.class) || opMode == null || !opMode.name().equals(name)) {
            throw new AssertionError("Expected Driver Station entry " + name);
        }
    }

    private static void assertPoint(Pose pose, double x, double y) {
        if (Math.hypot(pose.x() - x, pose.y() - y) > 1e-6) {
            throw new AssertionError("Unexpected route endpoint " + pose);
        }
    }

    private static void assertHeading(double actual, double expected) {
        double delta = actual - expected;
        if (Math.abs(Math.atan2(Math.sin(delta), Math.cos(delta))) > 1e-6) {
            throw new AssertionError("Unexpected route heading " + Math.toDegrees(actual));
        }
    }
}
