package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/** Old entry points remain source-compatible but cannot expose a competing autonomous. */
public final class AutoPathRegression {
    public static void main(String[] args) {
        if (!AutoPath.class.isAnnotationPresent(Disabled.class)
                || !org.firstinspires.ftc.teamcode.autonomous.BioBuzzAuto.class.isAssignableFrom(AutoPath.class)
                || !BioBuzzAuto.class.isAnnotationPresent(Disabled.class)) {
            throw new AssertionError("Compatibility/legacy autonomous must remain disabled");
        }
        if (org.firstinspires.ftc.teamcode.autonomous.BioBuzzAuto.class.isAnnotationPresent(Disabled.class)) {
            throw new AssertionError("Intended full autonomous must be selectable");
        }
        Autonomous name=org.firstinspires.ftc.teamcode.autonomous.BioBuzzAuto.class.getAnnotation(Autonomous.class);
        if (!name.name().equals("BioBuzz Pedro Auto")) throw new AssertionError("Unexpected Driver Station name");
        System.out.println("Passed compatibility and autonomous registration checks.");
    }
}
