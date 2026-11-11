package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/** The same tested navigation and pauses, without initializing any mechanisms. */
@Autonomous(name = "AutoPath (Drive Only)", group = "Autonomous")
public final class AutoPathDriveOnly extends AutoPath {
    @Override protected boolean useMechanisms() { return false; }
}
