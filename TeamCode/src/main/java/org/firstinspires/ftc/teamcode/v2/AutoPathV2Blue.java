package org.firstinspires.ftc.teamcode.v2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "AutoPath V2 Blue", group = "Autonomous", preselectTeleOp = "BioBuzz Blue TeleOp V2")
public final class AutoPathV2Blue extends AutoPathV2 {
    @Override
    protected Robot.Alliance alliance() {
        return Robot.Alliance.BLUE;
    }
}
