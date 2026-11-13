package org.firstinspires.ftc.teamcode.v2;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name = "AutoPath V2 Red", group = "Autonomous", preselectTeleOp = "BioBuzz Red TeleOp V2")
public final class AutoPathV2Red extends AutoPathV2 {
    @Override
    protected Robot.Alliance alliance() {
        return Robot.Alliance.RED;
    }
}
