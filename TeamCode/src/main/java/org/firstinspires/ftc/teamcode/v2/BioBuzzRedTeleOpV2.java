package org.firstinspires.ftc.teamcode.v2;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "BioBuzz Red TeleOp V2", group = "Competition")
public final class BioBuzzRedTeleOpV2 extends BioBuzzTeleOpV2 {
    @Override
    protected Robot.Alliance teleOpAlliance() {
        return Robot.Alliance.RED;
    }
}
