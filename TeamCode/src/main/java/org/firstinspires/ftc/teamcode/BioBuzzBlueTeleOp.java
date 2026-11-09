package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BioBuzz Blue TeleOp", group = "Competition")
public final class BioBuzzBlueTeleOp extends BioBuzzTeleOp {
    @Override
    protected Robot.Alliance teleOpAlliance() {
        return Robot.Alliance.BLUE;
    }
}
