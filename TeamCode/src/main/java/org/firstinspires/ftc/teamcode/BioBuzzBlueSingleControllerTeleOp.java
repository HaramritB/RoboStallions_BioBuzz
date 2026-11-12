package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BioBuzz Blue TeleOp (Single Controller)", group = "Competition")
public final class BioBuzzBlueSingleControllerTeleOp extends BioBuzzTeleOp {
    @Override
    protected Robot.Alliance teleOpAlliance() {
        return Robot.Alliance.BLUE;
    }

    @Override
    protected boolean singleController() {
        return true;
    }
}
