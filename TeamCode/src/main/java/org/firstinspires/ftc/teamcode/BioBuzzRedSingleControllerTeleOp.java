package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BioBuzz Red TeleOp (Single Controller)", group = "Competition")
public final class BioBuzzRedSingleControllerTeleOp extends BioBuzzTeleOp {
    @Override
    protected Robot.Alliance teleOpAlliance() {
        return Robot.Alliance.RED;
    }

    @Override
    protected boolean singleController() {
        return true;
    }
}
