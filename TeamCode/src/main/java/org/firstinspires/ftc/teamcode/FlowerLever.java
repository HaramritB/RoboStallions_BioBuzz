package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

/** Shared measured flower-lever positions and PWM behavior for TeleOp and Auto. */
public final class FlowerLever {
    public static final double DEPLOYED_POSITION = 0.435;
    public static final double RETRACTED_POSITION = 0.1;
    private final Servo servo;
    private boolean commanded;
    private boolean deployed;

    /** Mapping the lever does not move it during INIT. */
    public FlowerLever(Servo servo) { this.servo = servo; }

    public static void command(Servo servo, boolean deployed) {
        servo.setPosition(deployed ? DEPLOYED_POSITION : RETRACTED_POSITION);
        if (servo instanceof PwmControl) ((PwmControl)servo).setPwmEnable();
    }

    public void setDeployed(boolean deployed) {
        commanded = true; // Also retry retraction during cleanup if this hardware write fails.
        this.deployed = deployed;
        command(servo, deployed);
    }

    public void refresh() {
        if (commanded && servo instanceof PwmControl && !((PwmControl)servo).isPwmEnabled()) {
            command(servo, deployed);
        }
    }

    public void stop() { if (commanded) setDeployed(false); }
}
