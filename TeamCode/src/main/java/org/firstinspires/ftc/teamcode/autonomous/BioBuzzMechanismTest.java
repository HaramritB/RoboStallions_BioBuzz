package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Robot;

/** Stationary, operator-held mechanism verification; never maps or commands drivetrain/flower. */
@TeleOp(name="BioBuzz Auto Mechanism Test", group="Development")
public class BioBuzzMechanismTest extends OpMode {
    private BioBuzzAutoMechanisms mechanisms;
    private Command volley;
    private boolean shootingMode, started, attempted, finished, commandEnded;
    private long began;
    private String status="INIT";
    private String failure="";

    protected BioBuzzAutoMechanisms createMechanisms() {
        return BioBuzzAutoMechanisms.initialize(hardwareMap, Robot.lastAlliance);
    }
    protected long nanoTime() { return System.nanoTime(); }
    @Override public void init() {
        cleanup();
        mechanisms=null; volley=null; shootingMode=false; started=false;
        attempted=false; finished=false; commandEnded=false; began=0; failure="";
        try {
            mechanisms=createMechanisms();
            mechanisms.manualIntake(false); // Zero all motors; unused gate stays untouched.
            mechanisms.observe();
            status="READY - stationary mechanism test";
        } catch (RuntimeException error) { fail("INIT: " + error.getMessage()); }
        publish();
    }
    @Override public void init_loop() {
        if (gamepad1.dpadUpWasPressed()) shootingMode=true;
        if (gamepad1.dpadDownWasPressed()) shootingMode=false;
        if (gamepad1.aWasPressed()) Robot.lastAlliance=Robot.Alliance.BLUE;
        if (gamepad1.bWasPressed()) Robot.lastAlliance=Robot.Alliance.RED;
        if (mechanisms!=null && failure.isEmpty()) {
            try { mechanisms.setAlliance(Robot.lastAlliance); mechanisms.observe(); }
            catch (RuntimeException error) { fail(error.getMessage()); }
        }
        publish();
    }
    @Override public void start() { started=true; status="STARTED - outputs idle until LB+A held"; publish(); }
    @Override public void loop() {
        if (!started || finished || !failure.isEmpty() || mechanisms==null) { publish(); return; }
        try {
            boolean held=gamepad1.left_bumper && gamepad1.a;
            if (!held) {
                if (attempted) { status="Released - stopped; re-INIT for another test"; finished=true; cleanup(); }
                else mechanisms.observe();
            } else {
                if (!attempted) {
                    attempted=true; began=nanoTime();
                    if (shootingMode) {
                        mechanisms.observe();
                        volley=mechanisms.command(BioBuzzAutoMechanisms.Action.shoot(4,
                                BioBuzzAutoConfig.SHOOT_TIMEOUT_S),this::nanoTime,this::fail);
                        volley.start();
                    } else mechanisms.manualIntake(true);
                }
                if (!failure.isEmpty()) { publish(); return; }
                if (!shootingMode && (nanoTime()-began)/1e9>=BioBuzzAutoConfig.COLLECTION_TIMEOUT_S) {
                    fail("Intake test time limit reached; stored-ball count is still unverified");
                } else {
                    mechanisms.update();
                    if (shootingMode) {
                        volley.execute();
                        if (volley.done()) {
                            volley.end(EndCondition.NATURALLY); commandEnded=true;
                            if (failure.isEmpty()) status="Four shots verified; inspect actual scoring physically";
                            finished=true; cleanup();
                        } else status="Four-shot test: aiming / RPM / feed / sensor verification";
                    } else status="Intake running - operator checks storage; NO four-ball completion claim";
                }
            }
        } catch (RuntimeException error) { fail(error.getClass().getSimpleName()+": "+error.getMessage()); }
        publish();
    }
    @Override public void stop() { finished=true; cleanup(); }
    private void fail(String reason) {
        if (failure.isEmpty()) failure=reason;
        finished=true; cleanup();
    }
    private void cleanup() {
        // Do not reenter a command's end callback through its failure consumer.
        if (volley!=null && !commandEnded) {
            commandEnded=true;
            try { volley.end(EndCondition.INTERRUPTED); }
            catch (RuntimeException error) { if (failure.isEmpty()) failure=error.getMessage(); }
        }
        if (mechanisms!=null) {
            try { mechanisms.stop(); }
            catch (RuntimeException error) { if (failure.isEmpty()) failure=error.getMessage(); }
        }
    }
    private void publish() {
        telemetry.addData("Mode", shootingMode ? "4-SHOT VOLLEY (D-pad down: intake)" : "MANUAL INTAKE (D-pad up: shoot)");
        telemetry.addData("Control", "After START hold LB+A; release stops; one attempt per INIT");
        telemetry.addData("Alliance", Robot.lastAlliance);
        telemetry.addData("Status", status);
        telemetry.addData("Failure", failure.isEmpty() ? "none" : failure);
        telemetry.addData("Mechanisms", mechanisms==null ? "unavailable" : mechanisms.state());
        telemetry.addData("Controller diagnostics", mechanisms==null ? "unavailable" : mechanisms.diagnostics());
        telemetry.addData("Intake policy", mechanisms==null ? "unavailable" : mechanisms.intakePolicy());
        telemetry.addData("Route / FLOWER lever", "Never controlled by this OpMode");
        telemetry.update();
    }
}
