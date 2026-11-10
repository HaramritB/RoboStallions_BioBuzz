package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.Map;

/** Actual stationary test OpMode callbacks, deterministic time, recording mechanism port. */
public final class MechanismTestRegression {
    static int checks;
    static void check(boolean condition,String message) { checks++; if (!condition) throw new AssertionError(message); }
    static final class Test extends BioBuzzMechanismTest {
        final BioBuzzAutoRegression.Mech hardware=new BioBuzzAutoRegression.Mech();
        final Map<String,Object> values=new HashMap<>();
        long now=1_000_000_000L;
        Test() {
            gamepad1=new Gamepad();
            telemetry=(Telemetry)Proxy.newProxyInstance(Telemetry.class.getClassLoader(),new Class<?>[]{Telemetry.class},(p,m,a)-> {
                if (m.getName().equals("addData")) { values.put((String)a[0],a[1]); return null; }
                if (m.getName().equals("update")) return true;
                return null;
            });
        }
        @Override protected BioBuzzAutoMechanisms createMechanisms() { return new BioBuzzAutoMechanisms(hardware); }
        @Override protected long nanoTime() { return now; }
        void hold() { gamepad1.left_bumper=true; gamepad1.a=true; loop(); }
        void advance(double s) { now+=(long)(s*1e9); loop(); }
        void shootMode() throws Exception {
            gamepad1.dpad_up=true;
            java.lang.reflect.Method method=Gamepad.class.getDeclaredMethod("updateEdgeDetection"); method.setAccessible(true); method.invoke(gamepad1);
            init_loop();
        }
    }
    public static void main(String[] args) throws Exception {
        Test intake=new Test(); intake.init(); intake.init_loop();
        check(intake.hardware.controls==0,"INIT only observes/zeros; no mechanism activation");
        intake.start(); intake.loop(); check(intake.hardware.controls==0,"START alone cannot activate mechanisms");
        intake.hold(); check(intake.hardware.request==BioBuzzAutoMechanisms.Kind.COLLECT,"Intake requires LB+A held");
        intake.gamepad1.a=false; intake.loop();
        check(intake.hardware.request==BioBuzzAutoMechanisms.Kind.NONE && intake.hardware.closes>0,"Release stops all outputs");
        int controls=intake.hardware.controls; intake.hold();
        check(intake.hardware.controls==controls,"Rearming requires a fresh INIT");
        intake.init(); intake.start(); intake.hold(); intake.advance(10.1);
        check(intake.hardware.request==BioBuzzAutoMechanisms.Kind.NONE,"Manual intake is bounded even if held indefinitely");
        Test shoot=new Test(); shoot.init(); shoot.shootMode(); shoot.start(); shoot.hold();
        check(shoot.hardware.request==BioBuzzAutoMechanisms.Kind.SHOOT,"Four-shot mode starts on held controls");
        for (int i=0;i<4;i++) { shoot.hardware.ball=true; shoot.loop(); shoot.hardware.ball=false; shoot.hardware.shots++; shoot.loop(); }
        shoot.advance(.21);
        check(shoot.hardware.shots==4 && shoot.hardware.closes>0 && shoot.hardware.request==BioBuzzAutoMechanisms.Kind.NONE,
                "Four verified shots complete with hardware cleanup");
        Test interrupted=new Test(); interrupted.init(); interrupted.shootMode(); interrupted.start(); interrupted.hold(); interrupted.stop();
        check(interrupted.hardware.closes>0 && interrupted.hardware.request==BioBuzzAutoMechanisms.Kind.NONE,"FTC Stop interrupts active volley");
        Test invalid=new Test(); invalid.init(); invalid.start(); invalid.hardware.sensorsReady=false; invalid.hold();
        check(invalid.hardware.controls==0 && invalid.hardware.closes>0,"Fresh sensor failure prevents actuator output");
        System.out.println("Passed "+checks+" stationary mechanism test lifecycle checks.");
    }
}
