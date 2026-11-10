package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Robot;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/** SDK HardwareMap override and recording real SDK interfaces; no stubbed Pedro/Ivy API. */
public final class MechanismIsolationRegression {
    static int checks;
    static void check(boolean ok,String message) { checks++; if (!ok) throw new AssertionError(message); }
    static final class Motor implements InvocationHandler {
        final List<String> trace=new ArrayList<>();
        boolean failStop;
        double power=1;
        final DcMotorEx device=(DcMotorEx)Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),new Class<?>[]{DcMotorEx.class},this);
        @Override public Object invoke(Object proxy,Method method,Object[] args) {
            String name=method.getName();
            if (name.equals("setPower")) {
                trace.add(name+":"+args[0]);
                if (failStop) throw new IllegalStateException("simulated write failure");
                power=(double)args[0]; return null;
            }
            if (name.equals("getVelocity")) return 0.0;
            if (name.equals("getCurrentPosition")) return 0;
            if (name.equals("setVelocity")) { trace.add(name+":"+args[0]); return null; }
            if (name.equals("setDirection") || name.equals("setZeroPowerBehavior") || name.equals("setMode")) {
                trace.add(name+":"+args[0]); return null;
            }
            if (name.equals("setVelocityPIDFCoefficients")) {
                trace.add(name+":"+java.util.Arrays.toString(args)); return null;
            }
            throw new AssertionError("Unexpected motor call " + name);
        }
    }
    static final class MapFixture extends HardwareMap {
        final Map<String,Motor> motors=new HashMap<>();
        final List<String> lookups=new ArrayList<>();
        int servoWrites;
        final Servo gate=(Servo)Proxy.newProxyInstance(Servo.class.getClassLoader(),new Class<?>[]{Servo.class},(proxy,method,args)-> {
            if (method.getName().equals("setPosition")) { servoWrites++; return null; }
            throw new AssertionError("Unexpected servo call " + method.getName());
        });
        MapFixture() {
            super(null,null);
            for (String name:new String[]{"intake","transmission","turret","shooter","frontleft","frontright","backleft","backright"}) motors.put(name,new Motor());
        }
        @Override public <T> T get(Class<? extends T> type,String name) {
            lookups.add(name);
            if (name.equals("torqueservo1")) return type.cast(gate);
            if (motors.containsKey(name)) return type.cast(motors.get(name).device);
            throw new AssertionError("Unexpected required device " + name);
        }
        @Override public <T> T tryGet(Class<? extends T> type,String name) { lookups.add(name); return null; }
        @Override public <T> List<T> getAll(Class<? extends T> type) { return Collections.emptyList(); }
    }
    public static void main(String[] args) throws Exception {
        MapFixture only=new MapFixture(); Robot mechanisms=new Robot(); mechanisms.initMechanismsOnly(only,false);
        for (String name:new String[]{"frontleft","frontright","backleft","backright"}) {
            check(!only.lookups.contains(name) && only.motors.get(name).trace.isEmpty(),"Mechanism init cannot touch " + name);
        }
        check(only.servoWrites==0,"INIT cannot move the gate");
        check(!only.motors.get("turret").trace.toString().contains("STOP_AND_RESET"),"Mechanism-only init preserves turret zero");
        MapFixture whole=new MapFixture(); Robot robot=new Robot(); robot.init(whole,false);
        for (String name:new String[]{"intake","transmission","turret","shooter"}) {
            check(only.motors.get(name).trace.equals(whole.motors.get(name).trace),"TeleOp and Auto retain identical initialization for " + name);
        }
        check(whole.motors.get("frontleft").trace.toString().contains("REVERSE"),"Whole Robot init retains old drivetrain direction");
        BioBuzzAutoMechanisms.RobotHardware adapter=new BioBuzzAutoMechanisms.RobotHardware();
        adapter.robot.intake=only.motors.get("intake").device;
        adapter.robot.transmission=only.motors.get("transmission").device;
        adapter.robot.turret=only.motors.get("turret").device;
        adapter.robot.shooter=only.motors.get("shooter").device;
        adapter.robot.gate=only.gate;
        only.motors.get("turret").failStop=true;
        try { adapter.close(); throw new AssertionError("Simulated cleanup error should surface"); }
        catch (IllegalStateException expected) {
            for (String name:new String[]{"shooter","intake","transmission"}) {
                check(only.motors.get(name).power==0,"Cleanup failure cannot skip stopping " + name);
            }
        }
        check(only.servoWrites==0,"Unused mechanisms must not create an INIT/cleanup gate command");
        MapFixture burst=new MapFixture();
        BioBuzzAutoMechanisms.RobotHardware firing=new BioBuzzAutoMechanisms.RobotHardware();
        firing.robot.initMechanismsOnly(burst,false);
        firing.robot.targetFresh=true; firing.robot.targetVisible=true; firing.robot.targetTx=0;
        firing.robot.ballPresent=true; firing.robot.gateOpen=true;
        java.lang.reflect.Field timer=Robot.class.getDeclaredField("feedTimer"); timer.setAccessible(true);
        ((com.qualcomm.robotcore.util.ElapsedTime)timer.get(firing.robot)).reset();
        firing.control(BioBuzzAutoMechanisms.Kind.SHOOT);
        check(firing.robot.gateOpen,"Existing feed pulse continues through a flywheel RPM dip");
        firing.robot.targetFresh=false;
        firing.control(BioBuzzAutoMechanisms.Kind.SHOOT);
        check(!firing.robot.gateOpen && burst.motors.get("transmission").power==0
                && burst.motors.get("intake").power==0,"Stale aiming data closes gate and stops feeding");
        firing.close();
        System.out.println("Passed " + checks + " mechanism initialization/isolation/cleanup checks.");
    }
}
