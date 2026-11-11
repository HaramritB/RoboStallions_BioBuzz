package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import android.content.SharedPreferences;
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
        double velocity, commandedVelocity;
        final DcMotorEx device=(DcMotorEx)Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),new Class<?>[]{DcMotorEx.class},this);
        @Override public Object invoke(Object proxy,Method method,Object[] args) {
            String name=method.getName();
            if (name.equals("setPower")) {
                trace.add(name+":"+args[0]);
                if (failStop) throw new IllegalStateException("simulated write failure");
                power=(double)args[0]; return null;
            }
            if (name.equals("getVelocity")) return velocity;
            if (name.equals("getCurrentPosition")) return 0;
            if (name.equals("setVelocity")) { commandedVelocity=(double)args[0]; trace.add(name+":"+args[0]); return null; }
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
        final List<Double> gatePositions=new ArrayList<>();
        final Servo gate=(Servo)Proxy.newProxyInstance(Servo.class.getClassLoader(),new Class<?>[]{Servo.class},(proxy,method,args)-> {
            if (method.getName().equals("setPosition")) { servoWrites++; gatePositions.add((double)args[0]); return null; }
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
    static final SharedPreferences PREFS=(SharedPreferences)Proxy.newProxyInstance(
            SharedPreferences.class.getClassLoader(),new Class<?>[]{SharedPreferences.class},(proxy,method,args)-> {
                if (method.getName().equals("contains")) return false;
                if (method.getName().equals("getFloat") || method.getName().equals("getBoolean")) return args[1];
                throw new AssertionError("Unexpected preferences call "+method.getName());
            });
    static class TestRobot extends Robot {
        @Override protected SharedPreferences gatePreferences(HardwareMap map) { return PREFS; }
    }
    static final class SensorRobot extends Robot {
        int wholeSensorReads, mechanismSensorReads;
        double heading, rate;
        @Override public void updateSensors() { wholeSensorReads++; }
        @Override public void updateMechanismSensors(double heading,double rate) {
            mechanismSensorReads++; this.heading=heading; this.rate=rate;
            ballDistanceCm=10;
        }
    }
    public static void main(String[] args) throws Exception {
        MapFixture only=new MapFixture(); Robot mechanisms=new TestRobot(); mechanisms.initMechanismsOnly(only,false);
        for (String name:new String[]{"frontleft","frontright","backleft","backright"}) {
            check(!only.lookups.contains(name) && only.motors.get(name).trace.isEmpty(),"Mechanism init cannot touch " + name);
        }
        check(only.servoWrites==0,"INIT cannot move the gate");
        check(!only.motors.get("turret").trace.toString().contains("STOP_AND_RESET"),"Mechanism-only init preserves turret zero");
        MapFixture whole=new MapFixture(); Robot robot=new TestRobot(); robot.init(whole,false);
        for (String name:new String[]{"intake","transmission","turret","shooter"}) {
            check(only.motors.get(name).trace.equals(whole.motors.get(name).trace),"TeleOp and Auto retain identical initialization for " + name);
        }
        check(whole.motors.get("frontleft").trace.toString().contains("REVERSE"),"Whole Robot init retains old drivetrain direction");
        BioBuzzAutoMechanisms.RobotHardware adapter=new BioBuzzAutoMechanisms.RobotHardware(new TestRobot());
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
        MapFixture intakeMap=new MapFixture();
        BioBuzzAutoMechanisms.RobotHardware intakePort=new BioBuzzAutoMechanisms.RobotHardware(new TestRobot());
        intakePort.robot.initMechanismsOnly(intakeMap,false);
        intakePort.control(BioBuzzAutoMechanisms.Kind.COLLECT);
        check(intakeMap.motors.get("intake").power==Robot.INTAKE_POWER,"Intake request runs the existing motor interface");
        check(intakeMap.motors.get("shooter").power==0 && intakeMap.motors.get("turret").power==0,"Collection cannot run shooter or turret");
        intakePort.robot.ballPresent=true;
        intakePort.control(BioBuzzAutoMechanisms.Kind.COLLECT);
        check(intakeMap.motors.get("intake").power==Robot.INTAKE_POWER,"Feeder presence does not stop open-loop intake motor");
        check(!intakeMap.lookups.contains("flower"),"Autonomous mechanisms never map the flower lever");
        check(intakePort.robot.gateController.command==Robot.GATE_CLOSED,"Collection uses main's calibrated gate controller");
        intakePort.robot.gateController.closedPosition=.67;
        intakePort.close();
        check(intakePort.robot.gateController.command==.67,"Stop honors saved closed endpoint, not a hardcoded default");
        for (String name:new String[]{"intake","transmission","shooter","turret"}) {
            check(intakeMap.motors.get(name).power==0,"Completion stops "+name);
        }
        SensorRobot sensorRobot=new SensorRobot();
        sensorRobot.shooter=intakeMap.motors.get("shooter").device;
        sensorRobot.turret=intakeMap.motors.get("turret").device;
        BioBuzzAutoMechanisms.RobotHardware sensorPort=new BioBuzzAutoMechanisms.RobotHardware(sensorRobot);
        sensorPort.headingCw=()->-90; sensorPort.turnRateCw=()->-5;
        sensorPort.observe();
        check(sensorRobot.wholeSensorReads==0 && sensorRobot.mechanismSensorReads==1,
                "Pedro mode cannot update Pinpoint again through Robot.updateSensors");
        check(sensorRobot.heading==-90 && sensorRobot.rate==-5,"Preserve main turret clockwise heading convention");
        continuousVolley();
        calibratedRpm();
        lostAim();
        System.out.println("Passed " + checks + " mechanism initialization/isolation/cleanup checks.");
    }

    static BioBuzzAutoMechanisms.RobotHardware shootingPort(MapFixture map) {
        Robot robot=new TestRobot() { @Override public void updateSensors() { } };
        robot.initMechanismsOnly(map,false);
        robot.targetFresh=true; robot.targetVisible=true; robot.targetTx=0;
        robot.targetDistanceMeters=1.90; robot.ballPresent=true; robot.ballDistanceCm=2;
        return new BioBuzzAutoMechanisms.RobotHardware(robot);
    }

    static void continuousVolley() {
        MapFixture manualMap=new MapFixture();
        Robot manual=new TestRobot(); manual.initMechanismsOnly(manualMap,false);
        manual.updateManualFeeder(false,-Robot.INTAKE_POWER); // Working TeleOp gamepad 1 RT.
        double intakeReference=manualMap.motors.get("intake").power;
        double transmissionReference=manualMap.motors.get("transmission").power;

        MapFixture map=new MapFixture();
        BioBuzzAutoMechanisms.RobotHardware port=shootingPort(map);
        BioBuzzAutoMechanisms mechanisms=new BioBuzzAutoMechanisms(port);
        long[] now={1_000_000_000L};
        mechanisms.observe();
        com.pedropathing.ivy.Command volley=mechanisms.command(
                BioBuzzAutoMechanisms.Action.shoot(4,7),()->now[0],reason->{throw new AssertionError(reason);});
        volley.start();
        for(int i=0;i<3;i++) { mechanisms.update(); volley.done(); }
        check(map.gatePositions.size()==1 && map.gatePositions.get(0)==Robot.GATE_CLOSED,
                "One calibrated CLOSED command before shooting; no cycling while spinning up");
        check(!port.robot.gateOpen && map.motors.get("intake").power==0 && map.motors.get("transmission").power==0,
                "Gate closed and both feed motors stopped below target RPM");
        map.motors.get("shooter").velocity=3250*Robot.SHOOTER_TICKS_PER_REV/60;
        mechanisms.update(); volley.done();
        check(port.robot.gateOpen && map.motors.get("intake").power==intakeReference
                && map.motors.get("transmission").power==transmissionReference,
                "First at-speed sample opens the gate and starts both motors in RT direction together");
        check(map.gatePositions.size()==2 && map.gatePositions.get(1)==Robot.GATE_OPEN,
                "One calibrated OPEN command starts the entire volley");
        map.motors.get("shooter").velocity=2400*Robot.SHOOTER_TICKS_PER_REV/60;
        now[0]+=500_000_000L; mechanisms.update(); volley.done();
        check(port.robot.gateOpen && port.robot.shotsFired==0 && map.gatePositions.size()==2,
                "RPM dips and the old pulse duration cannot close the gate or fabricate a shot");
        check(map.motors.get("intake").power==intakeReference && map.motors.get("transmission").power==transmissionReference,
                "Both feed motors stay running through RPM dips");
        for(int ball=1;ball<=4;ball++) {
            port.robot.ballPresent=true; mechanisms.update(); volley.done();
            port.robot.ballPresent=false; mechanisms.update(); volley.done();
            check(port.robot.shotsFired==ball,"Count one observed ball exit, not a timed gate pulse");
            if(ball<4) {
                mechanisms.update(); volley.done();
                check(port.robot.shotsFired==ball,"Repeated empty readings cannot count another ball");
                check(port.robot.gateOpen && map.gatePositions.size()==2,
                        "Keep the gate open between balls without extra servo commands");
                check(map.motors.get("intake").power==intakeReference && map.motors.get("transmission").power==transmissionReference,
                        "Keep both motors continuously in RT direction between balls");
            }
        }
        check(!port.robot.gateOpen && map.motors.get("intake").power==0 && map.motors.get("transmission").power==0,
                "The fourth observed exit closes the gate and stops motors before another feed command");
        now[0]+=210_000_000L; mechanisms.update();
        check(volley.done(),"Four exits complete the burst after result settling");
        volley.end(com.pedropathing.ivy.behaviors.EndCondition.NATURALLY); mechanisms.stop();
        check(map.gatePositions.equals(java.util.Arrays.asList(Robot.GATE_CLOSED,Robot.GATE_OPEN,Robot.GATE_CLOSED)),
                "A complete four-ball volley commands CLOSED, OPEN, CLOSED exactly once each");
    }

    static void calibratedRpm() {
        MapFixture map=new MapFixture();
        BioBuzzAutoMechanisms.RobotHardware port=shootingPort(map);
        double[] distances={1.41,1.60,1.76,1.90,1.83};
        double[] rpm={3150,3150,3200,3250,3225};
        for(int i=0;i<distances.length;i++) {
            port.robot.targetDistanceMeters=distances[i]; port.control(BioBuzzAutoMechanisms.Kind.SHOOT);
            check(Math.abs(port.robot.targetRpm-rpm[i])<1e-8,"Use calibrated/interpolated RPM at "+distances[i]+" m");
            check(Math.abs(map.motors.get("shooter").commandedVelocity-rpm[i]*Robot.SHOOTER_TICKS_PER_REV/60)<1e-8,
                    "Send that same calibrated target to the shooter motor");
        }
        port.close();
    }

    static void lostAim() {
        MapFixture map=new MapFixture();
        BioBuzzAutoMechanisms.RobotHardware port=shootingPort(map);
        map.motors.get("shooter").velocity=3250*Robot.SHOOTER_TICKS_PER_REV/60;
        port.control(BioBuzzAutoMechanisms.Kind.SHOOT);
        check(port.robot.gateOpen,"Ready shooter starts burst immediately");
        port.robot.targetFresh=false;
        try { port.control(BioBuzzAutoMechanisms.Kind.SHOOT); throw new AssertionError("Lost aim must abort the burst"); }
        catch(IllegalStateException expected) {
            check(!port.robot.gateOpen && map.motors.get("intake").power==0 && map.motors.get("transmission").power==0,
                    "Loss of aiming aborts instead of cycling the gate and restarting mid-volley");
        }
        port.close();
    }
}
