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
        check(intakeMap.motors.get("intake").power==-Robot.INTAKE_POWER
                        && intakeMap.motors.get("transmission").power==-Robot.TRANSMISSION_INDEX_POWER,
                "Collecting runs intake AND transmission in the shooting-feed (gamepad 1 RT) direction");
        check(intakeMap.motors.get("shooter").power==0 && intakeMap.motors.get("turret").power==0,"Collection cannot run shooter or turret");
        intakePort.robot.ballPresent=true;
        intakePort.control(BioBuzzAutoMechanisms.Kind.COLLECT);
        check(intakeMap.motors.get("intake").power==-Robot.INTAKE_POWER
                        && intakeMap.motors.get("transmission").power==-Robot.TRANSMISSION_INDEX_POWER,
                "A ball at the feeder sensor does not stop collection; the closed gate holds the stack");
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
        timedContinuousVolley();
        calibratedRpm();
        stableSpeedBeforeFeeding();
        startupNeedsTarget();
        noTagFallback();
        forcedFeed();
        lostAim();
        readyShooterBetweenCommands();
        alwaysOnIntake();
        hivePreAim();
        travellingTurretHoldsGate();
        secondVolleyGeometry();
        System.out.println("Passed " + checks + " mechanism initialization/isolation/cleanup checks.");
    }

    static final long STEP_NS=20_000_000L;
    static final long GATE_WAIT_NS=Math.round(BioBuzzAutoConfig.GATE_OPEN_WAIT_SECONDS*1e9);

    static BioBuzzAutoMechanisms.RobotHardware shootingPort(MapFixture map) {
        return shootingPort(map,System::nanoTime);
    }

    static BioBuzzAutoMechanisms.RobotHardware shootingPort(MapFixture map, java.util.function.LongSupplier clock) {
        Robot robot=new TestRobot() { @Override public void updateSensors() { } };
        robot.initMechanismsOnly(map,false);
        robot.targetFresh=true; robot.targetVisible=true; robot.targetTx=0;
        robot.targetDistanceMeters=1.62; robot.ballPresent=true; robot.ballDistanceCm=2;
        return new BioBuzzAutoMechanisms.RobotHardware(robot,clock);
    }

    static double ticks(double rpm) { return rpm*Robot.SHOOTER_TICKS_PER_REV/60; }

    /** Step until readiness commands the gate OPEN (motors must still be stopped then). */
    static long openGate(BioBuzzAutoMechanisms.RobotHardware port,long[] now,Runnable tick) {
        for(int i=0;i<250 && !port.robot.gateOpen;i++) { now[0]+=STEP_NS; tick.run(); }
        check(port.robot.gateOpen,"Continuously sampled stable speed must open the gate");
        return now[0];
    }

    /** Step until both feed motors run (after the gate-open wait). */
    static void qualifySpeed(BioBuzzAutoMechanisms.RobotHardware port,long[] now,Runnable tick) {
        for(int i=0;i<250 && !port.shootingBurstActive();i++) { now[0]+=STEP_NS; tick.run(); }
        check(port.shootingBurstActive(),"Continuously sampled stable speed must qualify the burst");
    }

    static void continuousVolley() {
        MapFixture manualMap=new MapFixture();
        Robot manual=new TestRobot(); manual.initMechanismsOnly(manualMap,false);
        manual.updateManualFeeder(false,-Robot.INTAKE_POWER); // Working TeleOp gamepad 1 RT.
        double intakeReference=manualMap.motors.get("intake").power;
        double transmissionReference=manualMap.motors.get("transmission").power;

        MapFixture map=new MapFixture();
        long[] now={1_000_000_000L};
        BioBuzzAutoMechanisms.RobotHardware port=shootingPort(map,()->now[0]);
        BioBuzzAutoMechanisms mechanisms=new BioBuzzAutoMechanisms(port);
        mechanisms.observe();
        com.pedropathing.ivy.Command volley=mechanisms.command(
                BioBuzzAutoMechanisms.Action.shoot(4,7),()->now[0],reason->{throw new AssertionError(reason);});
        Runnable tick=()->{ mechanisms.update(); volley.done(); };
        volley.start();
        for(int i=0;i<3;i++) tick.run();
        check(map.gatePositions.size()==1 && map.gatePositions.get(0)==Robot.GATE_CLOSED,
                "One calibrated CLOSED command before shooting; no cycling while spinning up");
        check(!port.robot.gateOpen && map.motors.get("intake").power==0 && map.motors.get("transmission").power==0,
                "Gate closed and both feed motors stopped below target RPM");
        map.motors.get("shooter").velocity=ticks(3235);
        tick.run();
        check(!port.robot.gateOpen,"A single in-band sample must not open the gate");
        long opened=openGate(port,now,tick);
        check(map.gatePositions.size()==2 && map.gatePositions.get(1)==Robot.GATE_OPEN,
                "One calibrated OPEN command starts the entire volley");
        check(map.motors.get("intake").power==0 && map.motors.get("transmission").power==0 && !port.shootingBurstActive(),
                "The gate opens FIRST; intake and transmission stay stopped while it travels");
        now[0]=opened+GATE_WAIT_NS-STEP_NS; tick.run();
        check(map.motors.get("intake").power==0 && map.motors.get("transmission").power==0,
                "Feed motors wait the full gate-open time");
        now[0]=opened+GATE_WAIT_NS; tick.run();
        check(port.shootingBurstActive() && map.motors.get("intake").power==intakeReference
                        && map.motors.get("transmission").power==transmissionReference,
                "After the gate has opened, both motors start together in RT direction");
        map.motors.get("shooter").velocity=ticks(2400);
        now[0]+=500_000_000L; tick.run();
        check(port.robot.gateOpen && port.robot.shotsFired==0 && map.gatePositions.size()==2,
                "RPM dips and the old pulse duration cannot close the gate or fabricate a shot");
        check(map.motors.get("intake").power==intakeReference && map.motors.get("transmission").power==transmissionReference,
                "Both feed motors stay running through RPM dips");
        for(int ball=1;ball<=4;ball++) {
            port.robot.ballPresent=true; tick.run();
            port.robot.ballPresent=false; tick.run();
            check(port.robot.shotsFired==ball,"Count one observed ball exit, not a timed gate pulse");
            if(ball<4) {
                tick.run();
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
        long[] now={1_000_000_000L};
        BioBuzzAutoMechanisms.RobotHardware port=shootingPort(map,()->now[0]);
        double[] distances={1.39,1.52,1.62,1.455,1.57};
        double[] rpm={3215,3220,3235,3217.5,3227.5};
        for(int i=0;i<distances.length;i++) {
            port.robot.targetDistanceMeters=distances[i];
            for(int frame=0;frame<BioBuzzAutoConfig.DISTANCE_FILTER_SAMPLES;frame++) {
                now[0]+=STEP_NS; port.control(BioBuzzAutoMechanisms.Kind.SHOOT);
            }
            check(Math.abs(port.robot.targetRpm-rpm[i])<1e-8,"Use calibrated/interpolated RPM at "+distances[i]+" m");
            check(Math.abs(map.motors.get("shooter").commandedVelocity-ticks(rpm[i]))<1e-8,
                    "Send that same calibrated target to the shooter motor");
        }
        port.robot.targetDistanceMeters=1.39;
        now[0]+=STEP_NS; port.control(BioBuzzAutoMechanisms.Kind.SHOOT);
        check(Math.abs(port.robot.targetRpm-3227.5)<1e-8,"One outlier distance frame cannot change the calibrated RPM");
        port.robot.targetDistanceMeters=1.57;
        now[0]+=STEP_NS; port.control(BioBuzzAutoMechanisms.Kind.SHOOT);
        port.robot.targetFresh=false; port.robot.targetDistanceMeters=Double.NaN;
        now[0]+=500_000_000L; port.control(BioBuzzAutoMechanisms.Kind.SHOOT);
        check(Math.abs(port.robot.targetRpm-3227.5)<1e-8,"A brief tag dropout holds the last calibrated distance RPM");
        now[0]+=(long)(BioBuzzAutoConfig.DISTANCE_HOLD_SECONDS*1e9); port.control(BioBuzzAutoMechanisms.Kind.SHOOT);
        check(port.robot.targetRpm==Robot.TELEOP_DEFAULT_RPM,"A stale distance falls back to TeleOp's no-tag table default");
        port.close();
    }

    static void stableSpeedBeforeFeeding() {
        MapFixture map=new MapFixture();
        long[] now={1_000_000_000L};
        BioBuzzAutoMechanisms.RobotHardware port=shootingPort(map,()->now[0]);
        Runnable shoot=()->port.control(BioBuzzAutoMechanisms.Kind.SHOOT);
        shoot.run();
        // The old +/-100 RPM single-sample check would accept 3136 for a 3235 target.
        map.motors.get("shooter").velocity=ticks(3136);
        for(int i=0;i<25;i++) { now[0]+=STEP_NS; shoot.run(); }
        check(!port.robot.gateOpen && map.motors.get("intake").power==0 && map.motors.get("transmission").power==0,
                "A flywheel still 99 RPM below target cannot release balls");
        map.motors.get("shooter").velocity=ticks(3235);
        for(int i=0;i<10;i++) { now[0]+=STEP_NS; shoot.run(); }
        check(!port.robot.gateOpen,"A brief crossing through target RPM is not stable readiness");
        map.motors.get("shooter").velocity=ticks(3184);
        now[0]+=STEP_NS; shoot.run();
        map.motors.get("shooter").velocity=ticks(3336);
        now[0]+=STEP_NS; shoot.run();
        check(!port.robot.gateOpen,"RPM dips and overshoot reset the qualifying window");
        map.motors.get("shooter").velocity=ticks(3235);
        for(int i=0;i<14;i++) { now[0]+=STEP_NS; shoot.run(); }
        check(!port.robot.gateOpen,"Require the complete stable-speed duration after a disturbance");
        now[0]+=200_000_000L; shoot.run();
        check(!port.robot.gateOpen,"An old sample followed by a long loop gap cannot qualify speed");
        for(int i=0;i<14;i++) { now[0]+=STEP_NS; shoot.run(); }
        check(!port.robot.gateOpen,"Rebuild the stable window from fresh samples after a gap");
        // A 100-RPM trim change must restart qualification. The new table's entire 20-RPM
        // spread is inside Auto's +/-50 window and intentionally below its 25-RPM reset threshold.
        port.robot.rpmTrim=-100;
        map.motors.get("shooter").velocity=ticks(3135);
        long targetChanged=-1;
        for(int i=0;i<40 && !port.robot.gateOpen;i++) {
            now[0]+=STEP_NS; shoot.run();
            if(targetChanged<0 && port.robot.targetRpm+port.robot.rpmTrim==3135) targetChanged=now[0];
        }
        check(targetChanged>0 && port.robot.gateOpen && port.robot.targetRpm==3235,
                "Open only after sustained measured speed at the calibrated target plus trim");
        check((now[0]-targetChanged)/1e9>=BioBuzzAutoConfig.SHOOT_READY_STABLE_SECONDS-1e-9,
                "Do not reuse stable time from the previous RPM target");
        port.robot.targetDistanceMeters=1.39;
        now[0]+=STEP_NS; shoot.run();
        check(port.robot.targetRpm==3235 && Math.abs(map.motors.get("shooter").commandedVelocity-ticks(3135))<1e-8,
                "Distance-frame jitter cannot change the qualified RPM target during a stationary burst");
        map.motors.get("shooter").velocity=ticks(2400);
        now[0]+=STEP_NS; shoot.run();
        check(port.robot.gateOpen && port.diagnostics().contains("feed-start RPM 3135/3135")
                        && port.diagnostics().contains("min burst RPM 2400"),
                "Keep the gate open during feeding and report starting speed versus load-induced speed loss");
        port.stopOutputs();
        port.robot.rpmTrim=0;
        port.robot.targetDistanceMeters=1.39;
        map.motors.get("shooter").velocity=ticks(3215);
        long restarted=now[0]+STEP_NS;
        now[0]=restarted; shoot.run();
        check(!port.robot.gateOpen,"A new run/volley cannot reuse the previous readiness latch");
        for(int i=0;i<2;i++) {
            now[0]+=100_000_000L; shoot.run();
            check(!port.robot.gateOpen,"Even a warm flywheel must satisfy the fresh-start minimum settle interval");
        }
        for(int i=0;i<5 && !port.robot.gateOpen;i++) { now[0]+=100_000_000L; shoot.run(); }
        check(port.robot.gateOpen && (now[0]-restarted)/1e9>=BioBuzzAutoConfig.SHOOT_MIN_SPINUP_SECONDS,
                "A repeated run can qualify with its own new stable samples");
        port.close();
    }

    static void timedContinuousVolley() {
        MapFixture map=new MapFixture();
        long[] now={1_000_000_000L};
        BioBuzzAutoMechanisms.RobotHardware port=shootingPort(map,()->now[0]);
        BioBuzzAutoMechanisms mechanisms=new BioBuzzAutoMechanisms(port);
        mechanisms.observe();
        com.pedropathing.ivy.Command volley=mechanisms.command(
                BioBuzzAutoMechanisms.Action.shootFor(4,2,10),()->now[0],reason->{throw new AssertionError(reason);});
        volley.start(); mechanisms.update(); volley.done();
        now[0]+=9_500_000_000L; mechanisms.update(); volley.done();
        check(!port.robot.gateOpen && map.motors.get("intake").power==0,
                "A stalled flywheel (0 RPM) is never force-fed and gets its spin-up allowance without consuming feed time");
        map.motors.get("shooter").velocity=ticks(3235);
        mechanisms.update(); volley.done();
        qualifySpeed(port,now,()->{ mechanisms.update(); volley.done(); });
        check(port.robot.gateOpen && map.motors.get("intake").power==-Robot.INTAKE_POWER
                && map.motors.get("transmission").power==-Robot.TRANSMISSION_INDEX_POWER,
                "Timed feed starts after speed qualification and the gate wait, in RT direction");
        // Keep the feeder reading occupied throughout: four separate ball edges cannot be counted.
        now[0]+=1_999_000_000L; mechanisms.update(); volley.done();
        check(port.robot.gateOpen && port.robot.shotsFired==0,
                "Continuous feed remains active for its full duration without fabricating ball counts");
        now[0]+=1_000_000L; mechanisms.update(); volley.done();
        check(!port.robot.gateOpen && map.motors.get("intake").power==0 && map.motors.get("transmission").power==0,
                "End timed feed with no further actuator output even if the feeder cannot distinguish all four balls");
        now[0]+=210_000_000L; mechanisms.update();
        check(volley.done(),"A completed timed burst does not throw the old overall mechanism timeout");
        volley.end(com.pedropathing.ivy.behaviors.EndCondition.NATURALLY); mechanisms.stop();
        check(map.gatePositions.equals(java.util.Arrays.asList(Robot.GATE_CLOSED,Robot.GATE_OPEN,Robot.GATE_CLOSED)),
                "Uncounted timed volley still commands the gate closed/open/closed once each");
    }

    static void startupNeedsTarget() {
        MapFixture map=new MapFixture();
        long[] now={1_000_000_000L};
        BioBuzzAutoMechanisms.RobotHardware port=shootingPort(map,()->now[0]);
        port.robot.targetFresh=false; port.robot.targetDistanceMeters=Double.NaN;
        map.motors.get("shooter").velocity=ticks(Robot.TELEOP_DEFAULT_RPM);
        port.control(BioBuzzAutoMechanisms.Kind.SHOOT);
        now[0]+=1_000_000_000L; port.control(BioBuzzAutoMechanisms.Kind.SHOOT);
        check(!port.robot.gateOpen && map.motors.get("intake").power==0 && map.motors.get("transmission").power==0,
                "Within the aim grace period, an unaimed shooter waits for the tag");
        port.robot.targetFresh=true; port.robot.targetDistanceMeters=1.62;
        map.motors.get("shooter").velocity=ticks(3235);
        port.control(BioBuzzAutoMechanisms.Kind.SHOOT);
        qualifySpeed(port,now,()->port.control(BioBuzzAutoMechanisms.Kind.SHOOT));
        check(port.robot.gateOpen && port.diagnostics().contains("(aimed)"),
                "Fresh aimed distance and stable calibrated RPM allow feeding");
        port.close();
    }

    static void noTagFallback() {
        MapFixture map=new MapFixture();
        long[] now={1_000_000_000L};
        BioBuzzAutoMechanisms.RobotHardware port=shootingPort(map,()->now[0]);
        port.robot.targetFresh=false; port.robot.targetVisible=false; port.robot.targetDistanceMeters=Double.NaN;
        map.motors.get("shooter").velocity=ticks(Robot.TELEOP_DEFAULT_RPM);
        long began=now[0];
        port.control(BioBuzzAutoMechanisms.Kind.SHOOT);
        long opened=openGate(port,now,()->port.control(BioBuzzAutoMechanisms.Kind.SHOOT));
        check((opened-began)/1e9>=BioBuzzAutoConfig.SHOOT_AIM_GRACE_SECONDS,
                "Wait the full aim grace period for the HIVE tag before a no-tag volley");
        check(port.robot.targetRpm==Robot.TELEOP_DEFAULT_RPM && port.diagnostics().contains("calibrated fallback"),
                "With no tag, shoot at TeleOp's calibrated no-tag RPM instead of ending the autonomous");
        port.close();
    }

    static void forcedFeed() {
        MapFixture map=new MapFixture();
        long[] now={1_000_000_000L};
        BioBuzzAutoMechanisms.RobotHardware port=shootingPort(map,()->now[0]);
        // Low battery: the flywheel holds 120 RPM under its 3235 target, outside the +/-50 band.
        map.motors.get("shooter").velocity=ticks(3115);
        long began=now[0];
        port.control(BioBuzzAutoMechanisms.Kind.SHOOT);
        long opened=openGate(port,now,()->port.control(BioBuzzAutoMechanisms.Kind.SHOOT));
        check((opened-began)/1e9>=BioBuzzAutoConfig.SHOOT_FORCE_FEED_SECONDS && port.diagnostics().contains("forced"),
                "A flywheel that cannot settle in band is fed at the force time instead of timing out");
        qualifySpeed(port,now,()->port.control(BioBuzzAutoMechanisms.Kind.SHOOT));
        check(map.motors.get("intake").power==-Robot.INTAKE_POWER,"Forced volleys still wait for the gate before feeding");
        port.close();
    }

    static void lostAim() {
        MapFixture map=new MapFixture();
        long[] now={1_000_000_000L};
        BioBuzzAutoMechanisms.RobotHardware port=shootingPort(map,()->now[0]);
        map.motors.get("shooter").velocity=ticks(3235);
        port.control(BioBuzzAutoMechanisms.Kind.SHOOT);
        qualifySpeed(port,now,()->port.control(BioBuzzAutoMechanisms.Kind.SHOOT));
        check(port.robot.gateOpen,"Qualified shooter starts the continuous burst");
        port.robot.targetFresh=false; port.robot.targetDistanceMeters=Double.NaN;
        for(int i=0;i<50;i++) { now[0]+=STEP_NS; port.control(BioBuzzAutoMechanisms.Kind.SHOOT); } // One full second.
        check(port.robot.gateOpen && map.motors.get("intake").power==-Robot.INTAKE_POWER
                        && map.motors.get("transmission").power==-Robot.TRANSMISSION_INDEX_POWER,
                "Losing the tag mid-burst keeps feeding the short volley instead of aborting the autonomous");
        check(port.robot.targetRpm==3235 && map.gatePositions.size()==2,
                "Hold the latched calibrated RPM without cycling the gate");
        check(port.diagnostics().contains("aim lost 50 feed loops"),"Report aim loss during the burst");
        port.close();
    }

    /** AutoPath shot-2 fix: remember the HIVE from a stopped, centered sighting, then predict it. */
    static void hivePreAim() {
        MapFixture map=new MapFixture();
        long[] now={1_000_000_000L};
        BioBuzzAutoMechanisms.RobotHardware port=shootingPort(map,()->now[0]);
        double[] pose={0,0,0}; // x in, y in, heading clockwise degrees
        double[] turn={0};
        port.headingCw=()->pose[2]; port.turnRateCw=()->turn[0];
        port.xIn=()->pose[0]; port.yIn=()->pose[1];
        map.motors.get("shooter").velocity=ticks(3235);
        double range=1.62/0.0254;
        port.control(BioBuzzAutoMechanisms.Kind.NONE);
        check(port.hiveFixes==0,"One pose sample cannot show the chassis is still");
        for (int i=0;i<5;i++) { now[0]+=STEP_NS; port.control(BioBuzzAutoMechanisms.Kind.NONE); }
        check(port.hiveFixes==0,"Not still long enough yet: no HIVE fix");
        for (int i=0;i<15;i++) { now[0]+=STEP_NS; port.control(BioBuzzAutoMechanisms.Kind.NONE); }
        // Encoder 0 points TURRET_ZERO_DIRECTION_DEG (180 = backward) from the robot front.
        double zero=Math.toRadians(BioBuzzAutoConfig.TURRET_ZERO_DIRECTION_DEG);
        check(port.hiveFixes>0 && Math.abs(port.hiveX-range*Math.cos(-zero))<1e-6 && Math.abs(port.hiveY-range*Math.sin(-zero))<1e-6,
                "Still chassis + sighting at encoder 0, even between volleys: the HIVE is where the zeroed turret points");
        check(Math.abs(port.hiveX+range)<1e-6,"Zeroed backward: a centered sighting at encoder 0 is behind the robot");
        java.util.function.DoubleSupplier heading=port.headingCw;
        port.headingCw=null; // observe() then uses the fixture's no-op sensor update
        port.observe();
        port.headingCw=heading;
        check(port.diagnostics().contains("HIVE at"),"Telemetry reports the located HIVE");

        int fixes=port.hiveFixes;
        for (int i=0;i<25;i++) { now[0]+=STEP_NS; pose[1]+=0.5; port.control(BioBuzzAutoMechanisms.Kind.NONE); }
        check(port.hiveFixes==fixes,"Sightings while driving are not used (camera latency would misplace the HIVE)");
        pose[1]=0;
        for (int i=0;i<25;i++) { now[0]+=STEP_NS; port.control(BioBuzzAutoMechanisms.Kind.NONE); }
        fixes=port.hiveFixes; turn[0]=30;
        for (int i=0;i<25;i++) { now[0]+=STEP_NS; port.control(BioBuzzAutoMechanisms.Kind.NONE); }
        check(port.hiveFixes==fixes,"Sightings while the chassis turns are not used");
        turn[0]=0;
        map.motors.get("turret").velocity=300*Robot.TURRET_TICKS_PER_DEG;
        for (int i=0;i<25;i++) { now[0]+=STEP_NS; port.control(BioBuzzAutoMechanisms.Kind.NONE); }
        check(port.hiveFixes==fixes,"Sightings while the turret slews fast are not used");
        map.motors.get("turret").velocity=0;

        port.robot.targetFresh=false; port.robot.targetVisible=false; port.robot.targetTx=Double.NaN;
        port.stopFeed();
        pose[1]=range; // drove left by the same distance, no vision; HIVE now behind-right
        now[0]+=STEP_NS; port.control(BioBuzzAutoMechanisms.Kind.NONE);
        check(sameAngle(port.preAimDeg+BioBuzzAutoConfig.TURRET_ZERO_DIRECTION_DEG,135),
                "Odometry predicts the HIVE 135 deg right of the robot front after driving left");
        check(sameAngle(port.preAimDeg,-45),"...which is encoder -45 with the turret zeroed backward");
        pose[2]=-90; // and turned 90 deg left
        now[0]+=STEP_NS; port.control(BioBuzzAutoMechanisms.Kind.NONE);
        check(sameAngle(port.preAimDeg+BioBuzzAutoConfig.TURRET_ZERO_DIRECTION_DEG,225),
                "A left turn adds 90 deg to the predicted right turn");
        port.close();

        // A tag that is not centered still locates the HIVE: turret angle plus the tag offset.
        BioBuzzAutoMechanisms.RobotHardware off=shootingPort(new MapFixture(),()->now[0]);
        off.headingCw=()->0; off.turnRateCw=()->0; off.xIn=()->0; off.yIn=()->0;
        off.robot.targetTx=10/Robot.TURRET_VISION_POWER_SIGN; // 10 deg clockwise of the camera center
        for (int i=0;i<20;i++) { now[0]+=STEP_NS; off.control(BioBuzzAutoMechanisms.Kind.NONE); }
        double offBearing=-Math.toRadians(BioBuzzAutoConfig.TURRET_ZERO_DIRECTION_DEG+10);
        check(off.hiveFixes>0 && Math.abs(off.robot.visionErrorDegrees()-10)<1e-9
                        && Math.abs(off.hiveX-range*Math.cos(offBearing))<1e-6
                        && Math.abs(off.hiveY-range*Math.sin(offBearing))<1e-6,
                "An off-center sighting (e.g. while unwinding at START) still gives the HIVE position");
        off.close();

        BioBuzzAutoMechanisms.RobotHardware noPose=shootingPort(new MapFixture(),()->now[0]);
        noPose.headingCw=()->0; noPose.turnRateCw=()->0;
        noPose.control(BioBuzzAutoMechanisms.Kind.SHOOT);
        check(noPose.hiveFixes==0 && Double.isNaN(noPose.preAimDeg),
                "Without a pose supplier the turret keeps the original sweep");
        noPose.close();
    }

    static boolean sameAngle(double a,double b) {
        double d=a-b; d-=360*Math.floor((d+180)/360); return Math.abs(d)<1e-6;
    }

    /** AutoPath's real geometry: START (56,8) facing 270, spot 2 (59.8,126.9) facing 90, blue HIVE ~(95,71). */
    static void secondVolleyGeometry() {
        long[] now={1_000_000_000L};
        double[] turretEncoder={0};
        Robot robot=new TestRobot() {
            @Override public void updateSensors() { }
            @Override public double turretDegrees() { return turretEncoder[0]; }
        };
        robot.initMechanismsOnly(new MapFixture(),false);
        robot.targetFresh=true; robot.targetVisible=true; robot.targetTx=0; robot.ballPresent=true; robot.ballDistanceCm=2;
        BioBuzzAutoMechanisms.RobotHardware port=new BioBuzzAutoMechanisms.RobotHardware(robot,()->now[0]);
        double[] pose={56,8,-270}; // Pedro heading 270 -> clockwise -270
        port.headingCw=()->pose[2]; port.turnRateCw=()->0; port.xIn=()->pose[0]; port.yIn=()->pose[1];
        double hx=95.2, hy=70.7;
        double physical=Robot.turretDegreesToward(56,8,-270,hx,hy); // HIVE ~148 deg left, behind the robot
        double encoder=physical-BioBuzzAutoConfig.TURRET_ZERO_DIRECTION_DEG;
        encoder-=360*Math.floor((encoder+180)/360);
        check(Math.abs(encoder-32)<1,"Zeroed backward, the turret locks at START near encoder +32 (got " + encoder + ")");
        turretEncoder[0]=encoder;
        robot.targetDistanceMeters=Math.hypot(hx-56,hy-8)*0.0254;
        for (int i=0;i<30;i++) { now[0]+=STEP_NS; port.control(BioBuzzAutoMechanisms.Kind.NONE); }
        check(port.hiveFixes>0 && Math.abs(port.hiveX-hx)<1e-6 && Math.abs(port.hiveY-hy)<1e-6,
                "The START lock puts the HIVE at its real field position (was mirrored behind START)");
        robot.targetFresh=false; robot.targetVisible=false; robot.targetTx=Double.NaN;
        pose[0]=59.7632; pose[1]=126.893; pose[2]=-90; // spot 2, Pedro heading 90
        now[0]+=STEP_NS; port.control(BioBuzzAutoMechanisms.Kind.NONE);
        double want=Robot.turretDegreesToward(59.7632,126.893,-90,hx,hy); // ~148 deg right
        check(sameAngle(port.preAimDeg+BioBuzzAutoConfig.TURRET_ZERO_DIRECTION_DEG,want) && Math.abs(want-147.8)<1,
                "At spot 2 the turret pre-aims ~148 deg right of the robot front, at the HIVE");
        check(sameAngle(port.preAimDeg,-32.2) || Math.abs(port.preAimDeg+32.2)<0.1,
                "...which is encoder ~-32, well inside the soft limits");
        port.close();
    }

    /** The 2nd volley fired at the 2 s aim grace while the turret was still turning toward the HIVE. */
    static void travellingTurretHoldsGate() {
        for (String state:new String[]{"PRE-AIM","UNWIND RIGHT","TRACK","REACQUIRE LAST TAG","SEARCH RIGHT","NO CAMERA"}) {
            MapFixture map=new MapFixture();
            long[] now={1_000_000_000L};
            Robot robot=new TestRobot() {
                @Override public void updateSensors() { }
                @Override public String turretAimState() { return state; }
            };
            robot.initMechanismsOnly(map,false);
            robot.targetFresh=false; robot.targetVisible=false; robot.targetDistanceMeters=Double.NaN;
            robot.ballPresent=true; robot.ballDistanceCm=2;
            BioBuzzAutoMechanisms.RobotHardware port=new BioBuzzAutoMechanisms.RobotHardware(robot,()->now[0]);
            map.motors.get("shooter").velocity=ticks(Robot.TELEOP_DEFAULT_RPM);
            long began=now[0];
            port.control(BioBuzzAutoMechanisms.Kind.SHOOT);
            long opened=openGate(port,now,()->port.control(BioBuzzAutoMechanisms.Kind.SHOOT));
            double waited=(opened-began)/1e9;
            boolean blind=state.startsWith("SEARCH") || state.equals("NO CAMERA");
            check(waited>=BioBuzzAutoConfig.SHOOT_AIM_GRACE_SECONDS,state + ": never feeds unaimed before the aim grace");
            check(blind ? waited<BioBuzzAutoConfig.SHOOT_FORCE_FEED_SECONDS : waited>=BioBuzzAutoConfig.SHOOT_FORCE_FEED_SECONDS,
                    state + (blind ? ": nothing to turn to, so the 2 s grace applies"
                            : ": turret still turning to the HIVE keeps the gate closed until the force-feed time"));
            port.close();
        }
    }

    static void alwaysOnIntake() {
        MapFixture map=new MapFixture();
        long[] now={1_000_000_000L};
        BioBuzzAutoMechanisms.RobotHardware port=shootingPort(map,()->now[0]);
        BioBuzzAutoMechanisms mechanisms=new BioBuzzAutoMechanisms(port);
        mechanisms.setFaultTolerant(true);
        mechanisms.observe();
        mechanisms.keepShooterReady(true); mechanisms.keepIntakeRunning(true);
        mechanisms.update();
        double in=-Robot.INTAKE_POWER, tx=-Robot.TRANSMISSION_INDEX_POWER;
        check(map.motors.get("intake").power==in && map.motors.get("transmission").power==tx && !port.robot.gateOpen,
                "With no command, the always-on intake collects inward against the closed gate");
        map.motors.get("shooter").velocity=ticks(3235);
        com.pedropathing.ivy.Command volley=mechanisms.command(
                BioBuzzAutoMechanisms.Action.shootFor(4,BioBuzzAutoConfig.SHOOT_BURST_SECONDS,BioBuzzAutoConfig.SHOOT_SPINUP_TIMEOUT_S),
                ()->now[0],reason->{throw new AssertionError(reason);});
        Runnable tick=()->{ mechanisms.update(); volley.done(); };
        volley.start(); now[0]+=STEP_NS; tick.run();
        check(map.motors.get("intake").power==in && map.motors.get("transmission").power==0 && !port.robot.gateOpen,
                "During spin-up the intake roller runs, but the transmission stops packing balls into the closed gate");
        long opened=openGate(port,now,tick);
        double relief=-Math.signum(tx)*BioBuzzAutoConfig.GATE_RELIEF_TRANSMISSION_POWER;
        check(map.motors.get("intake").power==in && map.motors.get("transmission").power==relief,
                "As the gate starts opening, the transmission briefly backs balls OFF the gate");
        now[0]=opened+Math.round(BioBuzzAutoConfig.GATE_RELIEF_SECONDS*1e9); tick.run();
        check(port.robot.gateOpen && map.motors.get("intake").power==in && map.motors.get("transmission").power==0,
                "After the back-off, the transmission waits for the gate to finish opening");
        now[0]=opened+GATE_WAIT_NS; tick.run();
        check(port.shootingBurstActive() && map.motors.get("intake").power==in && map.motors.get("transmission").power==tx,
                "Once the gate is open, both feed the shot inward");
        long burstEnd=now[0]+Math.round(BioBuzzAutoConfig.SHOOT_BURST_SECONDS*1e9);
        now[0]=burstEnd; tick.run();
        check(!port.robot.gateOpen && map.motors.get("intake").power==in && map.motors.get("transmission").power==0,
                "The burst closes the gate; the intake stays on and the transmission waits for the gate to close");
        now[0]=burstEnd+GATE_WAIT_NS; tick.run();
        check(map.motors.get("intake").power==in && map.motors.get("transmission").power==tx,
                "After the gate has closed, collecting resumes immediately");
        now[0]+=210_000_000L; mechanisms.update();
        check(volley.done(),"The volley still completes normally");
        volley.end(com.pedropathing.ivy.behaviors.EndCondition.NATURALLY);
        check(map.motors.get("intake").power==in,"Ending the volley command does not stop the always-on intake");
        mechanisms.stop();
        check(map.motors.get("intake").power==0 && map.motors.get("transmission").power==0,"STOP still stops the intake");
    }

    static void readyShooterBetweenCommands() {
        MapFixture map=new MapFixture();
        long[] now={1_000_000_000L};
        BioBuzzAutoMechanisms.RobotHardware port=shootingPort(map,()->now[0]);
        BioBuzzAutoMechanisms mechanisms=new BioBuzzAutoMechanisms(port);
        mechanisms.setFaultTolerant(true);
        mechanisms.observe();
        mechanisms.manualIntake(false);
        check(map.gatePositions.isEmpty(),"INIT idle still sends no gate command");
        mechanisms.keepShooterReady(true);
        mechanisms.update();
        check(port.robot.shooterOn && Math.abs(map.motors.get("shooter").commandedVelocity-ticks(3235))<1e-8,
                "Ready mode spins the flywheel at the calibrated table RPM between commands");
        check(!port.robot.gateOpen && map.motors.get("intake").power==0 && map.motors.get("transmission").power==0,
                "Ready mode never opens the gate or feeds");
        mechanisms.manualIntake(true); now[0]+=STEP_NS; mechanisms.update();
        check(map.motors.get("intake").power==-Robot.INTAKE_POWER && port.robot.shooterOn,
                "Intake runs while the flywheel stays ready");
        List<String> shooterTrace=map.motors.get("shooter").trace;
        int before=shooterTrace.size();
        mechanisms.manualIntake(false);
        check(map.motors.get("intake").power==0 && !shooterTrace.subList(before,shooterTrace.size()).contains("setPower:0.0"),
                "Stopping intake does not shut the flywheel down");
        map.motors.get("shooter").velocity=ticks(3235);
        com.pedropathing.ivy.Command volley=mechanisms.command(
                BioBuzzAutoMechanisms.Action.shootFor(4,BioBuzzAutoConfig.SHOOT_BURST_SECONDS,BioBuzzAutoConfig.SHOOT_SPINUP_TIMEOUT_S),
                ()->now[0],reason->{throw new AssertionError(reason);});
        volley.start();
        long began=now[0];
        qualifySpeed(port,now,()->{ mechanisms.update(); volley.done(); });
        check((now[0]-began)/1e9<BioBuzzAutoConfig.SHOOT_MIN_SPINUP_SECONDS+BioBuzzAutoConfig.GATE_OPEN_WAIT_SECONDS+0.1,
                "A warm, aimed flywheel feeds right after the settle and gate-open waits");
        before=shooterTrace.size();
        now[0]+=Math.round(BioBuzzAutoConfig.SHOOT_BURST_SECONDS*1e9); mechanisms.update(); volley.done();
        check(!port.robot.gateOpen && map.motors.get("intake").power==0 && map.motors.get("transmission").power==0
                        && !shooterTrace.subList(before,shooterTrace.size()).contains("setPower:0.0") && port.robot.shooterOn,
                "Ending the burst closes the gate and stops feeding while the flywheel stays ready");
        now[0]+=210_000_000L; mechanisms.update();
        check(volley.done(),"Ready-mode volley completes normally");
        volley.end(com.pedropathing.ivy.behaviors.EndCondition.NATURALLY);
        mechanisms.stop();
        check(map.motors.get("shooter").power==0 && map.motors.get("turret").power==0,"STOP shuts the ready flywheel and turret down");
    }
}
