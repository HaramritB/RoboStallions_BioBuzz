package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;

/** Real iterative OpMode callbacks with a recording telemetry interface and unapproved config. */
public final class AutoLifecycleRegression {
    static int checks;
    static void check(boolean ok,String message) { checks++; if (!ok) throw new AssertionError(message); }
    static Object field(BioBuzzAuto auto,String name) throws Exception {
        Field f=BioBuzzAuto.class.getDeclaredField(name); f.setAccessible(true); return f.get(auto);
    }
    static void field(BioBuzzAuto auto,String name,Object value) throws Exception {
        Field f=BioBuzzAuto.class.getDeclaredField(name); f.setAccessible(true); f.set(auto,value);
    }
    static void edges(Gamepad gamepad) throws Exception {
        // The real event loop updates edge monitors when a gamepad packet arrives.
        java.lang.reflect.Method method=Gamepad.class.getDeclaredMethod("updateEdgeDetection");
        method.setAccessible(true); method.invoke(gamepad);
    }
    static final class TestAuto extends BioBuzzAuto {
        BioBuzzAutoRegression.Tracking tracking;
        BioBuzzAutoRegression.Drive drive;
        final BioBuzzAutoRegression.Mech mechanismPort=new BioBuzzAutoRegression.Mech();
        @Override protected BioBuzzAutoMechanisms createMechanisms() {
            mechanismPort.ball=true;
            return new BioBuzzAutoMechanisms(mechanismPort);
        }
        @Override protected com.pedropathing.follower.Follower createFollower() {
            drive=new BioBuzzAutoRegression.Drive();
            tracking=new BioBuzzAutoRegression.Tracking(new BioBuzzAutoRegression.Navigation(),drive);
            return tracking;
        }
    }
    public static void main(String[] args) throws Exception {
        TestAuto auto=new TestAuto();
        int[] telemetryUpdates={0};
        auto.telemetry=(Telemetry)Proxy.newProxyInstance(Telemetry.class.getClassLoader(),new Class<?>[]{Telemetry.class},
                (proxy,method,parameters)-> {
                    if (method.getName().equals("update")) { telemetryUpdates[0]++; return true; }
                    if (method.getReturnType()==boolean.class) return false;
                    if (method.getReturnType()==int.class) return 0;
                    return null;
                });
        auto.gamepad1=new Gamepad();
        Command stale=Command.build(); Scheduler.schedule(stale);
        auto.init();
        check(!Scheduler.isScheduled(stale),"INIT clears stale scheduler state");
        check(field(auto,"follower")!=null && field(auto,"routine")==null && auto.tracking.followed==0
                && auto.drive.writes==0,"INIT initializes tested components without driving");
        check((int)field(auto,"selectedPaths")==5 && (boolean)field(auto,"fullRoute"),"Full five-path scoring routine is the default");
        auto.gamepad1.y=true; edges(auto.gamepad1); auto.init_loop();
        check((int)field(auto,"selectedPaths")==1 && !(boolean)field(auto,"fullRoute"),"Y explicitly selects a movement-only test");
        auto.gamepad1.dpad_up=true; edges(auto.gamepad1); auto.init_loop();
        check((int)field(auto,"selectedPaths")==2 && !(boolean)field(auto,"fullRoute"),"INIT can select controlled first N test");
        auto.gamepad1.y=false; edges(auto.gamepad1); auto.gamepad1.y=true; edges(auto.gamepad1); auto.init_loop();
        check((int)field(auto,"selectedPaths")==5 && (boolean)field(auto,"fullRoute"),"Full operation is explicitly distinct from test mode");
        auto.start(); auto.start(); auto.loop();
        check((boolean)field(auto,"safeStopRequested") && field(auto,"routine")==null,"START waits for required FLOWER tip offset, without scheduling movement");
        auto.stop();
        BioBuzzAutoRegression.Mech hardware=new BioBuzzAutoRegression.Mech();
        field(auto,"mechanisms",new BioBuzzAutoMechanisms(hardware));
        field(auto,"mechanismsInitialized",true);
        auto.init();
        check(hardware.closes>0,"Repeated INIT cleans up even a prepared, unstarted mechanism subsystem");
        check(!(boolean)field(auto,"started") && !(boolean)field(auto,"safeStopRequested")
                && (int)field(auto,"selectedPaths")==5 && (boolean)field(auto,"fullRoute"),"Repeated INIT resets start, stop and route selection");
        auto.gamepad1.y=false; edges(auto.gamepad1); auto.gamepad1.y=true; edges(auto.gamepad1); auto.init_loop();
        auto.start(); auto.start();
        check(auto.tracking.followed==1,"Controlled movement can START once with tested components; no blanket geometry approval flags");
        check(telemetryUpdates[0]>0,"Real FTC telemetry is published");
        auto.stop(); Scheduler.reset();
        double oldReach=BioBuzzAutoConfig.INTAKE_TIP_FORWARD_IN;
        try {
            BioBuzzAutoConfig.INTAKE_TIP_FORWARD_IN=12; // Synthetic test offset, not a robot measurement.
            auto.init(); auto.init_loop(); auto.start(); auto.start(); auto.loop();
            check(field(auto,"routine")!=null && !((BioBuzzAutoRoutine)field(auto,"routine")).failed()
                    && (boolean)field(auto,"fullRoute"),"Full scoring START proceeds when measured offset is supplied");
            check(auto.tracking.followed==0 && auto.mechanismPort.request==BioBuzzAutoMechanisms.Kind.SHOOT,
                    "Default full mode shoots preloads before any path movement");
            auto.stop();
        } finally { BioBuzzAutoConfig.INTAKE_TIP_FORWARD_IN=oldReach; Scheduler.reset(); }
        System.out.println("Passed " + checks + " iterative OpMode lifecycle checks.");
    }
}
