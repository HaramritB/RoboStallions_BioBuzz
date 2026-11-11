package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;

/** Runs the Driver Station OpMode lifecycle with simulated navigation and recorded mechanism commands. */
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
        BioBuzzAutoRegression.Navigation navigation;
        long now = 1_000_000_000L;
        @Override protected long nanoTime() { return now; }
        final BioBuzzAutoRegression.Mech mechanismPort=new BioBuzzAutoRegression.Mech();
        @Override protected BioBuzzAutoMechanisms createMechanisms() {
            mechanismPort.ball=true;
            return new BioBuzzAutoMechanisms(mechanismPort);
        }
        @Override protected com.pedropathing.follower.Follower createFollower() {
            drive=new BioBuzzAutoRegression.Drive();
            navigation=new BioBuzzAutoRegression.Navigation();
            tracking=new BioBuzzAutoRegression.Tracking(navigation,drive);
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
        double measuredReach=BioBuzzAutoConfig.INTAKE_TIP_FORWARD_IN;
        try {
            BioBuzzAutoConfig.INTAKE_TIP_FORWARD_IN=Double.NaN;
            auto.start(); auto.start(); auto.loop();
            check((boolean)field(auto,"safeStopRequested") && field(auto,"routine")==null,
                    "Missing FLOWER offset prevents START without scheduling movement");
        } finally { BioBuzzAutoConfig.INTAKE_TIP_FORWARD_IN=measuredReach; }
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
        try {
            auto.init(); auto.init_loop(); auto.start(); auto.start(); auto.loop();
            check(field(auto,"routine")!=null && !((BioBuzzAutoRoutine)field(auto,"routine")).failed()
                    && (boolean)field(auto,"fullRoute"),"Full scoring START proceeds when measured offset is supplied");
            check(auto.tracking.followed==0 && auto.mechanismPort.request==BioBuzzAutoMechanisms.Kind.SHOOT,
                    "Default full mode shoots preloads before any path movement");
            runFullRoute(auto);
        } finally { auto.stop(); Scheduler.reset(); }
        System.out.println("Passed " + checks + " iterative OpMode lifecycle checks.");
    }

    private static void runFullRoute(TestAuto auto) throws Exception {
        BioBuzzAutoRoutine routine = (BioBuzzAutoRoutine) field(auto, "routine");
        BioBuzzAutoPaths paths = new BioBuzzAutoPaths(BioBuzzAutoConfig.INTAKE_TIP_FORWARD_IN,
                BioBuzzAutoConfig.INTAKE_TIP_LEFT_IN, true);
        PoseFactory poses = PoseFactory.degrees();
        double turnY = BioBuzzAutoConfig.PATH1_TURN_Y_IN;
        int initialShots = auto.mechanismPort.shots;
        shootVolley(auto);
        arrive(auto, routine, poses.of(paths.startPose.x(), turnY, Math.toDegrees(paths.startPose.heading())));
        check(routine.stage() == BioBuzzAutoRoutine.Stage.TURNING, "Inward move precedes the path-1 alignment turn");
        arrive(auto, routine, poses.of(paths.start(1).x(), turnY, Math.toDegrees(paths.start(1).heading())));
        arrive(auto, routine, paths.start(1));
        auto.loop();
        check(auto.mechanismPort.request == BioBuzzAutoMechanisms.Kind.COLLECT
                && auto.tracking.followed == 3, "Intake starts on straight path 1 after both alignment moves");
        arrive(auto, routine, paths.end(1));
        check(auto.mechanismPort.request == BioBuzzAutoMechanisms.Kind.NONE, "Path-1 arrival ends intake");
        arrive(auto, routine, paths.end(2));
        shootVolley(auto);
        check(routine.stage() == BioBuzzAutoRoutine.Stage.TURNING, "Path-2 volley precedes the position-holding turn");
        arrive(auto, routine, paths.start(3));
        arrive(auto, routine, paths.end(3));
        auto.loop();
        check(auto.mechanismPort.request == BioBuzzAutoMechanisms.Kind.COLLECT, "FLOWER intake starts during path 4");
        arrive(auto, routine, paths.end(4));
        advance(auto, BioBuzzAutoConfig.FLOWER_INTAKE_SECONDS - 0.1);
        check(routine.stage() == BioBuzzAutoRoutine.Stage.MECHANISM
                && auto.mechanismPort.request == BioBuzzAutoMechanisms.Kind.COLLECT,
                "FLOWER intake continues during the timed hold");
        advance(auto, 0.11);
        check(auto.tracking.followed == 7 && auto.mechanismPort.request == BioBuzzAutoMechanisms.Kind.NONE
                && BioBuzzAutoConfig.atPose(auto.tracking.currentPath().get(0), paths.end(4)),
                "Path 5 starts from the corrected FLOWER pose after intake stops");
        arrive(auto, routine, paths.end(5));
        shootVolley(auto);
        auto.loop();
        check(routine.stage() == BioBuzzAutoRoutine.Stage.COMPLETE
                && auto.mechanismPort.shots - initialShots == 12, "Full OpMode finishes all three four-shot volleys");
        check(!Scheduler.isScheduled(routine.command()) && auto.tracking.idle()
                && auto.mechanismPort.request == BioBuzzAutoMechanisms.Kind.NONE && auto.drive.stops > 0,
                "Full completion clears the schedule and stops the drivetrain and mechanisms");
    }

    private static void arrive(TestAuto auto, BioBuzzAutoRoutine routine, Pose expected) {
        check(BioBuzzAutoConfig.distance(routine.target(), expected) < 1e-8
                && Math.abs(BioBuzzAutoConfig.headingErrorDegrees(routine.target(), expected)) < 1e-7,
                "OpMode targets the expected pose for " + routine.segment());
        auto.navigation.setPose(expected);
        auto.tracking.hold(expected);
        auto.loop();
        advance(auto, 0.21);
    }

    private static void shootVolley(TestAuto auto) {
        auto.loop();
        check(auto.mechanismPort.request == BioBuzzAutoMechanisms.Kind.SHOOT, "Expected scoring volley is active");
        for (int shot = 1; shot <= 4; shot++) {
            auto.mechanismPort.ball = true;
            auto.loop();
            auto.mechanismPort.ball = false;
            auto.mechanismPort.shots++;
            int writes = auto.mechanismPort.controls;
            auto.loop();
            if (shot == 4) {
                check(auto.mechanismPort.controls == writes
                        && auto.mechanismPort.request == BioBuzzAutoMechanisms.Kind.NONE,
                        "The fourth observed shot stops feeding before another control update");
            }
        }
        advance(auto, 0.21);
    }

    private static void advance(TestAuto auto, double seconds) {
        auto.now += (long) (seconds * 1e9);
        auto.loop();
    }
}
