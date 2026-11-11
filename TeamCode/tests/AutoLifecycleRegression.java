package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;
import java.lang.reflect.Proxy;

/** Runs the Driver Station OpMode lifecycle with simulated drive and localization only. */
public final class AutoLifecycleRegression {
    static int checks;
    static void check(boolean ok,String message) { checks++; if (!ok) throw new AssertionError(message); }
    static Object field(BioBuzzAuto auto,String name) throws Exception {
        Field f=BioBuzzAuto.class.getDeclaredField(name); f.setAccessible(true); return f.get(auto);
    }
    static final class TestAuto extends BioBuzzAuto {
        BioBuzzAutoRegression.Tracking tracking;
        BioBuzzAutoRegression.Drive drive;
        BioBuzzAutoRegression.Navigation navigation;
        long now = 1_000_000_000L;
        @Override protected long nanoTime() { return now; }
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
        // No HardwareMap or gamepad is supplied. The only available ports are drive and localization.
        Command stale=Command.build(); Scheduler.schedule(stale);
        auto.init(); auto.init_loop();
        check(!Scheduler.isScheduled(stale),"INIT clears stale scheduler state");
        check(field(auto,"routine")==null && auto.tracking.followed==0 && auto.drive.writes==0,
                "INIT does not drive or schedule the route");
        check((boolean)field(auto,"localizationReady"),"Drive-only INIT is ready without mechanism hardware");

        double measuredReach=BioBuzzAutoConfig.INTAKE_TIP_FORWARD_IN;
        try {
            BioBuzzAutoConfig.INTAKE_TIP_FORWARD_IN=Double.NaN;
            auto.start(); auto.loop();
            check((boolean)field(auto,"safeStopRequested") && field(auto,"routine")==null,
                    "Missing FLOWER offset prevents START without scheduling movement");
        } finally { BioBuzzAutoConfig.INTAKE_TIP_FORWARD_IN=measuredReach; }

        auto.init();
        check(!(boolean)field(auto,"started") && !(boolean)field(auto,"safeStopRequested"),
                "Repeated INIT resets start and stop state");
        auto.init_loop(); auto.start(); auto.start(); auto.loop();
        BioBuzzAutoRoutine interrupted=(BioBuzzAutoRoutine)field(auto,"routine");
        check(auto.tracking.followed==1 && interrupted.pathCount==5,
                "START immediately begins the fixed route once, without a volley or mode selection");
        auto.stop(); auto.loop();
        check(!Scheduler.isScheduled(interrupted.command()) && auto.tracking.idle() && auto.drive.stops>0,
                "FTC Stop cancels and zeros the drivetrain during a movement");

        try {
            auto.init(); auto.init_loop(); auto.start(); auto.loop();
            runFullRoute(auto);
        } finally { auto.stop(); Scheduler.reset(); }
        check(telemetryUpdates[0]>0,"Real FTC telemetry is published");
        System.out.println("Passed " + checks + " iterative OpMode lifecycle checks.");
    }

    private static void runFullRoute(TestAuto auto) throws Exception {
        BioBuzzAutoRoutine routine = (BioBuzzAutoRoutine) field(auto, "routine");
        BioBuzzAutoPaths paths = new BioBuzzAutoPaths(BioBuzzAutoConfig.INTAKE_TIP_FORWARD_IN,
                BioBuzzAutoConfig.INTAKE_TIP_LEFT_IN, true);
        PoseFactory poses = PoseFactory.degrees();
        double turnY = BioBuzzAutoConfig.PATH1_TURN_Y_IN;
        arrive(auto, routine, poses.of(paths.startPose.x(), turnY, Math.toDegrees(paths.startPose.heading())));
        check(routine.stage() == BioBuzzAutoRoutine.Stage.TURNING, "Inward move precedes the path-1 alignment turn");
        arrive(auto, routine, poses.of(paths.start(1).x(), turnY, Math.toDegrees(paths.start(1).heading())));
        arrive(auto, routine, paths.start(1));
        check(auto.tracking.followed == 3 && routine.segment().equals("Path 1"),
                "Straight path 1 follows the return to its original entry");
        arrive(auto, routine, paths.end(1));
        check(routine.segment().equals("Path 2"), "Path 2 begins after verified path-1 arrival");
        arrive(auto, routine, paths.end(2));
        check(routine.stage() == BioBuzzAutoRoutine.Stage.TURNING && routine.segment().equals("Turn before path 3"),
                "Path 2 is followed by the in-place turn");
        arrive(auto, routine, paths.start(3));
        arrive(auto, routine, paths.end(3));
        check(routine.segment().equals("Path 4"), "Path 4 starts after path 3");
        arrive(auto, routine, paths.end(4));
        check(auto.tracking.followed == 7 && routine.segment().equals("Path 5")
                && BioBuzzAutoConfig.atPose(auto.tracking.currentPath().get(0), paths.end(4)),
                "Path 5 starts at the corrected FLOWER pose once path 4 settles");
        arrive(auto, routine, paths.end(5));
        check(routine.stage() == BioBuzzAutoRoutine.Stage.COMPLETE
                && !Scheduler.isScheduled(routine.command()) && auto.tracking.idle() && auto.drive.stops > 0,
                "All five paths complete and the OpMode cancels and zeros the drivetrain");
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

    private static void advance(TestAuto auto, double seconds) {
        auto.now += (long) (seconds * 1e9);
        auto.loop();
    }
}
