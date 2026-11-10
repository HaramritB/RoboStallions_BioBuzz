package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.Collections;
import java.util.Map;

/** Runs actual Ivy sequencing and Pedro paths against recording navigation/actuator ports. */
public final class BioBuzzAutoRegression {
    static int checks;
    static final PoseFactory P=PoseFactory.degrees();
    static void check(boolean ok,String message) { checks++; if (!ok) throw new AssertionError(message); }
    static void near(Pose a,Pose b,String message) {
        check(BioBuzzAutoConfig.distance(a,b)<1e-8 && Math.abs(BioBuzzAutoConfig.headingErrorDegrees(a,b))<1e-7,message);
    }
    static final class Navigation implements Localizer {
        Pose pose=new BioBuzzAutoPaths().startPose;
        Velocity velocity=Velocity.zero();
        @Override public void setPose(Pose p) { pose=p; }
        @Override public MotionState state() { return MotionState.ofVelocity(pose,velocity); }
        @Override public void update() { }
        @Override public void reset() { }
    }
    static final class Drive implements Drivetrain {
        int writes,stops;
        @Override public void drive(DrivePowers p,boolean manual) { writes++; }
        @Override public void stop() { stops++; }
        @Override public void stop(boolean brake) { stop(); }
        @Override public double maxScaling(DrivePowers a,DrivePowers b) { return 1; }
        @Override public double interpolateVelocity(double x,double y,double t) { return 1; }
        @Override public Map<String,Object> debug() { return Collections.emptyMap(); }
    }
    static final class Tracking extends Follower {
        int followed;
        Pose held;
        Tracking(Navigation navigation,Drive drive) { super(navigation,drive,new Foresight(Constants.foresightConfig)); }
        @Override public void follow(Path p) { followed++; super.follow(p); }
        @Override public void hold(Pose p) { held=p; super.hold(p); }
        @Override public void update() { } // Physical motion is explicitly simulated by the fixture.
    }
    static final class Mech implements BioBuzzAutoMechanisms.Hardware {
        boolean ball;
        int shots,controls,stops,closes,collected;
        boolean countedCollection;
        double angle;
        boolean sensorsReady=true;
        BioBuzzAutoMechanisms.Kind request=BioBuzzAutoMechanisms.Kind.NONE;
        @Override public BioBuzzAutoMechanisms.Observation observe() {
            return new BioBuzzAutoMechanisms.Observation(ball,shots,angle,sensorsReady,countedCollection ? collected : -1);
        }
        @Override public boolean supportsCountedCollection() { return countedCollection; }
        @Override public void control(BioBuzzAutoMechanisms.Kind kind) { request=kind; controls++; }
        @Override public void stopOutputs() { request=BioBuzzAutoMechanisms.Kind.NONE; stops++; }
        @Override public void close() { closes++; stopOutputs(); }
    }
    static BioBuzzAutoMechanisms.Action[] none() {
        BioBuzzAutoMechanisms.Action[] a=new BioBuzzAutoMechanisms.Action[BioBuzzAutoPaths.PATH_COUNT];
        for (int i=0;i<a.length;i++) a[i]=BioBuzzAutoMechanisms.Action.none();
        return a;
    }
    static final class Fixture {
        final BioBuzzAutoPaths paths=new BioBuzzAutoPaths();
        final Navigation nav=new Navigation();
        final Drive drive=new Drive();
        final Tracking follower=new Tracking(nav,drive);
        final Mech hardware=new Mech();
        final BioBuzzAutoMechanisms mechanisms=new BioBuzzAutoMechanisms(hardware);
        final long[] now={1_000_000_000L};
        final BioBuzzAutoRoutine routine;
        Fixture(int count) { this(count,none()); }
        Fixture(int count,BioBuzzAutoMechanisms.Action[] actions) {
            Scheduler.reset(); mechanisms.observe();
            routine=new BioBuzzAutoRoutine(follower,paths,count,mechanisms,actions,()->now[0]);
        }
        Fixture(BioBuzzAutoConfig.Plan plan) {
            Scheduler.reset(); hardware.countedCollection=true; hardware.ball=true; mechanisms.observe();
            routine=new BioBuzzAutoRoutine(follower,paths,5,mechanisms,plan,()->now[0]);
        }
        void start() { routine.start(); }
        void tick() { routine.update(); }
        void advance(double seconds) { now[0]+=(long)(seconds*1e9); tick(); }
        void arrive(Pose target) {
            nav.pose=target; follower.hold(target); tick(); advance(.21);
        }
        void drain() { for (int i=0;i<8;i++) tick(); }
    }
    public static void main(String[] args) {
        check(BioBuzzAuto.class.isAnnotationPresent(Disabled.class),"Physical Auto remains disabled");
        check(!BioBuzzAutoConfig.blockers(1,false).isEmpty(),"Review gates remain closed");
        check(BioBuzzAutoConfig.blockers(5,true).contains("FLOWER"),"Flag unresolved full route conflict");
        check(BioBuzzAutoConfig.hasMechanismActions(),"Full route includes user-confirmed mechanism mapping");
        check(BioBuzzAutoConfig.blockers(5,true).contains("Four-ball inventory"),"Full route blocks unverified four-ball inventory");
        check(BioBuzzAutoConfig.PRELOAD_BALL_COUNT==4 && BioBuzzAutoConfig.STAGED_BALL_COUNT==4
                && BioBuzzAutoConfig.FLOWER_BALL_COUNT==4,"User confirmed four balls in each of three volleys");
        for (BioBuzzAutoMechanisms.Action action:BioBuzzAutoConfig.actions(false)) {
            check(action.kind==BioBuzzAutoMechanisms.Kind.NONE,"First-N tests never activate scoring");
        }
        BioBuzzAutoPaths p=new BioBuzzAutoPaths();
        double[][] ends={{9.5657,9.2715,179.7546},{32.2934,127.6559,-84.7123},
                {46.9373,125.3042,90},{46.7883,140.2305,90.572},{38.0943,127.3583,-66.9637}};
        near(p.startPose,P.of(57.6323,9.0657,90),"Exact physical start definition");
        for (int i=1;i<=5;i++) {
            near(p.end(i),P.of(ends[i-1][0],ends[i-1][1],ends[i-1][2]),"Preserve input endpoint " + i);
            near(p.path(i).get(0),p.start(i),"Path start heading " + i);
            near(p.path(i).get(1),p.end(i),"Path end heading " + i);
            check(p.path(i)!=p.path(i),"Fresh reusable path " + i);
            if (i>1) check(BioBuzzAutoConfig.distance(p.start(i),p.end(i-1))==0,"Exact adjacent XY endpoints " + i);
            check(p.turnBefore(i)==(i==3),"Only path 3 needs an explicit turn");
        }
        near(p.path(1).get(.5),P.of((57.6323+9.5657)/2,(9.0657+9.2715)/2,(90+179.7546)/2),"Linear heading interpolation");
        // A known quadratic midpoint also verifies the original control point survived generation.
        Pose midpoint=p.path(2).curve.get(.5).toPose(0);
        check(Math.abs(midpoint.x()-(9.5657+2*40.923+32.2934)/4)<1e-8
                && Math.abs(midpoint.y()-(9.2715+2*34.4143+127.6559)/4)<1e-8,"Preserve Bezier control geometry");
        check(Math.abs(BioBuzzAutoConfig.headingErrorDegrees(P.of(0,0,1),P.of(0,0,359))-2)<1e-7,"Wrapped heading error");
        double velocityBefore=Constants.foresightConfig.maxVelocityConstraint.get();
        boolean skipBefore=Constants.foresightConfig.pathSkip.get();
        Fixture f=new Fixture(5);
        check(f.follower.followed==0 && f.hardware.controls==0,"INIT/construction must not drive mechanisms or paths");
        f.start(); f.start();
        check(f.follower.followed==1,"START schedules exactly once");
        check(Constants.foresightConfig.maxVelocityConstraint.get()==12,"Scoped slow path policy applied");
        f.drain(); check(f.follower.followed==1,"Path 2 cannot precede arrival");
        f.nav.pose=f.paths.end(1).withHeading(0); f.follower.hold(f.nav.pose); f.tick();
        check(f.follower.followed==1,"Correct XY with wrong heading is insufficient");
        f.nav.pose=f.paths.end(1); f.nav.velocity=new Velocity(5,0,0); f.tick();
        check(f.follower.followed==1,"Arrival must settle velocity");
        f.nav.velocity=Velocity.zero(); f.tick(); f.advance(.1);
        check(f.follower.followed==1,"Require sustained settling, not one sample");
        f.advance(.11);
        check(f.follower.followed==2,"Path 2 starts after verified path 1");
        f.arrive(f.paths.end(2));
        check(f.routine.stage()==BioBuzzAutoRoutine.Stage.TURNING && f.follower.followed==2,"Turn is a separate action");
        near(f.follower.held,f.paths.start(3),"Turn to 0 at the same XY");
        f.advance(.21); check(f.follower.followed==2,"Turn cannot finish without measured orientation");
        f.arrive(f.paths.start(3)); check(f.follower.followed==3,"Path 3 starts only after turn settles");
        f.arrive(f.paths.end(3)); f.arrive(f.paths.end(4)); f.arrive(f.paths.end(5)); f.drain();
        check(f.routine.stage()==BioBuzzAutoRoutine.Stage.COMPLETE && f.follower.followed==5,"Complete all five paths");
        check(f.follower.idle() && f.drive.stops>0 && f.hardware.closes>0,"Success stops drivetrain and all activated mechanisms");
        f.routine.stop(); Scheduler.reset();
        check(Constants.foresightConfig.maxVelocityConstraint.get()==velocityBefore
                && Constants.foresightConfig.pathSkip.get()==skipBefore,"Restore temporary constraints");
        Fixture first=new Fixture(1); first.start(); first.arrive(first.paths.end(1)); first.drain();
        check(first.routine.stage()==BioBuzzAutoRoutine.Stage.COMPLETE && first.follower.followed==1,"Controlled first-path test");
        Fixture timeout=new Fixture(5); timeout.start(); timeout.advance(16);
        check(timeout.routine.failed() && timeout.routine.failure().contains("timeout") && timeout.follower.followed==1,"Segment timeout blocks later movement");
        Fixture overall=new Fixture(5); overall.start(); overall.advance(28);
        check(overall.routine.failed() && overall.routine.failure().contains("Overall"),"Overall deadline before controller updates");
        Fixture turn=new Fixture(3); turn.start(); turn.arrive(turn.paths.end(1)); turn.arrive(turn.paths.end(2)); turn.advance(6);
        check(turn.routine.failed() && turn.follower.followed==2,"Failed turn blocks path 3");
        Fixture interrupted=new Fixture(5); interrupted.start(); interrupted.follower.stop(); interrupted.tick();
        check(interrupted.routine.failed() && interrupted.follower.followed==1,"Tracking interruption fails closed");
        Fixture replaced=new Fixture(5); replaced.start(); replaced.follower.follow(p.path(5)); replaced.tick();
        check(replaced.routine.failed(),"Unexpected path replacement fails");
        Fixture cancelled=new Fixture(5); cancelled.start(); Scheduler.cancel(cancelled.routine.command());
        check(cancelled.routine.failed() && cancelled.drive.stops>0 && cancelled.hardware.closes>0,"Ivy cancellation stops all outputs");
        Fixture stopped=new Fixture(5); stopped.start(); stopped.routine.stop(); stopped.tick(); stopped.start();
        check(stopped.routine.stage()==BioBuzzAutoRoutine.Stage.STOPPED && stopped.follower.followed==1,"Stop prevents re-scheduling");
        Fixture reset=new Fixture(5); reset.start(); Scheduler.reset(); reset.tick();
        check(reset.routine.failed(),"Unexpected scheduler reset is a failure");
        Fixture again=new Fixture(1); again.start(); check(again.follower.followed==1 && again.routine.failure().isEmpty(),"Fresh INIT has no stale scheduler/failure state"); again.routine.stop();
        Fixture badStart=new Fixture(5); badStart.nav.pose=P.of(0,0,0); badStart.start();
        check(badStart.routine.failed() && badStart.follower.followed==0,"Invalid start cannot move");
        checkFreshSampleGuard();
        mechanismCommands();
        scoringSequence();
        Scheduler.reset();
        System.out.println("Passed " + checks + " integrated autonomous regression checks.");
    }
    static void checkFreshSampleGuard() {
        Navigation nav=new Navigation(); Drive drive=new Drive(); boolean[] ready={false};
        Follower follower=new Follower(new BioBuzzAuto.CheckedLocalizer(nav,()->ready[0]),drive,new Foresight(Constants.foresightConfig));
        follower.follow(new BioBuzzAutoPaths().path(1));
        assertBlocked(follower,drive); ready[0]=true; nav.pose=P.of(Double.NaN,0,0); assertBlocked(follower,drive);
        nav.pose=new BioBuzzAutoPaths().startPose; nav.velocity=new Velocity(0,Double.POSITIVE_INFINITY,0); assertBlocked(follower,drive);
    }
    static void assertBlocked(Follower follower,Drive drive) {
        int writes=drive.writes;
        try { follower.update(); throw new AssertionError("Bad sample must fail"); }
        catch (IllegalStateException expected) { check(drive.writes==writes,"Reject fresh invalid sample BEFORE output"); }
    }
    static void mechanismCommands() {
        BioBuzzAutoMechanisms.Action[] actions=none(); actions[0]=BioBuzzAutoMechanisms.Action.shoot(1,5);
        Fixture shot=new Fixture(2,actions); shot.start(); shot.arrive(shot.paths.end(1));
        check(shot.routine.stage()==BioBuzzAutoRoutine.Stage.MECHANISM && shot.follower.followed==1,"Shoot waits for path arrival");
        shot.tick(); check(shot.hardware.request==BioBuzzAutoMechanisms.Kind.SHOOT,"Existing mechanism request runs nonblocking");
        shot.hardware.shots=1; shot.tick(); shot.advance(.21);
        check(shot.follower.followed==1,"Timer-derived shot counter alone cannot prove shooting");
        shot.hardware.ball=true; shot.tick(); shot.hardware.ball=false; shot.tick(); shot.advance(.21);
        check(shot.follower.followed==2 && shot.hardware.request==BioBuzzAutoMechanisms.Kind.NONE,"Counter plus sensor-clear event permits next path and stops feeding");
        shot.routine.stop();
        Fixture failShot=new Fixture(2,actions); failShot.start(); failShot.arrive(failShot.paths.end(1)); failShot.advance(6);
        check(failShot.routine.failed() && failShot.follower.followed==1 && failShot.hardware.closes>0,"Failed shot blocks later drive and cleans up");
        Fixture cancelShot=new Fixture(2,actions); cancelShot.start(); cancelShot.arrive(cancelShot.paths.end(1)); Scheduler.cancel(cancelShot.routine.command());
        check(cancelShot.routine.failed() && cancelShot.hardware.closes>0,"Interrupted mechanism cleans up");
        actions=none(); actions[0]=BioBuzzAutoMechanisms.Action.collectUntilBall(4);
        Fixture collect=new Fixture(2,actions); collect.start(); collect.arrive(collect.paths.end(1)); collect.tick(); collect.advance(.5);
        check(collect.follower.followed==1,"Collection must not succeed just because time passed");
        collect.hardware.ball=true; collect.tick(); collect.advance(.21);
        check(collect.follower.followed==2 && collect.hardware.stops>0,"Collection uses sustained ball sensor completion"); collect.routine.stop();
        Fixture occupied=new Fixture(2,actions); occupied.hardware.ball=true; occupied.mechanisms.observe();
        occupied.start(); occupied.arrive(occupied.paths.end(1));
        check(occupied.routine.failed() && occupied.hardware.controls==0,"Existing feeder occupancy cannot count as a new pickup");
        Fixture drift=new Fixture(2,actions); drift.start(); drift.arrive(drift.paths.end(1));
        int before=drift.hardware.controls; drift.nav.pose=P.of(0,0,0); drift.tick();
        check(drift.routine.failed() && drift.hardware.controls==before && drift.hardware.closes>0,"Lost waypoint blocks mechanism output before control");
        Fixture sensor=new Fixture(2,actions); sensor.start(); sensor.arrive(sensor.paths.end(1));
        before=sensor.hardware.controls; sensor.hardware.sensorsReady=false; sensor.tick();
        check(sensor.routine.failed() && sensor.hardware.controls==before,"Bad fresh mechanism sensor blocks output");
        actions=none(); actions[0]=BioBuzzAutoMechanisms.Action.centerTurret(4);
        Fixture center=new Fixture(2,actions); center.hardware.angle=20; center.start(); center.arrive(center.paths.end(1)); center.tick();
        check(center.follower.followed==1,"Turret centering requires actual encoder orientation");
        center.hardware.angle=0; center.tick(); center.advance(.21); check(center.follower.followed==2,"Centering success advances"); center.routine.stop();
    }

    static void volley(Fixture f) {
        for (int i=0;i<4;i++) {
            f.hardware.ball=true; f.tick();
            f.hardware.ball=false; f.hardware.shots++; f.tick();
            if (i<3) check(f.hardware.request==BioBuzzAutoMechanisms.Kind.SHOOT,"Volley cannot finish before four verified shots");
        }
        check(f.hardware.request==BioBuzzAutoMechanisms.Kind.NONE,"Stop feeding immediately at four shots");
        f.advance(.21);
    }

    static void scoringSequence() {
        BioBuzzAutoConfig.Plan plan=BioBuzzAutoConfig.scoringPlan();
        check(plan.beforeFirstPath.kind==BioBuzzAutoMechanisms.Kind.SHOOT && plan.beforeFirstPath.count==4,
                "Four preloads before path 1");
        check(plan.duringPath[0].kind==BioBuzzAutoMechanisms.Kind.COLLECT && plan.duringPath[0].count==4
                && plan.afterPath[0].kind==BioBuzzAutoMechanisms.Kind.NONE,"Collect during path 1, not after it");
        Fixture run=new Fixture(plan); run.start(); run.tick();
        check(run.follower.followed==0 && run.follower.holding() && run.hardware.request==BioBuzzAutoMechanisms.Kind.SHOOT,
                "Shoot preloads while holding start, before any path movement");
        volley(run); run.tick();
        check(run.follower.followed==1 && run.hardware.request==BioBuzzAutoMechanisms.Kind.COLLECT
                && run.routine.stage()==BioBuzzAutoRoutine.Stage.DRIVING_AND_COLLECTING,"Intake alongside path 1 after all preloads clear");
        run.hardware.collected=3; run.arrive(run.paths.end(1));
        check(run.follower.followed==1,"Path 2 cannot start after only three pickups, even after arrival");
        run.hardware.collected=4; run.tick(); run.advance(.21);
        check(run.follower.followed==2 && run.hardware.request==BioBuzzAutoMechanisms.Kind.NONE,"Verified four pickups AND arrival permit path 2");
        run.arrive(run.paths.end(2)); run.tick();
        check(run.hardware.request==BioBuzzAutoMechanisms.Kind.SHOOT && run.follower.followed==2,"Second volley follows path 2");
        volley(run);
        check(run.routine.stage()==BioBuzzAutoRoutine.Stage.TURNING,"Turn waits for all four path-2 shots");
        run.arrive(run.paths.start(3)); run.arrive(run.paths.end(3));
        check(run.follower.followed==4 && run.hardware.request==BioBuzzAutoMechanisms.Kind.NONE,"No invented action at path 3");
        run.arrive(run.paths.end(4)); run.tick();
        check(run.hardware.request==BioBuzzAutoMechanisms.Kind.COLLECT,"Intake-only FLOWER pickup after path 4");
        run.hardware.collected=7; run.tick(); run.advance(.21);
        check(run.follower.followed==4,"Three new FLOWER balls do not satisfy four-ball pickup");
        run.hardware.collected=8; run.tick(); run.advance(.21);
        check(run.follower.followed==5,"Four new FLOWER balls permit path 5");
        run.arrive(run.paths.end(5)); run.tick();
        check(run.hardware.request==BioBuzzAutoMechanisms.Kind.SHOOT,"Final volley follows path 5");
        volley(run); run.drain();
        check(run.routine.stage()==BioBuzzAutoRoutine.Stage.COMPLETE && run.hardware.shots==12
                && run.hardware.request==BioBuzzAutoMechanisms.Kind.NONE && run.hardware.closes>0 && run.drive.stops>0,
                "Twelve verified shot events finish with all outputs stopped");
        run.routine.stop();

        Fixture early=new Fixture(plan); early.start(); volley(early);
        early.hardware.collected=4; early.tick(); early.advance(.21);
        check(early.follower.followed==1,"Four pickups alone cannot substitute for path-1 arrival");
        early.routine.stop();
        check(early.hardware.closes>0 && early.drive.stops>0,"Stop during concurrent path/intake cleans both");

        Fixture timeout=new Fixture(plan); timeout.start(); volley(timeout); timeout.advance(10.1);
        check(timeout.routine.failed() && timeout.follower.followed==1,"Collection timeout during drive blocks path 2");
        Fixture reset=new Fixture(plan); reset.start(); volley(reset); reset.hardware.collected=3; reset.tick();
        reset.hardware.collected=0; reset.tick();
        check(reset.routine.failed() && reset.follower.followed==1,"Pickup counter reset aborts concurrent collection");
        Fixture cancelled=new Fixture(plan); cancelled.start(); volley(cancelled); Scheduler.cancel(cancelled.routine.command());
        check(cancelled.routine.failed() && cancelled.hardware.closes>0 && cancelled.drive.stops>0,
                "Cancelling concurrent pickup stops mechanisms and drivetrain");
        BioBuzzAutoMechanisms.Action[] actions=none(); actions[0]=BioBuzzAutoMechanisms.Action.collect(4,4);
        Fixture unsupported=new Fixture(2,actions); unsupported.start(); unsupported.arrive(unsupported.paths.end(1));
        check(unsupported.routine.failed() && unsupported.hardware.controls==0,"Presence-only hardware cannot execute a four-ball pickup");

        Fixture preloads=new Fixture(plan); preloads.start(); preloads.advance(7.1);
        check(preloads.routine.failed() && preloads.follower.followed==0,"Failed preload volley prevents path 1");
        Fixture unverified=new Fixture(plan); unverified.start(); unverified.tick();
        unverified.hardware.shots=4; unverified.tick(); int writes=unverified.hardware.controls; unverified.advance(.5);
        check(unverified.follower.followed==0 && unverified.hardware.controls==writes
                && unverified.hardware.request==BioBuzzAutoMechanisms.Kind.NONE,
                "Four timed counts alone cannot finish or keep feeding while missing clear events");
        unverified.routine.stop();
    }
}
