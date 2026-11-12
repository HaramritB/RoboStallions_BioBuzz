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
        int shots,controls,stops,closes;
        double angle;
        boolean sensorsReady=true;
        BioBuzzAutoMechanisms.Kind request=BioBuzzAutoMechanisms.Kind.NONE;
        @Override public BioBuzzAutoMechanisms.Observation observe() {
            return new BioBuzzAutoMechanisms.Observation(ball,shots,angle,sensorsReady);
        }
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
        final BioBuzzAutoPaths paths;
        final Navigation nav=new Navigation();
        final Drive drive=new Drive();
        final Tracking follower=new Tracking(nav,drive);
        final Mech hardware=new Mech();
        final BioBuzzAutoMechanisms mechanisms=new BioBuzzAutoMechanisms(hardware);
        final long[] now={1_000_000_000L};
        final BioBuzzAutoRoutine routine;
        Fixture(int count) { this(count,none()); }
        Fixture(int count,BioBuzzAutoMechanisms.Action[] actions) {
            paths=new BioBuzzAutoPaths();
            Scheduler.reset(); mechanisms.observe();
            routine=new BioBuzzAutoRoutine(follower,paths,count,mechanisms,actions,()->now[0]);
        }
        Fixture(BioBuzzAutoConfig.Plan plan) { this(plan,new BioBuzzAutoPaths()); }
        Fixture(BioBuzzAutoConfig.Plan plan,BioBuzzAutoPaths paths) {
            this.paths=paths;
            Scheduler.reset(); hardware.ball=true; mechanisms.observe();
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
        check(!BioBuzzAuto.class.isAnnotationPresent(Disabled.class),"Full autonomous is registered and enabled");
        check(BioBuzzAutoConfig.blockers(1,false).isEmpty(),"Tested components do not have stale approval gates");
        check(BioBuzzAutoConfig.blockers(5,true).isEmpty(),"Team-measured intake reach allows full route");
        check(BioBuzzAutoConfig.hasMechanismActions(),"Full route includes user-confirmed mechanism mapping");
        check(!BioBuzzAutoConfig.blockers(5,true).toLowerCase().contains("inventory"),"No stored-ball inventory approval blocks full route");
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
        // Synthetic offsets test geometry only; these are NOT robot measurements.
        BioBuzzAutoPaths shifted=new BioBuzzAutoPaths(12, 1.5);
        Pose center=shifted.end(4), tip=shifted.flowerTipPose;
        double heading=center.heading();
        near(P.of(center.x()+12*Math.cos(heading)-1.5*Math.sin(heading),
                center.y()+12*Math.sin(heading)+1.5*Math.cos(heading),Math.toDegrees(heading)),tip,
                "Measured forward/lateral offset reconstructs exact intake-tip target");
        near(shifted.start(5),shifted.end(4),"Corrected path 4 endpoint exactly matches path 5 start");
        near(shifted.path(4).get(1),center,"Path 4 actually uses corrected robot center");
        near(shifted.path(5).get(0),center,"Path 5 starts at corrected robot center");
        for (int i=1;i<=3;i++) near(shifted.end(i),p.end(i),"Unaffected endpoint "+i);
        near(shifted.end(5),p.end(5),"Park/last scoring target remains unchanged");
        try { new BioBuzzAutoPaths(Double.NaN,0); throw new AssertionError("Unknown offset accepted"); }
        catch (IllegalArgumentException expected) { check(true,"Unknown reach rejected before path construction"); }
        double velocityBefore=Constants.foresightConfig.maxVelocityConstraint.get();
        boolean skipBefore=Constants.foresightConfig.pathSkip.get();
        Fixture f=new Fixture(5);
        check(f.follower.followed==0 && f.hardware.controls==0,"INIT/construction must not drive mechanisms or paths");
        f.start(); f.start();
        check(f.follower.followed==1,"START schedules exactly once");
        check(Constants.foresightConfig.maxVelocityConstraint.get()==18
                && Constants.foresightConfig.maxPathSpeed.get()==0.35,"Path 1 uses the lower wall constraints");
        f.drain(); check(f.follower.followed==1,"Path 2 cannot precede arrival");
        f.nav.pose=f.paths.end(1).withHeading(0); f.follower.hold(f.nav.pose); f.tick();
        check(f.follower.followed==1,"Correct XY with wrong heading is insufficient");
        f.nav.pose=f.paths.end(1); f.nav.velocity=new Velocity(5,0,0); f.tick();
        check(f.follower.followed==1,"Arrival must settle velocity");
        f.nav.velocity=Velocity.zero(); f.tick(); f.advance(.1);
        check(f.follower.followed==1,"Require sustained settling, not one sample");
        f.advance(.11);
        check(f.follower.followed==2,"Path 2 starts after verified path 1");
        check(Constants.foresightConfig.maxVelocityConstraint.get()==36
                && Constants.foresightConfig.maxPathSpeed.get()==0.60,"Path 2 uses the faster transit constraints");
        f.arrive(f.paths.end(2));
        check(f.routine.stage()==BioBuzzAutoRoutine.Stage.TURNING && f.follower.followed==2,"Turn is a separate action");
        near(f.follower.held,f.paths.start(3),"Turn to 0 at the same XY");
        f.advance(.21); check(f.follower.followed==2,"Turn cannot finish without measured orientation");
        f.arrive(f.paths.start(3)); check(f.follower.followed==3,"Path 3 starts only after turn settles");
        check(Constants.foresightConfig.maxVelocityConstraint.get()==36,"Path 3 uses the faster transit constraints");
        f.arrive(f.paths.end(3));
        check(Constants.foresightConfig.maxVelocityConstraint.get()==18
                && Constants.foresightConfig.maxPathSpeed.get()==0.35,"Path 4 slows before FLOWER");
        f.arrive(f.paths.end(4));
        check(Constants.foresightConfig.maxVelocityConstraint.get()==18,"Path 5 keeps the lower departure constraint");
        f.arrive(f.paths.end(5)); f.drain();
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
        straightPickupAlignment();
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
        actions=none(); actions[0]=BioBuzzAutoMechanisms.Action.intakeFor(1,4);
        Fixture collect=new Fixture(2,actions); collect.start(); collect.arrive(collect.paths.end(1)); collect.tick(); collect.advance(.5);
        check(collect.follower.followed==1,"Timed intake continues for the configured window");
        collect.hardware.ball=true; collect.tick(); collect.advance(.21);
        check(collect.follower.followed==1,"Presence does not finish intake early");
        int outputs=collect.hardware.controls; collect.advance(.3);
        check(collect.follower.followed==2 && collect.hardware.controls==outputs && collect.hardware.stops>0,
                "Time window ends without a stored count; no extra output after duration"); collect.routine.stop();
        Fixture occupied=new Fixture(2,actions); occupied.hardware.ball=true; occupied.mechanisms.observe();
        occupied.start(); occupied.arrive(occupied.paths.end(1)); occupied.tick();
        check(!occupied.routine.failed() && occupied.hardware.request==BioBuzzAutoMechanisms.Kind.COLLECT,
                "An occupied feeder does not prevent an open-loop intake attempt"); occupied.routine.stop();
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
        check(plan.duringPath[0].kind==BioBuzzAutoMechanisms.Kind.COLLECT && plan.duringPath[0].count==0
                && plan.duringPath[0].durationSeconds==0 && plan.afterPath[0].kind==BioBuzzAutoMechanisms.Kind.NONE,
                "Path-1 intake has no inventory target or separate after-path pickup");
        double fullVelocity=Constants.foresightConfig.maxVelocityConstraint.get();
        double fullSpeed=Constants.foresightConfig.maxPathSpeed.get();
        Fixture run=new Fixture(plan); run.start(); run.tick();
        check(run.follower.followed==0 && run.follower.holding() && run.hardware.request==BioBuzzAutoMechanisms.Kind.SHOOT,
                "Shoot preloads while holding start, before any path movement");
        volley(run); run.tick();
        check(Constants.foresightConfig.maxVelocityConstraint.get()==fullVelocity
                && Constants.foresightConfig.maxPathSpeed.get()==fullSpeed,"Full routine preserves tested factory speed policy");
        check(run.follower.followed==1 && run.hardware.request==BioBuzzAutoMechanisms.Kind.COLLECT
                && run.routine.stage()==BioBuzzAutoRoutine.Stage.DRIVING_AND_COLLECTING,"Intake alongside path 1 after all preloads clear");
        run.hardware.ball=false; run.advance(.5);
        check(run.follower.followed==1 && run.hardware.request==BioBuzzAutoMechanisms.Kind.COLLECT,
                "Intake remains active until path-1 arrival even without a ball detection");
        run.arrive(run.paths.end(1));
        check(run.follower.followed==2 && run.hardware.request==BioBuzzAutoMechanisms.Kind.NONE,
                "Path-1 arrival ends intake and starts path 2 without waiting for stored-ball feedback");
        run.arrive(run.paths.end(2)); run.tick();
        check(run.hardware.request==BioBuzzAutoMechanisms.Kind.SHOOT && run.follower.followed==2,"Second volley follows path 2");
        volley(run);
        check(run.routine.stage()==BioBuzzAutoRoutine.Stage.TURNING,"Turn waits for all four path-2 shots");
        run.arrive(run.paths.start(3)); run.arrive(run.paths.end(3));
        check(run.follower.followed==4 && run.hardware.request==BioBuzzAutoMechanisms.Kind.NONE,"No invented action at path 3");
        run.tick();
        check(run.hardware.request==BioBuzzAutoMechanisms.Kind.COLLECT
                && run.routine.stage()==BioBuzzAutoRoutine.Stage.DRIVING_AND_COLLECTING,
                "FLOWER rollers run during path 4 approach");
        run.arrive(run.paths.end(4)); run.tick();
        check(run.hardware.request==BioBuzzAutoMechanisms.Kind.COLLECT,"Intake continues while holding the FLOWER waypoint");
        run.hardware.ball=false; run.advance(BioBuzzAutoConfig.FLOWER_INTAKE_SECONDS-.1);
        check(run.follower.followed==4 && run.hardware.request==BioBuzzAutoMechanisms.Kind.COLLECT,"FLOWER intake waits for its duration");
        run.advance(.11);
        check(run.follower.followed==5 && run.hardware.request==BioBuzzAutoMechanisms.Kind.NONE,
                "FLOWER window ends and starts path 5 without claiming four stored balls");
        run.arrive(run.paths.end(5)); run.tick();
        check(run.hardware.request==BioBuzzAutoMechanisms.Kind.SHOOT,"Final volley follows path 5");
        volley(run); run.drain();
        check(run.routine.stage()==BioBuzzAutoRoutine.Stage.COMPLETE && run.hardware.shots==12
                && run.hardware.request==BioBuzzAutoMechanisms.Kind.NONE && run.hardware.closes>0 && run.drive.stops>0,
                "Twelve verified shot events finish with all outputs stopped");
        run.routine.stop();

        Fixture early=new Fixture(plan); early.start(); volley(early);
        early.hardware.ball=true; early.tick(); early.advance(.21);
        check(early.follower.followed==1 && early.hardware.request==BioBuzzAutoMechanisms.Kind.COLLECT,
                "Feeder presence cannot substitute for path arrival or stop travel intake");
        early.routine.stop();
        check(early.hardware.closes>0 && early.drive.stops>0,"Stop during concurrent path/intake cleans both");
        Fixture timeout=new Fixture(plan); timeout.start(); volley(timeout); timeout.advance(16);
        check(timeout.routine.failed() && timeout.follower.followed==1,"Stalled path/intake timeout blocks path 2");
        Fixture cancelled=new Fixture(plan); cancelled.start(); volley(cancelled); Scheduler.cancel(cancelled.routine.command());
        check(cancelled.routine.failed() && cancelled.hardware.closes>0 && cancelled.drive.stops>0,
                "Cancelling concurrent intake stops mechanisms and drivetrain");
        Fixture interrupted=new Fixture(plan); interrupted.start(); volley(interrupted); interrupted.follower.stop(); interrupted.tick();
        check(interrupted.routine.failed() && interrupted.hardware.closes>0,"Path interruption stops concurrent intake");
        Fixture preloads=new Fixture(plan); preloads.start(); preloads.advance(7.1);
        check(preloads.routine.failed() && preloads.follower.followed==0,"Failed preload volley prevents path 1");
        Fixture unverified=new Fixture(plan); unverified.start(); unverified.tick();
        unverified.hardware.shots=4; unverified.tick(); int writes=unverified.hardware.controls; unverified.advance(.5);
        check(unverified.follower.followed==0 && unverified.hardware.controls==writes
                && unverified.hardware.request==BioBuzzAutoMechanisms.Kind.NONE,
                "Four timed counts alone cannot finish or keep feeding while missing clear events");
        unverified.routine.stop();
        try { BioBuzzAutoMechanisms.Action.intakeFor(Double.NaN,4); throw new AssertionError("NaN window"); }
        catch (IllegalArgumentException expected) { check(true,"Reject nonfinite intake duration"); }
        try { BioBuzzAutoMechanisms.Action.intakeFor(5,4); throw new AssertionError("Window exceeds deadline"); }
        catch (IllegalArgumentException expected) { check(true,"Reject intake window beyond timeout"); }
    }
    static void straightPickupAlignment() {
        BioBuzzAutoPaths paths=new BioBuzzAutoPaths(BioBuzzAutoConfig.INTAKE_TIP_FORWARD_IN,0,true);
        Pose entry=paths.start(1), end=paths.end(1), center=paths.end(4);
        near(center,P.of(46.8722690231,131.8198167846,90.572),"Measured FLOWER center");
        near(paths.start(5),center,"Measured path 5 departure matches FLOWER arrival");
        double halfLength=BioBuzzAutoConfig.ROBOT_LENGTH_IN/2;
        double halfWidth=BioBuzzAutoConfig.ROBOT_WIDTH_IN/2;
        for (int i=0;i<=1000;i++) {
            Pose sample=paths.path(1).get(i/1000.0);
            double cross=(sample.x()-entry.x())*(end.y()-entry.y())-(sample.y()-entry.y())*(end.x()-entry.x());
            double extent=Math.abs(Math.sin(sample.heading()))*halfLength+Math.abs(Math.cos(sample.heading()))*halfWidth;
            if (Math.abs(cross)>1e-7 || sample.y()<extent
                    || Math.abs(BioBuzzAutoConfig.headingErrorDegrees(sample,end))>1e-7) {
                throw new AssertionError("Path 1 must stay straight, aligned and inside the lower wall");
            }
        }
        check(true,"Measured rectangular footprint clears lower wall throughout straight path 1");
        check(BioBuzzAutoConfig.PATH1_TURN_Y_IN-Math.hypot(halfLength,halfWidth)>BioBuzzAutoConfig.POSITION_TOLERANCE_IN,
                "Inward turn clears the measured diagonal plus arrival tolerance");
        Fixture run=new Fixture(BioBuzzAutoConfig.scoringPlan(),paths);
        run.start(); volley(run); run.tick();
        check(run.follower.followed==1 && run.hardware.request==BioBuzzAutoMechanisms.Kind.NONE,
                "After shooting, move inward before alignment or intake");
        double turnY=BioBuzzAutoConfig.PATH1_TURN_Y_IN;
        run.arrive(P.of(entry.x(),turnY,Math.toDegrees(paths.startPose.heading())));
        check(run.routine.stage()==BioBuzzAutoRoutine.Stage.TURNING,"Turn only after inward movement settles");
        run.arrive(P.of(entry.x(),turnY,Math.toDegrees(entry.heading())));
        check(run.follower.followed==2 && run.hardware.request==BioBuzzAutoMechanisms.Kind.NONE,
                "Return to entry at aligned heading before intake");
        run.arrive(entry); run.tick();
        check(run.follower.followed==3 && run.hardware.request==BioBuzzAutoMechanisms.Kind.COLLECT,
                "Intake starts with the original straight pickup line");
        run.routine.stop();
    }
}
