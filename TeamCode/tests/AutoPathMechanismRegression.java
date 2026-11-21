package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import org.firstinspires.ftc.teamcode.autonomous.BioBuzzAutoMechanisms;
import org.firstinspires.ftc.teamcode.autonomous.BioBuzzAutoConfig;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PwmControl;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.List;

/** Actual AutoPath composition + actual mechanism commands, with recording sensors/outputs. */
public final class AutoPathMechanismRegression {
    static int checks;
    static void check(boolean ok,String message) { checks++; if(!ok) throw new AssertionError(message); }
    static final class Lever {
        double position=Double.NaN;
        boolean pwm, failRetract;
        final List<Double> commands=new ArrayList<>();
        final Servo servo=(Servo)Proxy.newProxyInstance(Servo.class.getClassLoader(),
                new Class<?>[]{Servo.class,PwmControl.class},(p,m,a)-> {
                    switch(m.getName()) {
                        case "setPosition":
                            if(failRetract && (double)a[0]==FlowerLever.RETRACTED_POSITION) throw new IllegalStateException("simulated flower retraction failure");
                            position=(double)a[0]; commands.add(position); return null;
                        case "setPwmEnable": pwm=true; return null;
                        case "isPwmEnabled": return pwm;
                        default: throw new AssertionError("Unexpected flower servo method "+m.getName());
                    }
                });
    }
    static final class Hardware implements BioBuzzAutoMechanisms.Hardware {
        boolean ball=true, valid=true, failClose, feedReady=true, ready, intakeAlways;
        int shots, controls, shootControls, closes, fullStops;
        BioBuzzAutoMechanisms.Kind output=BioBuzzAutoMechanisms.Kind.NONE;
        @Override public BioBuzzAutoMechanisms.Observation observe() {
            return new BioBuzzAutoMechanisms.Observation(ball,shots,0,valid);
        }
        @Override public void control(BioBuzzAutoMechanisms.Kind request) {
            controls++;
            if(request==BioBuzzAutoMechanisms.Kind.SHOOT) shootControls++;
            output=request==BioBuzzAutoMechanisms.Kind.SHOOT && !feedReady ? BioBuzzAutoMechanisms.Kind.NONE : request;
        }
        @Override public boolean shootingBurstActive() { return output==BioBuzzAutoMechanisms.Kind.SHOOT; }
        @Override public void setShooterIdle(boolean spinning) { ready=spinning; }
        @Override public void setIntakeAlways(boolean running) { intakeAlways=running; }
        boolean continuous; double continuousRpm=Double.NaN;
        @Override public void setContinuousFeed(boolean on, double rpm) { continuous=on; continuousRpm=on ? rpm : Double.NaN; }
        boolean intakeOn() { return intakeAlways || continuous || output!=BioBuzzAutoMechanisms.Kind.NONE; }
        /** Gate open + intake + transmission feeding: a volley's window, or continuous feeding after it. */
        boolean feeding() { return output==BioBuzzAutoMechanisms.Kind.SHOOT || continuous; }
        @Override public void stopFeed() { output=BioBuzzAutoMechanisms.Kind.NONE; }
        @Override public void stopOutputs() {
            fullStops++; ready=false; intakeAlways=false; continuous=false; continuousRpm=Double.NaN;
            output=BioBuzzAutoMechanisms.Kind.NONE;
        }
        @Override public void close() {
            closes++; stopOutputs();
            if(failClose) throw new IllegalStateException("simulated mechanism close failure");
        }
    }
    static final class Fixture {
        final AutoPathRegression.Navigation nav=new AutoPathRegression.Navigation();
        final AutoPathRegression.Drive drive=new AutoPathRegression.Drive();
        final AutoPathRegression.Tracking follower=new AutoPathRegression.Tracking(nav,drive);
        final Hardware hardware=new Hardware();
        final BioBuzzAutoMechanisms mechanisms=new BioBuzzAutoMechanisms(hardware);
        final Lever lever=new Lever();
        final FlowerLever flower=new FlowerLever(lever.servo);
        final Command route;
        long now=1_000_000_000L;
        Fixture() { this(true); }
        Fixture(boolean matchPolicy) {
            Scheduler.reset();
            mechanisms.setFaultTolerant(matchPolicy); // AutoPath INIT policy.
            mechanisms.observe();
            flower.setDeployed(false); // Mirrors AutoPath INIT: flower OFF as soon as INIT is pressed.
            check(lever.commands.equals(java.util.Collections.singletonList(FlowerLever.RETRACTED_POSITION)) && lever.pwm,
                    "INIT commands the flower OFF (0) with PWM enabled");
            route=AutoPath.route(follower,()->now,mechanisms,flower);
            check(follower.pathsStarted==0 && hardware.controls==0,"Construction cannot activate drive or mechanisms");
            check(lever.commands.size()==1,"Building the route does not move the flower again");
        }
        // Mirrors AutoPath START: flywheel ready + turret tracking, retract flower, schedule route.
        long startedAt;
        void start() {
            startedAt=now;
            mechanisms.keepShooterReady(true); mechanisms.keepIntakeRunning(true);
            flower.setDeployed(false); Scheduler.schedule(route); tick();
            check(hardware.intakeAlways && hardware.intakeOn(),"START turns the intake on for the whole route");
        }
        void tick() { mechanisms.update(); Scheduler.execute(); flower.refresh(); } // Same order as the AutoPath loop.
        // Nested Ivy groups settle child completion and parent advancement on separate loops.
        void drain() { for(int i=0;i<3;i++) tick(); }
        void advance(double seconds) { now+=(long)(seconds*1e9); tick(); }
        void arrive() { nav.setPose(follower.currentPath().endPose()); follower.atEnd=true; tick(); }
        void volley() {
            tick();
            check(hardware.output==BioBuzzAutoMechanisms.Kind.SHOOT,"Volley runs the real shooting action");
            for(int i=0;i<4;i++) {
                hardware.ball=true; tick();
                hardware.ball=false; hardware.shots++; tick();
            }
            check(hardware.output==BioBuzzAutoMechanisms.Kind.SHOOT,"Sensor count alone must not prematurely end timed continuous feeding");
            advance(BioBuzzAutoConfig.SHOOT_BURST_SECONDS);
            check(hardware.output==BioBuzzAutoMechanisms.Kind.NONE,"End the feed window before another actuator write");
            advance(.21); drain();
        }
        void toSecondShotApproach() {
            start();
            check(follower.pathsStarted==0 && follower.idle(),"Preloads shoot without moving the chassis");
            check(hardware.ready,"START keeps the flywheel spinning and turret tracking for the whole route");
            volley(); tick();
            check(follower.pathsStarted==1 && hardware.output==BioBuzzAutoMechanisms.Kind.COLLECT,
                    "Intake accompanies path 1: paths="+follower.pathsStarted+" output="+hardware.output+" request="+mechanisms.state());
            checkSpeed(SPEED,"Path 1 (to the wall) runs at route speed");
            check(hardware.ready && hardware.fullStops==0,"Volleys and intake never shut the ready flywheel down");
            arrive(); drain();
            check(follower.pathsStarted==2 && hardware.output==BioBuzzAutoMechanisms.Kind.COLLECT,
                    "Wall intake continues through path 2");
            checkSpeed(BioBuzzAutoConfig.PICKUP_SPEED_FRACTION,
                    "Path 2 into the wall balls runs at pickup speed so balls are intaken, not batted away");
            arrive(); drain();
            check(follower.idle() && follower.pathsStarted==2 && hardware.output==BioBuzzAutoMechanisms.Kind.COLLECT,
                    "No wall shuffle: path 2's end stops the drive and keeps intaking during the pause");
            advance(AutoPath.WALL_PAUSE_NS/1e9-.001);
            check(follower.pathsStarted==2 && hardware.output==BioBuzzAutoMechanisms.Kind.COLLECT,
                    "Keep the (0.7 s) wall pause with intake");
            advance(.001); drain();
            for (int path=3; path<=5; path++) {
                check(follower.pathsStarted==path && hardware.intakeOn(),
                        "The intake keeps running during transit on path "+path);
                checkSpeed(SPEED,"Path "+path+" runs at route speed");
                if (path<5) { arrive(); drain(); }
            }
            check(!hardware.continuous,"No continuous feeding before volley 2");
            check(!lever.commands.contains(FlowerLever.DEPLOYED_POSITION) && lever.position==FlowerLever.RETRACTED_POSITION,
                    "The flower stays OFF for the preloads, the wall pickup and the drive to the 2nd shooting spot");
        }
        void close(boolean interrupted) {
            try { AutoPath.stopAll(follower,mechanisms,route,flower); }
            catch(IllegalStateException expected) { if(!interrupted) throw expected; }
            check(follower.idle() && drive.stops>0 && hardware.closes>0
                    && hardware.output==BioBuzzAutoMechanisms.Kind.NONE,"Cleanup stops all outputs despite interrupted commands");
            check(!Scheduler.isScheduled(route),"Cleanup clears route scheduling");
            if(!lever.failRetract && !lever.commands.isEmpty()) check(lever.position==FlowerLever.RETRACTED_POSITION,
                    "End of the OpMode, STOP and failures retract the flower lever to OFF");
        }
    }
    static final double SPEED=AutoPath.SPEED_FRACTION;
    static void checkSpeed(double fraction,String message) {
        boolean slow=fraction!=SPEED;
        check(Constants.foresightConfig.maxPathSpeed.get()==fraction
                && (!slow || Constants.foresightConfig.maxVelocityConstraint.get()==BioBuzzAutoConfig.PICKUP_MAX_VELOCITY_IN_S),message);
    }
    public static void main(String[] args) {
        fullRoute(false);
        wallStall();
        fullRoute(true);
        delayedVolley();
        missingShotCounts();
        slowSpinup();
        sensorFailure();
        stopDuringIntake();
        cleanupFailure();
        commandFailure();
        stopAtFlower(false);
        stopAtFlower(true);
        lateParks();
        cleanShotCancellation();
        System.out.println("Passed "+checks+" AutoPath shooting/intake integration checks.");
    }
    static void fullRoute(boolean stalled) {
        double originalSpeed=Constants.foresightConfig.maxPathSpeed.get();
        Fixture f=new Fixture();
        try {
            f.toSecondShotApproach();
            f.arrive(); f.drain();
            check(f.lever.position==FlowerLever.RETRACTED_POSITION && !f.lever.commands.contains(FlowerLever.DEPLOYED_POSITION),
                    "The flower stays OFF on arrival at the 2nd shooting spot");
            f.tick();
            check(f.hardware.output==BioBuzzAutoMechanisms.Kind.SHOOT && f.lever.position==FlowerLever.RETRACTED_POSITION,
                    "The flower stays OFF while volley 2 is shooting");
            check(f.hardware.continuous && Double.isNaN(f.hardware.continuousRpm),
                    "Volley 2 arms continuous feeding at its own RPM, so the gate never closes after it");
            f.volley(); // Feed window ends 2.21 s after arrival here.
            check(f.follower.pathsStarted==5 && f.follower.idle(),"Volley 2 stays still after an early finish");
            check(f.hardware.feeding(),"After volley 2's window the gate stays open and both feed motors keep running");
            f.advance(AutoPath.VOLLEY2_MIN_STOP_NS/1e9-2.21-.002);
            check(f.follower.pathsStarted==5,"Volley 2 stops at least four seconds");
            f.advance(.003); f.drain();
            check(f.follower.pathsStarted==6,"Path 6 begins once volley 2's four-second stop ends");
            check(f.hardware.feeding() && Double.isNaN(f.hardware.continuousRpm),
                    "Path 6 keeps shooting at volley 2's RPM (no drop yet)");
            check(f.lever.position==FlowerLever.RETRACTED_POSITION && !f.lever.commands.contains(FlowerLever.DEPLOYED_POSITION),
                    "The flower stays OFF through path 6");
            checkSpeed(SPEED,"Path 6 (strafe under the FLOWER) runs at route speed");
            f.arrive(); f.drain();
            check(f.follower.pathsStarted==7 && f.hardware.feeding(),"Still shooting into the FLOWER approach");
            check(f.hardware.continuousRpm==AutoPath.FLOWER_RPM && AutoPath.FLOWER_RPM==3180,
                    "From the start of path 7 the flywheel runs at a fixed 3180 RPM, not the distance table");
            checkSpeed(BioBuzzAutoConfig.PICKUP_SPEED_FRACTION,"Path 7 FLOWER intake runs at pickup speed");
            check(f.lever.position==FlowerLever.DEPLOYED_POSITION && f.lever.pwm,"The flower deploys ON right before path 7");
            f.lever.pwm=false; f.tick();
            check(f.lever.pwm && f.lever.position==FlowerLever.DEPLOYED_POSITION,"Restore PWM while holding the deployed flower lever");
            if(stalled) {
                f.nav.setPose(PoseFactory.degrees().of(42.40,131.0,90)); f.tick();
                f.advance(AutoPath.FLOWER_STALL_NS/1e9-.001);
                check(f.follower.pathsStarted==7 && !f.follower.idle(),"Keep pushing for the (1 s) FLOWER progress window");
                f.advance(.001);
            } else f.arrive();
            f.drain();
            check(f.follower.pathsStarted==8 && !f.follower.idle()
                            && Constants.foresightConfig.maxPathSpeed.get()==BioBuzzAutoConfig.PICKUP_SPEED_FRACTION,
                    "At the FLOWER the slight shuffle runs at pickup speed");
            check(f.hardware.feeding() && f.hardware.continuousRpm==3180,"Still shooting at 3180 RPM during the FLOWER shuffle");
            f.arrive(); f.drain();
            check(f.follower.idle() && f.follower.pathsStarted==8,"After the shuffle the drive stops at the FLOWER");
            long leaveAt=f.startedAt+Math.round(AutoPath.PARK_LEAVE_BY_S*1e9);
            check(f.now<leaveAt-1_000_000_000L,"Test reaches the FLOWER well before the park time");
            f.now=leaveAt-1_000_000L; f.tick(); f.drain();
            check(f.follower.idle() && f.follower.pathsStarted==8 && Scheduler.isScheduled(f.route),
                    "The robot stays at the FLOWER until the park time");
            check(f.hardware.feeding() && f.hardware.continuousRpm==3180 && f.hardware.fullStops==0,
                    "Volley 3: the shooting sequence stays on at 3180 RPM at the FLOWER");
            check(f.lever.position==FlowerLever.DEPLOYED_POSITION,"The flower stays ON at the FLOWER (cleanup retracts it)");
            Pose atFlower=f.nav.state().pose();
            f.now=leaveAt; f.tick(); f.drain(); f.drain();
            check(f.follower.pathsStarted==9 && f.follower.currentPath().get(0).distance(atFlower)<1e-8
                            && f.follower.currentPath().endPose().distance(PoseFactory.degrees().of(15.7092,122.0158,90))<1e-6,
                    "At PARK_LEAVE_BY_S the robot parks with path 8 from where it is");
            check(Constants.foresightConfig.maxPathSpeed.get()==AutoPath.SPEED_FRACTION,"The park runs at route speed");
            check(f.hardware.feeding(),"Still shooting while parking");
            f.arrive(); f.drain();
            check(f.follower.idle() && Scheduler.isScheduled(f.route),"Parked: the robot holds still until the auto ends");
        } finally { f.close(false); }
        check(Constants.foresightConfig.maxPathSpeed.get()==originalSpeed,"Path speed scopes restore after scoring");
    }
    static void wallStall() {
        Fixture f=new Fixture();
        try {
            f.start(); f.volley(); f.tick(); f.arrive(); f.drain();
            check(f.follower.pathsStarted==2,"Test reaches the wall approach");
            // Balls jam the intake 12 in short of POINT_2.
            f.nav.setPose(PoseFactory.degrees().of(8.1905,12.0319,-90.9351)); f.tick();
            int stops=f.drive.stops;
            f.advance(AutoPath.WALL_STALL_NS/1e9-.001);
            check(f.follower.pathsStarted==2 && !f.follower.idle(),"Keep pushing while the wall stall window runs");
            f.advance(.001); f.drain();
            check(f.follower.pathsStarted==2 && f.follower.idle() && f.drive.stops>stops
                            && f.hardware.output==BioBuzzAutoMechanisms.Kind.COLLECT,
                    "A blocked wall approach stops pushing and starts the intaking wall pause instead of hanging");
            f.advance(AutoPath.WALL_PAUSE_NS/1e9); f.drain();
            check(f.follower.pathsStarted==3,"The route continues to the second shot after a wall stall");
        } finally { f.close(false); }
    }
    /** Still driving into the FLOWER at PARK_LEAVE_BY_S: park right away from where the robot is. */
    static void lateParks() {
        Fixture f=new Fixture();
        try {
            f.toSecondShotApproach(); f.arrive(); f.drain();
            f.volley(); f.advance(2); f.drain(); f.drain(); // Volley 2's four-second stop.
            f.arrive(); f.drain();
            check(f.follower.pathsStarted==7 && !f.follower.idle(),"Still driving into the FLOWER on path 7");
            long leaveAt=f.startedAt+Math.round(AutoPath.PARK_LEAVE_BY_S*1e9);
            f.nav.setPose(PoseFactory.degrees().of(42.10,120.0,90)); f.now=leaveAt-10_000_000L; f.tick();
            f.nav.setPose(PoseFactory.degrees().of(42.10,121.0,90)); f.now=leaveAt-1_000_000L; f.tick(); f.drain();
            check(f.follower.pathsStarted==7,"Before the park time, path 7 keeps going");
            f.nav.setPose(PoseFactory.degrees().of(42.10,122.0,90)); f.now=leaveAt; f.tick(); f.drain(); f.drain();
            check(f.follower.pathsStarted==8 && f.follower.currentPath().get(0).distance(PoseFactory.degrees().of(42.10,122.0,90))<1e-8
                            && f.follower.currentPath().endPose().distance(PoseFactory.degrees().of(15.7092,122.0158,90))<1e-6,
                    "At the park time the robot parks from wherever it is, even mid path 7");
        } finally { f.close(false); }
    }
    static void delayedVolley() {
        Fixture f=new Fixture();
        try {
            f.toSecondShotApproach(); f.hardware.feedReady=false; f.arrive(); f.drain(); f.advance(4.1);
            check(f.follower.pathsStarted==5,"Pause completion alone cannot bypass unfinished shooting");
            f.hardware.feedReady=true;
            f.volley();
            check(f.follower.pathsStarted==5,"Volley 2 stays still for 0.5 s after a late feed window");
            f.advance(AutoPath.VOLLEY2_HOLD_AFTER_NS/1e9); f.drain(); f.drain();
            check(f.follower.pathsStarted==6,"A late successful volley permits path 6 after the hold");
        } finally { f.close(false); }
    }
    static void missingShotCounts() {
        Fixture f=new Fixture();
        try {
            f.start(); f.advance(AutoPath.SHORT_FEED_S-.001);
            check(f.follower.pathsStarted==0 && f.hardware.output==BioBuzzAutoMechanisms.Kind.SHOOT,
                    "Give a full feed window even with no distinct sensor exits");
            int shootControls=f.hardware.shootControls; f.advance(.001);
            check(f.hardware.output==BioBuzzAutoMechanisms.Kind.NONE && f.hardware.shootControls==shootControls,
                    "A stuck/uncounted ball sensor cannot cause endless feeding");
            f.advance(.21); f.drain();
            check(f.follower.pathsStarted==1 && f.hardware.shots==0,
                    "Completed preload feed starts path 1 without requiring or fabricating four sensor counts");
        } finally { f.close(false); }
    }
    static void slowSpinup() {
        Fixture f=new Fixture();
        try {
            f.hardware.feedReady=false; f.start();
            f.advance(BioBuzzAutoConfig.SHOOT_SPINUP_TIMEOUT_S-.1);
            check(f.follower.pathsStarted==0 && f.hardware.output==BioBuzzAutoMechanisms.Kind.NONE,
                    "Slow spin-up must not open the gate or start the path");
            f.hardware.feedReady=true;
            f.volley();
            check(f.follower.pathsStarted==1,"Spin-up time does not consume the subsequent complete feed window");
        } finally { f.close(false); }
    }
    static void sensorFailure() {
        Fixture f=new Fixture();
        try {
            f.toSecondShotApproach(); f.arrive(); f.drain();
            f.hardware.valid=false;
            f.tick(); // Previously threw and ended the whole autonomous.
            check(f.mechanisms.failure().isEmpty() && f.mechanisms.warning().contains("invalid feeder sensor"),
                    "A bad feeder-sensor sample is reported, not fatal");
            f.volley();
            f.advance(4-BioBuzzAutoConfig.SHOOT_BURST_SECONDS-.21+.001); f.drain(); f.drain();
            check(f.follower.pathsStarted==6,"The route keeps shooting and driving through sensor glitches");
        } finally { f.close(false); }
        Fixture strict=new Fixture(false);
        try {
            strict.toSecondShotApproach(); strict.arrive(); strict.drain();
            int outputs=strict.hardware.controls; strict.hardware.valid=false;
            try { strict.tick(); throw new AssertionError("Strict sensor failure must abort"); }
            catch(IllegalStateException expected) {
                check(strict.follower.pathsStarted==5 && strict.hardware.controls==outputs,
                        "Diagnostic (strict) mode still blocks output on invalid sensors");
            }
        } finally { strict.close(false); }
    }
    static void stopDuringIntake() {
        Fixture f=new Fixture();
        f.start(); f.volley(); f.tick();
        check(f.hardware.output==BioBuzzAutoMechanisms.Kind.COLLECT,"Test interrupts active path-owned intake");
        f.close(false);
    }
    static void cleanupFailure() {
        Fixture f=new Fixture();
        f.hardware.failClose=true;
        f.close(true); // INIT/stop with an unscheduled route, plus a failing mechanism device.
        check(f.drive.stops>0,"Mechanism cleanup failure cannot skip drivetrain stop");
    }
    static void commandFailure() {
        Fixture f=new Fixture();
        try {
            f.hardware.feedReady=false; f.start();
            f.advance(BioBuzzAutoConfig.SHOOT_SPINUP_TIMEOUT_S-.1);
            check(f.follower.pathsStarted==0,"Keep waiting for the volley until its backstop");
            f.advance(.2);
            check(f.hardware.output==BioBuzzAutoMechanisms.Kind.NONE && f.mechanisms.warning().contains("Skipped volley"),
                    "A volley that can never feed is skipped and reported at the backstop");
            f.advance(.21); f.drain();
            check(f.follower.pathsStarted==1 && f.hardware.output==BioBuzzAutoMechanisms.Kind.COLLECT
                            && f.mechanisms.failure().isEmpty(),
                    "The route continues to the wall pickup instead of ending the autonomous");
        } finally { f.close(false); }
        Fixture strict=new Fixture(false);
        try {
            strict.hardware.feedReady=false; strict.start(); strict.now+=(long)((BioBuzzAutoConfig.SHOOT_SPINUP_TIMEOUT_S+.1)*1e9);
            try { Scheduler.execute(); throw new AssertionError("Strict shot command must abort at its deadline"); }
            catch(IllegalStateException expected) {
                check(strict.follower.pathsStarted==0,"Strict failure inside Ivy cannot advance into path 1");
            }
        } finally { strict.close(false); }
    }
    static void stopAtFlower(boolean servoFailure) {
        Fixture f=new Fixture();
        f.toSecondShotApproach(); f.arrive(); f.drain(); f.volley(); f.advance(4); f.drain(); f.drain(); f.arrive(); f.drain();
        check(f.lever.position==FlowerLever.DEPLOYED_POSITION,"Test stops during an active flower deployment");
        f.lever.failRetract=servoFailure;
        f.close(servoFailure);
        check(f.hardware.output==BioBuzzAutoMechanisms.Kind.NONE && f.drive.stops>0,
                "Failed flower retraction cannot prevent stopping intake and drivetrain");
    }
    static void cleanShotCancellation() {
        Fixture f=new Fixture();
        f.start();
        check(f.hardware.output==BioBuzzAutoMechanisms.Kind.SHOOT,"Test cancels an active shooting command");
        f.close(false); // Normal STOP must not throw 'Mechanism command interrupted: SHOOT'.
        check(f.mechanisms.failure().isEmpty(),"Cleanup must not invent a shooting failure or mask the original error");
    }
}
