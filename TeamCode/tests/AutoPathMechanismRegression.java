package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
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
        boolean ball=true, valid=true, failClose;
        int shots, controls, closes;
        BioBuzzAutoMechanisms.Kind output=BioBuzzAutoMechanisms.Kind.NONE;
        @Override public BioBuzzAutoMechanisms.Observation observe() {
            return new BioBuzzAutoMechanisms.Observation(ball,shots,0,valid);
        }
        @Override public void control(BioBuzzAutoMechanisms.Kind request) { controls++; output=request; }
        @Override public void stopOutputs() { output=BioBuzzAutoMechanisms.Kind.NONE; }
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
        Fixture() {
            Scheduler.reset(); mechanisms.observe();
            route=AutoPath.route(follower,()->now,mechanisms,flower);
            check(follower.pathsStarted==0 && hardware.controls==0,"Construction cannot activate drive or mechanisms");
            check(lever.commands.isEmpty(),"Mapping flower and building the route cannot move the servo in INIT");
        }
        void start() { flower.setDeployed(false); Scheduler.schedule(route); tick(); }
        void tick() { mechanisms.update(); Scheduler.execute(); }
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
            check(hardware.output==BioBuzzAutoMechanisms.Kind.NONE,"Fourth shot stops feeding before another control update");
            advance(.21); drain();
        }
        void toPath2() {
            start();
            check(follower.pathsStarted==0 && follower.idle(),"Preloads shoot without moving the chassis");
            volley(); tick();
            check(follower.pathsStarted==1 && hardware.output==BioBuzzAutoMechanisms.Kind.COLLECT,
                    "Intake accompanies path 1: paths="+follower.pathsStarted+" output="+hardware.output+" request="+mechanisms.state());
            arrive();
            check(hardware.output==BioBuzzAutoMechanisms.Kind.NONE && follower.idle(),"Path-1 arrival stops intake and drive");
            advance(.999);
            check(follower.pathsStarted==1,"Retain the tested one-second path-1 pause");
            advance(.001);
            check(follower.pathsStarted==2,"Path 2 starts after pause");
            check(!lever.commands.contains(FlowerLever.DEPLOYED_POSITION),"Preloads and paths 1/2 must not deploy the flower lever");
        }
        void close(boolean interrupted) {
            try { AutoPath.stopAll(follower,mechanisms,route,flower); }
            catch(IllegalStateException expected) { if(!interrupted) throw expected; }
            check(follower.idle() && drive.stops>0 && hardware.closes>0
                    && hardware.output==BioBuzzAutoMechanisms.Kind.NONE,"Cleanup stops all outputs despite interrupted commands");
            check(!Scheduler.isScheduled(route),"Cleanup clears route scheduling");
            if(!lever.failRetract && !lever.commands.isEmpty()) check(lever.position==FlowerLever.RETRACTED_POSITION,
                    "Normal completion, STOP and failures retract the flower lever");
        }
    }
    public static void main(String[] args) {
        fullRoute(false);
        fullRoute(true);
        delayedVolley();
        missingShots();
        sensorFailure();
        stopDuringIntake();
        cleanupFailure();
        commandFailure();
        stopAtFlower(false);
        stopAtFlower(true);
        System.out.println("Passed "+checks+" AutoPath shooting/intake integration checks.");
    }
    static void fullRoute(boolean stalled) {
        double originalSpeed=Constants.foresightConfig.maxPathSpeed.get();
        Fixture f=new Fixture();
        try {
            f.toPath2();
            f.arrive(); f.volley();
            check(f.follower.pathsStarted==2 && f.follower.idle(),"An early volley cannot shorten the four-second path-2 stop");
            f.advance(3.789);
            check(f.follower.pathsStarted==2,"Path-2 stop lasts the full four seconds");
            f.advance(.001);
            f.drain();
            check(f.follower.pathsStarted==3,"Path 3 begins after both pause and volley finish");
            f.arrive(); f.tick();
            check(f.follower.pathsStarted==4 && f.hardware.output==BioBuzzAutoMechanisms.Kind.COLLECT,"Intake runs during FLOWER approach");
            check(f.lever.position==FlowerLever.DEPLOYED_POSITION && f.lever.pwm,"Deploy the servo at TeleOp's ON position during path 4");
            f.lever.pwm=false; f.tick();
            check(f.lever.pwm && f.lever.position==FlowerLever.DEPLOYED_POSITION,"Restore PWM while holding the deployed flower lever");
            if(stalled) {
                f.nav.setPose(PoseFactory.degrees().of(46.88,131.0,90)); f.tick();
                f.advance(3.999);
                check(f.follower.pathsStarted==4,"Preserve the four-second FLOWER progress window");
                f.advance(.001);
            } else f.arrive();
            check(f.follower.idle() && f.follower.pathsStarted==4,"Stop pushing during the FLOWER pickup window");
            f.advance(BioBuzzAutoConfig.FLOWER_INTAKE_SECONDS-.001);
            check(f.follower.pathsStarted==4 && f.hardware.output==BioBuzzAutoMechanisms.Kind.COLLECT,"FLOWER intake runs its configured duration");
            check(f.lever.position==FlowerLever.DEPLOYED_POSITION,"Keep the lever deployed throughout the FLOWER pickup window");
            f.advance(.001);
            f.drain();
            check(f.follower.pathsStarted==5 && f.hardware.output==BioBuzzAutoMechanisms.Kind.NONE,"Path 5 follows timed pickup, with intake stopped");
            check(f.lever.position==FlowerLever.RETRACTED_POSITION,"Command retraction before starting path 5");
            if(stalled) check(f.follower.currentPath().get(0).distance(PoseFactory.degrees().of(46.88,131.0,90))<1e-8,
                    "Stall recovery still rebases path 5 to the actual departure point");
            check(Constants.foresightConfig.maxPathSpeed.get()==.70,"Keep the tested path speed");
            f.arrive();
            check(f.follower.idle() && Scheduler.isScheduled(f.route),"Path-5 arrival stops the drive and waits for final shooting");
            f.volley();
            check(f.hardware.shots==12 && !Scheduler.isScheduled(f.route),"All three four-shot volleys finish the scoring route");
        } finally { f.close(false); }
        check(Constants.foresightConfig.maxPathSpeed.get()==originalSpeed,"Path speed scopes restore after scoring");
    }
    static void delayedVolley() {
        Fixture f=new Fixture();
        try {
            f.toPath2(); f.arrive(); f.advance(4.1);
            check(f.follower.pathsStarted==2,"Pause completion alone cannot bypass unfinished shooting");
            f.volley();
            check(f.follower.pathsStarted==3,"A late successful volley permits path 3");
        } finally { f.close(true); }
    }
    static void missingShots() {
        Fixture f=new Fixture();
        try {
            f.start(); f.hardware.shots=4; f.tick(); f.advance(.21);
            check(f.follower.pathsStarted==0,"Counter alone cannot bypass missing ball-clear events");
            try { f.advance(BioBuzzAutoConfig.SHOOT_TIMEOUT_S); throw new AssertionError("Missing shot verification must time out"); }
            catch(IllegalStateException expected) { check(f.follower.pathsStarted==0,"Failed preloads cannot start path 1"); }
        } finally { f.close(true); }
    }
    static void sensorFailure() {
        Fixture f=new Fixture();
        try {
            f.toPath2(); f.arrive();
            int outputs=f.hardware.controls; f.hardware.valid=false;
            try { f.tick(); throw new AssertionError("Sensor failure must abort"); }
            catch(IllegalStateException expected) {
                check(f.follower.pathsStarted==2 && f.hardware.controls==outputs,"Invalid fresh sensors block output and the next path");
            }
        } finally { f.close(true); }
    }
    static void stopDuringIntake() {
        Fixture f=new Fixture();
        f.start(); f.volley(); f.tick();
        check(f.hardware.output==BioBuzzAutoMechanisms.Kind.COLLECT,"Test interrupts active path-owned intake");
        f.close(true);
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
            f.start(); f.now+=(long)((BioBuzzAutoConfig.SHOOT_TIMEOUT_S+.1)*1e9);
            try { Scheduler.execute(); throw new AssertionError("Shot command must abort at its deadline"); }
            catch(IllegalStateException expected) {
                check(f.follower.pathsStarted==0,"Failure inside Ivy cannot advance into path 1");
            }
        } finally { f.close(true); }
    }
    static void stopAtFlower(boolean servoFailure) {
        Fixture f=new Fixture();
        f.toPath2(); f.arrive(); f.volley(); f.advance(4); f.drain(); f.arrive(); f.drain();
        check(f.lever.position==FlowerLever.DEPLOYED_POSITION,"Test stops during an active flower deployment");
        f.lever.failRetract=servoFailure;
        f.close(true);
        check(f.hardware.output==BioBuzzAutoMechanisms.Kind.NONE && f.drive.stops>0,
                "Failed flower retraction cannot prevent stopping intake and drivetrain");
    }
}
