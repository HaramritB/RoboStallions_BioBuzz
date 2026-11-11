package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.function.LongSupplier;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

/** A fresh instance owns one bounded run; geometry is generated from the saved JSON. */
public final class BioBuzzAutoRoutine {
    public enum Stage { READY, DRIVING, DRIVING_AND_COLLECTING, TURNING, MECHANISM, COMPLETE, FAILED, STOPPED }
    private final Follower follower;
    private final BioBuzzAutoMechanisms mechanisms;
    private final LongSupplier clock;
    private final Command command;
    private Stage stage=Stage.READY;
    private Pose target;
    private String failure="";
    private String segment="none";
    private long runStart;
    private long stepStart;
    private boolean scheduled;
    public final int pathCount;

    static BioBuzzAutoRoutine driveOnly(Follower follower, BioBuzzAutoPaths paths, LongSupplier clock) {
        return new BioBuzzAutoRoutine(follower, paths, BioBuzzAutoPaths.PATH_COUNT,
                new BioBuzzAutoMechanisms(), BioBuzzAutoConfig.plan(false), clock);
    }

    public BioBuzzAutoRoutine(Follower follower, BioBuzzAutoPaths paths, int pathCount,
                             BioBuzzAutoMechanisms mechanisms, BioBuzzAutoConfig.Plan plan) {
        this(follower,paths,pathCount,mechanisms,plan,System::nanoTime);
    }
    public BioBuzzAutoRoutine(Follower follower, BioBuzzAutoPaths paths, int pathCount,
                             BioBuzzAutoMechanisms mechanisms, BioBuzzAutoMechanisms.Action[] actions) {
        this(follower,paths,pathCount,mechanisms,actions,System::nanoTime);
    }
    BioBuzzAutoRoutine(Follower follower, BioBuzzAutoPaths paths, int pathCount,
                      BioBuzzAutoMechanisms mechanisms, BioBuzzAutoMechanisms.Action[] actions, LongSupplier clock) {
        this(follower,paths,pathCount,mechanisms,new BioBuzzAutoConfig.Plan(
                BioBuzzAutoMechanisms.Action.none(),BioBuzzAutoConfig.actions(false),actions),clock);
    }
    BioBuzzAutoRoutine(Follower follower, BioBuzzAutoPaths paths, int pathCount,
                      BioBuzzAutoMechanisms mechanisms, BioBuzzAutoConfig.Plan plan, LongSupplier clock) {
        if (pathCount<1 || pathCount>BioBuzzAutoPaths.PATH_COUNT) {
            throw new IllegalArgumentException("Invalid path count");
        }
        this.follower=follower; this.mechanisms=mechanisms; this.clock=clock; this.pathCount=pathCount;
        target=paths.startPose;
        List<Command> steps=new ArrayList<>();
        if (plan.beforeFirstPath.kind!=BioBuzzAutoMechanisms.Kind.NONE) {
            steps.add(stationaryAction(plan.beforeFirstPath,paths.startPose,"Shoot preloads before path 1"));
        }
        boolean movementOnly=plan.movementOnly();
        Pose pathOneEntry=paths.start(1);
        if (Math.abs(BioBuzzAutoConfig.headingErrorDegrees(pathOneEntry,paths.startPose))>1e-6) {
            PoseFactory poses=PoseFactory.degrees();
            double turnY=Math.max(paths.startPose.y(), BioBuzzAutoConfig.PATH1_TURN_Y_IN);
            Pose awayFromWall=poses.of(paths.startPose.x(),turnY,Math.toDegrees(paths.startPose.heading()));
            Pose aligned=poses.of(pathOneEntry.x(),turnY,Math.toDegrees(pathOneEntry.heading()));
            boolean needsInwardMove=turnY>paths.startPose.y();
            if (needsInwardMove) {
                steps.add(movement(constrain(line(paths.startPose,awayFromWall).constant(paths.startPose),movementOnly),
                        paths.startPose,awayFromWall,"Move inward before path 1"));
            }
            steps.add(movement(null,awayFromWall,aligned,"Align with straight pickup line"));
            if (needsInwardMove) {
                steps.add(movement(constrain(line(aligned,pathOneEntry).constant(pathOneEntry),movementOnly),
                        aligned,pathOneEntry,"Return to path 1 entry"));
            }
        }
        for (int i=1;i<=pathCount;i++) {
            if (paths.turnBefore(i)) steps.add(movement(null,paths.end(i-1),paths.start(i),"Turn before path " + i));
            Path path=constrain(paths.path(i),movementOnly);
            steps.add(movement(path,paths.start(i),paths.end(i),"Path " + i,plan.duringPath[i-1]));
            if (plan.afterPath[i-1].kind!=BioBuzzAutoMechanisms.Kind.NONE) {
                steps.add(stationaryAction(plan.afterPath[i-1],paths.end(i),"After path " + i));
            }
        }
        steps.add(instant(() -> {
            if (!blocked()) {
                stopOutputs();
                stage=Stage.COMPLETE;
            }
        }));
        command=sequential(steps.toArray(new Command[0]));
    }

    private static Path constrain(Path path, boolean movementOnly) {
        return path.with(
                Constants.foresightConfig.maxVelocityConstraint.at(movementOnly
                        ? BioBuzzAutoConfig.PATH_SPEED_IN_S : Constants.foresightConfig.maxVelocityConstraint.get()),
                Constants.foresightConfig.maxPathSpeed.at(movementOnly
                        ? BioBuzzAutoConfig.PATH_SPEED_FRACTION : Constants.foresightConfig.maxPathSpeed.get()),
                Constants.foresightConfig.pathSkip.at(false));
    }

    /** Schedule at most once; test and full route share the same lifecycle. */
    public void start() {
        if (scheduled || stage!=Stage.READY) return;
        scheduled=true; runStart=clock.getAsLong();
        Scheduler.schedule(command);
        if (failed()) Scheduler.cancel(command);
    }

    /** One active FTC iteration. Time/state checks run before any controller output. */
    public void update() {
        if (!scheduled || finished()) return;
        try {
            if (elapsedSeconds()>=BioBuzzAutoConfig.AUTO_TIMEOUT_S) { fail("Overall autonomous timeout"); return; }
            if (!Scheduler.isScheduled(command)) { fail("Routine cancelled or scheduler reset"); return; }
            follower.update();
            if (stage==Stage.MECHANISM && (!follower.holding()
                    || !BioBuzzAutoConfig.atPose(follower.pose(),target)
                    || !BioBuzzAutoConfig.settled(follower.velocity()))) {
                fail("Lost stationary waypoint during mechanism action: " + segment);
                return;
            }
            mechanisms.update();
            Scheduler.execute();
        } catch (RuntimeException error) {
            fail(error.getClass().getSimpleName() + ": " + error.getMessage());
        } finally {
            // Avoid mutating Ivy's active composition from inside a child callback.
            if (failed()) Scheduler.cancel(command);
        }
    }

    private Command stationaryAction(BioBuzzAutoMechanisms.Action spec, Pose pose, String name) {
        Command action=mechanisms.command(spec,clock,this::fail);
        final boolean[] active={false};
        return Command.build().requiring(follower,mechanisms).setStart(() -> {
            if (blocked()) return;
            if (!BioBuzzAutoConfig.atPose(follower.pose(),pose)
                    || !BioBuzzAutoConfig.settled(follower.velocity())) { fail("Invalid mechanism starting pose: " + name); return; }
            active[0]=true;
            stage=Stage.MECHANISM; segment=name; target=pose; stepStart=clock.getAsLong();
            follower.hold(pose);
            action.start();
        }).setExecute(() -> { if (active[0] && !blocked()) action.execute(); })
                .setDone(() -> !active[0] || blocked() || action.done())
                .setEnd(reason -> { if (active[0]) action.end(reason); });
    }

    private Command movement(Path path, Pose start, Pose end, String name) {
        return movement(path,start,end,name,BioBuzzAutoMechanisms.Action.none());
    }
    private Command movement(Path path, Pose start, Pose end, String name, BioBuzzAutoMechanisms.Action during) {
        final Command collection=during.kind==BioBuzzAutoMechanisms.Kind.NONE ? null : mechanisms.command(during,clock,this::fail);
        final boolean[] collectionStarted={false}, collectionEnded={collection==null};
        final boolean[] active={false};
        final long[] stableSince={-1};
        return (path==null ? Command.build() : follow(follower,path)).requiring(follower,mechanisms)
                .setStart(() -> {
                    if (blocked()) return;
                    if (!BioBuzzAutoConfig.atPose(follower.pose(),start)
                            || !BioBuzzAutoConfig.settled(follower.velocity())) { fail("Invalid start pose/motion: " + name); return; }
                    active[0]=true; stableSince[0]=-1;
                    stage=path==null ? Stage.TURNING : collection==null ? Stage.DRIVING : Stage.DRIVING_AND_COLLECTING;
                    segment=name; target=end; stepStart=clock.getAsLong();
                    if (collection!=null) {
                        collectionStarted[0]=true;
                        collection.start();
                        if (blocked()) return;
                    }
                    if (path==null) follower.hold(end); else follower.follow(path);
                }).setExecute(() -> {
                    if (active[0] && !blocked() && !collectionEnded[0]) collection.execute();
                }).setDone(() -> {
                    if (!active[0] || blocked()) return true;
                    if (!collectionEnded[0] && collection.done()) {
                        collectionEnded[0]=true;
                        collection.end(EndCondition.NATURALLY);
                    }
                    if (blocked()) return true;
                    if (!BioBuzzAutoConfig.finite(follower.pose()) || !BioBuzzAutoConfig.finite(follower.velocity())) {
                        fail("Invalid localization: " + name); return true;
                    }
                    long now=clock.getAsLong();
                    double deadline=path==null ? BioBuzzAutoConfig.TURN_TIMEOUT_S : BioBuzzAutoConfig.SEGMENT_TIMEOUT_S;
                    if ((now-stepStart)/1e9>=deadline) { fail("Waypoint/heading timeout: " + name); return true; }
                    if (path!=null && follower.following()) {
                        stableSince[0]=-1;
                        if (follower.currentPath()!=path) { fail("Path replaced unexpectedly: " + name); return true; }
                        return false;
                    }
                    if (!follower.holding()) { fail("Path interrupted: " + name); return true; }
                    if (!BioBuzzAutoConfig.atPose(follower.pose(),end) || !BioBuzzAutoConfig.settled(follower.velocity())) {
                        stableSince[0]=-1; return false;
                    }
                    if (stableSince[0]<0) stableSince[0]=now;
                    boolean arrived=(now-stableSince[0])/1e9>=BioBuzzAutoConfig.SETTLE_TIME_S;
                    if (arrived && collectionStarted[0] && !collectionEnded[0]) {
                        collectionEnded[0]=true;
                        collection.end(EndCondition.NATURALLY); // Path owns intake lifetime; no inventory prerequisite.
                    }
                    return arrived;
                }).setEnd(reason -> {
                    if (collectionStarted[0] && !collectionEnded[0]) {
                        collectionEnded[0]=true;
                        collection.end(EndCondition.INTERRUPTED);
                    }
                    if (active[0] && reason!=EndCondition.NATURALLY && !blocked()) fail("Ivy interrupted: " + name);
                });
    }
    private boolean blocked() { return stage==Stage.FAILED || stage==Stage.STOPPED || stage==Stage.COMPLETE; }
    public void fail(String reason) {
        if (stage==Stage.STOPPED) return;
        if (failure.isEmpty()) failure=reason;
        stage=Stage.FAILED;
        stopOutputs();
    }
    public void stop() {
        if (stage!=Stage.COMPLETE && stage!=Stage.FAILED) stage=Stage.STOPPED;
        try { Scheduler.cancel(command); }
        finally { stopOutputs(); }
    }
    private void stopOutputs() {
        // stop() only clears Pedro 3.0.2 navigation state; always write motor zero separately.
        try { follower.stop(); }
        finally {
            try { follower.drivetrain.stop(); }
            finally { mechanisms.stop(); }
        }
    }
    public Command command() { return command; }
    public Stage stage() { return stage; }
    public Pose target() { return target; }
    public String segment() { return segment; }
    public String failure() { return failure; }
    public boolean failed() { return stage==Stage.FAILED; }
    public boolean finished() { return stage==Stage.COMPLETE || stage==Stage.FAILED || stage==Stage.STOPPED; }
    public double elapsedSeconds() { return !scheduled ? 0 : (clock.getAsLong()-runStart)/1e9; }
    public double segmentSeconds() { return stage==Stage.READY ? 0 : (clock.getAsLong()-stepStart)/1e9; }
}
