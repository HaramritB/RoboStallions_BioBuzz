package org.firstinspires.ftc.teamcode.v2;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FlowerLever;
import org.firstinspires.ftc.teamcode.Robot;

import java.lang.reflect.Proxy;
import org.firstinspires.ftc.teamcode.autonomous.BioBuzzAutoMechanisms;

import java.util.Arrays;
import java.util.Collections;

/** Hardware-free checks for the V2 aiming, targeting, shooter and sequencing logic. */
public final class V2Regression {
    static int checks;

    static void check(boolean ok, String message) {
        checks++;
        if (!ok) throw new AssertionError(message);
    }

    static void near(double actual, double expected, double tolerance, String message) {
        check(Math.abs(actual - expected) <= tolerance, message + " (expected " + expected + ", got " + actual + ")");
    }

    public static void main(String[] args) {
        hiveTargeting();
        fieldAim();
        quietAim();
        onePersonControls();
        flowerCommands();
        distanceFilter();
        sequencer();
        mirror();
        voltageCompensation();
        burstEarlyEnd();
        System.out.println("Passed " + checks + " V2 regression checks.");
    }

    static void onePersonControls() {
        Gamepad pad = new Gamepad();
        pad.right_stick_x = 1;
        near(BioBuzzTeleOpV2.manualTurretPower(pad), 0, 0,
                "Chassis steering must never jog the turret in one-person TeleOp");
        pad.dpad_right = true;
        near(BioBuzzTeleOpV2.manualTurretPower(pad), 0.15, 0, "Held right D-pad jogs clockwise");
        near(BioBuzzTeleOpV2.manualTurretPower(pad), 0.15, 0, "Jog continues while held");
        pad.dpad_left = true;
        near(BioBuzzTeleOpV2.manualTurretPower(pad), 0, 0, "Opposing jog buttons cancel");
        pad.dpad_right = false;
        near(BioBuzzTeleOpV2.manualTurretPower(pad), -0.15, 0, "Held left D-pad jogs counterclockwise");
        pad.dpad_left = false;
        near(BioBuzzTeleOpV2.manualTurretPower(pad), 0, 0, "Releasing the D-pad ends manual jog");
        pad.right_trigger = 1;
        near(BioBuzzTeleOpV2.intakePower(pad), -Robot.INTAKE_POWER, 0, "RT collects inward");
        pad.left_trigger = 1;
        near(BioBuzzTeleOpV2.intakePower(pad), -Robot.INTAKE_POWER, 0, "RT retains original priority over LT");
        pad.right_trigger = 0;
        near(BioBuzzTeleOpV2.intakePower(pad), Robot.INTAKE_POWER, 0, "LT ejects outward");
        pad.left_trigger = 0;
        near(BioBuzzTeleOpV2.intakePower(pad), 0, 0, "Releasing triggers stops intake");
    }

    static void flowerCommands() {
        double[] position = {Double.NaN};
        Servo servo = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(), new Class<?>[]{Servo.class},
                (proxy, method, args) -> {
                    if (method.getName().equals("setPosition")) { position[0] = (double) args[0]; return null; }
                    throw new AssertionError("Unexpected servo call: " + method.getName());
                });
        FlowerLever flower = new FlowerLever(servo);
        flower.setDeployed(false);
        near(position[0], 0, 0, "Flower OFF commands 0 (not the deployed position)");
        flower.setDeployed(true);
        near(position[0], 0.18, 0, "Flower ON commands 0.18");
        flower.stop();
        near(position[0], 0, 0, "Auto cleanup retracts flower to 0");
    }

    // ================================================================ HIVE cell choice

    static HiveTargetV2.Tag tag(int id, double tx, double ty, double d) {
        return new HiveTargetV2.Tag(id, tx, ty, d);
    }

    static void hiveTargeting() {
        HiveTargetV2 hive = new HiveTargetV2();
        hive.update(Arrays.asList(tag(38, -2, 5, 2.0), tag(39, 0, 5, 1.9), tag(30, 9, 9, 1.0),
                tag(50, 9, 9, 1.0)), Robot.Alliance.BLUE);
        check(hive.cell == HiveTargetV2.Cell.LOW_IDS && hive.tagsUsed == 2, "Blue 38-41 is one CELL; red/other tags ignored");
        near(hive.txDeg, -1, 1e-9, "Aim uses the mean tx of the chosen CELL only");
        check(hive.distanceTagId == 39 && hive.distanceM == 1.9, "Distance is the nearest tag of the chosen CELL");

        hive.reset();
        hive.update(Arrays.asList(tag(38, -10, 5, 2.0), tag(43, 10, 12, 2.2)), Robot.Alliance.BLUE);
        check(hive.bothCellsVisible && hive.cell == HiveTargetV2.Cell.HIGH_IDS,
                "With both CELLS visible, the one higher in the image (larger ty) is the target");
        near(hive.txDeg, 10, 1e-9, "Both-CELL frames no longer average the two CELLS");
        near(hive.distanceM, 2.2, 1e-9, "Distance comes from the chosen CELL, not the nearer down CELL");

        hive.update(Arrays.asList(tag(38, -10, 12.5, 2.0), tag(43, 10, 12, 2.2)), Robot.Alliance.BLUE);
        check(hive.cell == HiveTargetV2.Cell.HIGH_IDS, "Hysteresis: a 0.5 deg ty difference does not flip the CELL");
        hive.update(Arrays.asList(tag(38, -10, 14, 2.0), tag(43, 10, 12, 2.2)), Robot.Alliance.BLUE);
        check(hive.cell == HiveTargetV2.Cell.LOW_IDS, "A clearly higher cluster takes over (HIVE tipped)");

        double sign = HiveTargetV2.UP_CELL_TY_SIGN;
        HiveTargetV2.UP_CELL_TY_SIGN = -1;
        hive.reset();
        hive.update(Arrays.asList(tag(38, -10, 5, 2.0), tag(43, 10, 12, 2.2)), Robot.Alliance.BLUE);
        check(hive.cell == HiveTargetV2.Cell.LOW_IDS, "UP_CELL_TY_SIGN=-1 handles an upside-down camera");
        HiveTargetV2.UP_CELL_TY_SIGN = sign;

        check(HiveTargetV2.cellOf(33, Robot.Alliance.RED) == HiveTargetV2.Cell.LOW_IDS
                && HiveTargetV2.cellOf(34, Robot.Alliance.RED) == HiveTargetV2.Cell.HIGH_IDS
                && HiveTargetV2.cellOf(38, Robot.Alliance.RED) == HiveTargetV2.Cell.NONE
                && HiveTargetV2.cellOf(45, Robot.Alliance.BLUE) == HiveTargetV2.Cell.HIGH_IDS,
                "Tag clusters: RED 30-33 / 34-37, BLUE 38-41 / 42-45");
        hive.update(Collections.<HiveTargetV2.Tag>emptyList(), Robot.Alliance.BLUE);
        check(hive.cell == HiveTargetV2.Cell.NONE && Double.isNaN(hive.txDeg), "No tags: no target");
        hive.update(Arrays.asList(tag(40, Double.NaN, 3, 2)), Robot.Alliance.BLUE);
        check(hive.cell == HiveTargetV2.Cell.NONE, "Non-finite tx is ignored");
    }

    // ================================================================ field-anchored aim

    static final double[] T = {0};

    static FieldAimV2 aim() {
        T[0] = 0;
        return new FieldAimV2(new Robot.TurretGains(), () -> T[0]);
    }

    static FieldAimV2.Inputs pose(double turret, double x, double y, double headingDeg) {
        FieldAimV2.Inputs in = new FieldAimV2.Inputs();
        in.turretDeg = turret;
        in.minDeg = Robot.TURRET_MIN_DEG;
        in.maxDeg = Robot.TURRET_MAX_DEG;
        in.poseValid = true;
        in.xIn = x;
        in.yIn = y;
        in.headingRad = Math.toRadians(headingDeg);
        in.cameraUsable = true;
        return in;
    }

    static int frame = 0;

    static FieldAimV2.Inputs withFrame(FieldAimV2.Inputs in, double error, double range, double ageMs) {
        in.observationFresh = true;
        in.errorDeg = error;
        in.rangeIn = range;
        in.frameTimestamp = ++frame;
        in.frameAgeMs = ageMs;
        return in;
    }

    static void fieldAim() {
        // A) Driving sideways keeps the turret on the remembered field target.
        FieldAimV2 a = aim();
        a.update(withFrame(pose(0, 0, 0, 0), 0, 100, 0));
        check(a.targetConfirmed(), "A camera frame confirms a field target");
        near(a.targetX(), 100, 1e-6, "Target placed 100 in ahead");
        near(a.targetY(), 0, 1e-6, "Target placed straight ahead");
        boolean sawFeedforward = false;
        for (int i = 1; i <= 50; i++) {
            T[0] += 10;
            double p = a.update(pose(0, 0, i, 0));
            if (i == 10) sawFeedforward = a.velocityTerm > 0 && p > 0;
        }
        near(a.goalDeg, Math.toDegrees(Math.atan2(50, 100)), 0.01,
                "After strafing 50 in left with no vision, the goal is 26.6 deg clockwise");
        check(a.state.startsWith("FIELD TRACK"), "Without fresh frames the turret tracks from odometry");
        check(sawFeedforward, "Strafing adds clockwise feedforward before any error builds");

        // B) Chassis rotation.
        a = aim();
        a.update(withFrame(pose(0, 0, 0, 0), 0, 100, 0));
        T[0] += 10;
        a.update(pose(0, 0, 0, 90));
        near(a.goalDeg, 90, 1e-6, "Robot turned 90 deg CCW: turret goal 90 deg clockwise");

        // C) Feedforward is no longer limited by the 0.22 tracking cap.
        a = aim();
        a.update(withFrame(pose(0, 0, 0, 0), 0, 100, 0));
        T[0] += 10;
        FieldAimV2.Inputs spin = pose(30, 0, 0, 30);
        spin.omegaRadS = Math.toRadians(300);
        spin.turretVelDegS = 300;
        double spinPower = a.update(spin);
        near(spinPower, new Robot.TurretGains().kV * 300 + Robot.TURRET_KS, 1e-9,
                "300 deg/s chassis spin gets kV*300 + kS feedforward");
        check(spinPower > Robot.TURRET_TRACK_MAX_POWER, "Spin feedforward exceeds the old 0.22 total cap");

        // D) Latency: the turret angle at capture time is used.
        a = aim();
        for (int i = 0; i <= 20; i++) {
            T[0] = i * 10;
            if (i < 20) a.update(pose(i, 0, 0, 0)); // turret sweeping 100 deg/s
        }
        a.update(withFrame(pose(20, 0, 0, 0), 5, 100, 100)); // captured at t=100 ms, turret 10 deg
        near(a.goalDeg, 15, 0.05, "Frame from 100 ms ago: goal = turret-at-capture (10) + error (5)");

        // E) Reachable equivalents and unwinding.
        near(FieldAimV2.nearestReachableAngle(200, 150, -250, 194), -160, 1e-9,
                "Beyond the right limit, the equivalent angle a turn away is used");
        near(FieldAimV2.nearestReachableAngle(100, -200, -250, 194), 100, 1e-9,
                "Only in-range equivalents are chosen");
        a = aim();
        a.update(withFrame(pose(150, 0, 0, 0), 0, 100, 0)); // target at turret 150
        T[0] += 10;
        a.update(pose(150, 0, 0, 50)); // robot turns 50 CCW -> wants 200 -> beyond 194
        check(a.state.equals("UNWIND LEFT") && a.returning(), "Passing the cable limit unwinds a full turn");

        // F) No odometry: camera-only, then search (as the original).
        a = aim();
        FieldAimV2.Inputs noPose = pose(0, 0, 0, 0);
        noPose.poseValid = false;
        a.update(withFrame(noPose, 4, 100, 0));
        check(a.state.equals("CAMERA ONLY"), "Without Pinpoint the camera goal is used");
        near(a.goalDeg, 4, 1e-9, "Camera-only goal = turret + error");
        T[0] += Robot.TURRET_REACQUIRE_MS + 10;
        FieldAimV2.Inputs noPose2 = pose(0, 0, 0, 0);
        noPose2.poseValid = false;
        a.update(noPose2);
        check(a.state.startsWith("SEARCH"), "Without Pinpoint and frames it searches, like the original");

        // G) Auto: pre-aim at a guess, sweep only if the guess shows nothing.
        a = aim();
        a.searchIfLost = true;
        a.setGuess(100, 0);
        a.update(pose(0, 0, 0, 0));
        check(a.state.equals("PRE-AIM GUESS") && a.targetIsGuess(), "Auto pre-aims at the guessed HIVE");
        T[0] += FieldAimV2.GUESS_SEARCH_AFTER_MS + 10;
        a.update(pose(0, 0, 0, 0));
        check(a.state.startsWith("SEARCH (guess empty)"), "An empty guess falls back to a sweep");
        T[0] += 10;
        a.update(withFrame(pose(0, 0, 0, 0), 3, 90, 0));
        check(a.targetConfirmed() && a.state.equals("TRACK"), "A sighting ends the search and confirms the target");
        a.suppressSearch = true;
        a.clearTarget();
        a.searchIfLost = true;
        T[0] += FieldAimV2.GUESS_SEARCH_AFTER_MS + 10;
        a.update(pose(0, 0, 0, 0));
        T[0] += FieldAimV2.GUESS_SEARCH_AFTER_MS + 10;
        double held = a.update(pose(0, 0, 0, 0));
        check(held == 0 && a.state.startsWith("HOLD"), "suppressSearch holds instead of sweeping (gate open)");

        // H) TeleOp before any sighting: hold the field direction.
        a = aim();
        a.update(pose(20, 0, 0, 0));
        check(a.state.equals("FIELD HOLD") && !a.targetConfirmed(), "TeleOp holds its starting field direction");
        T[0] += 10;
        a.update(pose(20, 0, 0, 30));
        near(a.goalDeg, 50, 1e-6, "Field hold counter-rotates 30 deg for a 30 deg chassis turn");

        // I) aimed() and J) trim.
        a = aim();
        a.update(withFrame(pose(0, 0, 0, 0), 1, 100, 0));
        check(a.aimed(0), "Within 3 deg of a fresh camera-confirmed goal is aimed");
        check(!a.aimed(5), "5 deg off is not aimed");
        T[0] += FieldAimV2.AIM_MAX_OBSERVATION_AGE_MS + 10;
        a.update(pose(0, 0, 0, 0));
        check(!a.aimed(1), "A camera confirmation older than 1 s is not aimed");
        a.aimTrimDeg = 2;
        a.update(pose(0, 0, 0, 0));
        near(a.goalDeg, 3, 1e-6, "Aim trim shifts the goal immediately");

        // K) Encoder outside the soft limits recovers inward.
        a = aim();
        double recover = a.update(pose(Robot.TURRET_MAX_DEG + 3, 0, 0, 0));
        check(a.state.equals("LIMIT RECOVERY LEFT") && recover < 0, "Above the right limit, drive back left");
    }

    // ================================================================ distance filter

    static void quietAim() {
        FieldAimV2 a = aim();
        a.update(withFrame(pose(0, 0, 0, 0), 0, 100, 0));
        double peakGoal = 0, peakPower = 0;
        for (int i = 1; i <= 80; i++) {
            T[0] += 20;
            FieldAimV2.Inputs in = withFrame(pose(0, 0, 0, 0), i % 2 == 0 ? 1.4 : -1.4, 100, 0);
            in.turretVelDegS = i % 2 == 0 ? 8 : -8; // quantized encoder rate on a still turret
            peakPower = Math.max(peakPower, Math.abs(a.update(in)));
            peakGoal = Math.max(peakGoal, Math.abs(a.goalDeg));
        }
        check(peakGoal < 0.3 && peakPower == 0,
                "Small alternating camera/encoder noise must not kick a settled turret");

        T[0] += 20;
        double correction = a.update(withFrame(pose(0, 0, 0, 0), 12, 100, 0));
        near(a.goalDeg, 12, 0.001, "A real target change is acquired immediately");
        check(correction > 0, "A large target change still starts the turret moving");

        // Small sustained bias must converge instead of being mistaken for noise forever.
        a = aim();
        a.update(withFrame(pose(0, 0, 0, 0), 0, 100, 0));
        for (int i = 0; i < 60; i++) {
            T[0] += 20;
            a.update(withFrame(pose(0, 0, 0, 0), 2, 100, 0));
        }
        near(a.goalDeg, 2, 0.001, "Sustained small error converges to the measured target");
        check(a.staticTerm > 0 && a.staticTerm < Robot.TURRET_KS,
                "Near-goal corrections receive a smaller friction kick");

        // Smoothing observations must not smooth away immediate odometry compensation.
        T[0] += 10;
        a.update(pose(0, 0, 0, 30));
        near(a.goalDeg, 32, 0.001, "Heading compensation remains immediate after vision smoothing");

        a = aim();
        FieldAimV2.Inputs in = pose(0, 0, 0, 0);
        in.poseValid = false;
        a.update(withFrame(in, 0, 100, 0));
        for (int i = 1; i <= 20; i++) {
            T[0] += 20;
            in = pose(0, 0, 0, 0);
            in.poseValid = false;
            near(a.update(withFrame(in, i % 2 == 0 ? 1.4 : -1.4, 100, 0)), 0, 0,
                    "Camera-only fallback also rejects small alternating frame noise");
        }
    }

    static void distanceFilter() {
        ShotDistanceFilterV2 f = new ShotDistanceFilterV2();
        f.update(0, 2.0, Double.NaN);
        f.update(10, 2.1, Double.NaN);
        double d = f.update(20, 5.0, Double.NaN);
        near(d, 2.1, 1e-9, "One bad frame cannot move the median");
        check(f.source == ShotDistanceFilterV2.Source.VISION, "Fresh frame: vision source");
        d = f.update(500, Double.NaN, Double.NaN);
        check(f.source == ShotDistanceFilterV2.Source.HELD && d == 2.1, "Short dropout holds the distance");
        d = f.update(1100, Double.NaN, 2.4);
        check(f.source == ShotDistanceFilterV2.Source.ODOMETRY && d == 2.4, "Long dropout uses odometry range");
        d = f.update(1200, Double.NaN, Double.NaN);
        check(f.source == ShotDistanceFilterV2.Source.NONE && Double.isNaN(d), "Nothing: NaN -> default RPM");
        near(Robot.rpmForDistance(Double.NaN), Robot.TELEOP_DEFAULT_RPM, 0, "NaN distance uses the default RPM");
    }

    // ================================================================ hold-to-shoot

    static void sequencer() {
        ShotSequencerV2 s = new ShotSequencerV2();
        s.update(0, true, false, false, false, false, true, false, 0);
        check(s.phase == ShotSequencerV2.Phase.SPINUP && !s.gateOpen && s.waitingFor.equals("flywheel speed"),
                "Hold-to-shoot waits for the flywheel with the gate closed");
        s.update(10, true, false, true, true, true, true, false, 0);
        s.update(100, true, false, true, true, true, true, false, 0);
        check(s.phase == ShotSequencerV2.Phase.SPINUP, "The flywheel must be stable for 150 ms first");
        s.update(170, true, false, true, true, false, true, false, 0);
        check(s.phase == ShotSequencerV2.Phase.SPINUP && s.waitingFor.equals("turret aim"), "Then it waits for aim");
        s.update(180, true, false, true, true, true, true, false, 0);
        check(s.phase == ShotSequencerV2.Phase.GATE_OPENING && s.gateOpen, "Ready: gate opens");
        check(s.transmissionPower > 0, "Gate opening starts with an outward relief pulse");
        s.update(290, true, false, true, true, true, true, false, 0);
        check(s.transmissionPower == 0, "After relief, nothing pushes into the moving gate");
        s.update(440, true, false, true, true, true, true, false, 0);
        check(s.phase == ShotSequencerV2.Phase.FEEDING && s.intakePower == ShotSequencerV2.INWARD_INTAKE
                && s.transmissionPower == ShotSequencerV2.INWARD_TRANSMISSION, "Then it feeds inward");
        s.update(450, false, false, true, true, true, true, false, 0);
        check(s.phase == ShotSequencerV2.Phase.IDLE && !s.gateOpen && s.transmissionPower == 0, "Release closes the gate");

        s = new ShotSequencerV2();
        s.update(0, true, true, false, true, false, true, false, 0);
        check(s.phase == ShotSequencerV2.Phase.GATE_OPENING, "Force shot skips aim once near speed");

        s = new ShotSequencerV2();
        s.update(0, true, false, true, true, true, false, true, -1);
        check(!s.gateOpen && s.transmissionPower == 0 && s.intakePower == -1, "No gate calibration: no gate, no transmission");

        s = new ShotSequencerV2();
        s.update(0, false, false, false, false, false, true, false, -1);
        check(s.transmissionPower == ShotSequencerV2.INWARD_TRANSMISSION, "Manual intake stacks balls against the closed gate");
        s.update(10, false, false, false, false, false, true, true, -1);
        check(s.gateOpen && s.transmissionPower > 0, "Manual gate open gets the relief pulse too");
        s.update(150, false, false, false, false, false, true, true, -1);
        check(s.transmissionPower == 0, "Manual feed waits for the gate to finish moving");
        s.update(300, false, false, false, false, false, true, true, -1);
        check(s.transmissionPower == ShotSequencerV2.INWARD_TRANSMISSION, "Then driver 1 feeds as before");
    }

    // ================================================================ mirroring

    static void mirror() {
        check(FieldMirrorV2.x(56, 8, Robot.Alliance.BLUE) == 56 && FieldMirrorV2.headingDeg(270, Robot.Alliance.BLUE) == 270,
                "Blue geometry is unchanged");
        near(FieldMirrorV2.x(56, 8, Robot.Alliance.RED), 88, 1e-9, "Red mirrors x across the center line");
        near(FieldMirrorV2.y(56, 8, Robot.Alliance.RED), 8, 1e-9, "Red keeps y when mirroring");
        near(FieldMirrorV2.headingDeg(0, Robot.Alliance.RED), 180, 1e-9, "Facing +x becomes facing -x");
        FieldMirrorV2.Mode mode = FieldMirrorV2.MODE;
        FieldMirrorV2.MODE = FieldMirrorV2.Mode.ROTATE_180;
        near(FieldMirrorV2.y(56, 8, Robot.Alliance.RED), 136, 1e-9, "Rotational mode flips y");
        near(FieldMirrorV2.headingDeg(0, Robot.Alliance.RED), 180, 1e-9, "Rotational mode turns headings 180");
        FieldMirrorV2.MODE = mode;
    }

    // ================================================================ shooter F

    static void voltageCompensation() {
        near(RobotV2.compensatedF(RobotV2.NOMINAL_VOLTAGE), Robot.SHOOTER_F, 1e-9, "Nominal voltage keeps F");
        near(RobotV2.compensatedF(11.5), Robot.SHOOTER_F * RobotV2.NOMINAL_VOLTAGE / 11.5, 1e-9,
                "A sagging battery raises F proportionally");
        near(RobotV2.compensatedF(5), Robot.SHOOTER_F * RobotV2.NOMINAL_VOLTAGE / 9.0, 1e-9, "Clamped at 9 V");
    }

    // ================================================================ early burst end

    static final class FakeHardware implements BioBuzzAutoMechanisms.Hardware {
        boolean feeding, finished;
        @Override public BioBuzzAutoMechanisms.Observation observe() {
            return new BioBuzzAutoMechanisms.Observation(false, 0, 0, true);
        }
        @Override public void control(BioBuzzAutoMechanisms.Kind request) {
            feeding = request == BioBuzzAutoMechanisms.Kind.SHOOT;
        }
        @Override public boolean shootingBurstActive() { return feeding; }
        @Override public boolean burstFinishedEarly(int plannedBalls) { return finished && plannedBalls == 4; }
        @Override public void stopOutputs() { feeding = false; }
        @Override public void close() { feeding = false; }
    }

    static void burstEarlyEnd() {
        long[] now = {1_000_000_000L};
        FakeHardware hw = new FakeHardware();
        BioBuzzAutoMechanisms m = new BioBuzzAutoMechanisms(hw);
        m.setFaultTolerant(true);
        m.observe();
        Command volley = m.command(BioBuzzAutoMechanisms.Action.shootFor(4, 2.0, 6.0), () -> now[0],
                reason -> { throw new AssertionError(reason); });
        volley.start();
        m.update();
        check(!volley.done(), "Burst starts");
        now[0] += 600_000_000L;
        m.update();
        check(!volley.done(), "Without the early signal the burst keeps its full window");
        hw.finished = true;
        now[0] += 10_000_000L;
        m.update();
        volley.done();
        now[0] += 210_000_000L;
        m.update();
        check(volley.done(), "The early signal (with the planned count) ends the burst at ~0.8 s instead of 2 s");
        volley.end(EndCondition.NATURALLY);
        m.stop();

        // Default adapters never finish early: the original timing is unchanged.
        BioBuzzAutoMechanisms.Hardware plain = new BioBuzzAutoMechanisms.Hardware() {
            @Override public BioBuzzAutoMechanisms.Observation observe() { return null; }
            @Override public void control(BioBuzzAutoMechanisms.Kind request) { }
            @Override public void stopOutputs() { }
            @Override public void close() { }
        };
        check(!plain.burstFinishedEarly(4), "The original adapter keeps the full timed burst");
    }
}
