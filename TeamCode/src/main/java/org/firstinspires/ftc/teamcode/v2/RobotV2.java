package org.firstinspires.ftc.teamcode.v2;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.autonomous.BioBuzzAutoConfig;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * V2 robot layer. Reuses Robot for hardware mapping, the gate controller, drive math, turret soft
 * limits / stall detection and the shooter's velocity PIDF, and replaces the sensing and aiming:
 *
 *  - MANUAL bulk caching, cleared once per loop (Robot uses AUTO, so repeated encoder reads in
 *    one loop each cost another hub transaction).
 *  - Ball sensor: distance only, one I2C read per loop instead of color + distance.
 *  - Vision: aims at the upward HIVE CELL's tag cluster only (HiveTargetV2).
 *  - Turret: field-anchored aim from odometry (FieldAimV2).
 *  - Shooter: F scaled by battery voltage; RPM from a filtered distance (ShotDistanceFilterV2).
 *
 * The original Robot/TeleOp/AutoPath code paths are untouched.
 */
public final class RobotV2 {

    public static final String VERSION = "BIOBUZZ-V2-ONE-PERSON-2026-11-12";

    // The tuner and V2 share Robot.SHOOTER_F_REFERENCE_VOLTS (13.3 V).
    public static double VOLTAGE_REFRESH_MS = 500;
    /** Rewrite the hub PIDF only when compensated F changes by more than this fraction. */
    public static double F_REWRITE_FRACTION = 0.01;
    /** Tight window for hold-to-shoot (the original TeleOp window was +/-100 RPM). */
    public static double AT_SPEED_RPM = 50;
    /** Per-OpMode window; Auto keeps 50 RPM while TeleOp can use a tighter indication. */
    public double atSpeedToleranceRpm = AT_SPEED_RPM;
    public static double FORCE_MIN_RPM_FRACTION = 0.90;
    /**
     * Height of the HIVE tags above the camera lens, inches. 0 = treat the straight-line tag
     * distance as horizontal range for aiming (small error). The RPM table always uses
     * straight-line distance, exactly as it was measured.
     */
    public static double TAG_HEIGHT_ABOVE_CAMERA_IN = 0.0;

    // ---- Odometry-only turret aim (TeleOp, 2026-11-14). The Limelight still gives shot distance.
    /**
     * Physical direction of turret encoder 0, degrees clockwise from the robot front. The team zeros
     * the turret pointing straight BACKWARD (AutoPath INIT, carried into TeleOp), so 180.
     */
    public static double TURRET_ZERO_DIRECTION_DEG = BioBuzzAutoConfig.TURRET_ZERO_DIRECTION_DEG;
    /*
     * Field geometry in the robot's own frame (origin at the corner behind START, inches).
     * Both alliances run the same AutoPath from the same START in their own frame and the field is
     * 180-degree symmetric, so our HIVE is in the same place for red and blue.
     * Official BIOBUZZ manual: the HIVE structure sits in the field center; the red and blue HIVES are
     * 25.5 in apart center-to-center; each HIVE's two CELLS are 18.84 in apart, one facing the
     * audience side and one the opposite side. AutoPath's aim at spot 2 puts our HIVE at x > center.
     * FIELD_SIZE_IN: inside size; the AutoPath HIVE estimate (y = 70.7) points to ~141 in, not Pedro's
     * nominal 144. Change it if your field measures differently.
     */
    public static double FIELD_SIZE_IN = 141.0;
    public static double HIVE_HALF_SPACING_IN = 25.5 / 2;  // Our HIVE center is this far +x of center.
    public static double CELL_HALF_SPACING_IN = 18.84 / 2; // Each CELL is this far from the HIVE center in y.
    /** The turret switches CELLS only after the robot is this far past the center line. */
    public static double CELL_SWITCH_HYSTERESIS_IN = 3.0;
    public static double fieldCenterIn() { return FIELD_SIZE_IN / 2; }
    public static double hiveXIn() { return fieldCenterIn() + HIVE_HALF_SPACING_IN; }
    /** CELL on the START half (y < center) or the far half. */
    public static double cellYIn(boolean farHalf) {
        return fieldCenterIn() + (farHalf ? CELL_HALF_SPACING_IN : -CELL_HALF_SPACING_IN);
    }
    /**
     * Re-localize (right stick click) only if odometry already puts the robot within this distance
     * of the park-corner pose and it is nearly stopped, so a stray click mid-field does nothing.
     */
    public static double RELOCALIZE_MAX_ERROR_IN = 36.0;
    public static double RELOCALIZE_MAX_SPEED_IN_S = 3.0;
    /** True in TeleOp: aim from odometry at the fixed HIVE position, never from the camera. */
    public boolean odometryAim;
    public String relocalizeStatus = "not used";
    public String turretZeroSource = "INIT position";
    /** Which of our HIVE's CELLS the turret aims at: the one on the robot's half of the field. */
    public boolean aimFarCell;
    private boolean aimCellChosen;
    /** Camera-measured CELL position (for checking the fixed coordinates; never used to aim). */
    public double cameraCellX = Double.NaN, cameraCellY = Double.NaN;

    public final Robot robot = new Robot();
    public final HiveTargetV2 hive = new HiveTargetV2();
    public final FieldAimV2 aim = new FieldAimV2(robot.turretGains, RobotV2::nowMs);
    public final ShotDistanceFilterV2 distanceFilter = new ShotDistanceFilterV2();
    private final FieldAimV2.Inputs aimInputs = new FieldAimV2.Inputs();

    private HardwareMap hardwareMap;
    private List<LynxModule> hubs = new ArrayList<>();
    private GoBildaPinpointDriver pinpoint;

    // Pose (field inches, heading CCW radians).
    public boolean poseValid;
    public double xIn = Double.NaN, yIn = Double.NaN, headingRad = Double.NaN, omegaRadS;
    public String poseStatus = "not read";

    // Vision.
    public boolean cameraUsable;
    public boolean frameFresh;
    public double frameTimestamp = Double.NaN;
    public double frameAgeMs = Double.NaN;
    public String visionStatus = "not read";
    public int rawTagCount;
    private final List<HiveTargetV2.Tag> tags = new ArrayList<>();
    private Robot.Alliance hiveAlliance;

    // Shooter.
    public double batteryVolts = Double.NaN;
    public double appliedF = Double.NaN;
    private double shooterP = Robot.SHOOTER_P;
    private double shooterReferenceF = Robot.SHOOTER_F;
    private double lastVoltageMs = Double.NEGATIVE_INFINITY;
    public double shotDistanceM = Double.NaN;
    public double baseRpm = Robot.TELEOP_DEFAULT_RPM;

    // Turret.
    public String turretMode = "AUTO";
    private double lastLed = Double.NaN;

    // Loop timing.
    public double loopMs, loopMsAvg, loopMsMax;
    private double lastLoopMs = Double.NaN, maxWindowStartMs;
    private double windowMax;

    public static double nowMs() {
        return System.nanoTime() / 1e6;
    }

    // =========================================================== init

    /** Full robot including drivetrain (TeleOp). Never resets the turret encoder or the Pinpoint. */
    public void initTeleOp(HardwareMap hw) {
        hardwareMap = hw;
        robot.init(hw, false);
        // TeleOp starts where AutoPath ended: keep its turret zero (pointing backward) when there is one.
        turretZeroSource = robot.adoptCarriedTurretReference() ? "from AutoPath (backward)" : "INIT position";
        odometryAim = true;
        finishInit();
    }

    /** Mechanisms only; Pedro owns the drivetrain (Auto). */
    public void initMechanisms(HardwareMap hw) {
        hardwareMap = hw;
        robot.initMechanismsOnly(hw, false);
        finishInit();
    }

    private void finishInit() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            hub.clearBulkCache();
        }
        pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, "pinpoint");
        hive.reset();
    }

    public void setAlliance(Robot.Alliance alliance) {
        if (alliance != robot.alliance || hiveAlliance != alliance) {
            robot.alliance = alliance;
            hiveAlliance = alliance;
            hive.reset();
            distanceFilter.reset();
            aim.clearTarget();
        }
        if (odometryAim) {
            aimCellChosen = false;
            chooseAimCell();
        }
    }

    /** Aim at our HIVE's CELL on the robot's half (START half y < center, else far half). */
    public void chooseAimCell() {
        if (!odometryAim) return;
        boolean far = aimFarCell;
        if (poseValid) {
            double center = fieldCenterIn();
            if (!aimCellChosen) far = yIn >= center;
            else if (far && yIn < center - CELL_SWITCH_HYSTERESIS_IN) far = false;
            else if (!far && yIn > center + CELL_SWITCH_HYSTERESIS_IN) far = true;
        }
        if (!aimCellChosen || far != aimFarCell) {
            aimFarCell = far;
            aim.setFieldTarget(hiveXIn(), cellYIn(far));
        }
        aimCellChosen = aimCellChosen || poseValid;
    }

    /** Turret angle clockwise from the robot front (encoder angle plus the zero direction). */
    public double turretPhysicalDeg() {
        return robot.turretDegrees() + (odometryAim ? TURRET_ZERO_DIRECTION_DEG : 0);
    }

    /**
     * Park-corner pose for re-localizing: the field corner next to the alliance parking zone, with the
     * robot pressed square into it. Heading snaps to the nearest 90 degrees.
     */
    public static double[] parkCornerPose(Robot.Alliance alliance, double headingRad) {
        double snapped = (Math.PI / 2) * Math.rint(headingRad / (Math.PI / 2));
        boolean lengthAlongY = Math.abs(Math.sin(snapped)) > 0.5;
        double halfX = (lengthAlongY ? BioBuzzAutoConfig.ROBOT_WIDTH_IN : BioBuzzAutoConfig.ROBOT_LENGTH_IN) / 2;
        double halfY = (lengthAlongY ? BioBuzzAutoConfig.ROBOT_LENGTH_IN : BioBuzzAutoConfig.ROBOT_WIDTH_IN) / 2;
        // Same corner for both alliances in the robot's own frame (same AutoPath): x = 0, far wall.
        return new double[]{halfX, FIELD_SIZE_IN - halfY, snapped};
    }

    /** Right stick click: reset the Pinpoint pose to the park corner, if plausibly there and stopped. */
    public boolean relocalizeAtParkCorner() {
        if (pinpoint == null || !poseValid) { relocalizeStatus = "refused: no Pinpoint pose"; return false; }
        double[] corner = parkCornerPose(robot.alliance, headingRad);
        double error = Math.hypot(corner[0] - xIn, corner[1] - yIn);
        double speed = Math.hypot(aim.chassisVxInS, aim.chassisVyInS);
        if (error > RELOCALIZE_MAX_ERROR_IN) {
            relocalizeStatus = String.format(Locale.US, "refused: odometry %.0f in from the park corner", error);
            return false;
        }
        if (!(speed <= RELOCALIZE_MAX_SPEED_IN_S)) {
            relocalizeStatus = String.format(Locale.US, "refused: moving %.1f in/s", speed);
            return false;
        }
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, corner[0], corner[1], AngleUnit.RADIANS, corner[2]));
        relocalizeStatus = String.format(Locale.US, "OK: moved %.1f in to (%.1f, %.1f, %.0f deg)",
                error, corner[0], corner[1], Math.toDegrees(corner[2]));
        setPose(corner[0], corner[1], corner[2], omegaRadS, true);
        poseStatus = "RELOCALIZED";
        return true;
    }

    // =========================================================== per loop

    /** First call every loop: fresh hub data and loop timing. */
    public void beginLoop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        double now = nowMs();
        if (Double.isFinite(lastLoopMs)) {
            loopMs = now - lastLoopMs;
            loopMsAvg = loopMsAvg == 0 ? loopMs : 0.9 * loopMsAvg + 0.1 * loopMs;
            windowMax = Math.max(windowMax, loopMs);
            if (now - maxWindowStartMs > 2000) {
                loopMsMax = windowMax;
                windowMax = 0;
                maxWindowStartMs = now;
            }
        }
        lastLoopMs = now;
    }

    /** TeleOp pose source. Reads the existing Pinpoint pose without resetting it. */
    public void readPinpoint() {
        if (pinpoint == null) {
            poseValid = false;
            poseStatus = "MISSING: configure Pinpoint as pinpoint";
            return;
        }
        pinpoint.update();
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        if (status != GoBildaPinpointDriver.DeviceStatus.READY) {
            poseValid = false;
            poseStatus = status.toString();
            return;
        }
        setPose(pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(AngleUnit.RADIANS),
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS), true);
        poseStatus = poseValid ? "PINPOINT READY" : "PINPOINT INVALID SAMPLE";
    }

    /** Auto pose source (Pedro follower pose, same Pinpoint frame). */
    public void setPose(double x, double y, double heading, double omega, boolean valid) {
        poseValid = valid && Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(heading);
        xIn = x;
        yIn = y;
        headingRad = heading;
        omegaRadS = Double.isFinite(omega) ? omega : 0;
        if (valid && !poseValid) poseStatus = "INVALID POSE";
        else if (valid) poseStatus = "PEDRO";
    }

    public void readVision() {
        frameFresh = false;
        cameraUsable = false;
        rawTagCount = 0;
        tags.clear();
        Limelight3A ll = robot.Limelight;
        if (hiveAlliance != robot.alliance) setAlliance(robot.alliance);
        if (ll == null) {
            visionStatus = "MISSING: hardware config name must be Limelight";
            hive.update(null, robot.alliance);
            return;
        }
        if (!ll.isRunning() || !ll.isConnected()) {
            visionStatus = "NO CAMERA UPDATES: check USB/power/config";
            hive.update(null, robot.alliance);
            return;
        }
        LLResult result = ll.getLatestResult();
        if (result == null) {
            visionStatus = "NO RESULT RECEIVED";
            hive.update(null, robot.alliance);
            return;
        }
        long staleness = result.getStaleness();
        if (staleness >= Robot.LIMELIGHT_STALE_MS) {
            visionStatus = "STALE RESULT";
            hive.update(null, robot.alliance);
            return;
        }
        if (result.getPipelineIndex() != Robot.LIMELIGHT_PIPELINE) {
            visionStatus = "WRONG PIPELINE: expected " + Robot.LIMELIGHT_PIPELINE;
            hive.update(null, robot.alliance);
            return;
        }
        cameraUsable = true;
        if (!result.isValid() || result.getFiducialResults() == null) {
            visionStatus = "NO APRILTAGS";
            hive.update(null, robot.alliance);
            return;
        }
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            rawTagCount++;
            double distance = Double.NaN;
            Pose3D pose = tag.getTargetPoseCameraSpace();
            if (pose != null) {
                Position p = pose.getPosition().toUnit(DistanceUnit.METER);
                double d = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
                // The SDK returns an all-zero pose when the pipeline has no pose data.
                if (Double.isFinite(d) && d > 0) distance = d;
            }
            tags.add(new HiveTargetV2.Tag(tag.getFiducialId(), tag.getTargetXDegrees(),
                    tag.getTargetYDegrees(), distance));
        }
        hive.update(tags, robot.alliance);
        if (hive.cell == HiveTargetV2.Cell.NONE) {
            visionStatus = "TAGS SEEN BUT NONE FOR " + robot.alliance;
            return;
        }
        double ts = result.getTimestamp();
        if (!Double.isFinite(ts) || ts == 0) ts = result.getControlHubTimeStamp();
        frameFresh = ts != frameTimestamp;
        frameTimestamp = ts;
        frameAgeMs = staleness + result.getCaptureLatency() + result.getTargetingLatency()
                + result.getParseLatency();
        visionStatus = hive.bothCellsVisible ? "BOTH CELLS VISIBLE" : "HIVE CELL VISIBLE";
    }

    /** One I2C read (distance) instead of color + distance. Drives the pollen LED. */
    public void readBall() {
        DistanceSensor sensor = robot.ballDistanceSensor;
        if (sensor == null) {
            robot.ballPresent = false;
            robot.ballDistanceCm = Double.NaN;
        } else {
            double d = sensor.getDistance(DistanceUnit.CM);
            robot.ballDistanceCm = d;
            robot.ballPresent = Double.isFinite(d) && d > 0 && d < Robot.BALL_PRESENT_CM;
        }
        refreshLed();
    }

    /** True while the X shooting sequence is running: the LED shows purple instead of pollen state. */
    public boolean shootingIndicator;

    /** Purple while shooting; otherwise green with pollen present, red without. */
    public static double ledColor(boolean shooting, boolean ballPresent) {
        return shooting ? Robot.LED_VIOLET : ballPresent ? Robot.LED_GREEN : Robot.LED_RED;
    }

    /** Writes the LED only when its color changes. */
    public void refreshLed() {
        double led = ledColor(shootingIndicator, robot.ballPresent);
        if (robot.led != null && led != lastLed) {
            robot.led.setPosition(led);
            lastLed = led;
        }
    }

    // =========================================================== turret

    /** Turret-frame vision error for the chosen CELL, degrees clockwise, or NaN. */
    public double visionErrorDeg() {
        if (!Double.isFinite(hive.txDeg)) return Double.NaN;
        return (hive.txDeg - Robot.TURRET_AIM_OFFSET_DEG) * Robot.TURRET_VISION_POWER_SIGN;
    }

    /** Horizontal range estimate for aiming, inches, or NaN. */
    public double aimRangeIn() {
        if (!Double.isFinite(hive.distanceM)) return Double.NaN;
        double d = hive.distanceM / 0.0254;
        double h = TAG_HEIGHT_ABOVE_CAMERA_IN;
        return h > 0 && d > h ? Math.sqrt(d * d - h * h) : d;
    }

    /**
     * Runs the field-anchored aim every loop (so it keeps learning from frames even under manual
     * control), then applies auto, manual or centering power through Robot's soft limits.
     */
    public void driveTurret(boolean autoAim, boolean center, double manualPower) {
        chooseAimCell();
        FieldAimV2.Inputs in = aimInputs;
        double zero = odometryAim ? TURRET_ZERO_DIRECTION_DEG : 0;
        in.turretDeg = robot.turretDegrees() + zero;
        in.turretVelDegS = robot.turret.getVelocity() / Robot.TURRET_TICKS_PER_DEG;
        in.minDeg = robot.turretMinDegrees() + zero;
        in.maxDeg = robot.turretMaxDegrees() + zero;
        in.poseValid = poseValid;
        in.xIn = xIn;
        in.yIn = yIn;
        in.headingRad = headingRad;
        in.omegaRadS = omegaRadS;
        in.cameraUsable = cameraUsable;
        in.observationFresh = frameFresh && hive.cell != HiveTargetV2.Cell.NONE;
        in.errorDeg = visionErrorDeg();
        in.rangeIn = aimRangeIn();
        in.frameTimestamp = frameTimestamp;
        in.frameAgeMs = frameAgeMs;
        double autoPower = aim.update(in);
        updateCameraCellCheck();

        if (center) {
            turretMode = "CENTER";
            robot.turretToAngle(0);
        } else if (!autoAim) {
            turretMode = "MANUAL";
            robot.setTurretPower(manualPower);
        } else {
            turretMode = "AUTO";
            robot.setTurretPower(autoPower);
        }
    }

    /** Where the camera says the CELL is (pose + turret + tag offset + range). Telemetry only. */
    private void updateCameraCellCheck() {
        double err = visionErrorDeg(), range = aimRangeIn();
        if (!odometryAim || !poseValid || !frameFresh || hive.cell == HiveTargetV2.Cell.NONE
                || !Double.isFinite(err) || !Double.isFinite(range)) return;
        double bearing = headingRad - Math.toRadians(turretPhysicalDeg() + err);
        cameraCellX = xIn + range * Math.cos(bearing);
        cameraCellY = yIn + range * Math.sin(bearing);
    }

    public boolean aimed() {
        return turretMode.equals("AUTO") && !robot.isTurretStalled() && !robot.turretRecovering()
                && aim.aimed(turretPhysicalDeg());
    }


    // =========================================================== shooter

    /** Per-OpMode tune: V2 TeleOp can test new gains without changing the Auto/A-B baseline. */
    public void setShooterTune(double p, double referenceF) {
        if (!Double.isFinite(p) || p < 0 || !Double.isFinite(referenceF) || referenceF < 0) {
            throw new IllegalArgumentException("Shooter gains must be finite and nonnegative");
        }
        shooterP = p;
        shooterReferenceF = referenceF;
        appliedF = Robot.compensatedShooterF(referenceF, batteryVolts);
        robot.shooter.setVelocityPIDFCoefficients(shooterP, Robot.SHOOTER_I, Robot.SHOOTER_D, appliedF);
        lastVoltageMs = Double.NEGATIVE_INFINITY;
    }

    /** Re-reads battery voltage now and then and scales the hub velocity F to match. */
    public void refreshVoltage() {
        double now = nowMs();
        if (now - lastVoltageMs < VOLTAGE_REFRESH_MS || hardwareMap == null) return;
        lastVoltageMs = now;
        double lowest = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (Double.isFinite(v) && v > 0) lowest = Math.min(lowest, v);
        }
        if (Double.isInfinite(lowest)) return;
        batteryVolts = lowest;
        double f = Robot.compensatedShooterF(shooterReferenceF, lowest);
        if (Double.isNaN(appliedF) || Math.abs(f - appliedF) > F_REWRITE_FRACTION * appliedF) {
            robot.shooter.setVelocityPIDFCoefficients(shooterP, Robot.SHOOTER_I, Robot.SHOOTER_D, f);
            appliedF = f;
        }
    }

    public static double compensatedF(double volts) {
        return Robot.compensatedShooterF(Robot.SHOOTER_F, volts);
    }

    /** Updates the filtered shot distance from this loop's frame and the remembered target. */
    public double updateShotDistance() {
        double fresh = frameFresh && Double.isFinite(hive.distanceM) ? hive.distanceM : Double.NaN;
        double odometry = Double.NaN;
        if (aim.targetConfirmed() && poseValid) {
            double r = aim.rangeToTargetIn(xIn, yIn, headingRad);
            double h = TAG_HEIGHT_ABOVE_CAMERA_IN;
            if (Double.isFinite(r)) odometry = Math.sqrt(r * r + h * h) * 0.0254;
        }
        shotDistanceM = distanceFilter.update(nowMs(), fresh, odometry);
        return shotDistanceM;
    }

    /** Commands the flywheel. baseRpmOverride (finite) latches a burst's RPM. */
    public void runShooter(boolean on, double trimRpm, double baseRpmOverride) {
        baseRpm = Double.isFinite(baseRpmOverride) ? baseRpmOverride : Robot.rpmForDistance(shotDistanceM);
        robot.shooterOn = on;
        robot.rpmTrim = trimRpm;
        robot.updateShooter(baseRpm);
    }

    public double commandedRpm() {
        return robot.targetRpm + robot.rpmTrim;
    }

    public boolean atSpeed() {
        return robot.shooterOn && Math.abs(robot.shooterRpm() - commandedRpm()) <= atSpeedToleranceRpm;
    }

    public boolean nearSpeed() {
        return robot.shooterOn && robot.shooterRpm() >= FORCE_MIN_RPM_FRACTION * commandedRpm();
    }

    // =========================================================== telemetry

    public void addTelemetry(Telemetry t) {
        t.addData("Code", VERSION);
        t.addData("Loop", "%.1f ms avg / %.1f ms max (2 s)", loopMsAvg, loopMsMax);
        t.addData("Pose", poseValid ? String.format(Locale.US, "%.1f, %.1f in, %.1f deg (%s)",
                xIn, yIn, Math.toDegrees(headingRad), poseStatus) : poseStatus);
        t.addData("Vision", "%s; raw tags %d", visionStatus, rawTagCount);
        t.addData("HIVE cell", "%s (%d tags)%s", HiveTargetV2.label(hive.cell, robot.alliance), hive.tagsUsed,
                hive.bothCellsVisible ? String.format(Locale.US, "; ty low %.1f / high %.1f",
                        hive.lowTyDeg, hive.highTyDeg) : "");
        t.addData("Turret", "%s / %s; %.1f deg -> goal %.1f; trim %+.1f%s",
                turretMode, aim.state, turretPhysicalDeg(), aim.goalDeg, aim.aimTrimDeg,
                odometryAim ? " (from robot front; zero " + turretZeroSource + ")" : "");
        if (odometryAim) {
            t.addData("Aim CELL", "%s half -> (%.1f, %.1f); camera saw (%s)", aimFarCell ? "FAR" : "START",
                    hiveXIn(), cellYIn(aimFarCell), Double.isFinite(cameraCellX)
                            ? String.format(Locale.US, "%.1f, %.1f", cameraCellX, cameraCellY) : "--");
            t.addData("Relocalize", "right stick click at the park corner: %s", relocalizeStatus);
        }
        t.addData("Turret power", "applied %+.2f (P %+.2f D %+.2f FF %+.2f S %+.2f) limit=%s",
                robot.turretAppliedPower(), aim.pTerm, aim.dTerm, aim.velocityTerm, aim.staticTerm,
                robot.turretPowerLimit());
        t.addData("Target", aim.hasTarget() ? String.format(Locale.US, "(%.1f, %.1f) %s, seen %.0f ms ago",
                aim.targetX(), aim.targetY(), aim.odometryOnly ? "fixed HIVE (odometry)"
                        : aim.targetConfirmed() ? "camera" : aim.targetIsGuess() ? "guess" : "hold",
                aim.observationAgeMs()) : "none");
        t.addData("Shot distance", "%s m (%s)", Double.isFinite(shotDistanceM)
                ? String.format(Locale.US, "%.2f", shotDistanceM) : "--", distanceFilter.source);
        t.addData("Shooter", "%s %.0f / %.0f RPM (base %.0f, trim %+.0f) %s", robot.shooterOn ? "ON" : "OFF",
                robot.shooterRpm(), commandedRpm(), baseRpm, robot.rpmTrim, atSpeed() ? "AT SPEED" : "");
        t.addData("RPM error", "%+.1f RPM; at-speed band +/-%.0f RPM",
                robot.shooterRpm() - commandedRpm(), atSpeedToleranceRpm);
        t.addData("Battery / F", "%.2f V / F=%.2f", batteryVolts, appliedF);
        t.addData("Shooter tune", "P=%.0f I=%.0f D=%.0f; reference F=%.2f at %.1f V",
                shooterP, Robot.SHOOTER_I, Robot.SHOOTER_D, shooterReferenceF, Robot.SHOOTER_F_REFERENCE_VOLTS);
        if (robot.isTurretStalled()) t.addLine("TURRET STALLED: check wiring, restart OpMode to reset.");
    }
}
