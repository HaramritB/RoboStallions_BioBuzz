package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * All of the robot's hardware, plus the logic that TeleOp and Auto share.
 *
 * Robot configuration names (must match the config on the Driver Station):
 *   Motors:  frontleft, frontright, backleft, backright   goBILDA 435 rpm Yellow Jacket
 *            intake                                       goBILDA 1620 rpm Yellow Jacket
 *            transmission                                 goBILDA 1150 rpm Yellow Jacket
 *            turret                                       goBILDA 117/118 rpm Yellow Jacket
 *            shooter                                      ServoCity 5000 Series
 *   Servos:  torqueservo1                                 gate, Swyft Torque Servo
 *
 * Optional, used if they're in the config and skipped if not:
 *   Servos:  led                                          goBILDA RGB Indicator Light
 *   I2C:     colorSensor                                  REV Color Sensor V3 (just below the gate)
 *   USB:     Limelight                                    Limelight 3A (on top of the shooter)
 * Auto feeding requires our HIVE tag. The TeleOp gate toggle is independent of vision.
 *
 * init() never moves a servo. Servos stay where they are until something commands them.
 *
 * Every number marked "tune" is a starting guess. Check it on the real robot.
 */
public class Robot {

    public static final String CODE_VERSION = "BIOBUZZ-CENTER-FIX-2026-11-02";

    public enum Alliance { RED, BLUE }

    /** The ball waiting at the gate. YELLOW is POLLEN (2.8 in), RED and BLUE are NECTAR (3.6 in). */
    public enum BallColor { NONE, YELLOW, RED, BLUE }

    /** Auto saves the alliance here so TeleOp starts on the same one. */
    public static Alliance lastAlliance = Alliance.RED;

    // ---------------------------------------------------------------------------------------------
    // Drivetrain
    // ---------------------------------------------------------------------------------------------
    // 435 rpm Yellow Jacket (13.7:1) has 384.5 encoder ticks per output shaft rev.
    // Belted 20T on the motor to 16T on the wheel, so the wheel turns 20/16 = 1.25x per motor rev.
    public static final double DRIVE_TICKS_PER_MOTOR_REV = 384.5;
    public static final double DRIVE_GEAR_RATIO = 20.0 / 16.0;
    public static final double WHEEL_DIAMETER_IN = 96.0 / 25.4;     // goBILDA 96 mm mecanum
    public static final double DRIVE_TICKS_PER_INCH =
            DRIVE_TICKS_PER_MOTOR_REV / (DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_IN);
    public static final double STRAFE_MULTIPLIER = 1.1;               // mecanum slips a little sideways

    // ---------------------------------------------------------------------------------------------
    // Turret: 117/118 rpm Yellow Jacket (50.9:1), 1425.1 ticks per rev, 1:1 to the turret
    // ---------------------------------------------------------------------------------------------
    public static final double TURRET_TICKS_PER_DEG = 1425.1 / 360.0;
    // Encoder angles relative to straight forward. Set your measured travel here (at most 360 deg).
    public static final double TURRET_MIN_DEG = -150;
    public static final double TURRET_MAX_DEG = 150;
    public static final double TURRET_KP = 0.02;               // tune: power per degree of aim error
    public static final double TURRET_KS = 0.04;               // tune: minimum power that moves the turret
    public static final double TURRET_DEADBAND_DEG = 1.0;
    public static final double TURRET_MAX_POWER = 0.6;
    public static final double TURRET_SEARCH_POWER = 0.15;    // tune: speed after the tag leaves view
    public static final double TURRET_RETURN_MAX_POWER = 0.35;// tune: speed across to the other end
    public static final double TURRET_RETURN_MARGIN_DEG = 5; // begin returning BEFORE the soft limit
    public static final double TURRET_RETURN_TOLERANCE_DEG = 1;
    public static final double TURRET_AIM_OFFSET_DEG = 0.0;    // tune: nudge if shots always miss to one side
    public static final double TURRET_VISION_POWER_SIGN = -1.0;//h flipped camera/motor feedback; +1 restores prior sign
    public static final int TURRET_AIM_TAG_ID = -1;           // -1 = group mean; e.g. 38 = center tag 38 only
    public static final double AIM_TOLERANCE_DEG = 3.0;        // gate won't open until aimed this close

    // ---------------------------------------------------------------------------------------------
    // Shooter: ServoCity 5000 Series, 28 ticks per rev, 20T to 20T GT2 belt (1:1), so these RPMs are
    // flywheel RPMs. 72 mm Rhino wheel plus two 60 mm steel flywheels (551 g*cm^2 each).
    // Rhino surface speed = pi * 0.072 m * rpm / 60, about 11.3 m/s at 3000 rpm.
    // ---------------------------------------------------------------------------------------------
    public static final double SHOOTER_TICKS_PER_REV = 28;
    public static final double RPM_YELLOW = 3000;              // tune: POLLEN
    public static final double RPM_RED = 3400;                 // tune: red NECTAR
    public static final double RPM_BLUE = 3400;                // tune: blue NECTAR
    public static final double SHOOTER_RPM_TOLERANCE = 100;    // gate won't open until this close to target
    public static final double RPM_TRIM_STEP = 50;
    // Starting values from the FTC motor PIDF tuning guide: F = 32767 / max ticks per second.
    public static final double SHOOTER_F = 32767.0 / (6000 * SHOOTER_TICKS_PER_REV / 60.0);
    public static final double SHOOTER_P = 0.1 * SHOOTER_F;
    public static final double SHOOTER_I = 0.1 * SHOOTER_P;
    public static final double SHOOTER_D = 0;

    // ---------------------------------------------------------------------------------------------
    // Gate, intake, transmission
    // ---------------------------------------------------------------------------------------------
    public static final double GATE_CLOSED = 0.0;              // tune: find it with the Robot Test OpMode
    public static final double GATE_OPEN = 0.14;
    // tune: find it with the Robot Test OpMode
    public static final long FEED_TIME_MS = 300;               // tune: gate open time for one ball
    public static final double INTAKE_POWER = 1.0;
    public static final double TRANSMISSION_INDEX_POWER = 0.6; // moving balls up to the gate
    public static final double TRANSMISSION_FEED_POWER = 1.0;  // pushing a ball through the open gate

    // ---------------------------------------------------------------------------------------------
    // Color sensor. Hold each ball in front of it and read "hue" on telemetry to tune these.
    // ---------------------------------------------------------------------------------------------
    public static final float COLOR_GAIN = 4;
    public static final double BALL_PRESENT_CM = 4.0;
    public static final double RED_HUE_ABOVE = 330;            // red wraps around 0
    public static final double RED_HUE_BELOW = 35;
    public static final double YELLOW_HUE_MIN = 50;
    public static final double YELLOW_HUE_MAX = 110;
    public static final double BLUE_HUE_MIN = 190;
    public static final double BLUE_HUE_MAX = 260;

    // ---------------------------------------------------------------------------------------------
    // goBILDA RGB Indicator Light servo positions
    // ---------------------------------------------------------------------------------------------
    public static final double LED_OFF = 0.0;
    public static final double LED_RED = 0.277;
    public static final double LED_YELLOW = 0.388;
    public static final double LED_GREEN = 0.500;
    public static final double LED_BLUE = 0.611;
    public static final double LED_VIOLET = 0.722;
    public static final double LED_WHITE = 1.0;

    // ---------------------------------------------------------------------------------------------
    // Limelight / BIOBUZZ AprilTags
    // ---------------------------------------------------------------------------------------------
    // This slot must be an AprilTag pipeline (family 36h11), set up in the Limelight web UI.
    public static final int LIMELIGHT_PIPELINE = 4;
    // Each HIVE has 2 CELLS, each with a cluster of 4 tags on its bottom, facing down at the field.
    // Only the CELL that's facing up shows its tags to the robots, so seeing one of our tags means
    // our HIVE has an open CELL above us.
    //   Red:  30-33 (scoring side CELL), 34-37 (audience side CELL)
    //   Blue: 38-41 (audience side CELL), 42-45 (scoring side CELL)
    public static final int RED_TAG_MIN = 30;
    public static final int RED_TAG_MAX = 37;
    public static final int BLUE_TAG_MIN = 38;
    public static final int BLUE_TAG_MAX = 45;
    public static final long TAG_HOLD_MS = 100;                // rides through a dropped frame or two
    public static final long LIMELIGHT_STALE_MS = 250;         // older results count as "no tag"

    // ---------------------------------------------------------------------------------------------
    // Hardware. led, colorSensor and Limelight are null when they aren't in the config.
    // ---------------------------------------------------------------------------------------------
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public DcMotorEx intake, transmission, turret, shooter;
    public Servo gate, led;
    public NormalizedColorSensor colorSensor;
    public DistanceSensor ballDistanceSensor;
    public Limelight3A Limelight;

    // ---------------------------------------------------------------------------------------------
    // State, refreshed by updateSensors() / the update methods each loop
    // ---------------------------------------------------------------------------------------------
    public Alliance alliance = lastAlliance;

    public boolean targetVisible;
    public boolean targetFresh;         // no matching tag in the latest usable result = false
    public double targetTx;              // degrees, + means the CELL is right of the crosshair
    public int targetId = -1;
    private boolean tagEverSeen;
    private final ElapsedTime sinceTagSeen = new ElapsedTime();
    private final TurretAimController turretAim =
            new TurretAimController(TURRET_MIN_DEG, TURRET_MAX_DEG);
    private String turretAimState = "MANUAL";
    private double turretRequestedPower, turretAppliedPower;
    private String turretPowerLimit = "none";
    private boolean visionConnected, visionRunning, visionEverConnected, visionResultValid;
    private boolean visionPipelineSwitchOk;
    private long visionResultAgeMs = -1;
    private int visionPipelineIndex = -1;
    private String visionPipelineType = "unknown";
    private int visionRawTagCount, visionMatchingIds, visionUsableMatches;
    private String visionTagDetails = "none";
    private String visionDebug = "Waiting for first update";

    public boolean ballPresent;
    public BallColor ballColor = BallColor.NONE;
    public double ballHue;
    public double ballDistanceCm;
    private final float[] hsv = new float[3];

    public boolean shooterOn;
    public BallColor shotColor = BallColor.YELLOW;   // the ball the shooter is set up for
    public double rpmTrim;
    public double targetRpm = RPM_YELLOW;

    public boolean gateOpen;
    public int shotsFired;
    private boolean feedingBall;
    private final ElapsedTime feedTimer = new ElapsedTime();

    /**
     * Never moves anything: motors get no power and servos get no position.
     *
     * @param resetEncoders true in Auto (turret must be pointing straight forward). TeleOp passes
     *                      false so it keeps the turret zero from Auto.
     */
    public void init(HardwareMap hw, boolean resetEncoders) {
        for (LynxModule hub : hw.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        frontLeft = hw.get(DcMotorEx.class, "frontleft");
        frontRight = hw.get(DcMotorEx.class, "frontright");
        backLeft = hw.get(DcMotorEx.class, "backleft");
        backRight = hw.get(DcMotorEx.class, "backright");
        intake = hw.get(DcMotorEx.class, "intake");
        transmission = hw.get(DcMotorEx.class, "transmission");
        turret = hw.get(DcMotorEx.class, "turret");
        shooter = hw.get(DcMotorEx.class, "shooter");

        // The 20T->16T stage is belted, so each wheel spins the same way as its motor. If pushing
        // forward doesn't drive forward, flip these.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        for (DcMotorEx m : new DcMotorEx[] {frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (resetEncoders) m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Positive intake / transmission power should move balls toward the shooter.
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transmission.setDirection(DcMotorSimple.Direction.REVERSE);
        transmission.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transmission.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Positive turret power must turn the turret right (clockwise seen from above), the same way
        // a positive Limelight tx points. If the turret runs away from the tag, flip this.
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (resetEncoders) turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);

        // A servo stays unpowered (and doesn't move) until its first setPosition(), so none here.
        gate = hw.get(Servo.class, "torqueservo1");
        led = hw.tryGet(Servo.class, "led");

        colorSensor = hw.tryGet(NormalizedColorSensor.class, "colorSensor");
        if (colorSensor != null) {
            colorSensor.setGain(COLOR_GAIN);
            if (colorSensor instanceof DistanceSensor) {
                ballDistanceSensor = (DistanceSensor) colorSensor;
            }
        }

        Limelight = hw.tryGet(Limelight3A.class, "Limelight");
        if (Limelight != null) {
            Limelight.setPollRateHz(100);
            visionPipelineSwitchOk = Limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
            Limelight.start();
        }

        alliance = lastAlliance;
    }

    /** Call once at the top of every loop, before using any sensor values. */
    public void updateSensors() {
        updateVision();
        updateBall();
    }

    private void updateVision() {
        visionRunning = Limelight != null && Limelight.isRunning();
        visionConnected = Limelight != null && Limelight.isConnected();
        visionEverConnected |= visionConnected;
        LLResult result = Limelight != null ? Limelight.getLatestResult() : null;
        visionResultValid = result != null && result.isValid();
        visionResultAgeMs = result != null && visionEverConnected ? result.getStaleness() : -1;
        visionPipelineIndex = result != null && visionEverConnected ? result.getPipelineIndex() : -1;
        visionPipelineType = result != null && visionEverConnected ? result.getPipelineType() : "unknown";
        visionRawTagCount = 0;
        visionMatchingIds = 0;
        visionUsableMatches = 0;
        StringBuilder tags = new StringBuilder();
        double txSum = 0;
        int firstMatchingId = -1;

        // Inspect ALL raw detections before applying the aim filter. Seeing a different ID is
        // different from the camera seeing nothing. These details may be stale; age is shown too.
        if (result != null && result.getFiducialResults() != null) {
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                visionRawTagCount++;
                if (visionRawTagCount <= 8) {
                    if (tags.length() > 0) tags.append("; ");
                    tags.append(tag.getFiducialId()).append(" tx=")
                            .append(String.format(java.util.Locale.US, "%.1f", tag.getTargetXDegrees()));
                }
                if (isOurHiveTag(tag.getFiducialId())) {
                    visionMatchingIds++;
                    if (Double.isFinite(tag.getTargetXDegrees())) {
                        txSum += tag.getTargetXDegrees();
                        visionUsableMatches++;
                        if (firstMatchingId < 0) firstMatchingId = tag.getFiducialId();
                    }
                }
            }
        }
        if (visionRawTagCount > 8) tags.append("; ...");
        visionTagDetails = tags.length() > 0 ? tags.toString() : "none";
        visionDebug = visionDiagnosis(Limelight != null, visionRunning, visionConnected,
                visionResultAgeMs, visionPipelineIndex, visionResultValid,
                visionRawTagCount, visionMatchingIds, visionUsableMatches);
        boolean seenNow = "MATCHING TAG VISIBLE".equals(visionDebug);
        if (seenNow) {
            targetTx = txSum / visionUsableMatches;
            targetId = firstMatchingId;
            tagEverSeen = true;
            sinceTagSeen.reset();
        }
        targetFresh = seenNow;
        targetVisible = seenNow || (visionConnected && visionRunning
                && visionPipelineIndex == LIMELIGHT_PIPELINE
                && tagEverSeen && sinceTagSeen.milliseconds() < TAG_HOLD_MS);
        if (!targetVisible) targetId = -1;
    }

    static String visionDiagnosis(boolean configured, boolean running, boolean connected,
                                  long ageMs, int pipeline, boolean valid,
                                  int rawTags, int matchingIds, int usableMatches) {
        if (!configured) return "MISSING: hardware config name must be Limelight";
        if (!running) return "POLLING NOT RUNNING / STARTING";
        if (!connected) return "NO CAMERA UPDATES: check USB/power/config";
        if (ageMs < 0) return "NO RESULT RECEIVED";
        if (ageMs >= LIMELIGHT_STALE_MS) return "STALE RESULT: not used for aiming";
        if (pipeline != LIMELIGHT_PIPELINE) return "WRONG PIPELINE: expected " + LIMELIGHT_PIPELINE;
        if (rawTags == 0) return "NO APRILTAGS: check pipeline type, tag family and camera view";
        if (!valid) return "TAGS REPORTED BUT RESULT INVALID";
        if (matchingIds == 0) return "TAGS SEEN BUT FILTERED OUT: check alliance / tag IDs";
        if (usableMatches == 0) return "MATCHING TAG OFFSET INVALID";
        return "MATCHING TAG VISIBLE";
    }

    /** Call in INIT and active loops. Raw camera visibility is separate from aim acceptance. */
    public void addVisionTelemetry(Telemetry telemetry) {
        telemetry.addData("Robot code", CODE_VERSION);
        telemetry.addData("Limelight", "configured=%s connected=%s polling=%s",
                Limelight != null, visionConnected, visionRunning);
        telemetry.addData("LL pipeline", "requested=%d actual=%d type=%s switch request=%s",
                LIMELIGHT_PIPELINE, visionPipelineIndex, visionPipelineType,
                visionPipelineSwitchOk ? "OK" : "FAILED / not configured");
        telemetry.addData("LL result", "valid=%s age=%s", visionResultValid,
                visionResultAgeMs < 0 ? "no updates received" : visionResultAgeMs + " ms");
        boolean rawVisible = visionConnected && visionRunning && visionResultAgeMs >= 0
                && visionResultAgeMs < LIMELIGHT_STALE_MS && visionRawTagCount > 0;
        telemetry.addData("AprilTag visible (raw)", rawVisible ? "YES" : "NO");
        telemetry.addData("Raw tags (last result)", "%d: %s", visionRawTagCount, visionTagDetails);
        telemetry.addData("Aim filter", "%s IDs %d-%d; %d matching", alliance,
                alliance == Alliance.RED ? RED_TAG_MIN : BLUE_TAG_MIN,
                alliance == Alliance.RED ? RED_TAG_MAX : BLUE_TAG_MAX, visionMatchingIds);
        telemetry.addData("Aim target visible", targetFresh ? "YES - fresh"
                : targetVisible ? "HELD briefly - no fresh match" : "NO");
        telemetry.addData("Aim measurement", "%s; tx=%.2f offset=%.2f error=%.2f deg%s",
                aimTargetLabel(), targetTx, TURRET_AIM_OFFSET_DEG,
                targetTx - TURRET_AIM_OFFSET_DEG, targetFresh ? "" : " (last measurement)");
        telemetry.addData("Turret command", "%s requested=%+.2f applied=%+.2f limit=%s",
                turretAimState, turretRequestedPower, turretAppliedPower, turretPowerLimit);
        telemetry.addData("Vision power sign", TURRET_VISION_POWER_SIGN);
        telemetry.addData("Vision debug", visionDebug);
    }

    private boolean isOurHiveTag(int id) {
        if (TURRET_AIM_TAG_ID >= 0 && id != TURRET_AIM_TAG_ID) return false;
        if (alliance == Alliance.RED) return id >= RED_TAG_MIN && id <= RED_TAG_MAX;
        return id >= BLUE_TAG_MIN && id <= BLUE_TAG_MAX;
    }

    private void updateBall() {
        if (colorSensor == null) {
            ballPresent = false;
            ballColor = BallColor.NONE;
            return;
        }
        Color.colorToHSV(colorSensor.getNormalizedColors().toColor(), hsv);
        ballHue = hsv[0];

        BallColor hueColor;
        if (ballHue >= RED_HUE_ABOVE || ballHue <= RED_HUE_BELOW) hueColor = BallColor.RED;
        else if (ballHue >= YELLOW_HUE_MIN && ballHue <= YELLOW_HUE_MAX) hueColor = BallColor.YELLOW;
        else if (ballHue >= BLUE_HUE_MIN && ballHue <= BLUE_HUE_MAX) hueColor = BallColor.BLUE;
        else hueColor = BallColor.NONE;

        if (ballDistanceSensor != null) {
            ballDistanceCm = ballDistanceSensor.getDistance(DistanceUnit.CM);
            ballPresent = ballDistanceCm < BALL_PRESENT_CM;
        } else {
            ballPresent = hueColor != BallColor.NONE;
        }
        ballColor = ballPresent ? hueColor : BallColor.NONE;
    }

    // ---------------------------------------------------------------------------------------------
    // Drive
    // ---------------------------------------------------------------------------------------------

    /** Robot-centric mecanum drive. All inputs are -1 to 1; + is forward, right, and clockwise. */
    public void drive(double forward, double strafe, double turn) {
        strafe *= STRAFE_MULTIPLIER;
        double fl = -1.0*(forward + strafe + turn);
        double fr = forward - strafe - turn;
        double bl = -1.0*(forward - strafe + turn);
        double br = forward + strafe - turn;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                                            Math.max(Math.abs(bl), Math.abs(br))));
        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    // ---------------------------------------------------------------------------------------------
    // Turret
    // ---------------------------------------------------------------------------------------------

    public double turretDegrees() {
        return turret.getCurrentPosition() / TURRET_TICKS_PER_DEG;
    }

    /**
     * Centers the visible tag/group. When lost, searches toward its last seen side. At a travel
     * boundary, returns across the allowed range. A safely reachable fresh target interrupts a
     * return. Manual control cancels it too. Positive power and encoder angles must both turn right.
     */
    public void aimTurret(boolean autoAim, double manualPower) {
        double power;
        if (autoAim && Limelight != null) {
            if (!targetFresh && targetVisible && !turretAim.returning) {
                // A short dropped detection is not evidence that the tag moved to the right.
                // Pause through the existing TAG_HOLD_MS grace period before starting a search.
                power = 0;
                turretAimState = "WAIT FOR FRESH TAG";
            } else {
                power = turretAim.update(turretDegrees(), targetFresh,
                        (targetTx - TURRET_AIM_OFFSET_DEG) * TURRET_VISION_POWER_SIGN);
                turretAimState = turretAim.state;
            }
        } else {
            turretAim.cancelReturn();
            turretAimState = autoAim ? "NO CAMERA" : "MANUAL";
            power = manualPower;
        }
        setTurretPower(power);
    }

    /** Drives the turret toward an encoder angle (0 = straight forward). Call every loop. */
    public void turretToAngle(double degrees) {
        turretAim.cancelReturn();
        turretAimState = "CENTER / POSITION";
        double error = Range.clip(degrees, TURRET_MIN_DEG, TURRET_MAX_DEG) - turretDegrees();
        double power = Math.abs(error) < TURRET_DEADBAND_DEG
                ? 0
                : TURRET_KP * error + Math.copySign(TURRET_KS, error);
        setTurretPower(power);
    }

    private void setTurretPower(double power) {
        turretRequestedPower = Double.isFinite(power) ? power : 0;
        if (!Double.isFinite(power)) power = 0;
        power = Range.clip(power, -TURRET_MAX_POWER, TURRET_MAX_POWER);
        double deg = turretDegrees();
        turretPowerLimit = "none";
        if (deg >= TURRET_MAX_DEG && power > 0) {
            power = 0;
            turretPowerLimit = "RIGHT ENCODER LIMIT";
        } else if (deg <= TURRET_MIN_DEG && power < 0) {
            power = 0;
            turretPowerLimit = "LEFT ENCODER LIMIT";
        }
        turretAppliedPower = power;
        turret.setPower(power);
    }

    /** Re-zero the turret. Only do this with the turret pointing straight forward. */
    public void zeroTurret() {
        turretAim.cancelReturn();
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean aimedAtTarget() {
        return !turretAim.returning && targetVisible
                && Math.abs(targetTx - TURRET_AIM_OFFSET_DEG) < AIM_TOLERANCE_DEG;
    }

    public String turretAimState() {
        return turretAimState;
    }

    public String aimTargetLabel() {
        return TURRET_AIM_TAG_ID >= 0 ? "TAG " + TURRET_AIM_TAG_ID : "MATCHING TAG GROUP MEAN";
    }

    /** Encoder/vision decision logic, kept independent of hardware so boundary cases can be tested. */
    static final class TurretAimController {
        private final double minDeg, maxDeg;
        private int searchDirection = 1; // no previous sighting: start searching right
        private boolean returning;
        private double returnGoal;
        private String state = "SEARCH RIGHT";

        TurretAimController(double minDeg, double maxDeg) {
            double span = maxDeg - minDeg;
            if (!Double.isFinite(minDeg) || !Double.isFinite(maxDeg)
                    || minDeg > 0 || maxDeg < 0 || span > 360
                    || span <= 2 * (TURRET_RETURN_MARGIN_DEG + TURRET_RETURN_TOLERANCE_DEG)) {
                throw new IllegalArgumentException("Turret range must include zero, span at most "
                        + "360 degrees, and leave room for the return margins.");
            }
            this.minDeg = minDeg;
            this.maxDeg = maxDeg;
        }

        double update(double angle, boolean visible, double error) {
            if (!Double.isFinite(angle) || (visible && !Double.isFinite(error))) {
                cancelReturn();
                state = "INVALID READING";
                return 0;
            }

            // The camera error estimates how far to turn from the current encoder angle.
            // Do not rewind merely because a reachable target is near one end of the range.
            double targetAngle = angle + error;
            boolean reachable = visible && angle >= minDeg && angle <= maxDeg
                    && targetAngle >= minDeg && targetAngle <= maxDeg;
            if (returning && reachable) returning = false;
            if (returning) {
                if (Math.abs(returnGoal - angle) > TURRET_RETURN_TOLERANCE_DEG) {
                    return limited(angle, positionPower(returnGoal - angle,
                            TURRET_RETURN_MAX_POWER));
                }
                returning = false;
            }

            double power;
            if (visible) {
                if (Math.abs(error) >= TURRET_DEADBAND_DEG) {
                    searchDirection = error > 0 ? 1 : -1;
                }
                power = positionPower(error, TURRET_MAX_POWER);
                state = "TRACK";
            } else {
                power = searchDirection * TURRET_SEARCH_POWER;
                state = searchDirection > 0 ? "SEARCH RIGHT" : "SEARCH LEFT";
            }

            if (angle > maxDeg || (!reachable && power > 0
                    && angle >= maxDeg - TURRET_RETURN_MARGIN_DEG)) {
                returnGoal = minDeg + TURRET_RETURN_MARGIN_DEG;
                returning = true;
                state = "RETURN LEFT";
            } else if (angle < minDeg || (!reachable && power < 0
                    && angle <= minDeg + TURRET_RETURN_MARGIN_DEG)) {
                returnGoal = maxDeg - TURRET_RETURN_MARGIN_DEG;
                returning = true;
                state = "RETURN RIGHT";
            }
            if (returning) power = positionPower(returnGoal - angle, TURRET_RETURN_MAX_POWER);
            return limited(angle, power);
        }

        void cancelReturn() {
            returning = false;
        }

        private double positionPower(double error, double maxPower) {
            if (Math.abs(error) < TURRET_DEADBAND_DEG) return 0;
            double power = TURRET_KP * error + Math.copySign(TURRET_KS, error);
            return Math.max(-maxPower, Math.min(maxPower, power));
        }

        private double limited(double angle, double power) {
            if ((angle >= maxDeg && power > 0) || (angle <= minDeg && power < 0)) return 0;
            return Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, power));
        }
    }

    // ---------------------------------------------------------------------------------------------
    // Shooter
    // ---------------------------------------------------------------------------------------------

    public static double rpmFor(BallColor color) {
        switch (color) {
            case RED:  return RPM_RED;
            case BLUE: return RPM_BLUE;
            default:   return RPM_YELLOW;
        }
    }

    /** Sets the flywheel speed for whichever ball is waiting at the gate. */
    public void updateShooter() {
        // Only switch while the gate is closed, so the ball being fired keeps its speed.
        // With no ball at the gate, stay set up for the last ball instead of spinning down.
        if (!gateOpen && ballColor != BallColor.NONE) shotColor = ballColor;
        targetRpm = rpmFor(shotColor);

        if (shooterOn) {
            setShooterRpm(targetRpm + rpmTrim);
        } else {
            shooter.setPower(0);
        }
    }

    public void setShooterRpm(double rpm) {
        shooter.setVelocity(rpm * SHOOTER_TICKS_PER_REV / 60.0);
    }

    public double shooterRpm() {
        return shooter.getVelocity() * 60.0 / SHOOTER_TICKS_PER_REV;
    }

    public boolean shooterAtSpeed() {
        return shooterOn && Math.abs(shooterRpm() - (targetRpm + rpmTrim)) < SHOOTER_RPM_TOLERANCE;
    }

    public boolean readyToShoot() {
        return aimedAtTarget() && shooterAtSpeed();
    }

    // ---------------------------------------------------------------------------------------------
    // Gate, transmission, intake
    // ---------------------------------------------------------------------------------------------

    /**
     * Runs the gate, transmission and intake. The gate is never open unless our HIVE tag is visible.
     * Balls go one at a time: wait until aimed and at the speed for that ball's color, open the gate
     * for FEED_TIME_MS, close it, and start over for the next ball.
     *
     * @param wantShoot   driver is holding shoot (or Auto wants to fire)
     * @param intakePower + pulls balls in, - spits them back out, 0 stops
     */
    public void updateFeeder(boolean wantShoot, double intakePower) {
        if (!targetVisible) {
            gateOpen = false;
        } else if (gateOpen) {
            if (feedTimer.milliseconds() > FEED_TIME_MS) {
                gateOpen = false;
                if (feedingBall) shotsFired++;
            }
        } else if (wantShoot && readyToShoot()) {
            gateOpen = true;
            feedingBall = ballPresent;
            feedTimer.reset();
        }
        gate.setPosition(gateOpen ? GATE_OPEN : GATE_CLOSED);

        updateFeederMotors(gateOpen, wantShoot, intakePower);
    }

    /** Manual TeleOp gate toggle; no vision, readiness check, or automatic closing timer. */
    public void updateManualFeeder(boolean toggleGate, double intakePower) {
        if (toggleGate) gateOpen = !gateOpen;
        gate.setPosition(gateOpen ? GATE_OPEN : GATE_CLOSED);
        updateFeederMotors(false, false, intakePower);
    }

    private void updateFeederMotors(boolean feedThroughGate, boolean wantShoot, double intakePower) {
        double transmissionPower;
        if (feedThroughGate) {
            transmissionPower = TRANSMISSION_FEED_POWER;
        } else if (intakePower < 0) {
            transmissionPower = -TRANSMISSION_INDEX_POWER;
        } else if ((intakePower > 0 || wantShoot) && !ballPresent) {
            // Move balls up until one is waiting at the gate, then stop so it isn't squished.
            transmissionPower = TRANSMISSION_INDEX_POWER;
        } else {
            transmissionPower = 0;
        }
        transmission.setPower(transmissionPower);
        intake.setPower(intakePower);
    }

    // ---------------------------------------------------------------------------------------------
    // LED
    // ---------------------------------------------------------------------------------------------

    /**
     * Green: firing or ready to fire.  Yellow/red/blue: that ball is at the gate.
     * White: something is at the gate but the color is unclear.  Violet: our HIVE tag is in view.
     * Off: nothing to report.
     */
    public void updateLed() {
        if (led == null) return;
        double color;
        if (gateOpen || (ballPresent && readyToShoot())) color = LED_GREEN;
        else if (ballColor == BallColor.YELLOW) color = LED_YELLOW;
        else if (ballColor == BallColor.RED) color = LED_RED;
        else if (ballColor == BallColor.BLUE) color = LED_BLUE;
        else if (ballPresent) color = LED_WHITE;
        else if (targetVisible) color = LED_VIOLET;
        else color = LED_OFF;
        led.setPosition(color);
    }

    /** Stops every motor. Servos are left where they are. */
    public void stop() {
        drive(0, 0, 0);
        turret.setPower(0);
        shooter.setPower(0);
        intake.setPower(0);
        transmission.setPower(0);
        if (Limelight != null) Limelight.stop();
    }
}
