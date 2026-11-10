
package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.content.SharedPreferences;
import android.graphics.Color;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.function.DoubleSupplier;

public class Robot {

    public static final String CODE_VERSION =
            "BIOBUZZ-GATE-CALIBRATION-2026-11-08";

    public enum Alliance { RED, BLUE }
    public enum BallColor { NONE, YELLOW, RED, BLUE }

    public static Alliance lastAlliance = Alliance.RED;

    // =========================================
    // DRIVETRAIN
    // =========================================

    public static final double DRIVE_TICKS_PER_MOTOR_REV = 384.5;
    public static final double DRIVE_GEAR_RATIO = 20.0 / 16.0;
    public static final double WHEEL_DIAMETER_IN = 96.0 / 25.4;

    public static final double DRIVE_TICKS_PER_INCH =
            DRIVE_TICKS_PER_MOTOR_REV /
                    (DRIVE_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_IN);

    public static final double STRAFE_MULTIPLIER = 1.1;

    // =========================================
    // TURRET
    // =========================================

    // 1150 RPM Yellow Jacket: 28 encoder counts times its exact internal gearbox ratio.
    public static final double TURRET_MOTOR_TICKS_PER_REV = 28.0 * (1.0 + 46.0 / 11.0);
    public static final int TURRET_MOTOR_GEAR_TEETH = 12;
    public static final int TURRET_GEAR_TEETH = 84;
    public static final double TURRET_MOTOR_REVS_PER_TURRET_REV =
            (double) TURRET_GEAR_TEETH / TURRET_MOTOR_GEAR_TEETH;
    public static final double TURRET_TICKS_PER_TURRET_REV =
            TURRET_MOTOR_TICKS_PER_REV * TURRET_MOTOR_REVS_PER_TURRET_REV;
    public static final double TURRET_TICKS_PER_DEG = TURRET_TICKS_PER_TURRET_REV / 360.0;

    // Measured physical endpoints, clockwise-positive from the forward zero.
    public static final int TURRET_MEASURED_MIN_TICKS = -724;
    public static final int TURRET_MEASURED_MAX_TICKS = 563;
    public static final double TURRET_LIMIT_CLEARANCE_DEG = 5.0;
    public static final int TURRET_LIMIT_CLEARANCE_TICKS =
            (int) Math.ceil(TURRET_LIMIT_CLEARANCE_DEG * TURRET_TICKS_PER_DEG);
    public static final int TURRET_MIN_TICKS = TURRET_MEASURED_MIN_TICKS + TURRET_LIMIT_CLEARANCE_TICKS;
    public static final int TURRET_MAX_TICKS = TURRET_MEASURED_MAX_TICKS - TURRET_LIMIT_CLEARANCE_TICKS;
    public static final double TURRET_MIN_DEG = TURRET_MIN_TICKS / TURRET_TICKS_PER_DEG;
    public static final double TURRET_MAX_DEG = TURRET_MAX_TICKS / TURRET_TICKS_PER_DEG;

    public static final double TURRET_KP = 0.02;
    public static final double TURRET_KD = 0.0;
    public static final double TURRET_KS = 0.04;
    public static final double TURRET_DEADBAND_DEG = 1.0;

    public static final double TURRET_MAX_POWER = 0.6;
    public static final double TURRET_SEARCH_POWER = 0.15;
    public static final double TURRET_TRACK_MAX_POWER = 0.22;
    // Initial velocity feedforward from nominal no-load turret speed; tune on the robot.
    public static final double TURRET_KV = 1.0 / (1150.0 * 6.0 / TURRET_MOTOR_REVS_PER_TURRET_REV);
    public static final double TURRET_ROTATION_DEADBAND_DEG_S = 2.0;
    public static final long TURRET_REACQUIRE_MS = 400;
    public static final double TURRET_RETURN_MAX_POWER = 0.35;
    public static final double TURRET_RETURN_MARGIN_DEG = 5;
    public static final double TURRET_RETURN_TOLERANCE_DEG = 1;

    public static final double TURRET_AIM_OFFSET_DEG = 0.0;

    // Preserve your latest camera/motor sign correction.
    public static final double TURRET_VISION_POWER_SIGN = -1.0;

    // -1 automatically locks one matching tag; it does not average a changing tag group.
    public static final int TURRET_AIM_TAG_ID = -1;
    public static final long TURRET_TAG_SWITCH_DELAY_MS = 250;

    public static final double AIM_TOLERANCE_DEG = 3.0;

    // =========================================
    // SHOOTER - 4000 RPM, TUNED PIDF
    // =========================================

    // ServoCity 5000 Series
    // 28 ticks/revolution, 1:1 belt ratio

    public static final double SHOOTER_TICKS_PER_REV = 28.0;

    public static final double RPM_YELLOW = 4000.0;
    public static final double RPM_RED = 4000.0;
    public static final double RPM_BLUE = 4000.0;

    public static final double TELEOP_DEFAULT_RPM = 3150.0;
    private static final double[] SHOOTER_DISTANCE_METERS = {1.41, 1.60, 1.76, 1.90};
    private static final double[] SHOOTER_DISTANCE_RPM = {3150, 3150, 3200, 3250};

    public static final double SHOOTER_RPM_TOLERANCE = 100.0;
    public static final double RPM_TRIM_STEP = 50.0;

    public static final double SHOOTER_P = 180.0;
    public static final double SHOOTER_I = 0.0;
    public static final double SHOOTER_D = 0.0;
    public static final double SHOOTER_F = 13.83;

    // =========================================
    // GATE / INTAKE / TRANSMISSION
    // =========================================

    // Measured on the redesigned gate. Saved recalibration may override this pair.
    public static final double GATE_OPEN = 0.41;
    public static final double GATE_CLOSED = 0.59;
    public static final String GATE_PREFERENCES = "biobuzz_gate_model_v2";
    public static final double GATE_JOG_STEP = 0.002;
    public static final double GATE_CALIBRATION_STEP = 0.01;

    public static final long FEED_TIME_MS = 300;

    public static final double INTAKE_POWER = 1.0;
    public static final double TRANSMISSION_INDEX_POWER = 0.6;
    public static final double TRANSMISSION_FEED_POWER = 1.0;

    // =========================================
    // COLOR SENSOR
    // =========================================

    public static final float COLOR_GAIN = 4;

    // Starting threshold: verify with pollen at the installed sensor position.
    public static final double BALL_PRESENT_CM = 4.0;

    public static final double RED_HUE_ABOVE = 330;
    public static final double RED_HUE_BELOW = 35;

    public static final double YELLOW_HUE_MIN = 50;
    public static final double YELLOW_HUE_MAX = 110;

    public static final double BLUE_HUE_MIN = 190;
    public static final double BLUE_HUE_MAX = 260;

    // =========================================
    // LED
    // =========================================

    // goBILDA RGB Indicator 3118-0808-0002, standard FTC Servo PWM range.
    public static final double LED_OFF = 0.0;
    // 1104 us: just above the light's 1100 us minimum for red (below it is OFF).
    public static final double LED_RED = 0.28;
    public static final double LED_YELLOW = 0.388;
    public static final double LED_GREEN = 0.500;
    public static final double LED_BLUE = 0.611;
    public static final double LED_VIOLET = 0.722;
    public static final double LED_WHITE = 1.0;

    // =========================================
    // LIMELIGHT
    // =========================================

    public static final int LIMELIGHT_PIPELINE = 4;

    public static final int RED_TAG_MIN = 30;
    public static final int RED_TAG_MAX = 37;

    public static final int BLUE_TAG_MIN = 38;
    public static final int BLUE_TAG_MAX = 45;

    public static final long TAG_HOLD_MS = 100;
    public static final long LIMELIGHT_STALE_MS = 250;

    // =========================================
    // HARDWARE
    // =========================================

    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public DcMotorEx intake, transmission, turret, shooter;

    public Servo gate, led;
    public GateController gateController;

    public NormalizedColorSensor colorSensor;
    public DistanceSensor ballDistanceSensor;

    // Capital L matches your latest hardware config.
    public Limelight3A Limelight;
    private GoBildaPinpointDriver pinpoint;
    // Pinpoint is CCW-positive; turret encoders and controller angles are CW-positive.
    private double chassisHeadingDegrees = Double.NaN;
    private double chassisTurnRateDegrees = Double.NaN;
    private String headingStatus = "MISSING: configure Pinpoint as pinpoint";

    // =========================================
    // ROBOT STATE
    // =========================================

    public Alliance alliance = lastAlliance;

    public boolean targetVisible;
    public boolean targetFresh;
    public double targetTx = Double.NaN;
    private double targetFrameTimestamp;
    private double targetFrameAgeMs;
    // Straight-line camera-to-tag range; NaN means no fresh valid pose.
    public double targetDistanceMeters = Double.NaN;
    public int targetDistanceTagId = -1;
    // Camera position relative to the tag's outward normal, in the tag's X/Z plane.
    public double targetFaceAngleDegrees = Double.NaN;
    public int targetId = -1;

    private boolean tagEverSeen;

    private final ElapsedTime sinceTagSeen = new ElapsedTime();
    private final AimTagLock aimTagLock = new AimTagLock();
    private Alliance visionAlliance;

    public final TurretGains turretGains = new TurretGains();

    private final TurretAimController turretAim =
            new TurretAimController(
                    TURRET_MIN_DEG,
                    TURRET_MAX_DEG,
                    turretGains
            );

    private String turretAimState = "MANUAL";

    private double turretRequestedPower;
    private double turretAppliedPower;
    private String turretPowerLimit = "none";

    private boolean visionConnected;
    private boolean visionRunning;
    private boolean visionEverConnected;
    private boolean visionResultValid;
    private boolean visionPipelineSwitchOk;

    private long visionResultAgeMs = -1;
    private int visionPipelineIndex = -1;

    private String visionPipelineType = "unknown";

    private int visionRawTagCount;
    private int visionMatchingIds;
    private int visionUsableMatches;

    private String visionTagDetails = "none";
    private String visionDebug = "Waiting for first update";

    public boolean ballPresent;
    public BallColor ballColor = BallColor.NONE;

    public double ballHue;
    public double ballDistanceCm = Double.NaN;

    private final float[] hsv = new float[3];

    public boolean shooterOn = false;

    public BallColor shotColor = BallColor.YELLOW;

    public double rpmTrim = 0.0;
    public double targetRpm = RPM_YELLOW;

    public boolean gateOpen = false;
    public int shotsFired = 0;

    private boolean feedingBall;

    private final ElapsedTime feedTimer = new ElapsedTime();

    // =========================================
    // INITIALIZATION
    // =========================================

    public void init(HardwareMap hw, boolean resetEncoders) {

        for (LynxModule hub : hw.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(
                    LynxModule.BulkCachingMode.AUTO
            );
        }

        frontLeft = hw.get(DcMotorEx.class, "frontleft");
        frontRight = hw.get(DcMotorEx.class, "frontright");
        backLeft = hw.get(DcMotorEx.class, "backleft");
        backRight = hw.get(DcMotorEx.class, "backright");

        intake = hw.get(DcMotorEx.class, "intake");
        transmission = hw.get(DcMotorEx.class, "transmission");

        turret = hw.get(DcMotorEx.class, "turret");
        shooter = hw.get(DcMotorEx.class, "shooter");

        // Drivetrain

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[] {
                frontLeft, frontRight, backLeft, backRight
        }) {
            m.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.BRAKE
            );

            if (resetEncoders) {
                m.setMode(
                        DcMotor.RunMode.STOP_AND_RESET_ENCODER
                );
            }

            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Intake

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Transmission

        transmission.setDirection(DcMotorSimple.Direction.REVERSE);
        transmission.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transmission.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Turret

        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (resetEncoders) {
            turret.setMode(
                    DcMotor.RunMode.STOP_AND_RESET_ENCODER
            );
        }

        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Read the existing localizer without resetting the Auto-to-TeleOp heading/pose.
        pinpoint = hw.tryGet(GoBildaPinpointDriver.class, "pinpoint");

        // =====================================
        // SHOOTER: APPLY TUNED PIDF
        // =====================================

        shooter.setDirection(
                DcMotorSimple.Direction.FORWARD
        );

        shooter.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        shooter.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        shooter.setVelocityPIDFCoefficients(
                SHOOTER_P,
                SHOOTER_I,
                SHOOTER_D,
                SHOOTER_F
        );

        // =====================================
        // SERVOS / SENSORS
        // =====================================

        gate = hw.get(Servo.class, "torqueservo1");
        gateController = new GateController(gate,
                hw.appContext.getSharedPreferences(GATE_PREFERENCES, Context.MODE_PRIVATE));
        if (!gateController.calibrated()) gateController.release();
        led = hw.tryGet(Servo.class, "led");
        if (led != null) {
            led.setDirection(Servo.Direction.FORWARD);
            led.scaleRange(0, 1);
            if (led instanceof PwmControl) {
                ((PwmControl) led).setPwmRange(new PwmControl.PwmRange(600, 2400));
            }
        }
        ballPresent = false;
        updateLed();

        colorSensor = hw.tryGet(
                NormalizedColorSensor.class,
                "colorsensor"
        );
        // Preserve compatibility with the previous configuration name.
        if (colorSensor == null) {
            colorSensor = hw.tryGet(NormalizedColorSensor.class, "colorSensor");
        }
        ballDistanceSensor = null;

        if (colorSensor != null) {
            colorSensor.setGain(COLOR_GAIN);

            if (colorSensor instanceof DistanceSensor) {
                ballDistanceSensor = (DistanceSensor) colorSensor;
            }
        }

        Limelight = hw.tryGet(
                Limelight3A.class,
                "Limelight"
        );

        if (Limelight != null) {
            Limelight.setPollRateHz(100);

            visionPipelineSwitchOk =
                    Limelight.pipelineSwitch(LIMELIGHT_PIPELINE);

            Limelight.start();
        }

        alliance = lastAlliance;
    }

    // =========================================
    // SENSORS
    // =========================================

    public void updateSensors() {
        updateHeading();
        updateVision();
        updateBall();
        updateLed();
    }

    private void updateHeading() {
        chassisHeadingDegrees = Double.NaN;
        chassisTurnRateDegrees = Double.NaN;
        if (pinpoint == null) return;

        pinpoint.update();
        headingStatus = pinpoint.getDeviceStatus().toString();
        if (pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) return;

        double heading = -pinpoint.getHeading(UnnormalizedAngleUnit.DEGREES);
        double turnRate = -pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        if (Double.isFinite(heading) && Double.isFinite(turnRate)) {
            chassisHeadingDegrees = heading;
            chassisTurnRateDegrees = turnRate;
        } else {
            headingStatus = "INVALID HEADING";
        }
    }

    private void updateVision() {

        visionRunning =
                Limelight != null && Limelight.isRunning();

        visionConnected =
                Limelight != null && Limelight.isConnected();

        visionEverConnected |= visionConnected;

        LLResult result =
                Limelight != null
                        ? Limelight.getLatestResult()
                        : null;

        visionResultValid =
                result != null && result.isValid();

        visionResultAgeMs =
                result != null && visionEverConnected
                        ? result.getStaleness()
                        : -1;

        visionPipelineIndex =
                result != null && visionEverConnected
                        ? result.getPipelineIndex()
                        : -1;

        visionPipelineType =
                result != null && visionEverConnected
                        ? result.getPipelineType()
                        : "unknown";

        visionRawTagCount = 0;
        visionMatchingIds = 0;
        visionUsableMatches = 0;

        StringBuilder tags = new StringBuilder();

        targetDistanceMeters = Double.NaN;
        targetDistanceTagId = -1;
        targetFaceAngleDegrees = Double.NaN;
        if (visionAlliance != alliance) {
            aimTagLock.reset();
            turretAim.cancelReturn();
            visionAlliance = alliance;
            tagEverSeen = false;
        }
        aimTagLock.beginFrame();

        if (result != null
                && result.getFiducialResults() != null) {

            for (LLResultTypes.FiducialResult tag :
                    result.getFiducialResults()) {

                visionRawTagCount++;

                if (visionRawTagCount <= 8) {

                    if (tags.length() > 0) {
                        tags.append("; ");
                    }

                    tags.append(tag.getFiducialId())
                            .append(" tx=")
                            .append(String.format(
                                    java.util.Locale.US,
                                    "%.1f",
                                    tag.getTargetXDegrees()
                            ));
                }

                if (isOurHiveTag(tag.getFiducialId())) {

                    visionMatchingIds++;

                    if (Double.isFinite(
                            tag.getTargetXDegrees())) {

                        aimTagLock.consider(tag.getFiducialId(), tag.getTargetXDegrees());
                        visionUsableMatches++;
                    }
                }
            }
        }

        if (visionRawTagCount > 8) {
            tags.append("; ...");
        }

        visionTagDetails =
                tags.length() > 0
                        ? tags.toString()
                        : "none";

        visionDebug = visionDiagnosis(
                Limelight != null,
                visionRunning,
                visionConnected,
                visionResultAgeMs,
                visionPipelineIndex,
                visionResultValid,
                visionRawTagCount,
                visionMatchingIds,
                visionUsableMatches
        );

        boolean seenNow = "MATCHING TAG VISIBLE".equals(visionDebug);
        if (seenNow) {
            int selectedId = aimTagLock.finishFrame(System.nanoTime() / 1e6);
            if (selectedId < 0) {
                seenNow = false;
                visionDebug = "LOCKED TAG TEMPORARILY LOST: ID " + aimTagLock.lockedId;
            } else {
                targetId = selectedId;
                targetTx = aimTagLock.selectedTx;
            }
        }

        if (seenNow) {
            targetFrameTimestamp = result.getTimestamp();
            if (!Double.isFinite(targetFrameTimestamp) || targetFrameTimestamp == 0) {
                targetFrameTimestamp = result.getControlHubTimeStamp();
            }
            targetFrameAgeMs = visionResultAgeMs + result.getCaptureLatency()
                    + result.getTargetingLatency() + result.getParseLatency();
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (tag.getFiducialId() != targetId) continue;
                Pose3D cameraPose = tag.getTargetPoseCameraSpace();
                if (cameraPose != null) {
                    Position position = cameraPose.getPosition().toUnit(DistanceUnit.METER);
                    targetDistanceMeters = cameraToTagDistanceMeters(position.x, position.y, position.z);
                    if (Double.isFinite(targetDistanceMeters)) targetDistanceTagId = targetId;
                }
                Pose3D cameraInTagSpace = tag.getCameraPoseTargetSpace();
                if (cameraInTagSpace != null) {
                    Position position = cameraInTagSpace.getPosition().toUnit(DistanceUnit.METER);
                    targetFaceAngleDegrees = tagFaceAngleDegrees(position.x, position.z);
                }
                break;
            }

            tagEverSeen = true;
            sinceTagSeen.reset();
        } else {
            // A retained TX must not look like a live measurement (or authorize a shot).
            targetTx = Double.NaN;
        }

        targetFresh = seenNow;

        targetVisible = seenNow ||
                (
                        visionConnected
                                && visionRunning
                                && visionPipelineIndex == LIMELIGHT_PIPELINE
                                && tagEverSeen
                                && sinceTagSeen.milliseconds() < TAG_HOLD_MS
                );

        if (!targetVisible) {
            targetId = -1;
        }
    }

    static String visionDiagnosis(
            boolean configured,
            boolean running,
            boolean connected,
            long ageMs,
            int pipeline,
            boolean valid,
            int rawTags,
            int matchingIds,
            int usableMatches
    ) {

        if (!configured) {
            return "MISSING: hardware config name must be Limelight";
        }

        if (!running) {
            return "POLLING NOT RUNNING / STARTING";
        }

        if (!connected) {
            return "NO CAMERA UPDATES: check USB/power/config";
        }

        if (ageMs < 0) return "NO RESULT RECEIVED";

        if (ageMs >= LIMELIGHT_STALE_MS) {
            return "STALE RESULT: not used for aiming";
        }

        if (pipeline != LIMELIGHT_PIPELINE) {
            return "WRONG PIPELINE: expected " + LIMELIGHT_PIPELINE;
        }

        if (rawTags == 0) {
            return "NO APRILTAGS: check pipeline and camera view";
        }

        if (!valid) {
            return "TAGS REPORTED BUT RESULT INVALID";
        }

        if (matchingIds == 0) {
            return "TAGS SEEN BUT FILTERED OUT: check alliance / tag IDs";
        }

        if (usableMatches == 0) {
            return "MATCHING TAG OFFSET INVALID";
        }

        return "MATCHING TAG VISIBLE";
    }

    static double cameraToTagDistanceMeters(double x, double y, double z) {
        if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(z)) {
            return Double.NaN;
        }
        double distance = Math.hypot(Math.hypot(x, y), z);
        // The SDK returns an all-zero pose when the pipeline has no pose data.
        return Double.isFinite(distance) && distance > 0 ? distance : Double.NaN;
    }

    static double tagFaceAngleDegrees(double sideMeters, double tagSpaceZ) {
        // This camera's t6c_ts reports a camera in front of a visible tag at negative Z.
        // Use -Z as front-face depth; do not reject valid negative-Z camera poses.
        double frontDepthMeters = -tagSpaceZ;
        if (!Double.isFinite(sideMeters) || !Double.isFinite(frontDepthMeters)
                || frontDepthMeters <= 0) {
            return Double.NaN;
        }
        return Math.toDegrees(Math.atan2(sideMeters, frontDepthMeters));
    }

    public void addVisionTelemetry(Telemetry telemetry) {
        if (Double.isFinite(targetFaceAngleDegrees)) {
            telemetry.addData("LL tag face angle", "ID %d: %+.1f deg (0=straight on, +=right, -=left)",
                    targetDistanceTagId, targetFaceAngleDegrees);
        } else {
            telemetry.addData("LL tag face angle", "unavailable (no fresh front-face pose)");
        }

        if (Double.isFinite(targetDistanceMeters)) {
            telemetry.addData("LL tag distance", "ID %d: %.2f m / %.1f in (locked aim tag)",
                    targetDistanceTagId, targetDistanceMeters, targetDistanceMeters / 0.0254);
        } else {
            telemetry.addData("LL tag distance", "unavailable (no fresh matching tag pose)");
        }


        telemetry.addData("Robot code", CODE_VERSION);

        telemetry.addData(
                "Limelight",
                "configured=%s connected=%s polling=%s",
                Limelight != null,
                visionConnected,
                visionRunning
        );

        telemetry.addData(
                "LL pipeline",
                "requested=%d actual=%d type=%s switch=%s",
                LIMELIGHT_PIPELINE,
                visionPipelineIndex,
                visionPipelineType,
                visionPipelineSwitchOk
        );

        telemetry.addData(
                "LL result",
                "valid=%s age=%d ms",
                visionResultValid,
                visionResultAgeMs
        );

        telemetry.addData(
                "Raw tags",
                "%d: %s",
                visionRawTagCount,
                visionTagDetails
        );

        telemetry.addData(
                "Vision debug",
                visionDebug
        );

        telemetry.addData(
                "Aim filter",
                "%s matching=%d usable=%d",
                alliance,
                visionMatchingIds,
                visionUsableMatches
        );

        telemetry.addData(
                "Turret command",
                "%s requested=%+.2f applied=%+.2f limit=%s",
                turretAimState,
                turretRequestedPower,
                turretAppliedPower,
                turretPowerLimit
        );
        telemetry.addData("Turret encoder", "%d ticks (soft limits %d to %d)",
                turret.getCurrentPosition(), TURRET_MIN_TICKS, TURRET_MAX_TICKS);
        telemetry.addData("Heading compensation", "%s; CW heading=%.1f rate=%.1f deg/s",
                headingStatus, chassisHeadingDegrees, chassisTurnRateDegrees);
        telemetry.addData("Aim target", aimTargetLabel());
        telemetry.addData("Aim gains", "P=%.4f I=0 D=%.5f kS=%.3f kV=%.6f DB=%.2f deg",
                turretGains.p, turretGains.d, turretGains.kS, turretGains.kV, turretGains.deadbandDeg);
        telemetry.addData("Aim terms", "P=%+.3f D=%+.3f velocity FF=%+.3f static=%+.3f settled=%s",
                turretAim.pTerm, turretAim.dTerm, turretAim.velocityTerm, turretAim.staticTerm,
                turretAim.settled);
        telemetry.addData("Aim error/rate", "%.2f deg / %.1f deg/s",
                turretAimErrorDegrees(), turret.getVelocity() / TURRET_TICKS_PER_DEG);
        if (Double.isFinite(turretAim.lastTargetAngle)) {
            telemetry.addData("Turret aim goal", "%.1f deg; observation age=%.0f ms",
                    turretAim.lastTargetAngle, turretAim.observationAgeMs());
        } else {
            telemetry.addData("Turret aim goal", "no observation");
        }
    }

    private boolean isOurHiveTag(int id) {

        if (TURRET_AIM_TAG_ID >= 0
                && id != TURRET_AIM_TAG_ID) {
            return false;
        }

        if (alliance == Alliance.RED) {
            return id >= RED_TAG_MIN && id <= RED_TAG_MAX;
        }

        return id >= BLUE_TAG_MIN && id <= BLUE_TAG_MAX;
    }

    private void updateBall() {

        ballDistanceCm = Double.NaN;
        if (colorSensor == null) {
            ballPresent = false;
            ballColor = BallColor.NONE;
            return;
        }

        Color.colorToHSV(
                colorSensor.getNormalizedColors().toColor(),
                hsv
        );

        ballHue = hsv[0];

        BallColor hueColor;

        if (ballHue >= RED_HUE_ABOVE
                || ballHue <= RED_HUE_BELOW) {

            hueColor = BallColor.RED;

        } else if (
                ballHue >= YELLOW_HUE_MIN
                        && ballHue <= YELLOW_HUE_MAX
        ) {

            hueColor = BallColor.YELLOW;

        } else if (
                ballHue >= BLUE_HUE_MIN
                        && ballHue <= BLUE_HUE_MAX
        ) {

            hueColor = BallColor.BLUE;

        } else {

            hueColor = BallColor.NONE;
        }

        if (ballDistanceSensor != null) {

            ballDistanceCm =
                    ballDistanceSensor.getDistance(DistanceUnit.CM);

            ballPresent =
                    Double.isFinite(ballDistanceCm)
                            && ballDistanceCm > 0
                            && ballDistanceCm < BALL_PRESENT_CM;

        } else {

            ballPresent = hueColor != BallColor.NONE;
        }

        ballColor = ballPresent
                ? hueColor
                : BallColor.NONE;
    }

    // =========================================
    // DRIVING
    // =========================================

    public void drive(
            double forward,
            double strafe,
            double turn
    ) {

        strafe *= STRAFE_MULTIPLIER;

        double fl = -1.0 * (forward + strafe + turn);
        double fr = forward - strafe - turn;

        double bl = -1.0 * (forward - strafe + turn);
        double br = forward + strafe - turn;

        double max = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(fl), Math.abs(fr)),
                        Math.max(Math.abs(bl), Math.abs(br))
                )
        );

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    // =========================================
    // TURRET
    // =========================================

    public double turretDegrees() {

        return turret.getCurrentPosition()
                / TURRET_TICKS_PER_DEG;
    }

    public void aimTurret(
            boolean autoAim,
            double manualPower
    ) {

        double power;

        if (autoAim && Limelight != null) {

            power = turretAim.update(
                    turretDegrees(),
                    targetFresh,
                    (targetTx - TURRET_AIM_OFFSET_DEG) * TURRET_VISION_POWER_SIGN,
                    targetFrameTimestamp,
                    visionConnected && visionRunning && visionPipelineIndex == LIMELIGHT_PIPELINE,
                    chassisHeadingDegrees,
                    chassisTurnRateDegrees,
                    turret.getVelocity() / TURRET_TICKS_PER_DEG,
                    targetFrameAgeMs
            );
            turretAimState = turretAim.state;
        } else {
            turretAim.cancelReturn();

            turretAimState = autoAim
                    ? "NO CAMERA"
                    : "MANUAL";

            power = manualPower;
        }

        setTurretPower(power);
    }

    public void turretToAngle(double degrees) {

        turretAim.cancelReturn();

        turretAimState = "CENTER / POSITION";

        double error =
                Range.clip(
                        degrees,
                        TURRET_MIN_DEG,
                        TURRET_MAX_DEG
                ) - turretDegrees();

        double power =
                Math.abs(error) < TURRET_DEADBAND_DEG
                        ? 0
                        : TURRET_KP * error
                        + Math.copySign(TURRET_KS, error);

        setTurretPower(power);
    }

    private void setTurretPower(double power) {

        turretRequestedPower =
                Double.isFinite(power) ? power : 0;

        if (!Double.isFinite(power)) power = 0;

        power = Range.clip(
                power,
                -TURRET_MAX_POWER,
                TURRET_MAX_POWER
        );

        int ticks = turret.getCurrentPosition();

        turretPowerLimit = "none";

        if (ticks >= TURRET_MAX_TICKS && power > 0) {

            power = 0;
            turretPowerLimit = "RIGHT ENCODER LIMIT";

        } else if (
                ticks <= TURRET_MIN_TICKS && power < 0
        ) {

            power = 0;
            turretPowerLimit = "LEFT ENCODER LIMIT";
        }

        turretAppliedPower = power;
        turret.setPower(power);
    }

    public void zeroTurret() {

        turretAim.cancelReturn();
        turret.setPower(0);

        turret.setMode(
                DcMotor.RunMode.STOP_AND_RESET_ENCODER
        );

        turret.setMode(
                DcMotor.RunMode.RUN_WITHOUT_ENCODER
        );
    }

    public boolean aimedAtTarget() {

        return !turretAim.returning
                && targetVisible
                && Math.abs(
                targetTx - TURRET_AIM_OFFSET_DEG
        ) < AIM_TOLERANCE_DEG;
    }

    public String turretAimState() {
        return turretAimState;
    }

    public String aimTargetLabel() {

        return TURRET_AIM_TAG_ID >= 0
                ? "TAG " + TURRET_AIM_TAG_ID
                : aimTagLock.lockedId >= 0 ? "LOCKED TAG " + aimTagLock.lockedId : "AUTO TAG LOCK";
    }

    public double turretAimErrorDegrees() {
        return turretAim.lastTargetAngle - turretDegrees();
    }

    public void resetTurretAim() {
        turretAim.cancelReturn();
    }

    /** Mutable per-OpMode gains; tuning does not persist or change competition defaults. */
    public static final class TurretGains {
        public double p = TURRET_KP;
        public double d = TURRET_KD;
        public double kS = TURRET_KS;
        public double kV = TURRET_KV;
        public double deadbandDeg = TURRET_DEADBAND_DEG;

        public void reset() {
            p = TURRET_KP;
            d = TURRET_KD;
            kS = TURRET_KS;
            kV = TURRET_KV;
            deadbandDeg = TURRET_DEADBAND_DEG;
        }

        public void quietStart() {
            reset();
            p = 0.012;
            kS = 0.020;
            deadbandDeg = 1.5;
        }
    }

    /** Keeps changes in the number/order of visible tags from moving the aim point. */
    static final class AimTagLock {
        private int lockedId = -1;
        private double lastSeenMs = Double.NEGATIVE_INFINITY;
        private int bestId;
        private double bestTx;
        private double lockedTx;
        private double selectedTx = Double.NaN;

        void beginFrame() {
            bestId = -1;
            bestTx = Double.NaN;
            lockedTx = Double.NaN;
        }

        void consider(int id, double tx) {
            if (id < 0 || !Double.isFinite(tx)) return;
            if (id == lockedId) lockedTx = tx;
            if (bestId < 0 || Math.abs(tx) < Math.abs(bestTx)
                    || (Math.abs(tx) == Math.abs(bestTx) && id < bestId)) {
                bestId = id;
                bestTx = tx;
            }
        }

        int finishFrame(double nowMs) {
            selectedTx = Double.NaN;
            if (Double.isFinite(lockedTx)) {
                selectedTx = lockedTx;
                lastSeenMs = nowMs;
            } else if (bestId >= 0 && (lockedId < 0
                    || nowMs - lastSeenMs >= TURRET_TAG_SWITCH_DELAY_MS)) {
                lockedId = bestId;
                selectedTx = bestTx;
                lastSeenMs = nowMs;
            }
            return Double.isFinite(selectedTx) ? lockedId : -1;
        }

        void reset() {
            lockedId = -1;
            lastSeenMs = Double.NEGATIVE_INFINITY;
            selectedTx = Double.NaN;
            beginFrame();
        }
    }

    // =========================================
    // TURRET AIM CONTROLLER
    // =========================================

    static final class TurretAimController {

        private final double minDeg;
        private final double maxDeg;
        private final DoubleSupplier clockMs;
        private final TurretGains gains;
        private int searchDirection = 1;
        private boolean returning;
        private double lastFrameTimestamp = Double.NaN;
        private double lastTargetAngle = Double.NaN;
        private double lastHeading = Double.NaN;
        private double lastObservationMs = Double.NEGATIVE_INFINITY;
        private String state = "SEARCH RIGHT";
        private boolean settled;
        private double pTerm;
        private double dTerm;
        private double velocityTerm;
        private double staticTerm;

        TurretAimController(double minDeg, double maxDeg) {
            this(minDeg, maxDeg, new TurretGains());
        }

        TurretAimController(double minDeg, double maxDeg, TurretGains gains) {
            this(minDeg, maxDeg, () -> System.nanoTime() / 1e6, gains);
        }

        TurretAimController(double minDeg, double maxDeg, DoubleSupplier clockMs) {
            this(minDeg, maxDeg, clockMs, new TurretGains());
        }

        TurretAimController(double minDeg, double maxDeg, DoubleSupplier clockMs, TurretGains gains) {
            double span = maxDeg - minDeg;
            if (!Double.isFinite(minDeg) || !Double.isFinite(maxDeg)
                    || !Double.isFinite(span) || minDeg > 0 || maxDeg < 0
                    || span <= 2 * (TURRET_RETURN_MARGIN_DEG + TURRET_RETURN_TOLERANCE_DEG)) {
                throw new IllegalArgumentException("Invalid turret range");
            }
            // Keep encoder angles unwrapped: the measured mechanism has more than 360 deg travel.
            this.minDeg = minDeg;
            this.maxDeg = maxDeg;
            this.clockMs = clockMs;
            this.gains = gains;
        }

        double update(double angle, boolean visible, double error, double frameTimestamp,
                      boolean cameraUsable, double heading, double turnRate,
                      double turretVelocity, double frameAgeMs) {
            pTerm = dTerm = velocityTerm = staticTerm = 0;
            if (!Double.isFinite(angle)
                    || (visible && (!Double.isFinite(error) || !Double.isFinite(frameTimestamp)
                    || !Double.isFinite(frameAgeMs) || frameAgeMs < 0))) {
                cancelReturn();
                state = "INVALID READING";
                return 0;
            }

            double nowMs = clockMs.getAsDouble();
            boolean headingAvailable = Double.isFinite(heading);
            if (headingAvailable && Double.isFinite(lastHeading)
                    && Double.isFinite(lastTargetAngle)) {
                // Subtract chassis rotation even when there is no new camera frame.
                double delta = heading - lastHeading;
                delta -= 360.0 * Math.floor((delta + 180.0) / 360.0);
                lastTargetAngle -= delta;
            }
            lastHeading = headingAvailable ? heading : Double.NaN;
            double rate = headingAvailable && Double.isFinite(turnRate) ? turnRate : 0;

            // A camera frame is one observation, regardless of how often the USB result is read.
            if (cameraUsable && visible && frameAgeMs < LIMELIGHT_STALE_MS
                    && frameTimestamp != lastFrameTimestamp) {
                lastFrameTimestamp = frameTimestamp;
                double velocity = Double.isFinite(turretVelocity) ? turretVelocity : 0;
                // First-order capture-time correction for motion during camera/USB latency.
                lastTargetAngle = angle + error - (velocity + rate) * frameAgeMs / 1000.0;
                lastObservationMs = nowMs - frameAgeMs;
                if (Math.abs(error) >= gains.deadbandDeg) {
                    searchDirection = error > 0 ? 1 : -1;
                }
            }

            returning = false;
            // An out-of-range encoder reading gets a short inward recovery, not a full-range sweep.
            if (angle < minDeg || angle > maxDeg) {
                double goal = angle < minDeg
                        ? minDeg + TURRET_RETURN_MARGIN_DEG : maxDeg - TURRET_RETURN_MARGIN_DEG;
                state = angle < minDeg ? "LIMIT RECOVERY RIGHT" : "LIMIT RECOVERY LEFT";
                returning = true;
                return limited(angle, positionPower(goal - angle, 0, TURRET_RETURN_MAX_POWER));
            }

            double goal = nearestReachableAngle(lastTargetAngle, angle);
            boolean tracking = cameraUsable && Double.isFinite(goal)
                    && nowMs - lastObservationMs < TURRET_REACQUIRE_MS;
            if (tracking) {
                lastTargetAngle = goal;
                double trackingError = goal - angle;
                // A full turn may be needed to unwind near a cable limit. Never wrap the encoder.
                returning = Math.abs(trackingError) > 180;
                state = returning ? (trackingError > 0 ? "UNWIND RIGHT" : "UNWIND LEFT")
                        : visible ? "TRACK" : "REACQUIRE LAST TAG";
                double feedforward = Math.abs(rate) >= TURRET_ROTATION_DEADBAND_DEG_S
                        ? -rate * gains.kV : 0;
                double velocity = Double.isFinite(turretVelocity) ? turretVelocity : 0;
                boolean moving = Math.abs(rate) >= TURRET_ROTATION_DEADBAND_DEG_S
                        || Math.abs(velocity) >= 5.0;
                // Enter hold inside DB; resume above DB+0.5 deg to avoid noise-driven kick reversals.
                if (moving || Math.abs(trackingError) >= gains.deadbandDeg + 0.5) settled = false;
                else if (Math.abs(trackingError) < gains.deadbandDeg) settled = true;
                if (settled) return 0;
                // D uses measured field angular velocity, avoiding derivative kicks from tag noise.
                double damping = returning ? 0 : gains.d * (-rate - velocity);
                return limited(angle, positionPower(trackingError, damping,
                        returning ? 0 : feedforward,
                        returning ? TURRET_RETURN_MAX_POWER : TURRET_TRACK_MAX_POWER));
            }

            // Search automatically before the first detection and after an expired target.
            // Reverse near either endpoint instead of latching a return to the far endpoint.
            if (angle >= maxDeg - TURRET_RETURN_MARGIN_DEG) searchDirection = -1;
            if (angle <= minDeg + TURRET_RETURN_MARGIN_DEG) searchDirection = 1;
            settled = false;
            state = searchDirection > 0 ? "SEARCH RIGHT" : "SEARCH LEFT";
            return limited(angle, searchDirection * TURRET_SEARCH_POWER);
        }

        private double nearestReachableAngle(double desired, double current) {
            if (!Double.isFinite(desired)) return Double.NaN;
            double firstTurn = Math.ceil((minDeg - desired) / 360.0);
            double lastTurn = Math.floor((maxDeg - desired) / 360.0);
            if (firstTurn > lastTurn) return Double.NaN;
            double nearestTurn = Math.rint((current - desired) / 360.0);
            nearestTurn = Math.max(firstTurn, Math.min(lastTurn, nearestTurn));
            return desired + nearestTurn * 360.0;
        }

        double observationAgeMs() {
            return clockMs.getAsDouble() - lastObservationMs;
        }

        void cancelReturn() {
            returning = false;
            lastFrameTimestamp = Double.NaN;
            lastTargetAngle = Double.NaN;
            lastHeading = Double.NaN;
            lastObservationMs = Double.NEGATIVE_INFINITY;
            settled = false;
            pTerm = dTerm = velocityTerm = staticTerm = 0;
        }

        private double positionPower(double error, double feedforward, double maxPower) {
            return positionPower(error, 0, feedforward, maxPower);
        }

        private double positionPower(double error, double damping, double feedforward, double maxPower) {
            pTerm = Math.abs(error) < gains.deadbandDeg ? 0 : gains.p * error;
            dTerm = damping;
            velocityTerm = feedforward;
            double power = pTerm + dTerm + velocityTerm;
            staticTerm = power == 0 ? 0 : Math.copySign(gains.kS, power);
            power += staticTerm;
            return Math.max(-maxPower, Math.min(maxPower, power));
        }

        private double limited(double angle, double power) {
            if (!Double.isFinite(power) || (angle >= maxDeg && power > 0)
                    || (angle <= minDeg && power < 0)) return 0;
            return Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, power));
        }
    }

    // =========================================
    // SHOOTER
    // =========================================

    /** Interpolate measured camera-to-AprilTag distances; clamp beyond the measured range. */
    public static double rpmForDistance(double meters) {
        if (!Double.isFinite(meters) || meters <= 0) return TELEOP_DEFAULT_RPM;
        if (meters <= SHOOTER_DISTANCE_METERS[0]) return SHOOTER_DISTANCE_RPM[0];
        for (int i = 1; i < SHOOTER_DISTANCE_METERS.length; i++) {
            if (meters <= SHOOTER_DISTANCE_METERS[i]) {
                double fraction = (meters - SHOOTER_DISTANCE_METERS[i - 1])
                        / (SHOOTER_DISTANCE_METERS[i] - SHOOTER_DISTANCE_METERS[i - 1]);
                return SHOOTER_DISTANCE_RPM[i - 1]
                        + fraction * (SHOOTER_DISTANCE_RPM[i] - SHOOTER_DISTANCE_RPM[i - 1]);
            }
        }
        return SHOOTER_DISTANCE_RPM[SHOOTER_DISTANCE_RPM.length - 1];
    }

    public double distanceBasedShooterRpm() {
        // targetVisible can outlive the latest detection during the short aim hold.
        return rpmForDistance(targetFresh ? targetDistanceMeters : Double.NaN);
    }

    public static double rpmFor(BallColor color) {

        switch (color) {
            case RED:
                return RPM_RED;

            case BLUE:
                return RPM_BLUE;

            default:
                return RPM_YELLOW;
        }
    }

    public void updateShooter() {

        if (!gateOpen && ballColor != BallColor.NONE) {
            shotColor = ballColor;
        }

        updateShooter(rpmFor(shotColor));
    }

    /** TeleOp supplies a distance-based base RPM; keep motor output/readiness on one target. */
    public void updateShooter(double baseRpm) {
        targetRpm = baseRpm;

        if (shooterOn) {

            setShooterRpm(targetRpm + rpmTrim);

        } else {

            shooter.setPower(0);
        }
    }

    public void setShooterRpm(double rpm) {

        shooter.setVelocity(
                rpm * SHOOTER_TICKS_PER_REV / 60.0
        );
    }

    public double shooterRpm() {

        return shooter.getVelocity()
                * 60.0 / SHOOTER_TICKS_PER_REV;
    }

    public boolean shooterAtSpeed() {

        return shooterOn
                && Math.abs(
                shooterRpm()
                        - (targetRpm + rpmTrim)
        ) < SHOOTER_RPM_TOLERANCE;
    }

    public boolean readyToShoot() {

        return gateCalibrated() && aimedAtTarget() && shooterAtSpeed();
    }

    public boolean gateCalibrated() {
        return gateController != null && gateController.calibrated();
    }

    public void addGateTelemetry(Telemetry telemetry) {
        telemetry.addData("Gate calibration", gateCalibrated()
                ? gateController.usingCodeDefaults ? "CODE DEFAULTS (measured)" : "SAVED for current model"
                : "REQUIRED - run Gate Calibration; no gate commands");
        if (gateController != null) {
            telemetry.addData("Gate endpoints", "OPEN=%.4f CLOSED=%.4f command=%.4f",
                    gateController.openPosition, gateController.closedPosition, gateController.command);
        }
    }

    /** Shared by calibration, TeleOp, Auto and Robot Test. Values describe commands, not shaft angle. */
    public static final class GateController {
        private final Servo servo;
        private final SharedPreferences preferences;
        public double openPosition = Double.NaN;
        public double closedPosition = Double.NaN;
        public double command = Double.NaN;
        public double markedOpen = Double.NaN;
        public double markedClosed = Double.NaN;
        public boolean usingCodeDefaults;
        private boolean armed;

        public GateController(Servo servo, SharedPreferences preferences) {
            this.servo = servo;
            this.preferences = preferences;
            openPosition = preferences.getFloat("open", Float.NaN);
            closedPosition = preferences.getFloat("closed", Float.NaN);
            if (preferences.getBoolean("disabled", false)) {
                openPosition = closedPosition = Double.NaN;
            } else if (!preferences.contains("open") && !preferences.contains("closed")) {
                openPosition = GATE_OPEN;
                closedPosition = GATE_CLOSED;
                usingCodeDefaults = true;
            }
        }

        private static boolean validPosition(double position) {
            return Double.isFinite(position) && position >= 0 && position <= 1;
        }

        private static boolean distinct(double open, double closed) {
            // Allow float persistence roundoff at the two-fine-step (0.001) boundary.
            return validPosition(open) && validPosition(closed) && Math.abs(open - closed) >= 0.000999;
        }

        public boolean calibrated() {
            return distinct(openPosition, closedPosition);
        }

        public boolean canRelease() {
            return servo instanceof PwmControl;
        }

        public void release() {
            if (canRelease()) ((PwmControl) servo).setPwmDisable();
            armed = false;
            command = Double.NaN;
        }

        /** Explicit operator action only: establish this reference with the linkage freed. */
        public void establishMidpoint() {
            markedOpen = markedClosed = Double.NaN;
            write(0.5);
        }

        public boolean jogCalibration(int direction, boolean fine) {
            if (!armed || !validPosition(command) || direction == 0) return false;
            write(Range.clip(command + Math.signum(direction) * (fine ? 0.0005 : GATE_CALIBRATION_STEP), 0, 1));
            return true;
        }

        public boolean markOpen() {
            if (!armed || !validPosition(command)) return false;
            markedOpen = command;
            return true;
        }

        public boolean markClosed() {
            if (!armed || !validPosition(command)) return false;
            markedClosed = command;
            return true;
        }

        public boolean saveMarks() {
            if (!distinct(markedOpen, markedClosed)) return false;
            boolean saved = preferences.edit().putFloat("open", (float) markedOpen)
                    .putFloat("closed", (float) markedClosed).putBoolean("disabled", false).commit();
            if (saved) {
                openPosition = markedOpen;
                closedPosition = markedClosed;
                usingCodeDefaults = false;
            }
            return saved;
        }

        public boolean forgetCalibration() {
            release();
            openPosition = closedPosition = markedOpen = markedClosed = Double.NaN;
            usingCodeDefaults = false;
            return preferences.edit().remove("open").remove("closed").putBoolean("disabled", true).commit();
        }

        public boolean commandSaved(boolean open) {
            if (!calibrated()) return false;
            write(open ? openPosition : closedPosition);
            return true;
        }

        public boolean jogWithinSaved(int direction) {
            if (!calibrated() || direction == 0) return false;
            double start = validPosition(command) ? command : closedPosition;
            write(Range.clip(start + Math.signum(direction) * GATE_JOG_STEP,
                    Math.min(openPosition, closedPosition), Math.max(openPosition, closedPosition)));
            return true;
        }

        private void write(double position) {
            if (!validPosition(position)) return;
            if (!armed || position != command) {
                servo.setPosition(position);
                if (canRelease()) ((PwmControl) servo).setPwmEnable();
            }
            command = position;
            armed = true;
        }
    }

    // =========================================
    // FEEDER
    // =========================================

    public void updateFeeder(
            boolean wantShoot,
            double intakePower
    ) {

        if (!gateCalibrated()) {
            gateOpen = feedingBall = false;
            transmission.setPower(0);
            intake.setPower(intakePower);
            return;
        }

        if (!targetVisible) {

            gateOpen = false;

        } else if (gateOpen) {

            if (feedTimer.milliseconds() > FEED_TIME_MS) {

                gateOpen = false;

                if (feedingBall) {
                    shotsFired++;
                }
            }

        } else if (wantShoot && readyToShoot()) {

            gateOpen = true;
            feedingBall = ballPresent;
            feedTimer.reset();
        }

        gateController.commandSaved(gateOpen);

        updateFeederMotors(
                gateOpen,
                wantShoot,
                intakePower
        );
    }

    // Manual TeleOp gate toggle, independent of vision.
    public void updateManualFeeder(
            boolean toggleGate,
            double intakePower
    ) {

        if (!gateCalibrated()) {
            gateOpen = false;
            transmission.setPower(0);
            intake.setPower(intakePower);
            return;
        }

        if (toggleGate) {
            gateOpen = !gateOpen;
        }

        gateController.commandSaved(gateOpen);

        updateFeederMotors(
                false,
                false,
                intakePower
        );
    }

    private void updateFeederMotors(
            boolean feedThroughGate,
            boolean wantShoot,
            double intakePower
    ) {

        double transmissionPower;

        if (feedThroughGate) {

            transmissionPower = TRANSMISSION_FEED_POWER;

        } else if (intakePower < 0) {

            transmissionPower = -TRANSMISSION_INDEX_POWER;

        } else if (
                (intakePower > 0 || wantShoot)
                        && !ballPresent
        ) {

            transmissionPower = TRANSMISSION_INDEX_POWER;

        } else {

            transmissionPower = 0;
        }

        transmission.setPower(transmissionPower);
        intake.setPower(intakePower);
    }

    // =========================================
    // LED
    // =========================================

    public void updateLed() {

        if (led == null) return;

        // Presence of any pollen color wins, independent of the gate/shooter/tag.
        led.setPosition(ballPresent ? LED_GREEN : LED_RED);
    }

    // =========================================
    // STOP
    // =========================================

    public void stop() {

        drive(0, 0, 0);

        turret.setPower(0);
        shooter.setPower(0);

        intake.setPower(0);
        transmission.setPower(0);

        if (Limelight != null) {
            Limelight.stop();
        }
    }
}
