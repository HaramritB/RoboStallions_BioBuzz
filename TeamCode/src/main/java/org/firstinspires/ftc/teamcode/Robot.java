
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Robot {

    public static final String CODE_VERSION =
            "BIOBUZZ-4000RPM-PIDF-2026-11-02";

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

    public static final double TURRET_TICKS_PER_DEG =
            1425.1 / 360.0;

    public static final double TURRET_MIN_DEG = -150;
    public static final double TURRET_MAX_DEG = 150;

    public static final double TURRET_KP = 0.02;
    public static final double TURRET_KS = 0.04;
    public static final double TURRET_DEADBAND_DEG = 1.0;

    public static final double TURRET_MAX_POWER = 0.6;
    public static final double TURRET_SEARCH_POWER = 0.15;
    public static final double TURRET_RETURN_MAX_POWER = 0.35;
    public static final double TURRET_RETURN_MARGIN_DEG = 5;
    public static final double TURRET_RETURN_TOLERANCE_DEG = 1;

    public static final double TURRET_AIM_OFFSET_DEG = 0.0;

    // Preserve your latest camera/motor sign correction.
    public static final double TURRET_VISION_POWER_SIGN = -1.0;

    // -1 means aim using the mean of matching tags.
    public static final int TURRET_AIM_TAG_ID = -1;

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

    public static final double SHOOTER_RPM_TOLERANCE = 100.0;
    public static final double RPM_TRIM_STEP = 50.0;

    public static final double SHOOTER_P = 180.0;
    public static final double SHOOTER_I = 0.0;
    public static final double SHOOTER_D = 0.0;
    public static final double SHOOTER_F = 13.83;

    // =========================================
    // GATE / INTAKE / TRANSMISSION
    // =========================================

    public static final double GATE_CLOSED = 0.0;
    public static final double GATE_OPEN = 0.14;

    public static final long FEED_TIME_MS = 300;

    public static final double INTAKE_POWER = 1.0;
    public static final double TRANSMISSION_INDEX_POWER = 0.6;
    public static final double TRANSMISSION_FEED_POWER = 1.0;

    // =========================================
    // COLOR SENSOR
    // =========================================

    public static final float COLOR_GAIN = 4;

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

    public static final double LED_OFF = 0.0;
    public static final double LED_RED = 0.277;
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

    public NormalizedColorSensor colorSensor;
    public DistanceSensor ballDistanceSensor;

    // Capital L matches your latest hardware config.
    public Limelight3A Limelight;

    // =========================================
    // ROBOT STATE
    // =========================================

    public Alliance alliance = lastAlliance;

    public boolean targetVisible;
    public boolean targetFresh;
    public double targetTx;
    public int targetId = -1;

    private boolean tagEverSeen;

    private final ElapsedTime sinceTagSeen = new ElapsedTime();

    private final TurretAimController turretAim =
            new TurretAimController(
                    TURRET_MIN_DEG,
                    TURRET_MAX_DEG
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
    public double ballDistanceCm;

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
        led = hw.tryGet(Servo.class, "led");

        colorSensor = hw.tryGet(
                NormalizedColorSensor.class,
                "colorSensor"
        );

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
        updateVision();
        updateBall();
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

        double txSum = 0;
        int firstMatchingId = -1;

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

                        txSum += tag.getTargetXDegrees();

                        visionUsableMatches++;

                        if (firstMatchingId < 0) {
                            firstMatchingId = tag.getFiducialId();
                        }
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

        boolean seenNow =
                "MATCHING TAG VISIBLE".equals(visionDebug);

        if (seenNow) {

            targetTx = txSum / visionUsableMatches;
            targetId = firstMatchingId;

            tagEverSeen = true;
            sinceTagSeen.reset();
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

    public void addVisionTelemetry(Telemetry telemetry) {

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
                    ballDistanceCm < BALL_PRESENT_CM;

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

            if (!targetFresh
                    && targetVisible
                    && !turretAim.returning) {

                power = 0;
                turretAimState = "WAIT FOR FRESH TAG";

            } else {

                power = turretAim.update(
                        turretDegrees(),
                        targetFresh,
                        (targetTx - TURRET_AIM_OFFSET_DEG)
                                * TURRET_VISION_POWER_SIGN
                );

                turretAimState = turretAim.state;
            }

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

        double degrees = turretDegrees();

        turretPowerLimit = "none";

        if (degrees >= TURRET_MAX_DEG && power > 0) {

            power = 0;
            turretPowerLimit = "RIGHT ENCODER LIMIT";

        } else if (
                degrees <= TURRET_MIN_DEG && power < 0
        ) {

            power = 0;
            turretPowerLimit = "LEFT ENCODER LIMIT";
        }

        turretAppliedPower = power;
        turret.setPower(power);
    }

    public void zeroTurret() {

        turretAim.cancelReturn();

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
                : "MATCHING TAG GROUP MEAN";
    }

    // =========================================
    // TURRET AIM CONTROLLER
    // =========================================

    static final class TurretAimController {

        private final double minDeg;
        private final double maxDeg;

        private int searchDirection = 1;

        private boolean returning = false;
        private double returnGoal;

        private String state = "SEARCH RIGHT";

        TurretAimController(
                double minDeg,
                double maxDeg
        ) {

            double span = maxDeg - minDeg;

            if (!Double.isFinite(minDeg)
                    || !Double.isFinite(maxDeg)
                    || minDeg > 0
                    || maxDeg < 0
                    || span > 360
                    || span <= 2 * (
                    TURRET_RETURN_MARGIN_DEG
                            + TURRET_RETURN_TOLERANCE_DEG
            )) {

                throw new IllegalArgumentException(
                        "Invalid turret range"
                );
            }

            this.minDeg = minDeg;
            this.maxDeg = maxDeg;
        }

        double update(
                double angle,
                boolean visible,
                double error
        ) {

            if (!Double.isFinite(angle)
                    || (visible && !Double.isFinite(error))) {

                cancelReturn();
                state = "INVALID READING";

                return 0;
            }

            double targetAngle = angle + error;

            boolean reachable =
                    visible
                            && angle >= minDeg
                            && angle <= maxDeg
                            && targetAngle >= minDeg
                            && targetAngle <= maxDeg;

            if (returning && reachable) {
                returning = false;
            }

            if (returning) {

                if (Math.abs(returnGoal - angle)
                        > TURRET_RETURN_TOLERANCE_DEG) {

                    return limited(
                            angle,
                            positionPower(
                                    returnGoal - angle,
                                    TURRET_RETURN_MAX_POWER
                            )
                    );
                }

                returning = false;
            }

            double power;

            if (visible) {

                if (Math.abs(error) >= TURRET_DEADBAND_DEG) {
                    searchDirection = error > 0 ? 1 : -1;
                }

                power = positionPower(
                        error,
                        TURRET_MAX_POWER
                );

                state = "TRACK";

            } else {

                power =
                        searchDirection * TURRET_SEARCH_POWER;

                state = searchDirection > 0
                        ? "SEARCH RIGHT"
                        : "SEARCH LEFT";
            }

            if (angle > maxDeg
                    || (!reachable && power > 0
                    && angle >= maxDeg - TURRET_RETURN_MARGIN_DEG)) {

                returnGoal =
                        minDeg + TURRET_RETURN_MARGIN_DEG;

                returning = true;
                state = "RETURN LEFT";

            } else if (
                    angle < minDeg
                            || (!reachable && power < 0
                            && angle <= minDeg + TURRET_RETURN_MARGIN_DEG)
            ) {

                returnGoal =
                        maxDeg - TURRET_RETURN_MARGIN_DEG;

                returning = true;
                state = "RETURN RIGHT";
            }

            if (returning) {

                power = positionPower(
                        returnGoal - angle,
                        TURRET_RETURN_MAX_POWER
                );
            }

            return limited(angle, power);
        }

        void cancelReturn() {
            returning = false;
        }

        private double positionPower(
                double error,
                double maxPower
        ) {

            if (Math.abs(error) < TURRET_DEADBAND_DEG) {
                return 0;
            }

            double power =
                    TURRET_KP * error
                            + Math.copySign(TURRET_KS, error);

            return Math.max(
                    -maxPower,
                    Math.min(maxPower, power)
            );
        }

        private double limited(
                double angle,
                double power
        ) {

            if ((angle >= maxDeg && power > 0)
                    || (angle <= minDeg && power < 0)) {

                return 0;
            }

            return Math.max(
                    -TURRET_MAX_POWER,
                    Math.min(TURRET_MAX_POWER, power)
            );
        }
    }

    // =========================================
    // SHOOTER
    // =========================================

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

        targetRpm = rpmFor(shotColor);

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

        return aimedAtTarget() && shooterAtSpeed();
    }

    // =========================================
    // FEEDER
    // =========================================

    public void updateFeeder(
            boolean wantShoot,
            double intakePower
    ) {

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

        gate.setPosition(
                gateOpen ? GATE_OPEN : GATE_CLOSED
        );

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

        if (toggleGate) {
            gateOpen = !gateOpen;
        }

        gate.setPosition(
                gateOpen ? GATE_OPEN : GATE_CLOSED
        );

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

        double color;

        if (gateOpen || (ballPresent && readyToShoot())) {

            color = LED_GREEN;

        } else if (ballColor == BallColor.YELLOW) {

            color = LED_YELLOW;

        } else if (ballColor == BallColor.RED) {

            color = LED_RED;

        } else if (ballColor == BallColor.BLUE) {

            color = LED_BLUE;

        } else if (ballPresent) {

            color = LED_WHITE;

        } else if (targetVisible) {

            color = LED_VIOLET;

        } else {

            color = LED_OFF;
        }

        led.setPosition(color);
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
