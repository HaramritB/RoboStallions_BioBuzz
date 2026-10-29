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
 *   USB:     limelight                                    Limelight 3A (on top of the shooter)
 * Without the Limelight the gate never opens in the competition TeleOp/Auto, since it only
 * opens while our HIVE tag is visible.
 *
 * init() never moves a servo. Servos stay where they are until something commands them.
 *
 * Every number marked "tune" is a starting guess. Check it on the real robot.
 */
public class Robot {

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
    public static final double TURRET_MIN_DEG = -150;          // tune: soft limits so wires don't wrap
    public static final double TURRET_MAX_DEG = 150;
    public static final double TURRET_KP = 0.02;               // tune: power per degree of aim error
    public static final double TURRET_KS = 0.04;               // tune: minimum power that moves the turret
    public static final double TURRET_DEADBAND_DEG = 1.0;
    public static final double TURRET_MAX_POWER = 0.6;
    public static final double TURRET_AIM_OFFSET_DEG = 0.0;    // tune: nudge if shots always miss to one side
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
    public static final double GATE_OPEN = 0.5;                // tune: find it with the Robot Test OpMode
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
    // Pipeline 0 must be an AprilTag pipeline (family 36h11), set up in the Limelight web UI.
    public static final int LIMELIGHT_PIPELINE = 0;
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
    // Hardware. led, colorSensor and limelight are null when they aren't in the config.
    // ---------------------------------------------------------------------------------------------
    public DcMotorEx frontLeft, frontRight, backLeft, backRight;
    public DcMotorEx intake, transmission, turret, shooter;
    public Servo gate, led;
    public NormalizedColorSensor colorSensor;
    public DistanceSensor ballDistanceSensor;
    public Limelight3A limelight;

    // ---------------------------------------------------------------------------------------------
    // State, refreshed by updateSensors() / the update methods each loop
    // ---------------------------------------------------------------------------------------------
    public Alliance alliance = lastAlliance;

    public boolean targetVisible;
    public double targetTx;              // degrees, + means the CELL is right of the crosshair
    public int targetId = -1;
    private boolean tagEverSeen;
    private final ElapsedTime sinceTagSeen = new ElapsedTime();

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

        limelight = hw.tryGet(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
            limelight.start();
        }

        alliance = lastAlliance;
    }

    /** Call once at the top of every loop, before using any sensor values. */
    public void updateSensors() {
        updateVision();
        updateBall();
    }

    private void updateVision() {
        boolean seenNow = false;
        LLResult result = limelight != null ? limelight.getLatestResult() : null;
        if (result != null && result.isValid() && result.getStaleness() < LIMELIGHT_STALE_MS) {
            double txSum = 0;
            int count = 0;
            for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                if (isOurHiveTag(tag.getFiducialId())) {
                    txSum += tag.getTargetXDegrees();
                    count++;
                    targetId = tag.getFiducialId();
                }
            }
            if (count > 0) {
                // The 4 tags sit evenly around the CELL opening, so their average is about its center.
                targetTx = txSum / count;
                seenNow = true;
                tagEverSeen = true;
                sinceTagSeen.reset();
            }
        }
        targetVisible = seenNow || (tagEverSeen && sinceTagSeen.milliseconds() < TAG_HOLD_MS);
        if (!targetVisible) targetId = -1;
    }

    private boolean isOurHiveTag(int id) {
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
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
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
     * Tracks our HIVE's CELL when autoAim is on and a tag is visible. Otherwise uses manualPower
     * (-1 to 1, + turns right). The Limelight rides on the shooter, so tx is the turret's aim error.
     */
    public void aimTurret(boolean autoAim, double manualPower) {
        double power;
        if (autoAim && targetVisible) {
            double error = targetTx - TURRET_AIM_OFFSET_DEG;
            power = Math.abs(error) < TURRET_DEADBAND_DEG
                    ? 0
                    : TURRET_KP * error + Math.copySign(TURRET_KS, error);
        } else {
            power = manualPower;
        }
        setTurretPower(power);
    }

    /** Drives the turret toward an encoder angle (0 = straight forward). Call every loop. */
    public void turretToAngle(double degrees) {
        double error = degrees - turretDegrees();
        double power = Math.abs(error) < TURRET_DEADBAND_DEG
                ? 0
                : TURRET_KP * error + Math.copySign(TURRET_KS, error);
        setTurretPower(power);
    }

    private void setTurretPower(double power) {
        power = Range.clip(power, -TURRET_MAX_POWER, TURRET_MAX_POWER);
        double deg = turretDegrees();
        if ((deg >= TURRET_MAX_DEG && power > 0) || (deg <= TURRET_MIN_DEG && power < 0)) power = 0;
        turret.setPower(power);
    }

    /** Re-zero the turret. Only do this with the turret pointing straight forward. */
    public void zeroTurret() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean aimedAtTarget() {
        return targetVisible && Math.abs(targetTx - TURRET_AIM_OFFSET_DEG) < AIM_TOLERANCE_DEG;
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

        double transmissionPower;
        if (gateOpen) {
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
        if (limelight != null) limelight.stop();
    }
}
