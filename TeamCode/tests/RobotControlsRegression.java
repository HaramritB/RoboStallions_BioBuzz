package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.Map;

/** Runs production gate/feeder/indicator methods with recording hardware and in-memory settings. */
public final class RobotControlsRegression {
    private static int checks;
    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }
    private static void near(double actual, double expected, String message) {
        check(Math.abs(actual - expected) < 1e-6, message + ": " + actual + " != " + expected);
    }

    private static final class Settings {
        final Map<String, Object> values = new HashMap<>();
        final Map<String, Object> pending = new HashMap<>();
        boolean storageWorks = true;
        int commits;
        final SharedPreferences.Editor editor = (SharedPreferences.Editor) Proxy.newProxyInstance(
                SharedPreferences.Editor.class.getClassLoader(), new Class<?>[]{SharedPreferences.Editor.class},
                (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "putFloat":
                        case "putBoolean": pending.put((String) args[0], args[1]); return proxy;
                        case "remove": pending.put((String) args[0], null); return proxy;
                        case "commit":
                            commits++;
                            if (storageWorks) {
                                for (Map.Entry<String, Object> entry : pending.entrySet()) {
                                    if (entry.getValue() == null) values.remove(entry.getKey());
                                    else values.put(entry.getKey(), entry.getValue());
                                }
                            }
                            pending.clear();
                            return storageWorks;
                        default: throw new AssertionError("Unexpected settings edit: " + method.getName());
                    }
                });
        final SharedPreferences prefs = (SharedPreferences) Proxy.newProxyInstance(
                SharedPreferences.class.getClassLoader(), new Class<?>[]{SharedPreferences.class},
                (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "getFloat":
                        case "getBoolean": return values.getOrDefault((String) args[0], args[1]);
                        case "contains": return values.containsKey((String) args[0]);
                        case "edit": return editor;
                        default: throw new AssertionError("Unexpected settings read: " + method.getName());
                    }
                });
    }

    private static final class Gate {
        int writes;
        int enables;
        int disables;
        double position;
        final Servo servo = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
                new Class<?>[]{Servo.class, PwmControl.class}, (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "setPosition": writes++; position = (double) args[0]; return null;
                        case "setPwmEnable": enables++; return null;
                        case "setPwmDisable": disables++; return null;
                        default: throw new AssertionError("Unexpected gate call: " + method.getName());
                    }
                });
    }

    private static final class Motor {
        double power;
        double velocity;
        int ticks;
        final DcMotorEx servo = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "setPower": power = (double) args[0]; return null;
                        case "setVelocity": velocity = (double) args[0]; return null;
                        case "getPower": return power;
                        case "getVelocity": return velocity;
                        case "getCurrentPosition": return ticks;
                        default: throw new AssertionError("Unexpected motor call: " + method.getName());
                    }
                });
    }

    public static void main(String[] args) {
        Gate gate = new Gate();
        Settings defaults = new Settings();
        Robot.GateController builtIn = new Robot.GateController(gate.servo, defaults.prefs);
        check(builtIn.calibrated() && builtIn.usingCodeDefaults, "Fresh installs use the measured code defaults");
        check(gate.writes == 0 && gate.enables == 0, "Loading defaults must not move the gate");
        builtIn.commandSaved(true);
        near(gate.position, 0.39, "Code default OPEN must be the measured 0.39");
        builtIn.commandSaved(false);
        near(gate.position, 0.59, "Code default CLOSED must be the measured 0.59");
        builtIn.forgetCalibration();
        check(!new Robot.GateController(gate.servo, defaults.prefs).calibrated(),
                "Explicit calibration clear must also disable code defaults after restart");
        defaults.values.remove("disabled");
        defaults.values.put("open", 0.42f);
        check(!new Robot.GateController(gate.servo, defaults.prefs).calibrated(),
                "A partial saved pair must not mix with code defaults");
        gate = new Gate();
        Settings settings = new Settings();
        settings.values.put("disabled", true);
        Robot.GateController control = new Robot.GateController(gate.servo, settings.prefs);
        check(gate.writes == 0 && gate.enables == 0, "Loading must not move or energize the gate");
        check(!control.calibrated(), "Explicit disable must block code defaults");
        check(!control.commandSaved(true) && !control.commandSaved(false), "Missing endpoints block both recalls");
        check(!control.jogCalibration(1, false), "No jog before a known reference is established");
        check(!control.jogWithinSaved(1), "Robot Test must not bypass missing calibration");
        check(!control.markOpen() && !control.markClosed(), "Unknown shaft command cannot be marked");
        check(!control.saveMarks() && settings.commits == 0, "Both marks are required before saving");
        control.release();
        check(gate.writes == 0 && gate.disables == 1, "Release disables PWM without sending a position");

        control.establishMidpoint();
        near(gate.position, 0.5, "Explicit freed-linkage reference action");
        control.jogCalibration(1, false);
        near(gate.position, 0.51, "One normal calibration press must move 0.01");
        control.jogCalibration(-1, true);
        near(gate.position, 0.5095, "One fine press must move only 0.0005");
        int writes = gate.writes;
        check(control.markOpen(), "Mark known open endpoint");
        check(gate.writes == writes, "Marking must not move the servo");
        for (int i = 0; i < 5; i++) control.jogCalibration(-1, false);
        check(control.markClosed(), "Mark known closed endpoint");
        writes = gate.writes;
        check(control.saveMarks(), "Save two distinct endpoints atomically");
        check(gate.writes == writes && settings.commits == 1, "Saving must not move the servo");
        near(control.openPosition, 0.5095, "Open may be numerically greater than closed");
        near(control.closedPosition, 0.4595, "Closed endpoint preserves measured direction");

        Robot.GateController restored = new Robot.GateController(gate.servo, settings.prefs);
        check(restored.calibrated() && gate.writes == writes, "Reload saved endpoints without a startup jump");
        restored.commandSaved(false);
        near(gate.position, 0.4595, "Recall measured closed endpoint");
        restored.commandSaved(true);
        near(gate.position, 0.5095, "Recall measured open endpoint");
        restored.jogWithinSaved(1);
        near(gate.position, 0.5095, "Robot Test jogging stays inside saved travel");
        for (int i = 0; i < 30; i++) restored.jogWithinSaved(-1);
        near(gate.position, 0.4595, "Robot Test lower bound");
        restored.release();
        writes = gate.writes;
        check(Double.isNaN(restored.command), "Releasing PWM invalidates the commanded reference");
        check(!restored.jogCalibration(1, false) && gate.writes == writes,
                "PWM release cannot be undone by a stale calibration jog");

        restored.establishMidpoint();
        restored.markOpen();
        restored.markClosed();
        check(!restored.saveMarks(), "Identical marks must not replace a valid saved calibration");
        restored.jogCalibration(1, false);
        restored.markClosed();
        settings.storageWorks = false;
        check(!restored.saveMarks(), "Storage failure must be reported");
        near(restored.openPosition, 0.5095, "Failed save must retain previous active endpoints");
        settings.storageWorks = true;

        Robot robot = new Robot();
        Motor transmission = new Motor();
        Motor intake = new Motor();
        Motor shooter = new Motor();
        Motor turret = new Motor();
        robot.gate = gate.servo;
        Settings disabled = new Settings();
        disabled.values.put("disabled", true);
        robot.gateController = new Robot.GateController(gate.servo, disabled.prefs);
        robot.transmission = transmission.servo;
        robot.intake = intake.servo;
        robot.shooter = shooter.servo;
        robot.turret = turret.servo;
        robot.targetVisible = true;
        robot.targetTx = 0;
        robot.shooterOn = true;
        shooter.velocity = robot.targetRpm * Robot.SHOOTER_TICKS_PER_REV / 60;
        writes = gate.writes;
        robot.updateManualFeeder(true, 1);
        robot.updateFeeder(true, 0);
        check(gate.writes == writes, "TeleOp and Auto must never send old defaults while uncalibrated");
        near(transmission.power, 0, "Uncalibrated feeding must stop transmission");
        check(!robot.readyToShoot(), "Uncalibrated gate must not authorize automatic feeding");
        robot.gateController = new Robot.GateController(gate.servo, settings.prefs);
        robot.updateManualFeeder(true, 0);
        near(gate.position, 0.5095, "TeleOp toggle uses saved open");
        robot.updateManualFeeder(true, 0);
        near(gate.position, 0.4595, "TeleOp toggle uses saved closed");
        robot.ballPresent = true;
        robot.updateFeeder(true, 0);
        near(gate.position, 0.5095, "Automatic shot uses saved open");
        near(transmission.power, Robot.TRANSMISSION_FEED_POWER, "Calibrated shot feeds");
        robot.targetVisible = false;
        robot.updateFeeder(true, 0);
        near(gate.position, 0.4595, "Tag loss closes to the saved endpoint");

        check(restored.forgetCalibration(), "Clear calibration persistently");
        check(!restored.calibrated(), "Clear disables gate control immediately");
        check(!new Robot.GateController(gate.servo, settings.prefs).calibrated(), "Clear survives restart");
        settings.values.remove("disabled");
        for (float[] endpoints : new float[][]{{Float.NaN, 0.5f}, {-0.1f, 0.2f}, {0.1f, 1.1f}, {0.2f, 0.2f}}) {
            settings.values.put("open", endpoints[0]);
            settings.values.put("closed", endpoints[1]);
            Robot.GateController invalid = new Robot.GateController(gate.servo, settings.prefs);
            writes = gate.writes;
            check(!invalid.commandSaved(true) && gate.writes == writes, "Reject invalid stored endpoints");
        }

        turret.ticks = Robot.TURRET_MAX_TICKS;
        robot.aimTurret(false, 0.3);
        near(turret.power, 0, "Positive turret soft limit remains enforced");
        robot.aimTurret(false, -0.3);
        near(turret.power, -0.3, "Inward turret movement remains available");
        turret.ticks = Robot.TURRET_MIN_TICKS;
        robot.aimTurret(false, -0.3);
        near(turret.power, 0, "Negative turret soft limit remains enforced");

        Robot indicatorRobot = new Robot();
        Gate indicator = new Gate();
        indicatorRobot.led = indicator.servo;
        indicatorRobot.gateOpen = true;
        indicatorRobot.targetVisible = true;
        indicatorRobot.ballColor = Robot.BallColor.YELLOW;
        indicatorRobot.updateLed();
        near(indicator.position, Robot.LED_RED,
                "No pollen stays red even with an open gate, a tag, and a stale color");
        for (Robot.BallColor color : Robot.BallColor.values()) {
            indicatorRobot.ballColor = color;
            indicatorRobot.ballPresent = true;
            indicatorRobot.gateOpen = false;
            indicatorRobot.updateLed();
            near(indicator.position, Robot.LED_GREEN,
                    "Pollen of any color (including unclassified) is green without shooter hardware");
        }
        indicatorRobot.ballPresent = false;
        indicatorRobot.updateLed();
        near(indicator.position, Robot.LED_RED, "Removing pollen immediately restores red");
        double redPulseUs = 600 + 1800 * Robot.LED_RED;
        check(redPulseUs >= 1100 && redPulseUs < 1110,
                "Red must stay above the manufacturer's OFF boundary");
        indicatorRobot.led = null;
        indicatorRobot.updateLed();
        for (double[] point : new double[][]{{1.39, 3215}, {1.52, 3220}, {1.62, 3235},
                {1.455, 3217.5}, {1.57, 3227.5}, {0.5, 3215}, {3.0, 3235}}) {
            near(Robot.rpmForDistance(point[0]), point[1], "Distance calibration/interpolation/clamp");
        }
        for (double invalid : new double[]{Double.NaN, Double.POSITIVE_INFINITY,
                Double.NEGATIVE_INFINITY, 0, -1}) {
            near(Robot.rpmForDistance(invalid), 3215, "Invalid distance defaults to 3215");
        }
        Robot distanceRobot = new Robot();
        Motor distanceShooter = new Motor();
        distanceRobot.shooter = distanceShooter.servo;
        distanceRobot.shooterOn = true;
        distanceRobot.gateOpen = true;
        distanceRobot.ballColor = Robot.BallColor.BLUE;
        distanceRobot.targetFresh = true;
        distanceRobot.targetDistanceMeters = 1.62;
        distanceRobot.updateShooter(distanceRobot.distanceBasedShooterRpm());
        near(distanceRobot.targetRpm, 3235, "TeleOp uses distance even with an open gate and colored ball");
        near(distanceShooter.velocity, 3235 * Robot.SHOOTER_TICKS_PER_REV / 60,
                "Distance target reaches the motor velocity command");
        check(distanceRobot.shooterAtSpeed(), "Readiness uses the same distance target as the motor");
        distanceRobot.targetFresh = false;
        distanceRobot.targetVisible = true;
        distanceRobot.updateShooter(distanceRobot.distanceBasedShooterRpm());
        near(distanceRobot.targetRpm, 3215, "Tag loss defaults immediately despite held visibility/stale distance");
        near(distanceShooter.velocity, 3215 * Robot.SHOOTER_TICKS_PER_REV / 60,
                "Fallback RPM reaches the motor");
        distanceRobot.targetFresh = true;
        distanceRobot.targetDistanceMeters = Double.NaN;
        near(distanceRobot.distanceBasedShooterRpm(), 3215, "Visible tag without a pose uses fallback");
        distanceRobot.targetDistanceMeters = 1.52;
        distanceRobot.rpmTrim = 50;
        distanceRobot.updateShooter(distanceRobot.distanceBasedShooterRpm());
        near(distanceShooter.velocity, 3270 * Robot.SHOOTER_TICKS_PER_REV / 60,
                "Manual trim remains an offset to distance RPM");
        distanceRobot.shooterOn = false;
        distanceShooter.power = 1;
        distanceRobot.updateShooter(distanceRobot.distanceBasedShooterRpm());
        near(distanceShooter.power, 0, "Automatic distance targets do not turn the shooter on");
        distanceRobot.updateShooter();
        near(distanceRobot.targetRpm, 3220,
                "No-argument shooting also uses the distance table rather than old color presets");
        double[] presets = Robot.shooterCalibrationRpms();
        presets[0] = 9999;
        near(Robot.rpmForDistance(1.39), 3215, "Changing a test preset copy cannot corrupt the shared table");
        System.out.println("Passed " + checks + " gate/robot control regression checks.");
    }
}
