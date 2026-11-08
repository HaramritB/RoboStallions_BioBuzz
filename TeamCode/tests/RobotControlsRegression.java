package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.HashMap;
import java.util.Map;

/** Runs the real Robot methods against recording hardware proxies, without moving a robot. */
public final class RobotControlsRegression {
    private static int checks;

    private static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    private static void near(double actual, double expected, String message) {
        check(Math.abs(actual - expected) < 1e-6, message + ": " + actual + " != " + expected);
    }

    private static final class Motor implements InvocationHandler {
        double power;
        double velocity;
        int ticks;
        boolean enabled = true;
        final DcMotorEx device = (DcMotorEx) Proxy.newProxyInstance(
                DcMotorEx.class.getClassLoader(), new Class<?>[]{DcMotorEx.class}, this);

        @Override public Object invoke(Object proxy, Method method, Object[] args) {
            switch (method.getName()) {
                case "setPower": power = (double) args[0]; return null;
                case "getPower": return power;
                case "getCurrentPosition": return ticks;
                case "getVelocity": return velocity;
                case "isMotorEnabled": return enabled;
                case "setMotorEnable": enabled = true; return null;
                case "setMode": return null;
                default: throw new AssertionError("Unexpected motor call " + method.getName());
            }
        }
    }

    private static final class Fixture {
        final Robot robot = new Robot();
        final Motor turret = new Motor();
        final Motor transmission = new Motor();
        final Motor intake = new Motor();
        final Motor shooter = new Motor();
        final Map<String, Float> stored = new HashMap<>();
        int servoWrites;
        double servoPosition;

        Fixture() throws Exception {
            robot.turret = turret.device;
            robot.transmission = transmission.device;
            robot.intake = intake.device;
            robot.shooter = shooter.device;
            robot.gatePosition = 0;
            robot.gate = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
                    new Class<?>[]{Servo.class}, (proxy, method, args) -> {
                        if (method.getName().equals("setPosition")) {
                            servoWrites++;
                            servoPosition = (double) args[0];
                            return null;
                        }
                        throw new AssertionError("Unexpected servo call " + method.getName());
                    });
            SharedPreferences.Editor editor = (SharedPreferences.Editor) Proxy.newProxyInstance(
                    SharedPreferences.Editor.class.getClassLoader(),
                    new Class<?>[]{SharedPreferences.Editor.class}, (proxy, method, args) -> {
                        if (method.getName().equals("putFloat")) {
                            stored.put((String) args[0], (float) args[1]);
                            return proxy;
                        }
                        if (method.getName().equals("apply")) return null;
                        throw new AssertionError("Unexpected editor call " + method.getName());
                    });
            SharedPreferences prefs = (SharedPreferences) Proxy.newProxyInstance(
                    SharedPreferences.class.getClassLoader(), new Class<?>[]{SharedPreferences.class},
                    (proxy, method, args) -> {
                        if (method.getName().equals("edit")) return editor;
                        throw new AssertionError("Unexpected preferences call " + method.getName());
                    });
            Field field = Robot.class.getDeclaredField("gatePreferences");
            field.setAccessible(true);
            field.set(robot, prefs);
        }
    }

    public static void main(String[] args) throws Exception {
        Fixture f = new Fixture();
        Robot r = f.robot;
        r.targetVisible = true;
        r.shooterOn = true;
        f.shooter.velocity = r.targetRpm * Robot.SHOOTER_TICKS_PER_REV / 60;
        r.updateFeeder(true, 1);
        check(f.servoWrites == 0, "Uncalibrated feeding must not move the servo");
        near(f.transmission.power, 0, "Uncalibrated feeding must stop transmission");
        r.resumeAutomaticGate();
        r.updateFeeder(true, 0);
        check(f.servoWrites == 0, "Shoot must not bypass missing calibration");

        r.saveGateOpenPosition();
        check(f.servoWrites == 0, "Saving the current open command must not move the servo");
        near(f.stored.get("open"), 0, "Open endpoint must be persisted");
        check(!r.gateCalibrated(), "One endpoint is insufficient");
        r.saveGateClosedPosition();
        check(!r.gateCalibrated(), "Identical endpoints must not enable feeding");
        r.jogGate(1);
        near(f.servoPosition, 0.005, "First nudge must be a small step, not a midpoint jump");
        r.saveGateClosedPosition();
        check(r.gateCalibrated(), "Distinct endpoints must enable feeding");
        near(f.stored.get("closed"), 0.005, "Closed endpoint must be persisted");

        int writes = f.servoWrites;
        r.updateFeeder(true, 1);
        check(f.servoWrites == writes, "Feeder must not overwrite a manual adjustment");
        near(f.transmission.power, 0, "Manual gate must stop automatic transmission");
        r.gateOpenPosition = 0.8;
        r.gateClosedPosition = 0.2;
        check(r.gateCalibrated(), "Open may be numerically greater than closed");
        r.openGateManually();
        near(f.servoPosition, 0.8, "Manual open must use the saved endpoint");
        r.closeGateManually();
        near(f.servoPosition, 0.2, "Manual close must use the saved endpoint");
        r.resumeAutomaticGate();
        r.updateFeeder(false, 0);
        near(f.servoPosition, 0.2, "Automatic idle must use calibrated closed");
        r.ballPresent = true;
        r.updateFeeder(true, 0);
        near(f.servoPosition, 0.8, "Automatic shot must use calibrated open");
        near(f.transmission.power, Robot.TRANSMISSION_FEED_POWER, "Shot must feed");
        r.targetVisible = false;
        r.updateFeeder(true, 0);
        near(f.servoPosition, 0.2, "Tag loss must close automatic gate");
        r.gatePosition = 0.999;
        r.jogGate(1);
        near(f.servoPosition, 1, "Nudges must stay within servo range");
        r.gatePosition = 0.001;
        r.jogGate(-1);
        near(f.servoPosition, 0, "Negative nudges must stay within servo range");

        f.turret.ticks = 0;
        f.turret.enabled = false;
        r.aimTurret(false, 0.3);
        check(f.turret.enabled, "Manual turret command must enable a disabled motor");
        near(f.turret.power, 0.3, "Manual turret must work without a visible tag");
        r.aimTurret(false, -1);
        near(f.turret.power, -Robot.TURRET_MAX_POWER, "Turret power must remain capped");
        f.turret.ticks = (int) Math.ceil(Robot.TURRET_MAX_DEG * Robot.TURRET_TICKS_PER_DEG);
        r.aimTurret(false, 0.3);
        check(r.turretLimitBlocked, "Positive soft limit must be reported");
        near(f.turret.power, 0, "Positive soft limit must block outward movement");
        r.aimTurret(false, -0.3);
        near(f.turret.power, -0.3, "Positive soft limit must allow inward movement");
        f.turret.ticks = (int) Math.floor(Robot.TURRET_MIN_DEG * Robot.TURRET_TICKS_PER_DEG);
        r.aimTurret(false, -0.3);
        check(r.turretLimitBlocked, "Negative soft limit must be reported");
        near(f.turret.power, 0, "Negative soft limit must block outward movement");
        r.aimTurret(false, 0.3);
        near(f.turret.power, 0.3, "Negative soft limit must allow inward movement");
        System.out.println("Passed " + checks + " robot control regression checks.");
    }
}
