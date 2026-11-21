package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedWriter;
import java.io.StringWriter;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.function.Consumer;

/** Runs actual tuner callbacks with recording motors, a deterministic clock and button edges. */
public final class FlywheelTuningRegression {
    static int checks;
    static void near(double actual, double expected, String message) {
        checks++;
        if (Math.abs(actual - expected) > 1e-8 || !Double.isFinite(actual)) {
            throw new AssertionError(message + ": " + actual + " != " + expected);
        }
    }
    static void check(boolean condition, String message) {
        checks++;
        if (!condition) throw new AssertionError(message);
    }

    static final class Motor {
        double power, velocity, p, f;
        final DcMotorEx port = (DcMotorEx) Proxy.newProxyInstance(DcMotorEx.class.getClassLoader(),
                new Class<?>[]{DcMotorEx.class}, (proxy, method, args) -> {
                    switch (method.getName()) {
                        case "setPower": power = (double) args[0]; return null;
                        case "setVelocity": velocity = (double) args[0]; return null;
                        case "getVelocity": return velocity;
                        case "setVelocityPIDFCoefficients":
                            p = (double) args[0]; f = (double) args[3]; return null;
                        default: throw new AssertionError("Unexpected motor call: " + method.getName());
                    }
                });
    }

    static final class Test extends FlywheelTuning {
        long now = 1_000_000_000L;
        final Motor wheel = new Motor(), feeder = new Motor(), collector = new Motor();
        double gatePosition;
        final java.io.File folder;
        @Override protected java.io.File datalogFolder() { return folder; }
        Test() throws Exception {
            folder = java.nio.file.Files.createTempDirectory("flywheel-logs").toFile();
            gamepad1 = new Gamepad();
            telemetry = (Telemetry) Proxy.newProxyInstance(Telemetry.class.getClassLoader(),
                    new Class<?>[]{Telemetry.class}, (p, m, a) -> m.getName().equals("update") ? true : null);
            SharedPreferences prefs = (SharedPreferences) Proxy.newProxyInstance(
                    SharedPreferences.class.getClassLoader(), new Class<?>[]{SharedPreferences.class},
                    (p, m, a) -> {
                        if (m.getName().equals("contains")) return false;
                        if (m.getName().equals("getBoolean") || m.getName().equals("getFloat")) return a[1];
                        throw new AssertionError("Unexpected preference call: " + m.getName());
                    });
            Servo gate = (Servo) Proxy.newProxyInstance(Servo.class.getClassLoader(),
                    new Class<?>[]{Servo.class}, (p, m, a) -> {
                        if (m.getName().equals("setPosition")) { gatePosition = (double) a[0]; return null; }
                        throw new AssertionError("Unexpected gate call: " + m.getName());
                    });
            set("shooter", wheel.port);
            set("transmission", feeder.port);
            set("intake", collector.port);
            set("gateController", new Robot.GateController(gate, prefs));
        }
        @Override protected long nanoTime() { return now; }
        void set(String name, Object value) throws Exception {
            Field f = FlywheelTuning.class.getDeclaredField(name);
            f.setAccessible(true); f.set(this, value);
        }
        void tick(long elapsedMs) throws Exception {
            now += elapsedMs * 1_000_000L;
            Method edges = Gamepad.class.getDeclaredMethod("updateEdgeDetection");
            edges.setAccessible(true); edges.invoke(gamepad1);
            loop();
        }
        void press(Consumer<Gamepad> button) throws Exception {
            button.accept(gamepad1); tick(20);
            gamepad1 = new Gamepad(); tick(20);
        }
    }

    public static void main(String[] args) throws Exception {
        Test t = new Test();
        t.start(); t.tick(20);
        near(t.wheel.power, 0, "START keeps flywheel off");
        near(t.wheel.p, 0, "Tuning starts with feedback disabled");
        near(t.wheel.f, Robot.V2_TELEOP_SHOOTER_F, "Starts from V2 TeleOp's reference F; missing voltage falls back to it");
        t.press(g -> g.left_stick_button = true);
        near(t.wheel.velocity, 3215.0 * 28 / 60, "First target is 3215 RPM");
        t.press(g -> g.y = true);
        near(t.wheel.velocity, 3220.0 * 28 / 60, "Y advances to 3220 RPM");
        t.press(g -> g.y = true);
        near(t.wheel.velocity, 3235.0 * 28 / 60, "Y advances to 3235 RPM");
        t.press(g -> g.y = true);
        near(t.wheel.velocity, 3300.0 * 28 / 60, "Y advances to 3300 RPM");
        t.press(g -> g.y = true);
        near(t.wheel.velocity, 3215.0 * 28 / 60, "Y wraps to 3215 RPM");

        t.press(g -> g.dpad_right = true);
        near(t.wheel.f, Robot.V2_TELEOP_SHOOTER_F + 0.1, "F starts with 0.1 adjustments");
        t.press(g -> g.dpad_left = true);
        // 0.1 -> 0.01 -> 0.001 -> 0.0001 -> 10
        for (int i = 0; i < 4; i++) t.press(g -> g.b = true);
        t.press(g -> g.dpad_up = true);
        near(t.wheel.p, 10, "P can be tuned in ten-point steps");
        t.press(g -> g.back = true);
        near(t.wheel.p, 0, "Back returns immediately to F-only tuning");
        t.press(g -> g.right_stick_button = true);
        near(t.wheel.p, 90, "Right stick click jumps to the 90 preset");
        t.press(g -> g.right_stick_button = true);
        near(t.wheel.p, 150, "Then 150");
        t.press(g -> g.right_stick_button = true);
        near(t.wheel.p, 180, "Then 180");
        t.press(g -> g.right_stick_button = true);
        near(t.wheel.p, 90, "Presets wrap back to 90");
        near(t.wheel.f, Robot.V2_TELEOP_SHOOTER_F, "Presets change P only");
        t.press(g -> g.back = true);

        t.gamepad1.right_bumper = true; t.tick(20);
        near(t.feeder.power, 0, "RB cannot feed through a closed gate");
        t.gamepad1.x = true; t.tick(20);
        near(t.gatePosition, Robot.GATE_OPEN, "X opens the gate");
        near(t.feeder.power, 0.30, "Opening unloads the gate briefly outward");
        t.gamepad1.x = false; t.tick(110);
        near(t.feeder.power, 0, "Then wait for gate travel");
        t.tick(150);
        near(t.feeder.power, -Robot.TRANSMISSION_INDEX_POWER, "RB feeds in V2's inward direction");
        near(t.collector.power, -Robot.INTAKE_POWER, "Burst also runs the intake inward");
        t.gamepad1.right_bumper = false; t.tick(20);
        near(t.feeder.power, 0, "Releasing RB stops feeding");
        t.press(g -> g.x = true);
        t.gamepad1.right_trigger = 1; t.tick(20);
        near(t.collector.power, -Robot.INTAKE_POWER, "RT loads balls inward");
        near(t.feeder.power, -Robot.TRANSMISSION_INDEX_POWER, "Loading stacks balls at the closed gate");
        t.gamepad1.right_trigger = 0; t.gamepad1.left_trigger = 1; t.tick(20);
        near(t.feeder.power, Robot.TRANSMISSION_INDEX_POWER, "LT reverses the transmission");
        t.gamepad1 = new Gamepad();

        near(Robot.compensatedShooterF(14, 13.3), 14, "13.3 V is the common normalization point");
        near(Robot.compensatedShooterF(14, 12), 14 * 13.3 / 12, "F scales with measured sag");
        near(Robot.compensatedShooterF(14, Double.NaN), 14, "Invalid voltage does not create invalid coefficients");

        StringWriter csv = new StringWriter();
        BufferedWriter writer = new BufferedWriter(csv);
        t.set("logWriter", writer); t.set("recording", true); t.set("recordingStartNs", t.now);
        t.tick(20); writer.flush();
        String[] fields = csv.toString().trim().split(",", -1);
        check(fields.length == 23, "Burst CSV contains 23 fields, including loop time and volley data");
        near(Double.parseDouble(fields[13]), 13.3, "CSV records the fixed F reference");
        near(Double.parseDouble(fields[14]), t.wheel.f, "CSV distinguishes the applied hub F");
        near(Double.parseDouble(fields[15]), 1, "CSV records whether the flywheel is running");
        near(Double.parseDouble(fields[18]), 20, "CSV records the loop time in ms");
        t.set("recording", false); t.set("logWriter", null);

        // A full volley through the OpMode: the summary row lands in shooter_volleys.csv.
        double target = 3215.0 * 28 / 60;
        t.press(g -> g.x = true); // gate open
        t.gamepad1 = new Gamepad();
        for (int i = 0; i < 30; i++) { t.wheel.velocity = target; t.tick(10); }
        double[] trace = {target, target - 70, target - 60, target - 20, target - 90, target - 70, target - 40,
                target - 110, target - 90, target - 60, target - 130, target - 100};
        t.gamepad1.right_bumper = true;
        for (double v : trace) { t.wheel.velocity = v; t.tick(10); }
        t.gamepad1.right_bumper = false;
        for (int i = 0; i < 60; i++) { t.wheel.velocity = target; t.tick(10); }
        java.io.File volleys = new java.io.File(t.folder, FlywheelTuning.VOLLEY_FILE);
        check(volleys.exists(), "A finished volley writes shooter_volleys.csv without pressing record");
        java.util.List<String> rows = java.nio.file.Files.readAllLines(volleys.toPath());
        // The earlier feed checks already finished one volley with no dips (constant speed).
        check(rows.size() == 3 && rows.get(0).startsWith("time_ms,volley,P"), "Volley file: header + one row per volley");
        near(Double.parseDouble(rows.get(1).split(",", -1)[12]), 0, "A volley with no speed dips counts no balls");
        String[] row = rows.get(2).split(",", -1);
        check(row.length == 18, "Volley row has 18 columns");
        near(Double.parseDouble(row[12]), 4, "Four dips are counted as four balls");
        check(Double.parseDouble(row[8]) > Double.parseDouble(row[11]), "Later balls launch slower in this trace");
        check(!row[16].isEmpty(), "Recovery time is reported");

        // Analyzer edge cases.
        BurstAnalyzer a = new BurstAnalyzer();
        double ms = 0;
        for (int i = 0; i < 20; i++) a.sample(ms += 10, 3250, 3250, false);
        for (double r : new double[]{3250, 3180, 3120, 3060, 3000, 2950}) a.sample(ms += 10, r, 3250, true);
        for (int i = 0; i < 200; i++) a.sample(ms += 10, 2950, 3250, false);
        check(!a.inVolley && a.dips == 1, "Two balls with no rise between them merge into one dip");
        check(Double.isNaN(a.recoveryMs), "No recovery within the window is reported as missing");
        near(a.drop(), 300, "Drop is measured from the pre-volley baseline");
        check(FlywheelTuning.gcd(1460, 1480) == 20 && FlywheelTuning.gcd(0, 1500) == 1500,
                "Velocity step detection uses the greatest common step");
        t.stop();
        near(t.wheel.power, 0, "STOP turns off the flywheel");
        near(t.feeder.power, 0, "STOP turns off the transmission");
        near(t.collector.power, 0, "STOP turns off the intake");
        near(t.gatePosition, Robot.GATE_CLOSED, "STOP closes the gate");
        System.out.println("Passed " + checks + " flywheel tuning regression checks.");
    }
}
