package org.firstinspires.ftc.teamcode.sim;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Laptop stand-ins for robot hardware, so subsystems can be tested without a Control Hub.
 *
 * <p>{@link SimMotor} is a {@link DcMotorEx} with simple physics: speed chases
 * {@code power x battery/12 x free speed} with a time constant, and the encoder integrates it.
 * It copies the SDK's rule that {@code setDirection} flips the power AND the encoder together,
 * and can also fake an encoder that counts the opposite way from its motor (the Maverick quirk
 * the hood code has a flag for).
 *
 * <p>Timing is real: subsystems use {@code ElapsedTime}, so {@link #run} sleeps a few ms per
 * loop like a real OpMode loop would take.
 */
public final class Sim {

    private Sim() {}

    /** A HardwareMap that only knows the devices you put in it. */
    public static class FakeHardwareMap extends HardwareMap {
        private final Map<String, Object> devices = new HashMap<>();

        public FakeHardwareMap() {
            super(null, null);
        }

        public FakeHardwareMap add(String name, Object device) {
            devices.put(name, device);
            return this;
        }

        public FakeHardwareMap battery(double volts) {
            VoltageSensor v = (VoltageSensor) Proxy.newProxyInstance(Sim.class.getClassLoader(),
                    new Class<?>[]{VoltageSensor.class}, (p, m, a) -> {
                        if (m.getName().equals("getVoltage")) return volts;
                        return defaultFor(m);
                    });
            voltageSensor.put("Control Hub", v);
            return this;
        }

        @Override
        @SuppressWarnings("unchecked")
        public <T> T tryGet(Class<? extends T> c, String name) {
            Object o = devices.get(name);
            return c.isInstance(o) ? (T) o : null;
        }
    }

    /** A simulated DC motor + encoder. */
    public static class SimMotor implements InvocationHandler {
        public final double ticksPerRev;
        public final double freeRpm;
        public final double tauS;
        public double batteryVolts = 12.0;
        /** Hardware quirk: encoder counts opposite to the motor it's on. */
        public boolean encoderInverted = false;
        /** Wheel held still (a jam). */
        public boolean stalled = false;
        /** Mechanical stops on the physical (motor-frame) position, ticks. NaN = none. */
        public double stopLow = Double.NaN, stopHigh = Double.NaN;
        /** Constant torque as a fraction of stall power, physical direction (gravity on a hood). */
        public double loadPower = 0.0;

        public DcMotorSimple.Direction direction = DcMotorSimple.Direction.FORWARD;
        public DcMotor.ZeroPowerBehavior zeroPower = DcMotor.ZeroPowerBehavior.UNKNOWN;
        public DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
        public double power;               // as commanded (before direction)
        public double physPos, physVel;    // ticks, ticks/s, motor frame
        private double encoderZero;
        public final List<Double> powerLog = new ArrayList<>();

        public final DcMotorEx motor;

        public SimMotor(double ticksPerRev, double freeRpm, double tauS) {
            this.ticksPerRev = ticksPerRev;
            this.freeRpm = freeRpm;
            this.tauS = tauS;
            motor = (DcMotorEx) Proxy.newProxyInstance(Sim.class.getClassLoader(),
                    new Class<?>[]{DcMotorEx.class}, this);
        }

        private double dirSign() {
            return direction == DcMotorSimple.Direction.REVERSE ? -1 : 1;
        }

        private double encSign() {
            return dirSign() * (encoderInverted ? -1 : 1);
        }

        /** Advances the physics by dt seconds. */
        public void step(double dt) {
            if (stalled) {
                physVel = 0;
                return;
            }
            double free = freeRpm / 60.0 * ticksPerRev;
            double applied = Math.max(-1, Math.min(1, power * dirSign() + loadPower));
            double target = applied * batteryVolts / 12.0 * free;
            physVel += (target - physVel) * Math.min(1, dt / tauS);
            physPos += physVel * dt;
            if (!Double.isNaN(stopLow) && physPos < stopLow) { physPos = stopLow; physVel = Math.max(0, physVel); }
            if (!Double.isNaN(stopHigh) && physPos > stopHigh) { physPos = stopHigh; physVel = Math.min(0, physVel); }
        }

        /** Knocks the speed down by this fraction, like a ball leaving the flywheel. */
        public void hit(double fraction) {
            physVel *= 1 - fraction;
        }

        public double rpm() {
            return physVel * dirSign() / ticksPerRev * 60.0;
        }

        @Override
        public Object invoke(Object proxy, Method m, Object[] a) {
            switch (m.getName()) {
                case "setPower": power = (Double) a[0]; powerLog.add(power); return null;
                case "getPower": return power;
                case "setDirection": direction = (DcMotorSimple.Direction) a[0]; return null;
                case "getDirection": return direction;
                case "setZeroPowerBehavior": zeroPower = (DcMotor.ZeroPowerBehavior) a[0]; return null;
                case "getZeroPowerBehavior": return zeroPower;
                case "setMode":
                    mode = (DcMotor.RunMode) a[0];
                    if (mode == DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                        encoderZero = physPos;
                        power = 0;
                    }
                    return null;
                case "getMode": return mode;
                case "getCurrentPosition": return (int) Math.round((physPos - encoderZero) * encSign());
                case "getVelocity":
                    if (a == null || a.length == 0) return physVel * encSign();
                    return defaultFor(m);
                case "getDeviceName": return "SimMotor";
                case "toString": return "SimMotor";
                case "hashCode": return System.identityHashCode(proxy);
                case "equals": return proxy == a[0];
                default: return defaultFor(m);
            }
        }
    }

    public static Object defaultFor(Method m) {
        Class<?> r = m.getReturnType();
        if (r == boolean.class) return false;
        if (r == int.class) return 0;
        if (r == long.class) return 0L;
        if (r == double.class) return 0.0;
        if (r == float.class) return 0f;
        return null;
    }

    /** A Telemetry that remembers every line, so tests can read what the drivers would see. */
    public static class FakeTelemetry implements InvocationHandler {
        public final List<String> lines = new ArrayList<>();
        public final Telemetry telemetry = (Telemetry) Proxy.newProxyInstance(
                Sim.class.getClassLoader(), new Class<?>[]{Telemetry.class}, this);

        @Override
        public Object invoke(Object proxy, Method m, Object[] a) {
            if (m.getName().equals("addLine") && a != null && a.length == 1) lines.add(String.valueOf(a[0]));
            if (m.getName().equals("addData") && a != null) {
                StringBuilder sb = new StringBuilder(String.valueOf(a[0])).append(": ");
                if (a.length == 2) sb.append(a[1]);
                else if (a.length >= 3 && a[1] instanceof String) sb.append(String.format((String) a[1], (Object[]) a[2]));
                lines.add(sb.toString());
            }
            if (m.getName().equals("clear") || m.getName().equals("clearAll")) lines.clear();
            return defaultFor(m);
        }

        public String all() {
            return String.join("\n", lines);
        }
    }

    /**
     * Feeds a button state into a gamepad the way the SDK does each loop (copy from a fresh
     * state), so the xWasPressed() edge detectors work.
     */
    public static void press(Gamepad gp, java.util.function.Consumer<Gamepad> buttons) {
        Gamepad down = new Gamepad();
        buttons.accept(down);
        gp.copy(down);
    }

    public static void release(Gamepad gp) {
        gp.copy(new Gamepad());
    }

    /** Runs {@code loop} for {@code seconds} of real time, stepping the motors between loops. */
    public static void run(double seconds, Runnable loop, SimMotor... motors) {
        long end = System.nanoTime() + (long) (seconds * 1e9);
        long last = System.nanoTime();
        while (System.nanoTime() < end) {
            long now = System.nanoTime();
            double dt = (now - last) / 1e9;
            last = now;
            for (SimMotor m : motors) m.step(dt);
            loop.run();
            sleep(5);
        }
    }

    public static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
