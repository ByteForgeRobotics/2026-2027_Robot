package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

/**
 * The shooter flywheel. Holds a set RPM with a PIDF loop so shots go the same distance as the
 * battery drains and after each ball pulls the speed down.
 *
 * <p>Call {@link #update()} once every loop, or nothing happens.
 *
 * <h2>Tuning (Mechanism Test OpMode + FTC Dashboard graph of target vs rpm)</h2>
 * <ol>
 *   <li>Set KP, KI, KD to 0. Raise KF until the rpm sits just under the target at a few
 *   different targets.</li>
 *   <li>Raise KP until it recovers fast after a shot without bouncing above the target.</li>
 *   <li>Only add KI if it always settles a little low. Usually not needed.</li>
 * </ol>
 * KF is scaled by {@code 12 V / battery voltage}, so a value tuned on a full battery still works
 * on a tired one.
 */
@Config
public class Flywheel {

    /** Configuration name on the Driver Station. */
    public static String NAME = "flywheel";
    public static boolean REVERSE = false;

    /** Encoder ticks per output-shaft turn. goBILDA 5203 1:1 (6000 rpm) = 28. */
    public static double TICKS_PER_REV = 28;

    public static double KP = 0.0004;
    public static double KI = 0.0;
    public static double KD = 0.0;
    /** Power per rpm. About 1 / free speed: 1 / 6000 for a bare 5203. */
    public static double KF = 0.000175;
    public static double KS = 0.02;

    /** {@link #atSpeed()} is true within this many rpm of the target. */
    public static double TOLERANCE_RPM = 75;

    /** Named speeds. Set the real ones once the shooter is built. */
    public static double RPM_CLOSE = 2800;
    public static double RPM_FAR = 3600;

    private final DcMotorEx motor;
    private final VoltageSensor battery;
    private final PIDFController pid = new PIDFController();

    private double targetRpm;
    private double rpm;
    private double power;
    private double volts = 12;
    private int loopsSinceVoltage = 1000;

    public Flywheel(HardwareMap hardwareMap) {
        motor = hardwareMap.tryGet(DcMotorEx.class, NAME);
        if (motor != null) {
            // Coast, not brake: slamming a heavy flywheel to a stop is hard on the gearbox.
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // our loop, not the hub's
        }
        battery = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next() : null;
    }

    public boolean isConnected() {
        return motor != null;
    }

    public void setRpm(double rpm) {
        if (rpm != targetRpm) {
            pid.reset();
        }
        targetRpm = Math.max(0, rpm);
    }

    public void stop() {
        setRpm(0);
    }

    /** Runs the loop. Call once per OpMode loop. */
    public void update() {
        if (motor == null) {
            return;
        }
        motor.setDirection(REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        rpm = motor.getVelocity() / TICKS_PER_REV * 60.0;

        // Reading the battery is a hub call, so only do it every 25 loops.
        if (battery != null && ++loopsSinceVoltage >= 25) {
            loopsSinceVoltage = 0;
            double v = battery.getVoltage();
            if (v > 6) {
                volts = v;
            }
        }

        if (targetRpm <= 0) {
            power = 0; // let it coast down
        } else {
            double kfScaled = KF * 12.0 / volts;
            power = pid.calculate(targetRpm, rpm, KP, KI, KD, kfScaled, KS);
            // Never drive it backwards to slow down; just ease off.
            power = Math.max(0, Math.min(1, power));
        }
        motor.setPower(power);
    }

    /** True when spinning and close enough to the target to shoot. */
    public boolean atSpeed() {
        return targetRpm > 0 && Math.abs(targetRpm - rpm) <= TOLERANCE_RPM;
    }

    public double getRpm() {
        return rpm;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getPower() {
        return power;
    }
}
