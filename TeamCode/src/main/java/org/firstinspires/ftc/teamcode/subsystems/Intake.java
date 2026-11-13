package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * The intake roller. Holds a set RPM with a PIDF loop, so it doesn't bog down when it grabs a
 * game piece. Positive rpm pulls in, negative spits out.
 *
 * <p>Call {@link #update()} once every loop, or nothing happens.
 *
 * <p>Tune the same way as {@link Flywheel}: KF first with the others at 0, then KP.
 *
 * <p>{@link #isJammed()} turns true if the roller is told to spin but has been stuck under
 * {@code JAM_FRACTION} of its target for {@code JAM_S} seconds, so TeleOp can warn the driver
 * or reverse it.
 */
@Config
public class Intake {

    /** Configuration name on the Driver Station. */
    public static String NAME = "intake";
    public static boolean REVERSE = false;

    /** Encoder ticks per output-shaft turn. goBILDA 5203 13.7:1 (435 rpm) = 384.5. */
    public static double TICKS_PER_REV = 384.5;

    public static double KP = 0.003;
    public static double KI = 0.0;
    public static double KD = 0.0;
    /** Power per rpm. About 1 / free speed: 1 / 435 for a 435 rpm motor. */
    public static double KF = 0.0023;
    public static double KS = 0.03;

    /** Keep under ~400 so there is headroom when it grabs a piece on a low battery. */
    public static double RPM_IN = 350;
    public static double RPM_OUT = -280;

    public static double JAM_FRACTION = 0.2;
    public static double JAM_S = 0.4;

    private final DcMotorEx motor;
    private final PIDFController pid = new PIDFController();
    private final ElapsedTime slowTimer = new ElapsedTime();

    private double targetRpm;
    private double rpm;
    private double power;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.tryGet(DcMotorEx.class, NAME);
        if (motor != null) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public boolean isConnected() {
        return motor != null;
    }

    public void setRpm(double rpm) {
        if (rpm != targetRpm) {
            pid.reset();
            slowTimer.reset();
        }
        targetRpm = rpm;
    }

    public void in() {
        setRpm(RPM_IN);
    }

    public void out() {
        setRpm(RPM_OUT);
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

        if (targetRpm == 0) {
            power = 0;
        } else {
            power = pid.calculate(targetRpm, rpm, KP, KI, KD, KF, KS);
            power = Math.max(-1, Math.min(1, power));
        }
        motor.setPower(power);

        // Restart the jam timer whenever it's actually spinning fast enough.
        if (targetRpm == 0 || Math.abs(rpm) >= Math.abs(targetRpm) * JAM_FRACTION) {
            slowTimer.reset();
        }
    }

    public boolean isJammed() {
        return targetRpm != 0 && slowTimer.seconds() > JAM_S;
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
