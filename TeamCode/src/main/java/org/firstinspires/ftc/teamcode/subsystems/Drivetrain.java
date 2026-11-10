package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The four mecanum wheels. Takes robot-relative commands in the same frame as
 * {@link Odometry} (+forward, +left, +counter-clockwise), so an auto can hand it controller
 * output without flipping signs.
 */
@Config
public class Drivetrain {

    public static double STRAFE_MULTIPLIER = 1.1;
    public static boolean REVERSE_FL = true;
    public static boolean REVERSE_FR = false;
    public static boolean REVERSE_BL = true;
    public static boolean REVERSE_BR = false;

    public static final String[] NAMES = {"FL", "FR", "BL", "BR"};
    private static final int FL = 0, FR = 1, BL = 2, BR = 3;

    private final DcMotorEx[] motors = new DcMotorEx[NAMES.length];
    private final double[] powers = new double[NAMES.length];

    public Drivetrain(HardwareMap hardwareMap) {
        for (int i = 0; i < NAMES.length; i++) {
            motors[i] = hardwareMap.tryGet(DcMotorEx.class, NAMES[i]);
            if (motors[i] != null) {
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        applyDirections();
    }

    /** True if the motor at this index (see {@link #NAMES}) was found. */
    public boolean hasMotor(int i) {
        return motors[i] != null;
    }

    /**
     * Drives relative to the robot. Each input is roughly -1 to 1; if they add up past 1 the
     * whole set is scaled down so the direction stays the same.
     *
     * @param forward +forward
     * @param left    +strafe left
     * @param turn    +counter-clockwise
     */
    public void drive(double forward, double left, double turn) {
        // Re-applied every call so flipping a flag in the dashboard takes effect live.
        // Free: the SDK only stores the direction, it doesn't talk to the hub.
        applyDirections();

        double y = forward;
        double x = -left * STRAFE_MULTIPLIER; // the mixing below wants +right
        double rx = -turn;                    // and +clockwise

        double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        powers[FL] = (y + x + rx) / denom;
        powers[BL] = (y - x + rx) / denom;
        powers[FR] = (y - x - rx) / denom;
        powers[BR] = (y + x - rx) / denom;

        for (int i = 0; i < NAMES.length; i++) {
            if (motors[i] != null) {
                // The SDK skips the hub write when the power hasn't changed.
                motors[i].setPower(powers[i]);
            }
        }
    }

    public void stop() {
        for (DcMotorEx m : motors) {
            if (m != null) {
                m.setPower(0);
            }
        }
    }

    private void applyDirections() {
        setDirection(FL, REVERSE_FL);
        setDirection(FR, REVERSE_FR);
        setDirection(BL, REVERSE_BL);
        setDirection(BR, REVERSE_BR);
    }

    private void setDirection(int i, boolean reverse) {
        if (motors[i] != null) {
            motors[i].setDirection(reverse
                    ? DcMotorSimple.Direction.REVERSE
                    : DcMotorSimple.Direction.FORWARD);
        }
    }
}
