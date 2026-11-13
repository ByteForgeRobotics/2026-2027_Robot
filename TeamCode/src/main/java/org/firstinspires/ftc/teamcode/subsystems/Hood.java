package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The shooter hood. A motor with an encoder, held at a target position (in encoder ticks) by a
 * PID loop.
 *
 * <p><b>Zero:</b> the encoder is zeroed when this object is created, so the hood has to be
 * all the way down (against its stop) when the OpMode is INITed. Every position is counted up
 * from there. If it gets knocked off, push it down and call {@link #zeroHere()}.
 *
 * <p>Call {@link #update()} once every loop, or nothing happens.
 *
 * <h2>Directions: two separate flags</h2>
 * {@code setDirection} flips the motor AND its encoder together, so it can't fix an encoder
 * that counts the opposite way from its own motor (the Maverick does this on a REV hub). Set
 * them in this order, robot on blocks, in Mechanism Test:
 * <ol>
 *   <li><b>REVERSE</b>: pick it so that dpad up (positive target) moves the hood UP. Look at the
 *   hood, not the numbers.</li>
 *   <li><b>ENCODER_REVERSED</b>: pick it so that pushing the hood UP by hand makes the ticks go
 *   UP. If it's wrong, the hood runs away to one end and the count races negative; the
 *   runaway cutoff below stops it and the screen says to flip this.</li>
 * </ol>
 *
 * <h2>Tuning (Mechanism Test OpMode)</h2>
 * <ol>
 *   <li>Push the hood by hand from down to fully up and read the ticks. Put that number,
 *   minus a little, in MAX_TICKS.</li>
 *   <li>Raise KP until it gets to the target quickly. If it overshoots and wobbles, add a
 *   little KD.</li>
 *   <li>If it stops just short every time, raise KS (the push needed to beat friction).</li>
 *   <li>If gravity pulls it down when it should be holding, raise KG.</li>
 * </ol>
 */
@Config
public class Hood {

    /** Configuration name on the Driver Station. */
    public static String NAME = "hood";
    /** Motor direction (also flips the encoder with it). See the class notes. */
    public static boolean REVERSE = true;
    /**
     * Flips only the encoder reading. The hood counting straight into the negatives while the
     * PID drives it means this is wrong. See the class notes.
     */
    public static boolean ENCODER_REVERSED = true;
    /**
     * Runaway cutoff: if the hood reads this many ticks past either soft limit, power is cut and
     * {@link #isFaulted()} turns true. Protects the gears while the flags above are being set.
     */
    public static int RUNAWAY_TICKS = 150;

    public static double KP = 0.01;
    public static double KI = 0.0;
    public static double KD = 0.0004;
    /** Push in the direction it needs to go, to beat friction. Only applied outside tolerance. */
    public static double KS = 0.05;
    /** Constant power to hold against gravity. Leave at 0 if the hood stays put unpowered. */
    public static double KG = 0.0;

    public static double MAX_POWER = 0.6;
    /**
     * Soft limits. The target is clamped between these. The CAD allows about 37 deg of travel
     * (~850 ticks); push the hood by hand end to end in Mechanism Test and set this a little
     * under what you read.
     */
    public static int MIN_TICKS = 0;
    public static int MAX_TICKS = 800;

    /**
     * Maverick 61:1 (1464 ticks per output turn) through the 32:180 sector = 22.875 ticks per
     * degree of hood, and the launch angle moves 1:1 with the hood.
     */
    public static double TICKS_PER_DEG = 1464.0 * 180.0 / 32.0 / 360.0;
    /**
     * Launch angle (deg above horizontal) when the hood sits at its zero / INIT spot.
     * PLACEHOLDER from CAD: measure it on the robot with a phone angle app on the hood lip.
     * If raising the tick count lowers the angle, flip REVERSE.
     */
    public static double ANGLE_AT_ZERO_DEG = 43.0;
    /** {@link #atTarget()} is true within this many ticks. */
    public static int TOLERANCE_TICKS = 5;

    /** Named positions. Set the real ones once the shooter is built. */
    public static int POS_DOWN = 0;
    public static int POS_CLOSE = 80;
    public static int POS_FAR = 200;

    private final DcMotorEx motor;
    private final PIDFController pid = new PIDFController();

    private int target;
    private int position;
    private double power;
    private boolean faulted;

    public Hood(HardwareMap hardwareMap) {
        motor = hardwareMap.tryGet(DcMotorEx.class, NAME);
        if (motor != null) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            applyDirection();
            zeroHere();
        }
    }

    public boolean isConnected() {
        return motor != null;
    }

    /** Calls the hood's current spot 0 and holds it there. */
    public void zeroHere() {
        if (motor == null) {
            return;
        }
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        target = 0;
        position = 0;
        faulted = false; // re-zeroing is how you clear a runaway
        pid.reset();
    }

    public void setTarget(int ticks) {
        int clamped = Math.max(MIN_TICKS, Math.min(MAX_TICKS, ticks));
        if (clamped != target) {
            pid.reset();
        }
        target = clamped;
    }

    /** Points the hood for this launch angle (deg above horizontal). */
    public void setAngleDeg(double deg) {
        setTarget((int) Math.round((deg - ANGLE_AT_ZERO_DEG) * TICKS_PER_DEG));
    }

    /** Current launch angle from the encoder. */
    public double getAngleDeg() {
        return ANGLE_AT_ZERO_DEG + position / TICKS_PER_DEG;
    }

    /** Moves the target by this many ticks. Handy for a manual trim on the dpad. */
    public void nudge(int ticks) {
        setTarget(target + ticks);
    }

    /** Runs the loop. Call once per OpMode loop. */
    public void update() {
        if (motor == null) {
            return;
        }
        applyDirection();
        int raw = motor.getCurrentPosition();
        position = ENCODER_REVERSED ? -raw : raw;

        if (position < MIN_TICKS - RUNAWAY_TICKS || position > MAX_TICKS + RUNAWAY_TICKS) {
            faulted = true;
        }
        if (faulted) {
            power = 0;
            motor.setPower(0);
            return;
        }

        int error = target - position;
        // kS is only wanted while moving; inside tolerance it would just buzz the motor.
        double ks = Math.abs(error) > TOLERANCE_TICKS ? KS : 0;
        power = pid.calculate(target, position, KP, KI, KD, 0, 0)
                + Math.copySign(ks, error) + KG;
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
        motor.setPower(power);
    }

    public boolean atTarget() {
        return !faulted && Math.abs(target - position) <= TOLERANCE_TICKS;
    }

    /**
     * True after a runaway: the hood went way past a soft limit, so power was cut. Almost always
     * means ENCODER_REVERSED is wrong. Fix the flag, put the hood down, then {@link #zeroHere()}.
     */
    public boolean isFaulted() {
        return faulted;
    }

    public void stop() {
        if (motor != null) {
            motor.setPower(0);
        }
    }

    public int getPosition() {
        return position;
    }

    public int getTarget() {
        return target;
    }

    public double getPower() {
        return power;
    }

    private void applyDirection() {
        motor.setDirection(REVERSE ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }
}
