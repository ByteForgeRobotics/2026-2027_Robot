package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * A plain PIDF loop shared by the hood, flywheel and intake.
 *
 * <pre>
 * power = kF * target + kS * sign(target) + kP * error + kI * integral + kD * d(error)/dt
 * </pre>
 *
 * <ul>
 *   <li><b>kF</b> (feedforward): power per unit of target. For a velocity loop this does most
 *   of the work: it's the power that would hold that speed with no error at all. Tune it
 *   first.</li>
 *   <li><b>kS</b> (static): the small power it takes just to get the motor turning against
 *   friction. Added in the direction of the target.</li>
 *   <li><b>kP / kI / kD</b>: fix whatever error is left.</li>
 * </ul>
 *
 * <p>Gains are passed in on every {@link #calculate} call instead of stored, so a subsystem
 * can keep them as {@code public static} fields and they change live from FTC Dashboard.
 */
public class PIDFController {

    /** The integral can't push more than this much power on its own. Stops wind-up. */
    public double integralLimit = 0.3;

    private final ElapsedTime timer = new ElapsedTime();
    private double integral;
    private double lastError;
    private boolean first = true;

    public double calculate(double target, double measured,
                            double kP, double kI, double kD, double kF, double kS) {
        double error = target - measured;
        double dt = timer.seconds();
        timer.reset();

        double derivative = 0;
        if (first || dt <= 0 || dt > 0.5) {
            // First loop, or the loop stalled: no trustworthy rate, so skip D and I this once.
            first = false;
        } else {
            derivative = (error - lastError) / dt;
            integral += error * dt;
            if (kI != 0) {
                double cap = integralLimit / Math.abs(kI);
                integral = Math.max(-cap, Math.min(cap, integral));
            }
        }
        lastError = error;

        double staticFf = target == 0 ? 0 : Math.copySign(kS, target);
        return kF * target + staticFf + kP * error + kI * integral + kD * derivative;
    }

    /** Forget the integral and the last error. Call it when the target jumps. */
    public void reset() {
        integral = 0;
        lastError = 0;
        first = true;
        timer.reset();
    }
}
