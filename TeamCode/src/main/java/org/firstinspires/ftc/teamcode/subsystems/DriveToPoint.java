package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Drives the robot in a straight line to a point on the field while holding a heading.
 *
 * <p>Every {@link #update()} works out how far off the robot is in field coordinates, turns
 * that into a power (more when far, less when close), rotates it into the robot's own frame,
 * and hands it to the {@link Drivetrain}. Mecanum can drive any direction without turning, so
 * the path is a straight line no matter which way the target is.
 *
 * <h2>Tuning, in order</h2>
 * <ol>
 *   <li>{@link #MIN_POWER}: raise until the robot creeps the last inch instead of stalling
 *       short. Too high and it hunts back and forth at the target.</li>
 *   <li>{@link #KP}: raise for a quicker approach, lower if it overshoots.</li>
 *   <li>{@link #KD}: raise if it still overshoots at the speed you want. It brakes using
 *       the Pinpoint's measured speed.</li>
 *   <li>{@link #KH}: raise if the heading drifts during the move, lower if it wobbles.</li>
 * </ol>
 */
@Config
public class DriveToPoint {

    /** Power per inch of error. 10 in away = 0.6, capped at {@link #MAX_POWER}. */
    public static double KP = 0.06;
    /** Power taken off per inch/second of speed. Brakes before the target. */
    public static double KD = 0.004;
    /** Turn power per radian of heading error. 10 degrees off = 0.17. */
    public static double KH = 1.0;

    public static double MAX_POWER = 0.5;
    public static double MAX_TURN = 0.4;
    /** Smallest drive power that still moves the robot. Below this it just sits and hums. */
    public static double MIN_POWER = 0.07;

    public static double POSITION_TOLERANCE_IN = 0.5;
    public static double HEADING_TOLERANCE_DEG = 2.0;
    /** Has to stay inside both tolerances this long to count as arrived. */
    public static double SETTLE_S = 0.15;

    private final Drivetrain drive;
    private final Odometry odometry;
    private final ElapsedTime settleTimer = new ElapsedTime();

    private double targetX, targetY, targetHeading;
    private double distanceError, headingError;
    private boolean inTolerance;

    public DriveToPoint(Drivetrain drive, Odometry odometry) {
        this.drive = drive;
        this.odometry = odometry;
    }

    /** Inches and radians, in the {@link Odometry} frame. */
    public void setTarget(double x, double y, double heading) {
        targetX = x;
        targetY = y;
        targetHeading = heading;
        inTolerance = false;
    }

    /**
     * Reads the odometry and drives one step toward the target. Call every loop. Reads the
     * Pinpoint itself, so don't also call {@link Odometry#update()} in the same loop.
     *
     * @return true once the robot has settled at the target. The motors are stopped then.
     */
    public boolean update() {
        odometry.update();

        double ex = targetX - odometry.getX();
        double ey = targetY - odometry.getY();
        distanceError = Math.hypot(ex, ey);
        headingError = AngleUnit.normalizeRadians(targetHeading - odometry.getHeading());

        boolean positionOk = distanceError <= POSITION_TOLERANCE_IN;
        boolean headingOk = Math.abs(headingError) <= Math.toRadians(HEADING_TOLERANCE_DEG);

        if (positionOk && headingOk) {
            if (!inTolerance) {
                inTolerance = true;
                settleTimer.reset();
            }
            if (settleTimer.seconds() >= SETTLE_S) {
                drive.stop();
                return true;
            }
        } else {
            inTolerance = false;
        }

        // Field-frame push: toward the target, minus a brake on how fast we're already going.
        double cx = 0.0, cy = 0.0;
        if (!positionOk) {
            cx = KP * ex - KD * odometry.getVelX();
            cy = KP * ey - KD * odometry.getVelY();
            double mag = Math.hypot(cx, cy);
            if (mag > MAX_POWER) {
                cx *= MAX_POWER / mag;
                cy *= MAX_POWER / mag;
            } else if (mag > 1e-6 && mag < MIN_POWER) {
                cx *= MIN_POWER / mag;
                cy *= MIN_POWER / mag;
            }
        }

        // Rotate field -> robot: what the field calls "toward the target" depends on which
        // way the robot is facing right now.
        double h = odometry.getHeading();
        double cos = Math.cos(h), sin = Math.sin(h);
        double forward = cx * cos + cy * sin;
        double left = -cx * sin + cy * cos;

        double turn = headingOk ? 0.0 : clip(KH * headingError, MAX_TURN);

        drive.drive(forward, left, turn);
        return false;
    }

    /** Inches from the target as of the last update. */
    public double getDistanceError() {
        return distanceError;
    }

    /** Degrees off the target heading as of the last update. */
    public double getHeadingErrorDeg() {
        return Math.toDegrees(headingError);
    }

    public double getTargetX() {
        return targetX;
    }

    public double getTargetY() {
        return targetY;
    }

    private static double clip(double v, double limit) {
        return Math.max(-limit, Math.min(limit, v));
    }
}
