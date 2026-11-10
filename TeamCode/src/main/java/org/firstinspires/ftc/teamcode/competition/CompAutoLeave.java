package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.DriveToPoint;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.PowerMonitor;

/**
 * Scores LEAVE (3 points in AUTO): drives straight off the wall and stops.
 *
 * <p>LEAVE only needs the robot to stop touching the perimeter wall (manual 10.5.4), so a
 * short straight move is enough. Uses the dead wheels through {@link DriveToPoint} to go an
 * exact distance. If the Pinpoint is missing, not ready, or the move goes wrong, it falls back
 * to driving on a timer so the points are still likely.
 *
 * <h2>Set up during INIT (gamepad 1)</h2>
 * <ul>
 *   <li>Dpad up / down: distance, 2 in steps.</li>
 *   <li>Dpad right / left: start delay, 1 s steps. Use this to stay out of a partner's way.</li>
 *   <li>Y: drive FORWARD or BACKWARD (pick whichever way points away from the wall).</li>
 * </ul>
 *
 * <p>After AUTO it preselects Comp TeleOp on the Driver Station.
 */
@Autonomous(name = "Comp Auto: Leave", group = "Competition", preselectTeleOp = "Comp TeleOp")
public class CompAutoLeave extends LinearOpMode {

    public static double DEFAULT_DISTANCE_IN = 12.0;
    /** Gives up on odometry after this long and falls back to the timer. */
    public static double ODOMETRY_TIMEOUT_S = 4.0;
    /** Stop trusting odometry if it hasn't moved this far after STUCK_S. */
    public static double STUCK_IN = 1.0;
    public static double STUCK_S = 1.0;
    /** Stop trusting odometry if it gets this much further from the target than its best. */
    public static double WRONG_WAY_IN = 3.0;

    /** Timer fallback: this power for this long. About a foot on a typical mecanum robot. */
    public static double TIMED_POWER = 0.4;
    public static double TIMED_S = 0.8;

    private Drivetrain drive;
    private Odometry odometry;

    @Override
    public void runOpMode() {
        drive = new Drivetrain(hardwareMap);
        odometry = new Odometry(hardwareMap);
        PowerMonitor power = new PowerMonitor(hardwareMap);
        odometry.reset(); // Robot is still in INIT, which the Pinpoint IMU needs.

        double distance = DEFAULT_DISTANCE_IN;
        double delay = 0.0;
        boolean forward = true;

        while (opModeInInit()) {
            if (gamepad1.dpadUpWasPressed()) distance = Math.min(48, distance + 2);
            if (gamepad1.dpadDownWasPressed()) distance = Math.max(4, distance - 2);
            if (gamepad1.dpadRightWasPressed()) delay = Math.min(25, delay + 1);
            if (gamepad1.dpadLeftWasPressed()) delay = Math.max(0, delay - 1);
            if (gamepad1.yWasPressed()) forward = !forward;

            odometry.update();
            power.update();
            telemetry.addLine("COMP AUTO: LEAVE");
            telemetry.addData("direction (Y)", forward ? "FORWARD" : "BACKWARD");
            telemetry.addData("distance (dpad up/down)", "%.0f in", distance);
            telemetry.addData("delay (dpad left/right)", "%.0f s", delay);
            telemetry.addLine();
            telemetry.addData("Pinpoint", odometry.getStatus()
                    + (odometry.isReady() ? "" : "  (will use timer instead)"));
            telemetry.addData("battery (V)", "%.2f", power.getVolts());
            for (int i = 0; i < Drivetrain.NAMES.length; i++) {
                if (!drive.hasMotor(i)) {
                    telemetry.addData("!! " + Drivetrain.NAMES[i], "NOT FOUND - check config");
                }
            }
            telemetry.update();
        }
        if (isStopRequested()) {
            return;
        }

        String result;
        try {
            ElapsedTime wait = new ElapsedTime();
            while (opModeIsActive() && wait.seconds() < delay) {
                status("WAITING", String.format("%.1f s", delay - wait.seconds()));
            }

            result = odometry.isReady() ? driveWithOdometry(distance, forward) : "NO PINPOINT";
            if (!result.equals("ARRIVED") && opModeIsActive()) {
                result += " -> used timer";
                driveTimed(forward);
            }
        } finally {
            drive.stop();
        }

        while (opModeIsActive()) {
            odometry.update();
            status("DONE", result);
        }
    }

    /** @return ARRIVED, or why it gave up. */
    private String driveWithOdometry(double distance, boolean forward) {
        odometry.update();
        double sx = odometry.getX(), sy = odometry.getY(), h = odometry.getHeading();
        double sign = forward ? 1.0 : -1.0;

        DriveToPoint follower = new DriveToPoint(drive, odometry);
        follower.setTarget(sx + sign * distance * Math.cos(h),
                sy + sign * distance * Math.sin(h), h);

        double closest = Double.POSITIVE_INFINITY;
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {
            if (follower.update()) {
                return "ARRIVED";
            }
            double dist = follower.getDistanceError();
            closest = Math.min(closest, dist);
            if (dist > closest + WRONG_WAY_IN) {
                return "WRONG WAY";
            }
            double moved = Math.hypot(odometry.getX() - sx, odometry.getY() - sy);
            if (timer.seconds() > STUCK_S && moved < STUCK_IN) {
                return "NOT MOVING";
            }
            if (timer.seconds() > ODOMETRY_TIMEOUT_S) {
                return "TIMED OUT";
            }
            status("DRIVING", String.format("%.1f in to go", dist));
        }
        return "STOPPED";
    }

    private void driveTimed(boolean forward) {
        ElapsedTime timer = new ElapsedTime();
        drive.drive(forward ? TIMED_POWER : -TIMED_POWER, 0, 0);
        while (opModeIsActive() && timer.seconds() < TIMED_S) {
            status("DRIVING (timer)", String.format("%.1f s", TIMED_S - timer.seconds()));
        }
        drive.stop();
    }

    private void status(String state, String detail) {
        telemetry.addData("state", state);
        telemetry.addData("detail", detail);
        telemetry.addData("x / y (in)", "%.1f / %.1f", odometry.getX(), odometry.getY());
        telemetry.update();
    }
}
