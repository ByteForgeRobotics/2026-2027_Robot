package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.subsystems.DriveToPoint;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;

/**
 * Drives in a straight line from wherever the robot starts to (TARGET_X, TARGET_Y) using the
 * dead wheels, then stops and shows where it ended up.
 *
 * <p>Start is (0, 0). +x is forward and +y is left, so the default (0, 10) is a 10 inch
 * strafe to the left. For 10 inches straight ahead, set TARGET_X = 10 and TARGET_Y = 0 in
 * the dashboard.
 *
 * <h2>If it doesn't work</h2>
 * <p>It stops itself and says why instead of running off:
 * <ul>
 *   <li>WRONG WAY: it got further from the target instead of closer. A pod direction is
 *       backwards, or the pods are plugged into the swapped ports. Use the init check.</li>
 *   <li>NOT MOVING: the wheels didn't move the dead wheels. Motors, or MIN_POWER too low.</li>
 *   <li>TIMED OUT: it got close but never settled. Tuning, see {@link DriveToPoint}.</li>
 * </ul>
 */
@Config
@Autonomous(name = "Auto Test", group = "Tests")
public class AutoTest extends LinearOpMode {

    public static double TARGET_X = 0.0;
    public static double TARGET_Y = 10.0;
    public static double TARGET_HEADING_DEG = 0.0;
    /** Gives up and stops if it hasn't arrived by now, so a bad tune can't run away. */
    public static double TIMEOUT_S = 5.0;
    /** Stops if it ends up this much further from the target than its closest point. */
    public static double WRONG_WAY_IN = 3.0;
    /** Stops if it hasn't moved this far after {@link #STUCK_S}. */
    public static double STUCK_IN = 1.0;
    public static double STUCK_S = 1.0;

    private Odometry odometry;
    private DriveToPoint follower;

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drive = new Drivetrain(hardwareMap);
        odometry = new Odometry(hardwareMap);
        follower = new DriveToPoint(drive, odometry);

        odometry.reset();
        telemetry.setMsTransmissionInterval(50);

        // Init doubles as a sensor check: push the robot by hand and watch the numbers.
        while (opModeInInit()) {
            odometry.update();
            for (int i = 0; i < Drivetrain.NAMES.length; i++) {
                if (!drive.hasMotor(i)) {
                    telemetry.addData(Drivetrain.NAMES[i], "NOT FOUND");
                }
            }
            telemetry.addData("Pinpoint", odometry.getStatus());
            telemetry.addData("target", "(%.1f, %.1f) in", TARGET_X, TARGET_Y);
            telemetry.addData("x (in)", "%.2f", odometry.getX());
            telemetry.addData("y (in)", "%.2f", odometry.getY());
            telemetry.addData("heading (deg)", "%.1f", Math.toDegrees(odometry.getHeading()));
            telemetry.addData("X / Y pod", "%d / %d",
                    odometry.getEncoderX(), odometry.getEncoderY());
            telemetry.addLine();
            telemetry.addLine("CHECK: push forward -> x goes UP");
            telemetry.addLine("       push left    -> y goes UP");
            telemetry.addLine("       turn left    -> heading goes UP");
            telemetry.addLine("Then set it back down still and press play.");
            telemetry.update();
        }

        if (!odometry.isConnected()) {
            hold("NO PINPOINT: check the config name is \"" + Odometry.PINPOINT_NAME + "\"");
            return;
        }

        // Pressing start right after init can land mid-calibration.
        ElapsedTime wait = new ElapsedTime();
        while (opModeIsActive() && !odometry.isReady() && wait.seconds() < 1.0) {
            odometry.update();
        }
        if (!odometry.isReady()) {
            hold("PINPOINT " + odometry.getStatus() + ": not driving on bad data");
            return;
        }

        double startX = odometry.getX(), startY = odometry.getY();
        follower.setTarget(TARGET_X, TARGET_Y, Math.toRadians(TARGET_HEADING_DEG));
        double closest = Double.POSITIVE_INFINITY;
        ElapsedTime timer = new ElapsedTime();
        String result = "TIMED OUT";

        try {
            while (opModeIsActive()) {
                if (follower.update()) {
                    result = "ARRIVED";
                    break;
                }
                double dist = follower.getDistanceError();
                closest = Math.min(closest, dist);
                if (dist > closest + WRONG_WAY_IN) {
                    result = "WRONG WAY";
                    break;
                }
                double moved = Math.hypot(odometry.getX() - startX, odometry.getY() - startY);
                if (timer.seconds() > STUCK_S && moved < STUCK_IN) {
                    result = "NOT MOVING";
                    break;
                }
                if (timer.seconds() > TIMEOUT_S) {
                    break;
                }
                show("DRIVING", timer.seconds());
            }
        } finally {
            drive.stop();
        }

        double driveTime = timer.seconds();
        while (opModeIsActive()) {
            odometry.update();
            show(result, driveTime);
        }
    }

    /** Keeps a message on screen until stop, since ending the OpMode would clear it. */
    private void hold(String message) {
        while (opModeIsActive()) {
            telemetry.addData("STOPPED", message);
            telemetry.update();
        }
    }

    private void show(String state, double seconds) {
        TelemetryPacket packet = new TelemetryPacket();
        put(packet, "state", state);
        put(packet, "time (s)", round2(seconds));
        put(packet, "x (in)", round2(odometry.getX()));
        put(packet, "y (in)", round2(odometry.getY()));
        put(packet, "heading (deg)", round2(Math.toDegrees(odometry.getHeading())));
        put(packet, "off by (in)", round2(Math.hypot(
                follower.getTargetX() - odometry.getX(),
                follower.getTargetY() - odometry.getY())));

        Canvas field = packet.fieldOverlay();
        field.setStroke("#4CAF50");
        field.strokeCircle(follower.getTargetX(), follower.getTargetY(), 2);
        field.setStroke("#3F51B5");
        Drawing.drawRobot(field,
                new Pose2d(odometry.getX(), odometry.getY(), odometry.getHeading()));

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.update();
    }

    /** One value to both displays. Numbers stay numbers so the dashboard can graph them. */
    private void put(TelemetryPacket packet, String key, Object value) {
        packet.put(key, value);
        telemetry.addData(key, value);
    }

    private static double round2(double v) {
        return Math.round(v * 100.0) / 100.0;
    }
}
