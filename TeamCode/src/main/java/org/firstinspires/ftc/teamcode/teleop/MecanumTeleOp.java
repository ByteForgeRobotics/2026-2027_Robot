package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;

/**
 * Driver-controlled mecanum drive, built on the Road Runner MecanumDrive so the
 * pose estimate stays live during TeleOp (useful for dashboard field view, and
 * for handing a pose off between auto and TeleOp).
 *
 * Controls:
 *   left stick   - translate
 *   right stick  - rotate
 *   right bumper - hold for slow mode
 *   options      - toggle field-centric / robot-centric
 *   back         - reset the field-centric heading to where the robot points now
 */
@TeleOp(name = "Mecanum TeleOp", group = "drive")
public class MecanumTeleOp extends LinearOpMode {
    /** Multiplier applied while the slow-mode button is held. */
    public static final double SLOW_MODE_SCALE = 0.35;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        boolean fieldCentric = true;

        telemetry.addLine("Ready. Options toggles field-centric, Back resets heading.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Keep the localizer fresh so the pose we read below is current.
            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            // *WasPressed() is a rising edge -- true only on the first loop the
            // button goes down, so a held button toggles once, not 50x/second.
            if (gamepad1.optionsWasPressed()) {
                fieldCentric = !fieldCentric;
            }

            // Zero the heading without disturbing the position estimate.
            if (gamepad1.backWasPressed()) {
                drive.localizer.setPose(new Pose2d(pose.position, 0));
            }

            double scale = gamepad1.right_bumper ? SLOW_MODE_SCALE : 1.0;

            // Stick axes are inverted relative to the field frame: pushing the
            // stick forward gives -y, and left gives -x.
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y * scale,
                    -gamepad1.left_stick_x * scale
            );

            // Field-centric: rotate the driver's input from the field frame into
            // the robot frame by undoing the robot's current heading.
            Vector2d translation = fieldCentric
                    ? pose.heading.inverse().times(input)
                    : input;

            drive.setDrivePowers(new PoseVelocity2d(
                    translation,
                    -gamepad1.right_stick_x * scale
            ));

            telemetry.addData("mode", fieldCentric ? "field-centric" : "robot-centric");
            telemetry.addData("slow", gamepad1.right_bumper);
            telemetry.addData("x", pose.position.x);
            telemetry.addData("y", pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(pose.heading.toDouble()));
            telemetry.update();
        }
    }
}
