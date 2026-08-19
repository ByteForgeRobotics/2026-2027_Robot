package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

/**
 * A worked example of a Road Runner 1.0 autonomous.
 *
 * IMPORTANT: this will not drive correctly until you have run the tuning
 * process in TeamCode/tuning -- an untuned drive will follow paths badly or not
 * at all. See README.md for the tuning order.
 *
 * Field coordinates are inches, headings are radians, and the origin is the
 * center of the field with +x away from the audience.
 */
@Autonomous(name = "Sample Autonomous", group = "auto")
public class SampleAutonomous extends LinearOpMode {
    // TODO: set this to where you actually place the robot before the match.
    private static final Pose2d START_POSE = new Pose2d(-36, -60, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);
        Lift lift = new Lift(hardwareMap);

        // Build the trajectory before start so the math is done during init,
        // not during the 30-second period.
        Action driveToScore = drive.actionBuilder(START_POSE)
                .splineTo(new Vector2d(-36, -24), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-12, -12, Math.toRadians(45)), Math.toRadians(45))
                .build();

        Action driveToPark = drive.actionBuilder(new Pose2d(-12, -12, Math.toRadians(45)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-58, -12, Math.toRadians(180)), Math.toRadians(180))
                .build();

        telemetry.addLine("Init complete. Trajectories built.");
        telemetry.addData("start pose", "%.1f, %.1f, %.0f deg",
                START_POSE.position.x, START_POSE.position.y,
                Math.toDegrees(START_POSE.heading.toDouble()));
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                // Drive and raise the lift at the same time -- ParallelAction
                // finishes when every child has finished.
                new ParallelAction(
                        driveToScore,
                        lift.toHigh()
                ),
                new SleepAction(0.5),   // let the robot settle before scoring
                lift.toLow(),
                new ParallelAction(
                        driveToPark,
                        lift.toDown()
                )
        ));

        Pose2d end = drive.localizer.getPose();
        telemetry.addData("end pose", "%.1f, %.1f, %.0f deg",
                end.position.x, end.position.y, Math.toDegrees(end.heading.toDouble()));
        telemetry.update();
    }
}
