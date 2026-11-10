package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.LimelightController;

/**
 * Vision readout for calibrating the Limelight mounting geometry.
 *
 * <p>Registered as {@code @TeleOp}, not {@code @Autonomous}: this is a diagnostic you run
 * while pushing the robot around, and an Autonomous OpMode gets no gamepad input.
 */
@TeleOp(name = "Limelight Test", group = "Tests")
public class LimelightTest extends LinearOpMode {

    /** Name of the Limelight in the robot configuration. Case-sensitive. */
    private static final String LIMELIGHT_NAME = "Limelight";

    /** Name of the IMU in the robot configuration, or null to skip heading entirely. */
    private static final String IMU_NAME = "imu";

    /** Control Hub mounting orientation — set these to match your robot. */
    private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    private static final int PIPELINE = 0;

    private LimelightController vision;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        vision = new LimelightController(hardwareMap, LIMELIGHT_NAME);
        imu = initImu();

        telemetry.setMsTransmissionInterval(11);
        vision.start(PIPELINE);

        // The original never called update() before waitForStart(), so the Driver Station
        // showed nothing during init — including whether the IMU came up.
        while (opModeInInit()) {
            telemetry.addData("Status", "Ready");
            telemetry.addData("Heading source", imu == null ? "none (assuming 0)" : "IMU");
            telemetry.addData("Pipeline", PIPELINE);
            telemetry.update();
        }

        try {
            while (opModeIsActive() && !isStopRequested()) {
                double heading = readHeadingDeg();
                vision.update(heading);

                telemetry.addData("heading (deg)", "%.1f", heading);

                LimelightController.TargetObservation obs = vision.getLatest();

                if (obs == null) {
                    // No live target and nothing left to hold: either never acquired, or the
                        // hold window expired. If the target IS in frame and this still shows,
                    // the geometry is being rejected — check CAMERA_PITCH_DEG and
                    // TARGET_HEIGHT_IN first.
                    telemetry.addData("Limelight", "no target");
                } else {
                    telemetry.addData("source", obs.fresh
                            ? "LIVE"
                            : String.format("HELD (%.0f ms old)", obs.ageMs));

                    Integer tagId = vision.getTrackedFiducialId();
                    telemetry.addData("tag", tagId == null ? "n/a" : tagId.toString());
                    telemetry.addData("tx", "raw %.2f | filtered %.2f",
                            vision.getRawTx(), vision.getSmoothedTx());
                    telemetry.addData("ty", "raw %.2f | filtered %.2f",
                            vision.getRawTy(), vision.getSmoothedTy());

                    telemetry.addData("range (in)", "%.1f", obs.rangeIn);
                    telemetry.addData("bearing (deg)", "%.1f", obs.bearingDeg);
                    telemetry.addData("target rel to robot", "X %.1f  Y %.1f",
                            obs.robotRelX, obs.robotRelY);
                    telemetry.addData("target rel, field frame", "X %.1f  Y %.1f",
                            obs.fieldRelX, obs.fieldRelY);

                    // Example of recovering the robot's own field position. Replace the
                    // constants with the real field coordinates of the target you are tracking.
                    double[] robotPos = obs.robotFieldPosition(72.0, 72.0);
                    telemetry.addData("robot field pos (example target)", "X %.1f  Y %.1f",
                            robotPos[0], robotPos[1]);
                }

                telemetry.update();
            }
        } finally {
            vision.stop();
        }
    }

    /** Returns a configured IMU, or null if one is not present — heading then falls back to 0. */
    private IMU initImu() {
        if (IMU_NAME == null) {
            return null;
        }
        try {
            IMU found = hardwareMap.get(IMU.class, IMU_NAME);
            found.initialize(new IMU.Parameters(
                    new RevHubOrientationOnRobot(LOGO_DIRECTION, USB_DIRECTION)));
            found.resetYaw();
            return found;
        } catch (Exception e) {
            // Missing from the configuration is not fatal for a vision readout: the
            // robot-relative numbers do not depend on heading.
            return null;
        }
    }

    private double readHeadingDeg() {
        if (imu == null) {
            return 0.0;
        }
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}