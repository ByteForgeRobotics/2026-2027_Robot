package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensController;
import org.firstinspires.ftc.teamcode.subsystems.HuskyLensController.PollenObservation;

/**
 * HuskyLens pollen-tracker readout, built for FTC Dashboard.
 *
 * <p>Open {@code http://192.168.43.1:8080/dash} on a laptop joined to the robot's WiFi. The
 * Field view shows the robot, the camera's floor footprint, and where
 * {@link HuskyLensController} thinks the nearest ball is. A small inset in the top-left corner
 * of the field is the camera frame itself (320x240), with every block the HuskyLens reported:
 * grey for rejected, yellow for candidates, green for the chosen target. Numeric values
 * (range, bearing, pixel position, loop rate) are all graphable from the Graph tab.
 *
 * <p>Everything also goes to the Driver Station, so it still works without a laptop.
 *
 * <h2>Controls</h2>
 * <ul>
 *   <li>Left stick / right stick: drive, same as {@code LocalizationTest}.</li>
 *   <li>Hold right bumper: chase the ball using {@link HuskyLensController#steer}. Only works
 *       when {@link #ALLOW_CHASE} is true (flip it in the dashboard Config tab).</li>
 *   <li>Y: re-run {@link HuskyLensController#start} (re-knock and re-select the algorithm),
 *       for when the camera was plugged in late or its protocol was just switched to I2C.</li>
 * </ul>
 *
 * <p>Registered as {@code @TeleOp} for the same reason as {@code LimelightTest}: a diagnostic
 * needs gamepad input, and Autonomous OpModes do not get any.
 *
 * <p>Driving is optional. If the Road Runner motor/IMU names are not in the configuration,
 * {@link MecanumDrive} fails to construct and the OpMode runs as a camera-only readout.
 */
@Config
@TeleOp(name = "HuskyLens Test", group = "Tests")
public class HuskyLensTest extends LinearOpMode {

    /** Name of the HuskyLens in the robot configuration. Case-sensitive. */
    public static String HUSKYLENS_NAME = "huskylens";

    /**
     * Let the right bumper hand the drivetrain to {@link HuskyLensController#steer}. Off by
     * default so a stray bumper press during a bench test does not move the robot.
     */
    public static boolean ALLOW_CHASE = false;

    /**
     * Also read blocks for every color ID, not just {@link HuskyLensController#POLLEN_ID}.
     * Costs a second I2C round trip per read. Turn on to see what else the camera is finding
     * (useful when yellow was learned as ID 2 by accident).
     */
    public static boolean SHOW_ALL_IDS = false;

    // ---------------------------------------------------------------------
    // Field-view drawing. The dashboard field is 144x144 in, origin at center.
    // ---------------------------------------------------------------------

    /** Camera inset: top-left corner and scale. 320 px * 0.2 = 64 in wide, 48 in tall. */
    private static final double INSET_X = -70.0;
    private static final double INSET_Y = 70.0;
    private static final double INSET_SCALE = 0.2;

    private static final String COLOR_ROBOT = "#3F51B5";
    private static final String COLOR_FOV = "#9E9E9E";
    private static final String COLOR_REJECTED = "#9E9E9E";
    private static final String COLOR_CANDIDATE = "#FFC107";
    private static final String COLOR_TARGET = "#4CAF50";
    private static final String COLOR_HELD = "#BDBDBD";

    private HuskyLensController tracker;
    private HuskyLens huskyLens;
    private MecanumDrive drive;

    private boolean connected;
    private HuskyLens.Block[] allBlocks = new HuskyLens.Block[0];
    private final ElapsedTime sinceAllRead = new ElapsedTime();

    // Rate counters, reset once a second.
    private final ElapsedTime rateWindow = new ElapsedTime();
    private int loopsInWindow = 0;
    private int freshInWindow = 0;
    private double loopHz = 0.0;
    private double freshHz = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        tracker = new HuskyLensController(hardwareMap, HUSKYLENS_NAME);
        // Same device instance the controller holds; used only for the all-IDs read.
        huskyLens = hardwareMap.get(HuskyLens.class, HUSKYLENS_NAME);
        drive = initDrive();

        telemetry.setMsTransmissionInterval(11);
        connected = tracker.start();

        while (opModeInInit()) {
            if (gamepad1.y) {
                connected = tracker.start();
            }
            telemetry.addData("Status", "Ready");
            telemetry.addData("HuskyLens", connected ? "connected" : "NOT FOUND (press Y to retry)");
            if (!connected) {
                telemetry.addLine("  check: Protocol Type = I2C on the camera, cable on an I2C port");
            }
            telemetry.addData("Drive", drive == null ? "none (camera-only)" : "MecanumDrive");
            telemetry.addData("Chase", ALLOW_CHASE ? "ENABLED (hold RB)" : "off");
            telemetry.addData("Dashboard", "http://192.168.43.1:8080/dash");
            telemetry.update();
        }

        rateWindow.reset();
        sinceAllRead.reset();

        try {
            while (opModeIsActive() && !isStopRequested()) {
                if (gamepad1.y) {
                    connected = tracker.start();
                }

                boolean fresh = tracker.update();
                PollenObservation obs = tracker.getLatest();
                readAllIdsIfEnabled();
                updateRates(fresh);

                Pose2d pose = driveAndLocalize(obs);

                TelemetryPacket packet = new TelemetryPacket();
                report(packet, obs);
                drawField(packet.fieldOverlay(), pose, obs);
                drawCameraInset(packet.fieldOverlay(), obs);
                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                telemetry.update();
            }
        } finally {
            if (drive != null) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            }
        }
    }

    // ---------------------------------------------------------------------
    // Sensing
    // ---------------------------------------------------------------------

    /** Reads every block regardless of ID, on the controller's cadence. */
    private void readAllIdsIfEnabled() {
        if (!SHOW_ALL_IDS || !connected) {
            allBlocks = new HuskyLens.Block[0];
            return;
        }
        if (sinceAllRead.milliseconds() < HuskyLensController.READ_PERIOD_MS) {
            return;
        }
        sinceAllRead.reset();
        HuskyLens.Block[] blocks = huskyLens.blocks();
        allBlocks = (blocks == null) ? new HuskyLens.Block[0] : blocks;
    }

    private void updateRates(boolean fresh) {
        loopsInWindow++;
        if (fresh) {
            freshInWindow++;
        }
        double sec = rateWindow.seconds();
        if (sec >= 1.0) {
            loopHz = loopsInWindow / sec;
            freshHz = freshInWindow / sec;
            loopsInWindow = 0;
            freshInWindow = 0;
            rateWindow.reset();
        }
    }

    // ---------------------------------------------------------------------
    // Driving
    // ---------------------------------------------------------------------

    /** Returns a MecanumDrive, or null if the drivetrain is not in the configuration. */
    private MecanumDrive initDrive() {
        try {
            return new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        } catch (Exception e) {
            // Missing motors or IMU. Not fatal: the camera readout does not need them.
            return null;
        }
    }

    /**
     * Applies stick or chase powers and returns the robot's pose for drawing. Without a
     * drivetrain the robot is drawn at the origin facing +X.
     */
    private Pose2d driveAndLocalize(PollenObservation obs) {
        if (drive == null) {
            return new Pose2d(0, 0, 0);
        }

        boolean chasing = ALLOW_CHASE && gamepad1.right_bumper && obs != null;
        if (chasing) {
            double[] powers = HuskyLensController.steer(obs);
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(powers[0], 0), powers[1]));
        } else {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x));
        }

        drive.updatePoseEstimate();
        return drive.localizer.getPose();
    }

    // ---------------------------------------------------------------------
    // Telemetry (Driver Station + dashboard)
    // ---------------------------------------------------------------------

    private void report(TelemetryPacket packet, PollenObservation obs) {
        put(packet, "connected", connected);
        put(packet, "loop Hz", round1(loopHz));
        put(packet, "fresh solves Hz", round1(freshHz));
        put(packet, "blocks (pollen id)", tracker.getLastBlocks().length);
        put(packet, "candidates", tracker.getCandidateCount());
        if (SHOW_ALL_IDS) {
            put(packet, "blocks (all ids)", allBlocks.length);
        }

        boolean chasing = ALLOW_CHASE && gamepad1.right_bumper && obs != null;
        put(packet, "mode", drive == null ? "camera only" : (chasing ? "CHASING" : "sticks"));

        if (obs == null) {
            // Nothing live and nothing held. If a ball is clearly in frame, the block is
            // being rejected: check HORIZON_ROW_PX and the aspect limits first, then
            // CAMERA_PITCH_DEG / CAMERA_HEIGHT_IN (a bad pitch puts the ray above the floor).
            put(packet, "target", "none");
            return;
        }

        put(packet, "target", obs.fresh ? "LIVE" : String.format("HELD (%.0f ms)", obs.ageMs));
        put(packet, "range (in)", round1(obs.rangeIn));
        put(packet, "range from width (in)", round1(obs.rangeFromWidthIn));
        put(packet, "bearing (deg)", round1(obs.bearingDeg));
        put(packet, "rel X (in)", round1(obs.robotRelX));
        put(packet, "rel Y (in)", round1(obs.robotRelY));
        put(packet, "pixel x", obs.pixelX);
        put(packet, "pixel y", obs.pixelY);
        put(packet, "pixel w", obs.pixelWidth);
        put(packet, "pixel h", obs.pixelHeight);

        double[] powers = HuskyLensController.steer(obs);
        put(packet, "steer forward", round2(powers[0]));
        put(packet, "steer turn", round2(powers[1]));
    }

    /** One value to both displays. Numbers stay numbers so the dashboard can graph them. */
    private void put(TelemetryPacket packet, String key, Object value) {
        packet.put(key, value);
        telemetry.addData(key, value);
    }

    private static double round1(double v) {
        return Math.round(v * 10.0) / 10.0;
    }

    private static double round2(double v) {
        return Math.round(v * 100.0) / 100.0;
    }

    // ---------------------------------------------------------------------
    // Dashboard field view
    // ---------------------------------------------------------------------

    /** Robot, camera floor footprint, and the solved ball, all in field coordinates. */
    private void drawField(Canvas c, Pose2d pose, PollenObservation obs) {
        c.setStroke(COLOR_ROBOT);
        Drawing.drawRobot(c, pose);

        drawFootprint(c, pose);

        if (obs == null) {
            return;
        }

        Vector2d ball = pose.times(new Vector2d(obs.robotRelX, obs.robotRelY));
        String color = obs.fresh ? COLOR_TARGET : COLOR_HELD;

        c.setStroke(color);
        c.setStrokeWidth(1);
        c.strokeLine(pose.position.x, pose.position.y, ball.x, ball.y);

        c.setFill(obs.fresh ? COLOR_CANDIDATE : COLOR_HELD);
        c.fillCircle(ball.x, ball.y, HuskyLensController.POLLEN_DIAMETER_IN / 2.0);
        c.setStroke(color);
        c.strokeCircle(ball.x, ball.y, HuskyLensController.POLLEN_DIAMETER_IN / 2.0);
    }

    /**
     * Wedge on the floor the camera can actually see, from the near edge (bottom row of the
     * image) out to {@link HuskyLensController#MAX_RANGE_IN}. If a ball sits outside this and
     * still shows up, the mounting constants are wrong.
     */
    private void drawFootprint(Canvas c, Pose2d pose) {
        double f = HuskyLensController.focalPx();
        double halfHfov = Math.toRadians(HuskyLensController.HFOV_DEG / 2.0);
        double halfVfov = Math.atan((HuskyLensController.IMAGE_HEIGHT_PX / 2.0) / f);
        double drop = HuskyLensController.CAMERA_HEIGHT_IN
                - HuskyLensController.POLLEN_CENTER_HEIGHT_IN;

        // Bottom row of the image looks down by (pitch + half vfov). Above the horizon the
        // near edge is undefined; fall back to the controller's minimum.
        double downAngle = -Math.toRadians(HuskyLensController.CAMERA_PITCH_DEG) + halfVfov;
        double near = downAngle > 0
                ? Math.max(HuskyLensController.MIN_RANGE_IN, drop / Math.tan(downAngle))
                : HuskyLensController.MIN_RANGE_IN;
        double far = HuskyLensController.MAX_RANGE_IN;

        double yaw = Math.toRadians(HuskyLensController.CAMERA_YAW_DEG);
        Vector2d camRel = new Vector2d(
                HuskyLensController.CAMERA_OFFSET_X_IN, HuskyLensController.CAMERA_OFFSET_Y_IN);

        double[] xs = new double[4];
        double[] ys = new double[4];
        double[][] corners = {
                {near, yaw - halfHfov}, {far, yaw - halfHfov},
                {far, yaw + halfHfov}, {near, yaw + halfHfov},
        };
        for (int i = 0; i < 4; i++) {
            double r = corners[i][0];
            double a = corners[i][1];
            // Camera frame is +X forward, +Y left, same as the robot frame.
            Vector2d rel = camRel.plus(new Vector2d(r * Math.cos(a), r * Math.sin(a)));
            Vector2d field = pose.times(rel);
            xs[i] = field.x;
            ys[i] = field.y;
        }

        c.setStroke(COLOR_FOV);
        c.setStrokeWidth(1);
        c.strokePolygon(xs, ys);

        Vector2d cam = pose.times(camRel);
        c.setFill(COLOR_FOV);
        c.fillCircle(cam.x, cam.y, 1.0);
    }

    // ---------------------------------------------------------------------
    // Dashboard camera inset
    // ---------------------------------------------------------------------

    /**
     * The 320x240 camera frame, drawn to scale in the top-left of the field. Image rows go
     * down, field Y goes up, so Y is flipped.
     */
    private void drawCameraInset(Canvas c, PollenObservation obs) {
        double w = HuskyLensController.IMAGE_WIDTH_PX * INSET_SCALE;
        double h = HuskyLensController.IMAGE_HEIGHT_PX * INSET_SCALE;

        c.setStroke(COLOR_FOV);
        c.setStrokeWidth(1);
        c.strokeRect(INSET_X, INSET_Y - h, w, h);

        // Horizon cutoff: blocks centered above this line are ignored by the controller.
        double horizonY = INSET_Y - HuskyLensController.HORIZON_ROW_PX * INSET_SCALE;
        c.strokeLine(INSET_X, horizonY, INSET_X + w, horizonY);

        // Rejected-by-ID blocks first, so the interesting ones draw on top.
        if (SHOW_ALL_IDS) {
            c.setStroke(COLOR_REJECTED);
            for (HuskyLens.Block b : allBlocks) {
                if (b.id != HuskyLensController.POLLEN_ID) {
                    strokeBlock(c, b);
                }
            }
        }

        for (HuskyLens.Block b : tracker.getLastBlocks()) {
            boolean candidate = HuskyLensController.looksLikePollen(b);
            c.setStroke(candidate ? COLOR_CANDIDATE : COLOR_REJECTED);
            c.setStrokeWidth(1);
            strokeBlock(c, b);
        }

        if (obs != null && obs.fresh) {
            c.setStroke(COLOR_TARGET);
            c.setStrokeWidth(2);
            double left = obs.pixelX - obs.pixelWidth / 2.0;
            double top = obs.pixelY - obs.pixelHeight / 2.0;
            strokeImageRect(c, left, top, obs.pixelWidth, obs.pixelHeight);
        }
    }

    private void strokeBlock(Canvas c, HuskyLens.Block b) {
        strokeImageRect(c, b.left, b.top, b.width, b.height);
    }

    /** Draws an image-space rectangle (pixels, top-left origin) inside the inset. */
    private void strokeImageRect(Canvas c, double left, double top, double width, double height) {
        double x = INSET_X + left * INSET_SCALE;
        double y = INSET_Y - (top + height) * INSET_SCALE;
        c.strokeRect(x, y, width * INSET_SCALE, height * INSET_SCALE);
    }
}
