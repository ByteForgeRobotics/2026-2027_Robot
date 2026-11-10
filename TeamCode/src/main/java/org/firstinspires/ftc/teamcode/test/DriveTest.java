package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.LimelightController;
import org.firstinspires.ftc.teamcode.subsystems.PowerMonitor;

import java.util.List;

/**
 * Robot-centric mecanum drive on the joysticks.
 *
 * <ul>
 *   <li>Left stick: drive forward/back and strafe.</li>
 *   <li>Right stick X: turn.</li>
 *   <li>Dpad up / down: raise or lower top speed by 0.1.</li>
 *   <li>Y: re-send the pod settings to the Pinpoint and zero its position.</li>
 * </ul>
 *
 * <p>If the robot drives wrong, fix the {@code REVERSE_*} flags (live from the dashboard
 * Config tab). Push the left stick straight up: every wheel should roll forward. Any wheel
 * that spins backwards gets its flag flipped. Once they're right, copy them into
 * {@code MecanumDrive}.
 *
 * <p>The Limelight readout (target, tag, range, bearing) shows under the motor powers so you
 * can drive around and watch it track. If the Limelight isn't plugged in or configured, the
 * drive still works and the readout says "NOT FOUND".
 *
 * <p>Battery voltage and current draw come from {@link PowerMonitor}, including the lowest
 * voltage seen so far. Watch that number while accelerating hard: if it dips under
 * {@link PowerMonitor#SAG_WARN_V} the battery is tired, and anything else that looks wrong in
 * this test is probably the battery rather than the code.
 *
 * <p>Each wheel also gets a running highest-current line ({@code FL amps peak} and friends).
 * Drive a straight line, then compare: the four should peak close together, and a wheel that
 * peaks well above the others is fighting something mechanical. Peaks here are sampled at
 * {@link PowerMonitor#READ_PERIOD_MS}, which keeps the drive loop quick but can clip a very
 * short spike — {@code Drive Motor Test} samples every loop if you need the true maximum.
 * Back zeroes all the min/peak history.
 *
 * <p>The dead wheels run through the goBILDA Pinpoint on an I2C port. Telemetry shows the
 * Pinpoint's position (x, y, heading) and each pod's raw encoder count. To check the pods:
 * push the robot straight forward and the X pod should count up and x should grow; push it
 * left and the Y pod should count up and y should grow. A pod that counts down gets its
 * {@code REVERSE_*_POD} flag flipped, then press Y. Y also zeroes the position, so keep the
 * robot still when you press it.
 */
@Config
@TeleOp(name = "Drive Test", group = "Tests")
public class DriveTest extends LinearOpMode {
    /** Top speed on the sticks, 0 to 1. Adjustable live with the dpad or the dashboard. */
    public static double TEST_POWER = 0.3;

    /**
     * Mecanum wheels strafe slower than they drive forward, so strafing gets a small boost
     * to make it feel the same. 1.0 turns it off.
     */
    public static double STRAFE_MULTIPLIER = 1.1;

    /** Stick values smaller than this count as zero, so a drifting stick doesn't creep. */
    public static double DEADZONE = 0.05;

    /** Motor directions. Usually one side of a mecanum drive is reversed; left is the guess. */
    public static boolean REVERSE_FL = true;
    public static boolean REVERSE_FR = false;
    public static boolean REVERSE_BL = true;
    public static boolean REVERSE_BR = false;

    /** Configuration names. The index constants below match this order. */
    private static final String[] NAMES = {"FL", "FR", "BL", "BR"};
    private static final int FL = 0, FR = 1, BL = 2, BR = 3;

    private final DcMotorEx[] motors = new DcMotorEx[NAMES.length];

    /** Name of the Limelight in the robot configuration. Case-sensitive. */
    public static String LIMELIGHT_NAME = "Limelight";
    public static int LIMELIGHT_PIPELINE = 0;

    /** Name of the Pinpoint in the robot configuration (I2C port). Case-sensitive. */
    public static String PINPOINT_NAME = "pinpoint";

    /** Which goBILDA pods are on the robot; sets ticks-per-mm. */
    public static GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    /**
     * Pod positions from the robot's center of rotation, in mm. X offset: how far left of
     * center the forward-facing (X) pod sits, right is negative. Y offset: how far forward of
     * center the sideways (Y) pod sits, back is negative. Only heading accuracy while turning
     * depends on these, so 0 is fine for a first check.
     */
    public static double X_POD_OFFSET_MM = -8.5;
    public static double Y_POD_OFFSET_MM = 16.5;

    /** Pod directions. Applied at init and whenever Y is pressed. */
    public static boolean REVERSE_X_POD = false;
    public static boolean REVERSE_Y_POD = false;

    private LimelightController limelight;
    private PowerMonitor powerMonitor;
    private GoBildaPinpointDriver pinpoint;

    private boolean lastUp, lastDown, lastY;

    @Override
    public void runOpMode() throws InterruptedException
    {

        limelight = initLimelight();
        pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, PINPOINT_NAME);
        configurePinpoint();
        powerMonitor = new PowerMonitor(hardwareMap);
        powerMonitor.watchMotors(NAMES, motors);
        telemetry.setMsTransmissionInterval(100);

        while (opModeInInit()) {
            for (int i = 0; i < NAMES.length; i++) {
                telemetry.addData(NAMES[i], motors[i] == null ? "NOT FOUND" : "ok");
            }
            telemetry.addData("Limelight", limelight == null ? "NOT FOUND" : "ok");
            telemetry.addData("Pinpoint", pinpoint == null
                    ? "NOT FOUND" : pinpoint.getDeviceStatus().toString());
            // Resting voltage, before anything draws current. Under about 12.5 V here means
            // the battery is already low and the drive test will not tell you much.
            powerMonitor.update();
            telemetry.addData("battery (V)", "%.2f", powerMonitor.getVolts());
            telemetry.addLine("Left stick: drive/strafe   Right stick: turn   Dpad: speed   Y: zero pods");
            telemetry.update();
        }

        // The init loop's readings are resting values; the sag numbers should only cover
        // driving.
        powerMonitor.resetPeaks();
        waitForStart();
        try {
            while (opModeIsActive()) {
                // Re-applied every loop so flipping a flag in the dashboard takes effect live.
                setDirection(FL, REVERSE_FL);
                setDirection(FR, REVERSE_FR);
                setDirection(BL, REVERSE_BL);
                setDirection(BR, REVERSE_BR);

                adjustSpeed();
                powerMonitor.update();
                if (gamepad1.back) {
                    powerMonitor.resetPeaks();
                }
                if (gamepad1.y && !lastY) {
                    configurePinpoint();
                }
                lastY = gamepad1.y;

                double y = -deadzone(gamepad1.left_stick_y); // stick up is negative
                double x = deadzone(gamepad1.left_stick_x) * STRAFE_MULTIPLIER;
                double rx = deadzone(gamepad1.right_stick_x);

                // Dividing by the biggest possible sum keeps every wheel within [-1, 1]
                // without changing the ratio between them, so combined moves stay the
                // right shape.
                double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

                double[] powers = new double[NAMES.length];
                powers[FL] = (y + x + rx) / denom * TEST_POWER;
                powers[BL] = (y - x + rx) / denom * TEST_POWER;
                powers[FR] = (y - x - rx) / denom * TEST_POWER;
                powers[BR] = (y + x - rx) / denom * TEST_POWER;

                telemetry.addData("speed", TEST_POWER);
                telemetry.addData("sticks", "y %.2f  x %.2f  turn %.2f", y, x, rx);
                for (int i = 0; i < NAMES.length; i++) {
                    if (motors[i] == null) {
                        telemetry.addData(NAMES[i], "NOT FOUND");
                        continue;
                    }
                    motors[i].setPower(powers[i]);
                    telemetry.addData(NAMES[i], "%.2f", powers[i]);
                }

                showPinpoint();

                telemetry.addLine();
                powerMonitor.report(telemetry::addData);

                showLimelight();
                telemetry.update();
            }
        } finally {
            for (DcMotorEx m : motors) {
                if (m != null) {
                    m.setPower(0);
                }
            }
            if (limelight != null) {
                limelight.stop();
            }
        }
    }

    /** Sends pod type, offsets and directions to the Pinpoint, then zeroes it. */
    private void configurePinpoint() {
        if (pinpoint == null) {
            return;
        }
        pinpoint.setEncoderResolution(POD_TYPE);
        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                REVERSE_X_POD ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                REVERSE_Y_POD ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    /** Reads the Pinpoint and adds position and both pod counts to telemetry. */
    private void showPinpoint() {
        telemetry.addLine();
        if (pinpoint == null) {
            telemetry.addData("Pinpoint", "NOT FOUND");
            return;
        }
        pinpoint.update();
        telemetry.addData("Pinpoint", "%s  (%.0f Hz)",
                pinpoint.getDeviceStatus(), pinpoint.getFrequency());
        telemetry.addData("x / y (in)", "%.2f / %.2f",
                pinpoint.getPosX(DistanceUnit.INCH), pinpoint.getPosY(DistanceUnit.INCH));
        telemetry.addData("heading (deg)", "%.1f", pinpoint.getHeading(AngleUnit.DEGREES));
        telemetry.addData("X pod (ticks)", pinpoint.getEncoderX());
        telemetry.addData("Y pod (ticks)", pinpoint.getEncoderY());
    }

    /** Returns a started Limelight, or null if it isn't in the configuration. */
    private LimelightController initLimelight() {
        try {
            LimelightController ll = new LimelightController(hardwareMap, LIMELIGHT_NAME);
            ll.start(LIMELIGHT_PIPELINE);
            return ll;
        } catch (Exception e) {
            return null;
        }
    }

    /**
     * Polls the Limelight and adds its readout to telemetry. No IMU here, so heading is 0:
     * range and bearing relative to the robot are still right.
     */
    private void showLimelight() {
        telemetry.addLine();
        if (limelight == null) {
            telemetry.addData("Limelight", "NOT FOUND");
            return;
        }
        limelight.update(0.0);

        LimelightController.TargetObservation obs = limelight.getLatest();
        if (obs == null) {
            telemetry.addData("Limelight", "no target");
        } else {
            telemetry.addData("Limelight", obs.fresh
                    ? "LIVE"
                    : String.format("HELD (%.0f ms old)", obs.ageMs));
            Integer tagId = limelight.getTrackedFiducialId();
            telemetry.addData("tag", tagId == null ? "n/a" : tagId.toString());
            telemetry.addData("tx / ty", "%.1f / %.1f",
                    limelight.getSmoothedTx(), limelight.getSmoothedTy());
            telemetry.addData("range (in)", "%.1f", obs.rangeIn);
            telemetry.addData("bearing (deg)", "%.1f", obs.bearingDeg);
        }

        showAllTags();
    }

    /**
     * One line per AprilTag in view, straight from the latest frame. Range and bearing use
     * the same geometry as the tracked tag, so they're only right for tags at
     * {@code TARGET_HEIGHT_IN}; a tag at another height shows "--" or a wrong range.
     */
    private void showAllTags() {
        List<LLResultTypes.FiducialResult> tags = limelight.getVisibleFiducials();
        telemetry.addData("tags in view", tags.size());
        for (LLResultTypes.FiducialResult tag : tags) {
            double tx = tag.getTargetXDegrees();
            double ty = tag.getTargetYDegrees();
            LimelightController.TargetObservation solved =
                    LimelightController.solve(tx, ty, 0.0);
            String where = solved == null
                    ? "--"
                    : String.format("%.1f in, %.1f deg", solved.rangeIn, solved.bearingDeg);
            telemetry.addData("  tag " + tag.getFiducialId(),
                    "%s  (tx %.1f  ty %.1f)", where, tx, ty);
        }
    }

    private void setDirection(int i, boolean reverse) {
        if (motors[i] != null) {
            motors[i].setDirection(reverse
                    ? DcMotorSimple.Direction.REVERSE
                    : DcMotorSimple.Direction.FORWARD);
        }
    }

    /** Dpad up/down nudges speed by 0.1, once per press. */
    private void adjustSpeed() {
        if (gamepad1.dpad_up && !lastUp) {
            TEST_POWER = Math.min(1.0, TEST_POWER + 0.1);
        }
        if (gamepad1.dpad_down && !lastDown) {
            TEST_POWER = Math.max(0.0, TEST_POWER - 0.1);
        }
        TEST_POWER = Math.round(TEST_POWER * 10.0) / 10.0;
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
    }

    private static double deadzone(double v) {
        return Math.abs(v) < DEADZONE ? 0.0 : v;
    }
}
