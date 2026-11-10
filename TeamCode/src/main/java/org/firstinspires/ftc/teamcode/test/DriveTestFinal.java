package org.firstinspires.ftc.teamcode.test;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.PowerMonitor;


/**
 * Lean robot-centric drive test that sends speed, battery and Pinpoint position to both the
 * Driver Station and FTC Dashboard (so they can be graphed).
 *
 * <ul>
 *   <li>Left stick: drive and strafe. Right stick X: turn.</li>
 *   <li>Dpad up / down: top speed +/- 0.1.</li>
 *   <li>Y: re-send pod settings to the Pinpoint, zero it, and clear battery peaks. Keep the
 *       robot still.</li>
 * </ul>
 *
 * <p>Motor directions here are a separate copy of the ones in
 * {@link org.firstinspires.ftc.teamcode.subsystems.Drivetrain}, so they can be flipped live
 * from the Dashboard. Keep the two in sync.
 */
@Config
@TeleOp(name = "Drive Motor Test Final", group = "Tests")
public class DriveTestFinal extends LinearOpMode {


    public static double MAX_DRIVE_POWER = 0.7;
    public static double STRAFE_MULTIPLIER = 1.1;
    public static double DEADZONE = 0.05;
    public static boolean REVERSE_FL = true;
    public static boolean REVERSE_FR = false;
    public static boolean REVERSE_BL = true;
    public static boolean REVERSE_BR = false;
    private static final String[] NAMES = {"FL", "FR", "BL", "BR"};
    private static final int FL = 0, FR = 1, BL = 2, BR = 3;
    public static String PINPOINT_NAME = "pinpoint";
    public static GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static double X_POD_OFFSET_MM = 0.0;
    public static double Y_POD_OFFSET_MM = 0.0;
    public static boolean REVERSE_X_POD = false;
    public static boolean REVERSE_Y_POD = false;
    private GoBildaPinpointDriver pinpoint;

    private final DcMotorEx[] motors = new DcMotorEx[NAMES.length];
    private final double[] powers = new double[NAMES.length];
    private PowerMonitor powerMonitor;

    @Override
    public void runOpMode() throws InterruptedException {
        for (int i = 0; i < NAMES.length; i++) {
            motors[i] = hardwareMap.tryGet(DcMotorEx.class, NAMES[i]);
            if (motors[i] != null) {
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, PINPOINT_NAME);
        configurePinpoint();

        // Battery only. No motors registered, so it never reads motor current.
        powerMonitor = new PowerMonitor(hardwareMap);

        telemetry.setMsTransmissionInterval(50);

        while (opModeInInit()) {
            for (int i = 0; i < NAMES.length; i++) {
                telemetry.addData(NAMES[i], motors[i] == null ? "NOT FOUND" : "ok");
            }
            // Wait for READY before driving; it reads CALIBRATING right after a reset.
            if (pinpoint == null) {
                telemetry.addData("Pinpoint", "NOT FOUND");
            } else {
                pinpoint.update();
                telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
            }
            powerMonitor.update();
            telemetry.addData("battery (V)", "%.2f", powerMonitor.getVolts());
            telemetry.addLine("Dpad up/down = speed  Y = reset Pinpoint (stand still!)");
            telemetry.update();
        }

        // Resting readings from init should not count toward the sag numbers.
        powerMonitor.resetPeaks();

        try {
            while (opModeIsActive()) {
                // Re-applied every loop so flipping a flag in the dashboard takes effect live.
                // Free: the SDK only stores the direction, it doesn't talk to the hub.
                setDirection(FL, REVERSE_FL);
                setDirection(FR, REVERSE_FR);
                setDirection(BL, REVERSE_BL);
                setDirection(BR, REVERSE_BR);

                adjustSpeed();
                powerMonitor.update();

                if (gamepad1.yWasPressed()) {
                    configurePinpoint();
                    powerMonitor.resetPeaks();
                }

                double y = -deadzone(gamepad1.left_stick_y); // stick up is negative
                double x = deadzone(gamepad1.left_stick_x) * STRAFE_MULTIPLIER;
                double rx = deadzone(gamepad1.right_stick_x);

                // Dividing by the biggest possible sum keeps every wheel within [-1, 1]
                // without changing the ratio between them, so combined moves stay the
                // right shape.
                double denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
                double maxPower = Range.clip(MAX_DRIVE_POWER, 0.0, 1.0);

                powers[FL] = (y + x + rx) / denom * maxPower;
                powers[BL] = (y - x + rx) / denom * maxPower;
                powers[FR] = (y - x - rx) / denom * maxPower;
                powers[BR] = (y + x - rx) / denom * maxPower;

                for (int i = 0; i < NAMES.length; i++) {
                    if (motors[i] != null) {
                        // The SDK skips the hub write when the power hasn't changed.
                        motors[i].setPower(powers[i]);
                    }
                }

                // Only what matters while driving. Problems show up as extra lines.
                TelemetryPacket packet = new TelemetryPacket();
                put(packet, "speed", maxPower);
                put(packet, "battery (V)", round2(powerMonitor.getVolts()));
                String power = powerMonitor.getStatus();
                if (!power.equals("OK")) {
                    put(packet, "battery", power + " (low " + round2(powerMonitor.getMinVolts()) + " V)");
                }
                showPinpoint(packet);
                for (int i = 0; i < NAMES.length; i++) {
                    if (motors[i] == null) {
                        put(packet, NAMES[i], "NOT FOUND");
                    }
                }

                FtcDashboard.getInstance().sendTelemetryPacket(packet);
                telemetry.update();
            }
        } finally {
            for (DcMotorEx m : motors) {
                if (m != null) {
                    m.setPower(0);
                }
            }
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
        if (gamepad1.dpadUpWasPressed()) {
            MAX_DRIVE_POWER = Math.min(1.0, MAX_DRIVE_POWER + 0.1);
        }
        if (gamepad1.dpadDownWasPressed()) {
            MAX_DRIVE_POWER = Math.max(0.0, MAX_DRIVE_POWER - 0.1);
        }
        MAX_DRIVE_POWER = Math.round(MAX_DRIVE_POWER * 10.0) / 10.0;
    }

    private static double deadzone(double v) {
        return Math.abs(v) < DEADZONE ? 0.0 : v;
    }

    /**
     * Pushes the dashboard pod settings to the Pinpoint and zeroes it. The IMU recalibrates
     * for about a quarter second, so the robot has to be still or the heading starts off.
     */
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

    /**
     * Adds position and heading. Status only shows when it isn't READY. One I2C read per
     * loop: everything here is in the Pinpoint's default bulk read.
     */
    private void showPinpoint(TelemetryPacket packet) {
        if (pinpoint == null) {
            put(packet, "Pinpoint", "NOT FOUND");
            return;
        }
        pinpoint.update();
        GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
        if (status != GoBildaPinpointDriver.DeviceStatus.READY) {
            put(packet, "Pinpoint", status.toString());
        }
        put(packet, "x (in)", round2(pinpoint.getPosX(DistanceUnit.INCH)));
        put(packet, "y (in)", round2(pinpoint.getPosY(DistanceUnit.INCH)));
        put(packet, "heading (deg)", round2(pinpoint.getHeading(AngleUnit.DEGREES)));
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
