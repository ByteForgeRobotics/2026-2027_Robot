package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.PowerMonitor;

/**
 * The match TeleOp. Robot-centric mecanum drive on gamepad 1.
 *
 * <ul>
 *   <li>Left stick: drive forward/back and strafe.</li>
 *   <li>Right stick X: turn.</li>
 *   <li>Hold left bumper or right trigger: slow mode, for lining up.</li>
 *   <li>Dpad up / down: raise or lower normal speed by 0.1.</li>
 * </ul>
 *
 * <p>Motor names and directions come from {@link Drivetrain} (FL, FR, BL, BR). If a wheel
 * spins the wrong way, flip its {@code REVERSE_*} flag there, not here.
 *
 * <p>Deliberately plain: no dashboard, no vision, no odometry. Fewer things to go wrong in a
 * match. A missing motor shows as a warning instead of crashing, so the robot still drives on
 * the wheels it has.
 */
// ======================================================================
// DO NOT TOUCH
// This is the match TeleOp. The team runs it at competitions.
// Change it on the test branch first and drive it on the real robot
// before it goes on the competition branch. Ask Ryan first.
// The name "Comp TeleOp" must match preselectTeleOp in CompAutoLeave.
// ======================================================================
@TeleOp(name = "Comp TeleOp", group = "Competition")
public class CompTeleOp extends LinearOpMode {

    /** Normal top speed, 0 to 1. Dpad changes it during the match. */
    public static double NORMAL_SPEED = 0.8;
    /** Top speed while slow mode is held. */
    public static double SLOW_SPEED = 0.35;
    /** Stick values smaller than this count as zero, so a drifting stick doesn't creep. */
    public static double DEADZONE = 0.05;

    @Override
    public void runOpMode() {
        Drivetrain drive = new Drivetrain(hardwareMap);
        PowerMonitor power = new PowerMonitor(hardwareMap);
        double speed = NORMAL_SPEED;

        while (opModeInInit()) {
            power.update();
            telemetry.addLine("COMP TELEOP - ready");
            addMissingMotors(drive);
            telemetry.addData("battery (V)", "%.2f", power.getVolts());
            if (power.getVolts() > 0 && power.getVolts() < 12.5) {
                telemetry.addLine("!! Battery is low. Swap it before the match.");
            }
            telemetry.update();
        }

        // Only count sag while driving, not the resting voltage from init.
        power.resetPeaks();

        try {
            while (opModeIsActive()) {
                if (gamepad1.dpadUpWasPressed()) {
                    speed = Math.min(1.0, speed + 0.1);
                }
                if (gamepad1.dpadDownWasPressed()) {
                    speed = Math.max(0.2, speed - 0.1);
                }
                speed = Math.round(speed * 10.0) / 10.0;

                boolean slow = gamepad1.left_bumper || gamepad1.right_trigger > 0.5;
                double max = slow ? SLOW_SPEED : speed;

                // Stick up is negative; Drivetrain wants +forward, +left, +counter-clockwise.
                double forward = -deadzone(gamepad1.left_stick_y) * max;
                double left = -deadzone(gamepad1.left_stick_x) * max;
                double turn = -deadzone(gamepad1.right_stick_x) * max;
                drive.drive(forward, left, turn);

                power.update();
                telemetry.addData("speed", slow ? "SLOW (" + SLOW_SPEED + ")" : speed);
                telemetry.addData("battery (V)", "%.2f   low %.2f",
                        power.getVolts(), power.getMinVolts());
                if (!power.getStatus().equals("OK")) {
                    telemetry.addData("battery", power.getStatus());
                }
                addMissingMotors(drive);
                telemetry.update();
            }
        } finally {
            drive.stop();
        }
    }

    private void addMissingMotors(Drivetrain drive) {
        for (int i = 0; i < Drivetrain.NAMES.length; i++) {
            if (!drive.hasMotor(i)) {
                telemetry.addData("!! " + Drivetrain.NAMES[i], "NOT FOUND - check config");
            }
        }
    }

    private static double deadzone(double v) {
        return Math.abs(v) < DEADZONE ? 0.0 : v;
    }
}
