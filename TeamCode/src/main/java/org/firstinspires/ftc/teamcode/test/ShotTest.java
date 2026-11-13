package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Field;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShotSolver;

/**
 * Tests {@link ShotSolver} on the real shooter. Pit / practice only.
 *
 * <p>Put the robot a measured distance from the goal (from the hood lip, flat on the floor),
 * dial that distance in, spin up, shoot. If shots land short at every distance, lower
 * {@code ShotSolver.SLIP}; long, raise it. If they're short only far away, the ball is probably
 * lighter or draggier than the solver thinks: weigh one and fix {@code BALL_MASS_KG}.
 *
 * <h2>Controls (gamepad 1)</h2>
 * <ul>
 *   <li>Dpad up / down: distance +/- 0.10 m. Dpad right / left: +/- 0.02 m.</li>
 *   <li>A: spin up and aim (flywheel to solver RPM, hood to solver angle). A again: stop.</li>
 *   <li>Right bumper (hold): feed with the intake. Only feeds once the flywheel is at speed.</li>
 *   <li>Left bumper (hold): intake out.</li>
 *   <li>Back: call the hood's current spot zero (also clears a hood runaway).</li>
 * </ul>
 *
 * <p>The hood must be at its zero spot (see {@code Hood.ANGLE_AT_ZERO_DEG}) when you press INIT.
 */
@Config
@TeleOp(name = "Shot Test", group = "Tests")
public class ShotTest extends LinearOpMode {

    /** Height of the target's center above the floor, meters. Default: the HIVE up-cell opening. */
    public static double TARGET_HEIGHT_M = Field.CELL_OPENING_HEIGHT_M;
    public static double START_DISTANCE_M = 1.5;

    @Override
    public void runOpMode() {
        Hood hood = new Hood(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry.setMsTransmissionInterval(50);

        double distance = START_DISTANCE_M;
        ShotSolver.Shot shot = ShotSolver.solve(distance, TARGET_HEIGHT_M);

        while (opModeInInit()) {
            telemetry.addLine("SHOT TEST - hood at its zero spot before INIT!");
            if (!hood.isConnected()) telemetry.addData("!! hood", "NOT FOUND - check config");
            if (!flywheel.isConnected()) telemetry.addData("!! flywheel", "NOT FOUND - check config");
            if (!intake.isConnected()) telemetry.addData("!! intake", "NOT FOUND - check config");
            telemetry.update();
        }

        boolean armed = false;
        double lastDistance = -1, lastHeight = -1;
        try {
            while (opModeIsActive()) {
                if (gamepad1.dpadUpWasPressed()) distance += 0.10;
                if (gamepad1.dpadDownWasPressed()) distance -= 0.10;
                if (gamepad1.dpadRightWasPressed()) distance += 0.02;
                if (gamepad1.dpadLeftWasPressed()) distance -= 0.02;
                distance = Math.max(0.3, Math.round(distance * 100) / 100.0);
                if (gamepad1.aWasPressed()) armed = !armed;
                if (gamepad1.backWasPressed()) hood.zeroHere();

                // Only re-solve when something changed (dpad or dashboard).
                if (distance != lastDistance || TARGET_HEIGHT_M != lastHeight) {
                    shot = ShotSolver.solve(distance, TARGET_HEIGHT_M);
                    lastDistance = distance;
                    lastHeight = TARGET_HEIGHT_M;
                }

                if (armed && shot.reachable) {
                    flywheel.setRpm(shot.rpm);
                    hood.setAngleDeg(shot.angleDeg);
                } else {
                    flywheel.stop();
                }

                if (gamepad1.right_bumper && armed && flywheel.atSpeed() && hood.atTarget()) {
                    intake.in();
                } else if (gamepad1.left_bumper) {
                    intake.out();
                } else {
                    intake.stop();
                }

                hood.update();
                flywheel.update();
                intake.update();

                telemetry.addData("distance (m)", "%.2f   target height %.2f", distance, TARGET_HEIGHT_M);
                telemetry.addData("solver", shot.toString());
                telemetry.addData("state", !armed ? "stopped (A to spin up)"
                        : flywheel.atSpeed() && hood.atTarget() ? "READY - hold RB to shoot" : "aiming...");
                telemetry.addData("flywheel", "%.0f / %.0f rpm", flywheel.getRpm(), flywheel.getTargetRpm());
                if (hood.isFaulted()) {
                    telemetry.addLine("!! HOOD RUNAWAY - power cut. Flip Hood.ENCODER_REVERSED, push the hood down, then zero it (Back).");
                }
                telemetry.addData("hood", "%.1f / %.1f deg (%d ticks)",
                        hood.getAngleDeg(), shot.angleDeg, hood.getPosition());
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.put("flywheel rpm", flywheel.getRpm());
                packet.put("flywheel target", flywheel.getTargetRpm());
                packet.put("hood deg", hood.getAngleDeg());
                packet.put("hood target deg", shot.angleDeg);
                dashboard.sendTelemetryPacket(packet);
            }
        } finally {
            flywheel.stop();
            flywheel.update();
            intake.stop();
            intake.update();
            hood.stop();
        }
    }
}
