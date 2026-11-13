package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Runs and tunes the hood, flywheel and intake. Pit / practice only.
 *
 * <p>Graph {@code flywheel rpm} against {@code flywheel target} (and the same for intake and
 * hood) in FTC Dashboard while changing the gains under Flywheel / Intake / Hood in the config
 * panel. Gains change live; copy the good numbers into the .java file afterward or they're lost
 * when the robot restarts.
 *
 * <h2>Controls (gamepad 1)</h2>
 * <ul>
 *   <li>A: flywheel on/off at {@code TEST_FLYWHEEL_RPM}. Dpad left / right: -/+ 100 rpm.</li>
 *   <li>Right bumper (hold): intake in. Left bumper (hold): intake out.</li>
 *   <li>Dpad up / down: hood +/- {@code HOOD_STEP} ticks.</li>
 *   <li>X: hood to POS_CLOSE. Y: hood to POS_FAR. B: hood to POS_DOWN.</li>
 *   <li>Back: call the hood's current spot zero.</li>
 * </ul>
 *
 * <p>The hood must be all the way down when you press INIT.
 */
@Config
@TeleOp(name = "Mechanism Test", group = "Tests")
public class MechanismTest extends LinearOpMode {

    public static double TEST_FLYWHEEL_RPM = 3000;
    public static int HOOD_STEP = 10;

    @Override
    public void runOpMode() {
        Hood hood = new Hood(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry.setMsTransmissionInterval(50);

        while (opModeInInit()) {
            telemetry.addLine("MECHANISM TEST - hood all the way down before INIT!");
            missing("hood", hood.isConnected());
            missing("flywheel", flywheel.isConnected());
            missing("intake", intake.isConnected());
            telemetry.update();
        }

        boolean flywheelOn = false;
        try {
            while (opModeIsActive()) {
                if (gamepad1.aWasPressed()) {
                    flywheelOn = !flywheelOn;
                }
                if (gamepad1.dpadRightWasPressed()) {
                    TEST_FLYWHEEL_RPM += 100;
                }
                if (gamepad1.dpadLeftWasPressed()) {
                    TEST_FLYWHEEL_RPM = Math.max(0, TEST_FLYWHEEL_RPM - 100);
                }
                flywheel.setRpm(flywheelOn ? TEST_FLYWHEEL_RPM : 0);

                if (gamepad1.right_bumper) {
                    intake.in();
                } else if (gamepad1.left_bumper) {
                    intake.out();
                } else {
                    intake.stop();
                }

                if (gamepad1.dpadUpWasPressed()) hood.nudge(HOOD_STEP);
                if (gamepad1.dpadDownWasPressed()) hood.nudge(-HOOD_STEP);
                if (gamepad1.xWasPressed()) hood.setTarget(Hood.POS_CLOSE);
                if (gamepad1.yWasPressed()) hood.setTarget(Hood.POS_FAR);
                if (gamepad1.bWasPressed()) hood.setTarget(Hood.POS_DOWN);
                if (gamepad1.backWasPressed()) hood.zeroHere();

                hood.update();
                flywheel.update();
                intake.update();

                telemetry.addData("flywheel", "%s  %.0f / %.0f rpm  pwr %.2f",
                        flywheel.atSpeed() ? "READY" : "spinning",
                        flywheel.getRpm(), flywheel.getTargetRpm(), flywheel.getPower());
                telemetry.addData("intake", "%s%.0f / %.0f rpm  pwr %.2f",
                        intake.isJammed() ? "JAMMED  " : "",
                        intake.getRpm(), intake.getTargetRpm(), intake.getPower());
                if (hood.isFaulted()) {
                    telemetry.addLine("!! HOOD RUNAWAY - power cut. Flip Hood.ENCODER_REVERSED, push the hood down, then zero it (Back).");
                }
                telemetry.addData("hood", "%s%d / %d ticks  pwr %.2f",
                        hood.atTarget() ? "" : "moving  ",
                        hood.getPosition(), hood.getTarget(), hood.getPower());
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.put("flywheel rpm", flywheel.getRpm());
                packet.put("flywheel target", flywheel.getTargetRpm());
                packet.put("intake rpm", intake.getRpm());
                packet.put("intake target", intake.getTargetRpm());
                packet.put("hood ticks", hood.getPosition());
                packet.put("hood target", hood.getTarget());
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

    private void missing(String name, boolean connected) {
        if (!connected) {
            telemetry.addData("!! " + name, "NOT FOUND - check config");
        }
    }
}
