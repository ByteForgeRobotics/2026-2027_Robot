package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.PowerMonitor;

/**
 * Spins each drive motor on its own so you can check wiring, direction, and encoders.
 *
 * <p>Motors are run raw: no direction reversal, no Road Runner. With positive power every
 * motor spins the way the SDK calls "forward" for that port, so on a mecanum robot the left
 * and right sides will turn opposite ways. Whichever side is backwards when the robot drives
 * is the side that needs {@code setDirection(REVERSE)}.
 *
 * <h2>Controls (gamepad 1)</h2>
 * <ul>
 *   <li>Hold X: FL. Hold Y: FR. Hold A: BL. Hold B: BR.</li>
 *   <li>Hold right bumper: all four at once.</li>
 *   <li>Hold left bumper while doing any of the above: run in reverse.</li>
 *   <li>Dpad up / down: raise or lower test power by 0.1.</li>
 *   <li>Back: zero all encoders and all min/peak power history.</li>
 * </ul>
 *
 * <p>Telemetry shows power, encoder ticks, velocity, and current for each motor, on the
 * Driver Station and in FTC Dashboard (graphable). A motor missing from the configuration
 * shows as "NOT FOUND" instead of crashing the OpMode.
 *
 * <p>Battery voltage and whole-robot current draw come from {@link PowerMonitor}, under the
 * per-motor lines, along with the highest current each motor has reached
 * ({@code FL amps peak} and friends). Peak is the number to compare between motors: spin each
 * one in turn at the same power and they should peak within a few tenths of an amp of each
 * other. One that peaks much higher is binding, mis-wired, or dragging a bad bearing — and the
 * live reading is easy to miss, because the spike happens in the instant the wheel breaks
 * loose. Peaks here are sampled every loop, not on the monitor's slower cadence, so short
 * spikes do get caught.
 */
@Config 
@TeleOp(name = "Drive Motor Test", group = "Diagnostics")
public class DriveMotorTest extends LinearOpMode {

    /** Starting power for the test. Adjustable live with the dpad or the dashboard. */
    public static double TEST_POWER = 0.3;

    /** Configuration names, in the same order as the buttons in the javadoc. */
    private static final String[] NAMES = {"FL", "FR", "BL", "BR"};

    private final DcMotorEx[] motors = new DcMotorEx[NAMES.length];

    private PowerMonitor powerMonitor;

    private boolean lastUp, lastDown;

    @Override
    public void runOpMode() throws InterruptedException {
        for (int i = 0; i < NAMES.length; i++) {
            motors[i] = hardwareMap.tryGet(DcMotorEx.class, NAMES[i]);
            if (motors[i] != null) {
                motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
        }
        resetEncoders();

        // Per-motor amps are already read every loop below, so those readings are handed to
        // the monitor rather than read twice. Sampling every loop instead of on the monitor's
        // slower cadence also means a brief stall spike actually lands in the peak.
        powerMonitor = new PowerMonitor(hardwareMap);
        powerMonitor.trackMotors(NAMES, motors);

        telemetry.setMsTransmissionInterval(50);

        while (opModeInInit()) {
            telemetry.addData("Status", "Ready. Robot on blocks, wheels off the ground!");
            for (int i = 0; i < NAMES.length; i++) {
                telemetry.addData(NAMES[i], motors[i] == null ? "NOT FOUND" : "ok");
            }
            powerMonitor.update();
            telemetry.addData("battery (V)", "%.2f", powerMonitor.getVolts());
            telemetry.addLine("X=FL  Y=FR  A=BL  B=BR  RB=all  LB=reverse");
            telemetry.update();
        }

        // Resting readings from init should not count toward the sag numbers.
        powerMonitor.resetPeaks();

        try {
            while (opModeIsActive()) {
                adjustPower();
                powerMonitor.update();
                if (gamepad1.back) {
                    resetEncoders();
                    powerMonitor.resetPeaks();
                }

                double power = gamepad1.left_bumper ? -TEST_POWER : TEST_POWER;
                boolean all = gamepad1.right_bumper;
                boolean[] held = {gamepad1.x, gamepad1.y, gamepad1.a, gamepad1.b};

                TelemetryPacket packet = new TelemetryPacket();
                put(packet, "test power", power);

                for (int i = 0; i < NAMES.length; i++) {
                    DcMotorEx m = motors[i];
                    if (m == null) {
                        put(packet, NAMES[i], "NOT FOUND");
                        continue;
                    }
                    double p = (all || held[i]) ? power : 0.0;
                    m.setPower(p);

                    double amps = m.getCurrent(CurrentUnit.AMPS);
                    powerMonitor.recordMotorAmps(i, amps);

                    put(packet, NAMES[i] + " power", p);
                    put(packet, NAMES[i] + " ticks", m.getCurrentPosition());
                    put(packet, NAMES[i] + " vel (tps)", Math.round(m.getVelocity()));
                    put(packet, NAMES[i] + " amps", round2(amps));
                }

                powerMonitor.report((key, value) -> put(packet, key, value));

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

    /** Dpad up/down nudges power by 0.1, once per press. */
    private void adjustPower() {
        if (gamepad1.dpad_up && !lastUp) {
            TEST_POWER = Math.min(1.0, TEST_POWER + 0.1);
        }
        if (gamepad1.dpad_down && !lastDown) {
            TEST_POWER = Math.max(0.0, TEST_POWER - 0.1);
        }
        TEST_POWER = round2(TEST_POWER);
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
    }

    private void resetEncoders() {
        for (DcMotorEx m : motors) {
            if (m != null) {
                m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
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
