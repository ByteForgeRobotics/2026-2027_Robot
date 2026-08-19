package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Example of a Road Runner "Action" subsystem: a single-motor lift held by a
 * simple proportional controller.
 *
 * The pattern to copy is at the bottom of this file. An Action is any object
 * with a run(packet) method that does one slice of work and returns true if it
 * wants to be called again. That lets the lift run *concurrently* with a
 * trajectory instead of blocking it -- see SampleAutonomous.
 *
 * TODO: replace the motor name, ticks, and gains with your robot's real values.
 */
@Config
public class Lift {
    /** Tunable live from FTC Dashboard while the robot is running. */
    public static double KP = 0.005;
    public static double TOLERANCE_TICKS = 20;
    public static double MAX_POWER = 0.8;
    public static double HOLD_POWER = 0.08;

    /** Encoder positions for the named heights. Measure these on the robot. */
    public static int DOWN_TICKS = 0;
    public static int LOW_TICKS = 500;
    public static int HIGH_TICKS = 1500;

    private final DcMotorEx motor;

    public Lift(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "lift");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    /** Drive the lift to a target and finish once it is close enough. */
    public Action goTo(int targetTicks) {
        return new GoToAction(targetTicks);
    }

    public Action toDown() {
        return goTo(DOWN_TICKS);
    }

    public Action toLow() {
        return goTo(LOW_TICKS);
    }

    public Action toHigh() {
        return goTo(HIGH_TICKS);
    }

    private class GoToAction implements Action {
        private final int target;

        GoToAction(int target) {
            this.target = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            int position = motor.getCurrentPosition();
            double error = target - position;

            packet.put("lift position", position);
            packet.put("lift target", target);

            if (Math.abs(error) <= TOLERANCE_TICKS) {
                // Small holding power fights gravity once we've arrived.
                motor.setPower(target > DOWN_TICKS ? HOLD_POWER : 0);
                return false; // done -- do not call again
            }

            double power = Math.max(-MAX_POWER, Math.min(MAX_POWER, KP * error));
            motor.setPower(power);
            return true; // keep running
        }
    }
}
