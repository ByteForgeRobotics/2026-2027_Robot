package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.sim.Sim;
import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

/** Mecanum mixing in {@link Drivetrain}: does each stick command move the robot the right way? */
public class DrivetrainTest {

    private static final double EPS = 1e-9;

    private static double[] cmd(SimRobot r, double f, double l, double t) {
        r.drive.drive(f, l, t);
        return r.commanded();
    }

    @Test
    public void forwardDrivesForward() {
        double[] c = cmd(new SimRobot(0, 0, 0), 0.5, 0, 0);
        assertEquals(0.5, c[0], EPS);
        assertEquals(0, c[1], EPS);
        assertEquals(0, c[2], EPS);
    }

    @Test
    public void leftStrafesLeftWithTheStrafeBoost() {
        double[] c = cmd(new SimRobot(0, 0, 0), 0, 0.5, 0);
        assertEquals(0, c[0], EPS);
        assertEquals(0.5 * Drivetrain.STRAFE_MULTIPLIER, c[1], EPS);
        assertEquals(0, c[2], EPS);
    }

    @Test
    public void positiveTurnIsCounterClockwise() {
        double[] c = cmd(new SimRobot(0, 0, 0), 0, 0, 0.5);
        assertEquals(0, c[0], EPS);
        assertEquals(0, c[1], EPS);
        assertEquals(0.5, c[2], EPS);
    }

    @Test
    public void fullStickInEveryDirectionStaysInsideOneAndKeepsDirection() {
        SimRobot r = new SimRobot(0, 0, 0);
        r.drive.drive(1, 1, 1);
        double max = 0;
        for (Sim.SimMotor w : r.wheels) max = Math.max(max, Math.abs(w.power));
        assertEquals(1.0, max, EPS);
        double[] c = r.commanded();
        // forward : left : turn ratio survives the scaling (left carries the strafe boost)
        assertEquals(c[0], c[2], EPS);
        assertEquals(c[0] * Drivetrain.STRAFE_MULTIPLIER, c[1], EPS);
    }

    @Test
    public void setsBrakeAndDirectionsFromTheFlags() {
        SimRobot r = new SimRobot(0, 0, 0);
        boolean[] flags = {Drivetrain.REVERSE_FL, Drivetrain.REVERSE_FR, Drivetrain.REVERSE_BL, Drivetrain.REVERSE_BR};
        for (int i = 0; i < 4; i++) {
            assertEquals(DcMotor.ZeroPowerBehavior.BRAKE, r.wheels[i].zeroPower);
            assertEquals(Drivetrain.NAMES[i], flags[i] ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD,
                    r.wheels[i].direction);
        }
    }

    @Test
    public void stopZeroesEveryWheel() {
        SimRobot r = new SimRobot(0, 0, 0);
        r.drive.drive(0.7, -0.2, 0.3);
        r.drive.stop();
        for (Sim.SimMotor w : r.wheels) assertEquals(0, w.power, 0);
    }

    @Test
    public void missingWheelStillDrivesTheOthers() {
        Sim.FakeHardwareMap hm = new Sim.FakeHardwareMap();
        Sim.SimMotor fr = new Sim.SimMotor(537.7, 312, 0.05);
        hm.add("FR", fr.motor);
        Drivetrain d = new Drivetrain(hm);
        assertFalse(d.hasMotor(0));
        assertTrue(d.hasMotor(1));
        d.drive(0.5, 0, 0);
        assertEquals(0.5, fr.power, EPS);
    }
}
