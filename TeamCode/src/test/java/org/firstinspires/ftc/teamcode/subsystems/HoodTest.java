package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.sim.Sim;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * Hood on a simulated Studica Maverick 61:1 (100 rpm, 1464 ticks/rev) through the 32:180
 * sector. The sim is built to match the flags in the code: the motor has to run REVERSE to lift
 * the hood, and its encoder counts the opposite way from the motor (the quirk
 * ENCODER_REVERSED exists for). The hood sits on its bottom stop at INIT and can travel
 * 37 deg (~846 ticks) before hitting the top.
 */
public class HoodTest {

    private boolean reverse, encReversed;
    private int maxTicks;
    private double kg;

    @Before
    public void save() {
        reverse = Hood.REVERSE;
        encReversed = Hood.ENCODER_REVERSED;
        maxTicks = Hood.MAX_TICKS;
        kg = Hood.KG;
    }

    @After
    public void restore() {
        Hood.REVERSE = reverse;
        Hood.ENCODER_REVERSED = encReversed;
        Hood.MAX_TICKS = maxTicks;
        Hood.KG = kg;
    }

    static final double TRAVEL_TICKS = 37 * 22.875;

    static Sim.SimMotor hoodMotor() {
        Sim.SimMotor m = new Sim.SimMotor(1464, 100, 0.03);
        m.encoderInverted = true;
        // Up is negative motor rotation (that's why REVERSE = true lifts it).
        m.stopHigh = 0;
        m.stopLow = -TRAVEL_TICKS;
        return m;
    }

    static Hood hood(Sim.SimMotor m) {
        return new Hood(new Sim.FakeHardwareMap().add(Hood.NAME, m.motor));
    }

    /** Hood height above its bottom stop, in ticks, from the physics (not the encoder). */
    static double trueTicks(Sim.SimMotor m) {
        return -m.physPos;
    }

    @Test
    public void missingMotorDoesNotCrash() {
        Hood h = new Hood(new Sim.FakeHardwareMap());
        assertFalse(h.isConnected());
        h.setTarget(300);
        h.update();
        h.stop();
    }

    @Test
    public void movesToATargetAndHolds() {
        Sim.SimMotor m = hoodMotor();
        Hood h = hood(m);
        h.setTarget(400);
        double[] peak = {0};
        long t0 = System.nanoTime();
        double[] arrived = {-1};
        Sim.run(1.5, () -> {
            h.update();
            peak[0] = Math.max(peak[0], h.getPosition());
            if (arrived[0] < 0 && h.atTarget()) arrived[0] = (System.nanoTime() - t0) / 1e9;
        }, m);
        System.out.printf("Hood 0 -> 400 ticks (17.5 deg): at target after %.2f s, overshoot %d ticks, ends at %d%n",
                arrived[0], (int) (peak[0] - 400), h.getPosition());
        assertTrue("never got there, at " + h.getPosition(), h.atTarget());
        assertTrue("slow: " + arrived[0], arrived[0] < 1.0);
        assertTrue("overshoot " + (peak[0] - 400), peak[0] - 400 < 20);
        assertEquals("encoder and real hood disagree", trueTicks(m), h.getPosition(), 2);
        assertFalse(h.isFaulted());
    }

    @Test
    public void goesBackDownToo() {
        Sim.SimMotor m = hoodMotor();
        Hood h = hood(m);
        h.setTarget(600);
        Sim.run(1.0, h::update, m);
        h.setTarget(100);
        Sim.run(1.0, h::update, m);
        assertTrue("at " + h.getPosition(), h.atTarget());
    }

    @Test
    public void wrongEncoderFlagTripsTheRunawayCutoff() {
        Hood.ENCODER_REVERSED = !Hood.ENCODER_REVERSED;
        Sim.SimMotor m = hoodMotor();
        Hood h = hood(m);
        h.setTarget(300);
        Sim.run(1.5, h::update, m);
        assertTrue("should have faulted", h.isFaulted());
        assertEquals(0, m.power, 0);
        h.setTarget(300);
        h.update();
        assertEquals("stays off until re-zeroed", 0, m.power, 0);
        h.zeroHere();
        assertFalse(h.isFaulted());
    }

    @Test
    public void targetIsClampedToTheSoftLimits() {
        Hood h = hood(hoodMotor());
        h.setTarget(5000);
        assertEquals(Hood.MAX_TICKS, h.getTarget());
        h.setTarget(-50);
        assertEquals(Hood.MIN_TICKS, h.getTarget());
    }

    @Test
    public void softLimitIsBelowTheRealTopStop() {
        assertTrue("MAX_TICKS " + Hood.MAX_TICKS + " is past the ~" + (int) TRAVEL_TICKS + " tick top stop",
                Hood.MAX_TICKS < TRAVEL_TICKS);
    }

    @Test
    public void angleAndTicksConvertBothWays() {
        Hood h = hood(hoodMotor());
        h.setAngleDeg(60);
        assertEquals(Math.round((60 - Hood.ANGLE_AT_ZERO_DEG) * Hood.TICKS_PER_DEG), h.getTarget());
        h.setAngleDeg(10); // below the bottom stop
        assertEquals(0, h.getTarget());
    }

    @Test
    public void everyAngleTheSolverCanAskForFitsTheHood() {
        // If the solver picks an angle the hood can't reach, the shot is silently wrong.
        double lo = Hood.ANGLE_AT_ZERO_DEG + Hood.MIN_TICKS / Hood.TICKS_PER_DEG;
        double hi = Hood.ANGLE_AT_ZERO_DEG + Hood.MAX_TICKS / Hood.TICKS_PER_DEG;
        System.out.printf("Hood reaches %.1f..%.1f deg; solver allowed %.1f..%.1f deg%n",
                lo, hi, ShotSolver.ANGLE_MIN_DEG, ShotSolver.ANGLE_MAX_DEG);
        assertTrue(ShotSolver.ANGLE_MIN_DEG >= lo - 0.05);
        assertTrue("solver max " + ShotSolver.ANGLE_MAX_DEG + " > hood max " + hi,
                ShotSolver.ANGLE_MAX_DEG <= hi + 0.05);
    }

    @Test
    public void holdsAgainstGravityWithoutKG() {
        // How far does the hood sag if gravity pulls with 6% of motor power and KG is 0?
        Sim.SimMotor m = hoodMotor();
        m.loadPower = 0.06; // positive = toward the bottom stop
        Hood h = hood(m);
        h.setTarget(500);
        Sim.run(1.5, h::update, m);
        System.out.printf("Hood with gravity load and KG=0: holds at %d (target 500)%n", h.getPosition());
        assertTrue("sagged to " + h.getPosition(), Math.abs(h.getPosition() - 500) <= 15);
        // Shooting waits on atTarget(). With KG = 0 the hood sits right at the edge of
        // TOLERANCE_TICKS, so READY can flicker or never come true. Reported, not asserted.
        System.out.printf("  sag %d ticks vs TOLERANCE_TICKS %d -> atTarget()=%s%n",
                500 - h.getPosition(), Hood.TOLERANCE_TICKS, h.atTarget());
    }
}
