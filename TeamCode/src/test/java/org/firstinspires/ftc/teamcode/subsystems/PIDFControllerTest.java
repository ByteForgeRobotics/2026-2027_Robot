package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.sim.Sim;
import org.junit.Test;

/** Laptop tests for {@link PIDFController}, the loop the hood, flywheel and intake share. */
public class PIDFControllerTest {

    @Test
    public void proportionalAndFeedforwardAddUp() {
        PIDFController pid = new PIDFController();
        // kF*target + kS*sign + kP*error, no D or I on the first call
        double out = pid.calculate(100, 80, 0.01, 0, 0, 0.002, 0.05);
        assertEquals(0.2 + 0.05 + 0.2, out, 1e-9);
    }

    @Test
    public void staticTermFollowsTheTargetsSign() {
        PIDFController pid = new PIDFController();
        assertEquals(-0.05, pid.calculate(-100, -100, 0, 0, 0, 0, 0.05), 1e-9);
        pid.reset();
        assertEquals(0, pid.calculate(0, 0, 0, 0, 0, 0, 0.05), 1e-9);
    }

    @Test
    public void firstCallSkipsDerivativeAndIntegral() {
        PIDFController pid = new PIDFController();
        // Huge D and I gains: the first call must not use them (no trustworthy dt yet).
        assertEquals(0, pid.calculate(10, 0, 0, 100, 100, 0, 0), 1e-9);
    }

    @Test
    public void derivativeSeesTheErrorChanging() {
        PIDFController pid = new PIDFController();
        pid.calculate(10, 0, 0, 0, 1, 0, 0);
        Sim.sleep(20);
        double out = pid.calculate(10, 5, 0, 0, 1, 0, 0); // error fell 10 -> 5 in ~20 ms
        assertTrue("D should brake (negative), got " + out, out < -50);
    }

    @Test
    public void integralIsCappedSoItCantWindUp() {
        PIDFController pid = new PIDFController();
        double out = 0;
        for (int i = 0; i < 60; i++) {
            out = pid.calculate(1000, 0, 0, 1.0, 0, 0, 0);
            Sim.sleep(10);
        }
        assertEquals(pid.integralLimit, out, 1e-6);
    }

    @Test
    public void aStalledLoopDoesNotSpikeD() {
        PIDFController pid = new PIDFController();
        pid.calculate(10, 0, 0, 0, 1, 0, 0);
        Sim.sleep(600); // a 0.6 s hiccup: skip D/I this once instead of trusting the gap
        assertEquals(0, pid.calculate(10, 10, 0, 0, 1, 0, 0), 1e-9);
    }

    @Test
    public void resetForgetsTheIntegral() {
        PIDFController pid = new PIDFController();
        for (int i = 0; i < 10; i++) {
            pid.calculate(100, 0, 0, 0.001, 0, 0, 0);
            Sim.sleep(10);
        }
        pid.reset();
        assertEquals(0, pid.calculate(100, 0, 0, 0.001, 0, 0, 0), 1e-9);
    }
}
