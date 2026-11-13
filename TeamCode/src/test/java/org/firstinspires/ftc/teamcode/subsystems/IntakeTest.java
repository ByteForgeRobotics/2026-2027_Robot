package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.sim.Sim;
import org.junit.Test;

/** Intake on a simulated goBILDA 435 rpm motor (384.5 ticks/rev). */
public class IntakeTest {

    static Sim.SimMotor motor() {
        return new Sim.SimMotor(384.5, 435 * 0.92, 0.06);
    }

    static Intake intake(Sim.SimMotor m) {
        return new Intake(new Sim.FakeHardwareMap().add(Intake.NAME, m.motor));
    }

    @Test
    public void missingMotorDoesNotCrash() {
        Intake i = new Intake(new Sim.FakeHardwareMap());
        assertFalse(i.isConnected());
        i.in();
        i.update();
    }

    @Test
    public void runsInAtSpeedWithoutAFalseJam() {
        Sim.SimMotor m = motor();
        Intake in = intake(m);
        in.in();
        boolean[] jamSeen = {false};
        Sim.run(1.0, () -> { in.update(); jamSeen[0] |= in.isJammed(); }, m);
        System.out.printf("Intake in: %.0f rpm (target %.0f)%n", in.getRpm(), Intake.RPM_IN);
        assertFalse("spin-up looked like a jam", jamSeen[0]);
        assertEquals(Intake.RPM_IN, in.getRpm(), 25);
    }

    @Test
    public void runsOutBackwards() {
        Sim.SimMotor m = motor();
        Intake in = intake(m);
        in.out();
        Sim.run(0.8, in::update, m);
        assertEquals(Intake.RPM_OUT, in.getRpm(), 25);
    }

    @Test
    public void detectsAJamAndClearsIt() {
        Sim.SimMotor m = motor();
        Intake in = intake(m);
        in.in();
        Sim.run(0.5, in::update, m);
        m.stalled = true;
        Sim.run(0.25, in::update, m);
        assertFalse("too eager", in.isJammed());
        Sim.run(0.4, in::update, m);
        assertTrue("missed the jam", in.isJammed());
        m.stalled = false;
        Sim.run(0.3, in::update, m);
        assertFalse(in.isJammed());
    }

    @Test
    public void stopCutsPower() {
        Sim.SimMotor m = motor();
        Intake in = intake(m);
        in.in();
        Sim.run(0.3, in::update, m);
        in.stop();
        in.update();
        assertEquals(0, m.power, 0);
        assertFalse(in.isJammed());
    }

    @Test
    public void staysAtSpeedOnALowBattery() {
        // No voltage compensation in the intake: check the loop still gets there at 11.5 V.
        Sim.SimMotor m = motor();
        m.batteryVolts = 11.5;
        Intake in = intake(m);
        in.in();
        Sim.run(1.0, in::update, m);
        System.out.printf("Intake on 11.5 V: %.0f rpm, power %.2f%n", in.getRpm(), in.getPower());
        assertEquals(Intake.RPM_IN, in.getRpm(), 30);
    }
}
