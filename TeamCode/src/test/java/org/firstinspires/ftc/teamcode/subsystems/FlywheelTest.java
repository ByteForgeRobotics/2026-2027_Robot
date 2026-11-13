package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.sim.Sim;
import org.junit.Test;

/**
 * Flywheel on a simulated goBILDA 6000 rpm motor (28 ticks/rev). The sim motor reaches 94% of
 * free speed at full power (a loaded flywheel never reaches the no-load number), with a 0.35 s
 * spin-up time constant.
 */
public class FlywheelTest {

    static Sim.SimMotor motor(double volts) {
        Sim.SimMotor m = new Sim.SimMotor(28, 6000 * 0.94, 0.35);
        m.batteryVolts = volts;
        return m;
    }

    static Flywheel flywheel(Sim.SimMotor m, double volts) {
        return new Flywheel(new Sim.FakeHardwareMap().add(Flywheel.NAME, m.motor).battery(volts));
    }

    @Test
    public void missingMotorDoesNotCrash() {
        Flywheel f = new Flywheel(new Sim.FakeHardwareMap());
        assertFalse(f.isConnected());
        f.setRpm(3000);
        f.update();
        assertFalse(f.atSpeed());
    }

    @Test
    public void setsUpTheMotorToCoast() {
        Sim.SimMotor m = motor(12);
        flywheel(m, 12);
        assertEquals(DcMotor.ZeroPowerBehavior.FLOAT, m.zeroPower);
        assertEquals(DcMotor.RunMode.RUN_WITHOUT_ENCODER, m.mode);
    }

    @Test
    public void reachesSpeedQuicklyAndHoldsIt() {
        Sim.SimMotor m = motor(12.6);
        Flywheel f = flywheel(m, 12.6);
        f.setRpm(Flywheel.RPM_FAR);
        long t0 = System.nanoTime();
        double[] firstReady = {-1};
        Sim.run(2.5, () -> {
            f.update();
            if (firstReady[0] < 0 && f.atSpeed()) firstReady[0] = (System.nanoTime() - t0) / 1e9;
        }, m);
        System.out.printf("Flywheel: %.0f rpm target, ready after %.2f s, settled at %.0f rpm%n",
                Flywheel.RPM_FAR, firstReady[0], f.getRpm());
        assertTrue("never reached speed", firstReady[0] > 0);
        assertTrue("took " + firstReady[0] + " s", firstReady[0] < 1.5);
        assertTrue("settled at " + f.getRpm(), f.atSpeed());
    }

    @Test
    public void sameSpeedOnAFullAndATiredBattery() {
        double[] settled = new double[2];
        double[] volts = {13.2, 11.5};
        for (int i = 0; i < 2; i++) {
            Sim.SimMotor m = motor(volts[i]);
            Flywheel f = flywheel(m, volts[i]);
            f.setRpm(3000);
            Sim.run(2.0, f::update, m);
            settled[i] = f.getRpm();
            assertTrue(volts[i] + " V settled at " + settled[i], f.atSpeed());
        }
        System.out.printf("Flywheel at 3000: %.0f rpm on 13.2 V, %.0f rpm on 11.5 V%n", settled[0], settled[1]);
    }

    @Test
    public void steadyStateErrorAcrossMotorEfficiency() {
        // The loop is feedforward + P only, so it always settles a bit off target. How far
        // depends on how close KF is to the real motor. Sweep how much of its free speed the
        // motor really makes and check atSpeed() still comes true.
        StringBuilder report = new StringBuilder("Flywheel steady error at 3000 rpm by motor efficiency:");
        boolean allOk = true;
        for (double eff = 0.80; eff <= 1.001; eff += 0.05) {
            Sim.SimMotor m = new Sim.SimMotor(28, 6000 * eff, 0.35);
            Flywheel f = flywheel(m, 12);
            f.setRpm(3000);
            Sim.run(1.8, f::update, m);
            double err = f.getRpm() - 3000;
            report.append(String.format("  %.0f%%: %+.0f rpm%s", eff * 100, err, f.atSpeed() ? "" : " (NOT atSpeed)"));
            if (eff >= 0.85 && eff <= 0.95 && !f.atSpeed()) allOk = false;
        }
        System.out.println(report);
        assertTrue(report.toString(), allOk);
    }

    @Test
    public void recoversAfterAShot() {
        Sim.SimMotor m = motor(12.6);
        Flywheel f = flywheel(m, 12.6);
        f.setRpm(3200);
        Sim.run(2.0, f::update, m);
        assertTrue(f.atSpeed());
        m.hit(0.15); // ball steals 15% of the speed
        f.update();
        assertFalse("should notice the dip", f.atSpeed());
        long t0 = System.nanoTime();
        double[] back = {-1};
        Sim.run(1.5, () -> {
            f.update();
            if (back[0] < 0 && f.atSpeed()) back[0] = (System.nanoTime() - t0) / 1e9;
        }, m);
        System.out.printf("Flywheel recovery after a 15%% dip at 3200 rpm: %.2f s%n", back[0]);
        assertTrue("never recovered", back[0] > 0);
        assertTrue("recovery " + back[0] + " s is slow for rapid fire", back[0] < 0.8);
    }

    @Test
    public void neverDrivesBackwardsAndCoastsWhenStopped() {
        Sim.SimMotor m = motor(12);
        Flywheel f = flywheel(m, 12);
        f.setRpm(4000);
        Sim.run(1.0, f::update, m);
        f.setRpm(1000); // big drop: must ease off, not reverse
        Sim.run(0.5, f::update, m);
        f.stop();
        f.update();
        for (double p : m.powerLog) assertTrue("negative power " + p, p >= 0);
        assertEquals(0, m.power, 0);
    }

    @Test
    public void negativeTargetIsTreatedAsStop() {
        Sim.SimMotor m = motor(12);
        Flywheel f = flywheel(m, 12);
        f.setRpm(-500);
        assertEquals(0, f.getTargetRpm(), 0);
    }

    @Test
    public void targetAboveWhatTheMotorCanDoNeverReportsReady() {
        Sim.SimMotor m = motor(12);
        Flywheel f = flywheel(m, 12);
        f.setRpm(7000);
        Sim.run(1.5, f::update, m);
        assertEquals(1.0, f.getPower(), 1e-9);
        assertFalse(f.atSpeed());
    }
}
