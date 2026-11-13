package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.sim.Sim;
import org.junit.Test;

import java.lang.reflect.Proxy;

/** Battery readouts and the sag / brownout warnings, from a fake voltage sensor. */
public class PowerMonitorTest {

    private final double[] volts = {12.8};

    private PowerMonitor monitor() {
        Sim.FakeHardwareMap hm = new Sim.FakeHardwareMap();
        VoltageSensor v = (VoltageSensor) Proxy.newProxyInstance(getClass().getClassLoader(),
                new Class<?>[]{VoltageSensor.class}, (p, m, a) ->
                        m.getName().equals("getVoltage") ? volts[0] : Sim.defaultFor(m));
        hm.voltageSensor.put("Control Hub", v);
        return new PowerMonitor(hm);
    }

    @Test
    public void noSensorSaysSo() {
        PowerMonitor p = new PowerMonitor(new Sim.FakeHardwareMap());
        assertFalse(p.isAvailable());
        p.update();
        assertEquals("NO SENSOR", p.getStatus());
    }

    @Test
    public void readsTheBatteryAndCallsItOk() {
        PowerMonitor p = monitor();
        assertTrue(p.update());
        assertEquals(12.8, p.getVolts(), 1e-9);
        assertEquals("OK", p.getStatus());
        assertFalse(p.hasCurrentSource());
    }

    @Test
    public void rateLimitsReads() {
        PowerMonitor p = monitor();
        assertTrue(p.update());
        assertFalse("second read inside 100 ms", p.update());
        Sim.sleep(PowerMonitor.READ_PERIOD_MS + 10);
        assertTrue(p.update());
    }

    @Test
    public void remembersTheWorstDipAndWarns() {
        PowerMonitor p = monitor();
        p.update();
        volts[0] = 10.5;
        Sim.sleep(110);
        p.update();
        volts[0] = 12.6;
        Sim.sleep(110);
        p.update();
        assertEquals(10.5, p.getMinVolts(), 1e-9);
        assertEquals("SAG", p.getStatus());
        assertFalse("now back up", p.isSagging());
        volts[0] = 8.5;
        Sim.sleep(110);
        p.update();
        assertEquals("BROWNOUT", p.getStatus());
        p.resetPeaks();
        volts[0] = 12.6;
        Sim.sleep(110);
        p.update();
        assertEquals("OK", p.getStatus());
    }

    @Test
    public void zeroVoltsIsIgnoredNotALowReading() {
        volts[0] = 0;
        PowerMonitor p = monitor();
        p.update();
        assertEquals("no reading", p.getStatus());
    }
}
