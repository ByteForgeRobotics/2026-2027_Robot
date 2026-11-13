package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * Laptop tests for {@link ShotSolver}. Run with:
 * <pre>./gradlew :TeamCode:testDebugUnitTest</pre>
 *
 * <p>The main check flies every answer through a separate, much finer simulation (time steps
 * of 10 microseconds, nothing shared with the solver) and makes sure the ball really arrives at
 * the target.
 */
public class ShotSolverTest {

    private static final double G = 9.80665;

    // ShotSolver settings are static (so Dashboard can edit them); save and restore them so one
    // test can't change another.
    private double airDensity, slip, exitHeight, angleMin, angleMax, rpmMin, rpmMax, mass;

    @Before
    public void save() {
        airDensity = ShotSolver.AIR_DENSITY;
        slip = ShotSolver.SLIP;
        exitHeight = ShotSolver.EXIT_HEIGHT_M;
        angleMin = ShotSolver.ANGLE_MIN_DEG;
        angleMax = ShotSolver.ANGLE_MAX_DEG;
        rpmMin = ShotSolver.RPM_MIN;
        rpmMax = ShotSolver.RPM_MAX;
        mass = ShotSolver.BALL_MASS_KG;
    }

    @After
    public void restore() {
        ShotSolver.AIR_DENSITY = airDensity;
        ShotSolver.SLIP = slip;
        ShotSolver.EXIT_HEIGHT_M = exitHeight;
        ShotSolver.ANGLE_MIN_DEG = angleMin;
        ShotSolver.ANGLE_MAX_DEG = angleMax;
        ShotSolver.RPM_MIN = rpmMin;
        ShotSolver.RPM_MAX = rpmMax;
        ShotSolver.BALL_MASS_KG = mass;
    }

    /** Height of the ball when it reaches horizontal distance x, from a fine time-step sim. */
    private static double independentHeight(double x, double angleDeg, double speed) {
        double area = Math.PI * ShotSolver.BALL_DIAMETER_M * ShotSolver.BALL_DIAMETER_M / 4;
        double k = 0.5 * ShotSolver.AIR_DENSITY * ShotSolver.DRAG_COEFF * area / ShotSolver.BALL_MASS_KG;
        double a = Math.toRadians(angleDeg);
        double px = 0, py = ShotSolver.EXIT_HEIGHT_M;
        double vx = speed * Math.cos(a), vy = speed * Math.sin(a);
        double dt = 1e-5;
        while (true) {
            double v = Math.hypot(vx, vy);
            double nx = px + vx * dt, ny = py + vy * dt;
            if (nx >= x) {
                return py + (x - px) / (nx - px) * (ny - py);
            }
            px = nx;
            py = ny;
            vx += -k * v * vx * dt;
            vy += (-G - k * v * vy) * dt;
            if (vx <= 0) {
                return Double.NaN;
            }
        }
    }

    @Test
    public void everyReachableShotHitsTheTarget() {
        int checked = 0;
        for (double h = 0.5; h <= 1.51; h += 0.25) {
            for (double d = 0.5; d <= 4.51; d += 0.25) {
                ShotSolver.Shot s = ShotSolver.solve(d, h);
                if (!s.reachable) continue;
                double miss = independentHeight(d, s.angleDeg, s.exitSpeedMps) - h;
                assertTrue(String.format("d=%.2f h=%.2f missed by %.1f mm (%s)", d, h, miss * 1000, s),
                        Math.abs(miss) < 0.01);
                checked++;
            }
        }
        assertTrue("expected most of the grid to be reachable, only " + checked, checked > 60);
    }

    @Test
    public void withNoDragMatchesTheExactFormula() {
        ShotSolver.AIR_DENSITY = 0;
        for (double d = 1; d <= 4; d += 0.5) {
            ShotSolver.Shot s = ShotSolver.solve(d, 1.0);
            assertTrue(s.reachable);
            double t = Math.toRadians(s.angleDeg), u = s.exitSpeedMps;
            double y = ShotSolver.EXIT_HEIGHT_M + d * Math.tan(t)
                    - G * d * d / (2 * u * u * Math.cos(t) * Math.cos(t));
            assertEquals("d=" + d, 1.0, y, 0.001);
        }
    }

    @Test
    public void reachableShotsStayInsideTheHoodAndRpmLimits() {
        for (double h = 0.5; h <= 1.51; h += 0.25) {
            for (double d = 0.3; d <= 6.01; d += 0.1) {
                ShotSolver.Shot s = ShotSolver.solve(d, h);
                if (!s.reachable) continue;
                String at = String.format("d=%.1f h=%.2f %s", d, h, s);
                assertTrue(at, s.angleDeg >= ShotSolver.ANGLE_MIN_DEG - 1e-9);
                assertTrue(at, s.angleDeg <= ShotSolver.ANGLE_MAX_DEG + 1e-9);
                assertTrue(at, s.rpm >= ShotSolver.RPM_MIN - 1e-6);
                assertTrue(at, s.rpm <= ShotSolver.RPM_MAX + 1e-6);
                assertFalse(at, Double.isNaN(s.flightTimeS));
            }
        }
    }

    @Test
    public void picksTheSlowestShotThatWorks() {
        // Brute force every 0.1 deg; the solver's pick should be within 1% of the true minimum.
        double d = 2.5, h = 1.0;
        ShotSolver.Shot s = ShotSolver.solve(d, h);
        double best = Double.MAX_VALUE;
        for (double deg = ShotSolver.ANGLE_MIN_DEG; deg <= ShotSolver.ANGLE_MAX_DEG; deg += 0.1) {
            double u = ShotSolver.speedForAngle(d, h, Math.toRadians(deg));
            if (!Double.isNaN(u)) best = Math.min(best, u);
        }
        assertEquals(best, s.exitSpeedMps, best * 0.01);
    }

    @Test
    public void fartherShotsNeedMoreRpm() {
        double last = 0;
        for (double d = 0.8; d <= 4.5; d += 0.1) {
            ShotSolver.Shot s = ShotSolver.solve(d, 1.0);
            assertTrue("d=" + d, s.reachable);
            assertTrue(String.format("d=%.1f rpm %.0f not above %.0f", d, s.rpm, last), s.rpm > last);
            last = s.rpm;
        }
    }

    @Test
    public void ballComesDownIntoTheGoal() {
        // Minimum-speed shots should arrive falling, not still rising.
        for (double d = 1; d <= 4; d += 0.5) {
            ShotSolver.Shot s = ShotSolver.solve(d, 1.0);
            assertTrue("d=" + d + " arrives at " + s.arrivalAngleDeg, s.arrivalAngleDeg < 0);
        }
    }

    @Test
    public void impossibleShotsAreMarkedOutOfRange() {
        assertFalse("20 m away", ShotSolver.solve(20, 1.0).reachable);
        assertFalse("3 m high from 10 cm away", ShotSolver.solve(0.1, 3.0).reachable);
        ShotSolver.RPM_MAX = 1500;
        assertFalse("4 m with the flywheel capped at 1500 rpm", ShotSolver.solve(4, 1.0).reachable);
    }

    @Test
    public void rpmAndSpeedConvertBothWays() {
        // 6000 rpm on a 38 mm wheel: surface 23.9 m/s, ball gets half.
        assertEquals(11.938, ShotSolver.speedForRpm(6000), 0.001);
        for (double rpm = 500; rpm <= 6000; rpm += 500) {
            assertEquals(rpm, ShotSolver.rpmForSpeed(ShotSolver.speedForRpm(rpm)), 1e-6);
        }
    }

    @Test
    public void lowerSlipAsksForMoreRpm() {
        double full = ShotSolver.solve(2.0, 1.0).rpm;
        ShotSolver.SLIP = 0.8;
        ShotSolver.Shot slipping = ShotSolver.solve(2.0, 1.0);
        assertEquals(full / 0.8, slipping.rpm, full * 0.01);
    }

    @Test
    public void heavierBallIsLessAffectedByDrag() {
        // Same shot, heavier ball -> drag matters less -> needs a little less speed.
        double light = ShotSolver.solve(3.0, 1.0).exitSpeedMps;
        ShotSolver.BALL_MASS_KG = 0.2;
        double heavy = ShotSolver.solve(3.0, 1.0).exitSpeedMps;
        assertTrue(heavy < light);
    }

    @Test
    public void fastEnoughToRunEveryLoop() {
        for (int i = 0; i < 200; i++) ShotSolver.solve(2.0, 1.0); // warm up the JIT
        int n = 500;
        long t0 = System.nanoTime();
        for (int i = 0; i < n; i++) ShotSolver.solve(1 + i * 0.005, 1.0);
        double ms = (System.nanoTime() - t0) / 1e6 / n;
        assertTrue(String.format("%.2f ms per solve", ms), ms < 2.0);
    }

    @Test
    public void hoodTicksPerDegreeMatchesTheGearing() {
        // Maverick 61:1 = 1464 ticks per output turn, 32:180 sector.
        assertEquals(22.875, Hood.TICKS_PER_DEG, 1e-9);
    }
}
