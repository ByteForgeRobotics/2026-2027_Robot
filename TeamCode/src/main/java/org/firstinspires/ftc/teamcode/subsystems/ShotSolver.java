package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

/**
 * Works out the hood angle and flywheel RPM to hit a target, with air drag.
 *
 * <p>Java port of the team's "BallisticSolver" Eigenslate graph:
 * <ol>
 *   <li>Ball speed off the shooter = 1/2 x SLIP x flywheel surface speed (ball rolls between a
 *   spinning wheel and a still hood).</li>
 *   <li>The flight is stepped forward in 20 small pieces of horizontal distance with gravity and
 *   quadratic drag (sphere, Cd 0.47).</li>
 *   <li>For each of 17 launch angles between ANGLE_MIN and ANGLE_MAX it finds the speed that
 *   lands on the target (Newton's method, 5 rounds), keeps the slowest one that's inside the RPM
 *   range, then searches 11 more angles around the best one.</li>
 * </ol>
 *
 * <p>All lengths are meters. {@code distanceM} is flat-ground distance from where the ball
 * leaves the hood to the target. {@code targetHeightM} is the target's height above the floor.
 * One solve is a few thousand cheap math steps, so it's fine to call every loop.
 *
 * <p>Plain Java, no hardware, so it can be tested on a laptop.
 */
@Config
public class ShotSolver {

    // ---- Game piece (AndyMark am-5851 pollen) ----
    public static double BALL_DIAMETER_M = 0.07112;
    /**
     * PLACEHOLDER: weigh a real one. 0.022 kg is what the CAD shell volume (22 cm^3) weighs in
     * a typical plastic. The CAD's own 175 g is wrong (it's set to steel).
     */
    public static double BALL_MASS_KG = 0.022;
    public static double DRAG_COEFF = 0.47;
    public static double AIR_DENSITY = 1.204;

    // ---- Shooter (from mainTestModel CAD) ----
    /** Height of the ball's center above the floor as it leaves the hood. Tape-measure this. */
    public static double EXIT_HEIGHT_M = 0.36;
    public static double FLYWHEEL_RADIUS_M = 0.038;
    /** Fudge for grip. 1.0 = perfect rolling. Lower it if real shots fall short of the math. */
    public static double SLIP = 1.0;

    /** Launch angles the hood can reach (deg above horizontal). CAD allows ~43-80. */
    public static double ANGLE_MIN_DEG = 45;
    public static double ANGLE_MAX_DEG = 78;
    /** Flywheel RPM the solver may pick. Free speed is 6000; leave headroom. */
    public static double RPM_MIN = 1000;
    public static double RPM_MAX = 5400;

    private static final double G = 9.80665;
    private static final int STEPS = 20;
    private static final double MIN_VX = 0.5;

    /** One answer. If {@link #reachable} is false the other numbers are the closest try. */
    public static class Shot {
        public final double angleDeg;
        public final double rpm;
        public final double exitSpeedMps;
        public final boolean reachable;
        public final double flightTimeS;
        /** Angle the ball is travelling at when it arrives; negative = coming down. */
        public final double arrivalAngleDeg;

        Shot(double angleDeg, double rpm, double exitSpeedMps, boolean reachable,
             double flightTimeS, double arrivalAngleDeg) {
            this.angleDeg = angleDeg;
            this.rpm = rpm;
            this.exitSpeedMps = exitSpeedMps;
            this.reachable = reachable;
            this.flightTimeS = flightTimeS;
            this.arrivalAngleDeg = arrivalAngleDeg;
        }

        @Override
        public String toString() {
            return String.format("%s angle %.1f deg, %.0f rpm (%.2f m/s), flight %.2f s, arrives at %.0f deg",
                    reachable ? "OK" : "OUT OF RANGE", angleDeg, rpm, exitSpeedMps,
                    flightTimeS, arrivalAngleDeg);
        }
    }

    /** Ball speed (m/s) for a flywheel RPM. */
    public static double speedForRpm(double rpm) {
        return 0.5 * SLIP * rpm * 2 * Math.PI / 60.0 * FLYWHEEL_RADIUS_M;
    }

    /** Flywheel RPM for a ball speed (m/s). */
    public static double rpmForSpeed(double speed) {
        return speed / (0.5 * SLIP * FLYWHEEL_RADIUS_M) * 60.0 / (2 * Math.PI);
    }

    private static double dragK() {
        double area = Math.PI * BALL_DIAMETER_M * BALL_DIAMETER_M / 4.0;
        return 0.5 * AIR_DENSITY * DRAG_COEFF * area / BALL_MASS_KG;
    }

    /**
     * Flies the ball to horizontal distance x. Returns {height, vx, vy, time}, or null if it
     * stalls (stops moving forward) first.
     */
    static double[] fly(double x, double angle, double speed) {
        double k = dragK();
        double[] s = {EXIT_HEIGHT_M, speed * Math.cos(angle), speed * Math.sin(angle), 0};
        double h = x / STEPS;
        double[] mid = new double[4];
        for (int i = 0; i < STEPS; i++) {
            if (s[1] <= MIN_VX) {
                return null;
            }
            double[] d1 = deriv(s, k);
            for (int j = 0; j < 4; j++) mid[j] = s[j] + h / 2 * d1[j];
            if (mid[1] <= MIN_VX) {
                return null;
            }
            double[] d2 = deriv(mid, k);
            for (int j = 0; j < 4; j++) s[j] += h * d2[j];
        }
        return s[1] > MIN_VX ? s : null;
    }

    /** d/dx of {height, vx, vy, time}. */
    private static double[] deriv(double[] s, double k) {
        double vx = s[1], vy = s[2];
        double v = Math.hypot(vx, vy);
        return new double[]{vy / vx, -k * v, (-G - k * v * vy) / vx, 1 / vx};
    }

    private static double heightAt(double x, double angle, double speed) {
        double[] s = fly(x, angle, speed);
        return s == null ? Double.NaN : s[0];
    }

    /** Speed (m/s) that hits the target at this angle, or NaN if none found. */
    static double speedForAngle(double x, double y, double angle) {
        // Start from the no-drag answer.
        double c = Math.cos(angle);
        double denom = 2 * c * c * (x * Math.tan(angle) - y + EXIT_HEIGHT_M);
        if (denom <= 0) {
            return Double.NaN; // angle too flat to ever reach that height
        }
        double u = Math.sqrt(G * x * x / denom);
        for (int i = 0; i < 5; i++) {
            double f = heightAt(x, angle, u) - y;
            double du = Math.max(1e-3, u * 1e-4);
            double slope = (heightAt(x, angle, u + du) - heightAt(x, angle, u - du)) / (2 * du);
            if (Double.isNaN(f) || Double.isNaN(slope) || slope <= 0) {
                // Stalled before the target: try faster.
                u *= 1.3;
                continue;
            }
            u -= f / slope;
            if (u <= 0) {
                return Double.NaN;
            }
        }
        double err = heightAt(x, angle, u) - y;
        return Double.isNaN(err) || Math.abs(err) > 0.01 ? Double.NaN : u;
    }

    /** Lower is better: the speed if it's in range, a big penalty otherwise. */
    private static double cost(double speed) {
        if (Double.isNaN(speed)) return 1e9;
        double lo = speedForRpm(RPM_MIN), hi = speedForRpm(RPM_MAX);
        if (speed < lo) return 1000 + lo - speed;
        if (speed > hi) return 2000 + speed;
        return speed;
    }

    /** Best shot at a target {@code distanceM} away and {@code targetHeightM} off the floor. */
    public static Shot solve(double distanceM, double targetHeightM) {
        double aLo = Math.toRadians(ANGLE_MIN_DEG), aHi = Math.toRadians(ANGLE_MAX_DEG);
        double step = (aHi - aLo) / 16;

        double bestA = aLo, bestCost = Double.MAX_VALUE;
        for (int j = 0; j <= 16; j++) {
            double a = aLo + j * step;
            double c = cost(speedForAngle(distanceM, targetHeightM, a));
            if (c < bestCost) { bestCost = c; bestA = a; }
        }
        double center = bestA;
        for (int j = 0; j <= 10; j++) {
            double a = Math.max(aLo, Math.min(aHi, center - step + j * step / 5));
            double c = cost(speedForAngle(distanceM, targetHeightM, a));
            if (c < bestCost) { bestCost = c; bestA = a; }
        }

        double u = speedForAngle(distanceM, targetHeightM, bestA);
        boolean ok = bestCost < 500;
        double t = Double.NaN, arrive = Double.NaN;
        if (!Double.isNaN(u)) {
            double[] s = fly(distanceM, bestA, u);
            if (s != null) {
                t = s[3];
                arrive = Math.toDegrees(Math.atan2(s[2], s[1]));
            }
        }
        return new Shot(Math.toDegrees(bestA), Double.isNaN(u) ? Double.NaN : rpmForSpeed(u),
                u, ok, t, arrive);
    }
}
