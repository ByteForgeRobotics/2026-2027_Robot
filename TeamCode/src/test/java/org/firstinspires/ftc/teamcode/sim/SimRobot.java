package org.firstinspires.ftc.teamcode.sim;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;

/**
 * A mecanum robot on a flat field. Reads the four wheel powers the {@link Drivetrain} sent,
 * turns them into robot motion (top speed 50 in/s, strafing 1/1.1 as fast, 0.12 s lag), and
 * reports the pose through a fake {@link Odometry} the way the Pinpoint would.
 */
public class SimRobot {
    public static final double VMAX_IN_S = 50;
    public static final double STRAFE_EFF = 1 / 1.1;
    public static final double TURN_RADIUS_IN = 11;
    public static final double LAG_S = 0.12;

    public final Sim.SimMotor[] wheels = new Sim.SimMotor[4];
    public final Drivetrain drive;
    public final FakeOdometry odometry;
    public double x, y, heading;          // field frame, inches / radians
    double vF, vL, w;                     // robot frame

    public SimRobot(double x, double y, double heading) {
        Sim.FakeHardwareMap hm = new Sim.FakeHardwareMap();
        for (int i = 0; i < 4; i++) {
            wheels[i] = new Sim.SimMotor(537.7, 312, 0.05);
            hm.add(Drivetrain.NAMES[i], wheels[i].motor);
        }
        drive = new Drivetrain(hm);
        odometry = new FakeOdometry(hm);
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    /** Commanded (forward, left, ccw) recovered from the four wheel powers. */
    public double[] commanded() {
        double fl = wheels[0].power, fr = wheels[1].power, bl = wheels[2].power, br = wheels[3].power;
        return new double[]{(fl + fr + bl + br) / 4, (-fl + fr + bl - br) / 4, (-fl + fr - bl + br) / 4};
    }

    public void step(double dt) {
        double[] c = commanded();
        double a = Math.min(1, dt / LAG_S);
        vF += (c[0] * VMAX_IN_S - vF) * a;
        vL += (c[1] * VMAX_IN_S * STRAFE_EFF - vL) * a;
        w += (c[2] * VMAX_IN_S / TURN_RADIUS_IN - w) * a;
        double cos = Math.cos(heading), sin = Math.sin(heading);
        x += (vF * cos - vL * sin) * dt;
        y += (vF * sin + vL * cos) * dt;
        heading += w * dt;
        heading = Math.atan2(Math.sin(heading), Math.cos(heading));
    }

    public double fieldVx() {
        return vF * Math.cos(heading) - vL * Math.sin(heading);
    }

    public double fieldVy() {
        return vF * Math.sin(heading) + vL * Math.cos(heading);
    }

    /** Runs {@code loop} (which returns true when done) for up to {@code maxS}; returns seconds used or -1. */
    public double run(double maxS, java.util.function.BooleanSupplier loop) {
        long t0 = System.nanoTime(), last = t0;
        while ((System.nanoTime() - t0) / 1e9 < maxS) {
            long now = System.nanoTime();
            step((now - last) / 1e9);
            last = now;
            if (loop.getAsBoolean()) return (now - t0) / 1e9;
            Sim.sleep(5);
        }
        return -1;
    }

    public class FakeOdometry extends Odometry {
        public boolean ready = true;

        FakeOdometry(Sim.FakeHardwareMap hm) {
            super(hm);
        }

        @Override public void update() {}
        @Override public boolean isReady() { return ready; }
        @Override public String getStatus() { return ready ? "READY" : "NOT FOUND"; }
        @Override public double getX() { return x; }
        @Override public double getY() { return y; }
        @Override public double getHeading() { return heading; }
        @Override public double getVelX() { return fieldVx(); }
        @Override public double getVelY() { return fieldVy(); }
        @Override public void setPose(double xIn, double yIn, double h) { x = xIn; y = yIn; heading = h; }
    }
}
