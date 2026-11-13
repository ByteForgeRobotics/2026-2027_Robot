package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.sim.SimRobot;
import org.junit.Test;

/**
 * {@link DriveToPoint} driving a simulated mecanum robot. Checks it arrives, holds heading and
 * never heads the wrong way, from several start headings (the field-to-robot rotation is the
 * part that's easy to get backwards).
 */
public class DriveToPointTest {

    private static String go(double sx, double sy, double sh, double tx, double ty, double th, double maxS) {
        SimRobot r = new SimRobot(sx, sy, sh);
        DriveToPoint d = new DriveToPoint(r.drive, r.odometry);
        d.setTarget(tx, ty, th);
        double start = Math.hypot(tx - sx, ty - sy);
        double[] worst = {0};
        double t = r.run(maxS, () -> {
            boolean done = d.update();
            worst[0] = Math.max(worst[0], Math.hypot(tx - r.x, ty - r.y) - start);
            return done;
        });
        double posErr = Math.hypot(tx - r.x, ty - r.y);
        double hErr = Math.toDegrees(Math.abs(Math.atan2(Math.sin(th - r.heading), Math.cos(th - r.heading))));
        String s = String.format("(%.0f,%.0f,%.0f deg) -> (%.0f,%.0f,%.0f deg): %s in %.2f s, off by %.2f in / %.1f deg, moved away by up to %.2f in",
                sx, sy, Math.toDegrees(sh), tx, ty, Math.toDegrees(th),
                t < 0 ? "DID NOT ARRIVE" : "arrived", t, posErr, hErr, worst[0]);
        System.out.println("DriveToPoint " + s);
        assertTrue(s, t > 0);
        assertTrue(s, posErr < DriveToPoint.POSITION_TOLERANCE_IN + 0.3);
        assertTrue(s, hErr < DriveToPoint.HEADING_TOLERANCE_DEG + 1);
        assertTrue("went the wrong way: " + s, worst[0] < 0.5);
        return s;
    }

    @Test
    public void leaveMoveFromTheRedStartHeading() {
        // What Comp Auto: Leave does: 12 in straight ahead, robot facing +y.
        go(0, 0, Math.toRadians(90), 0, 12, Math.toRadians(90), 4);
    }

    @Test
    public void straightAheadAtHeadingZero() {
        go(0, 0, 0, 24, 0, 0, 4);
    }

    @Test
    public void strafeSidewaysWhileFacingBackwards() {
        go(0, 0, Math.toRadians(180), 0, 20, Math.toRadians(180), 4);
    }

    @Test
    public void diagonalWithATurn() {
        go(10, -50, Math.toRadians(90), -20, -20, Math.toRadians(0), 6);
    }

    @Test
    public void backingUp() {
        go(0, 0, Math.toRadians(-45), -10, 10, Math.toRadians(-45), 4);
    }
}
