package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.junit.Test;

/**
 * The geometry inside {@link LimelightController#solve} and {@link HuskyLensController#solve}.
 * Each test places a target at a known spot, works out what the camera would see with a
 * separate pinhole model, and checks the solver gets the spot back.
 */
public class VisionMathTest {

    // ---------- Limelight ----------

    /** {tx, ty} the Limelight would report for a target at robot-frame (x fwd, y left). */
    private static double[] limelightSees(double x, double y) {
        double camF = x - LimelightController.CAMERA_OFFSET_X_IN;
        double camR = -(y - LimelightController.CAMERA_OFFSET_Y_IN);
        double up = LimelightController.TARGET_HEIGHT_IN - LimelightController.CAMERA_HEIGHT_IN;
        double p = Math.toRadians(LimelightController.CAMERA_PITCH_DEG);
        double fOpt = camF * Math.cos(p) + up * Math.sin(p);
        double uOpt = -camF * Math.sin(p) + up * Math.cos(p);
        return new double[]{Math.toDegrees(Math.atan(camR / fOpt)), Math.toDegrees(Math.atan(uOpt / fOpt))};
    }

    @Test
    public void limelightRecoversTargetPosition() {
        for (double x = 24; x <= 120; x += 24) {
            for (double y = -30; y <= 30; y += 15) {
                double[] t = limelightSees(x, y);
                LimelightController.TargetObservation o = LimelightController.solve(t[0], t[1], 0);
                assertNotNull("x=" + x + " y=" + y, o);
                assertEquals("x at " + x + "," + y, x, o.robotRelX, 0.01);
                assertEquals("y at " + x + "," + y, y, o.robotRelY, 0.01);
            }
        }
    }

    @Test
    public void limelightRotatesIntoTheFieldFrame() {
        double[] t = limelightSees(60, 0); // dead ahead
        LimelightController.TargetObservation o = LimelightController.solve(t[0], t[1], 90);
        assertEquals(0, o.fieldRelX, 0.01);   // robot faces +y, so ahead = +y on the field
        assertEquals(60, o.fieldRelY, 0.01);
        double[] robot = o.robotFieldPosition(10, 70);
        assertEquals(10, robot[0], 0.01);
        assertEquals(10, robot[1], 0.01);
    }

    @Test
    public void limelightRejectsImpossibleRays() {
        // A ray pointing well below the horizon can never reach a target above the camera.
        assertNull(LimelightController.solve(0, -60, 0));
    }

    @Test
    public void limelightTargetHeightMatchesTheHiveTags() {
        // Up cell's AprilTag sticker sits ~51 in high (manual Fig 9-9/9-10, vault notes). The
        // plane-intersection solve is only right if TARGET_HEIGHT_IN is the tag's height.
        assertEquals("TARGET_HEIGHT_IN vs the ~51 in up-cell tag height",
                51.0, LimelightController.TARGET_HEIGHT_IN, 1.5);
    }

    // ---------- HuskyLens ----------

    private static HuskyLens.Block block(int id, int x, int y, int w, int h) throws Exception {
        java.lang.reflect.Field uf = Class.forName("sun.misc.Unsafe").getDeclaredField("theUnsafe");
        uf.setAccessible(true);
        Object unsafe = uf.get(null);
        HuskyLens.Block b = (HuskyLens.Block) unsafe.getClass()
                .getMethod("allocateInstance", Class.class).invoke(unsafe, HuskyLens.Block.class);
        set(b, "id", id);
        set(b, "x", x);
        set(b, "y", y);
        set(b, "width", w);
        set(b, "height", h);
        return b;
    }

    private static void set(Object o, String name, int v) throws Exception {
        java.lang.reflect.Field f = HuskyLens.Block.class.getDeclaredField(name);
        f.setAccessible(true);
        f.setInt(o, v);
    }

    /** Pixel {x, y} where a pollen at robot (x fwd, y left) shows up. */
    private static double[] huskySees(double x, double y) {
        double f = HuskyLensController.focalPx();
        double camF = x - HuskyLensController.CAMERA_OFFSET_X_IN;
        double camR = -(y - HuskyLensController.CAMERA_OFFSET_Y_IN);
        double up = HuskyLensController.POLLEN_CENTER_HEIGHT_IN - HuskyLensController.CAMERA_HEIGHT_IN;
        double p = Math.toRadians(HuskyLensController.CAMERA_PITCH_DEG);
        double fOpt = camF * Math.cos(p) + up * Math.sin(p);
        double uOpt = -camF * Math.sin(p) + up * Math.cos(p);
        return new double[]{HuskyLensController.IMAGE_WIDTH_PX / 2.0 + f * camR / fOpt,
                HuskyLensController.IMAGE_HEIGHT_PX / 2.0 - f * uOpt / fOpt};
    }

    @Test
    public void huskyLensRecoversPollenPosition() throws Exception {
        for (double x = 16; x <= 48; x += 8) {
            for (double y = -10; y <= 10; y += 5) {
                double[] px = huskySees(x, y);
                HuskyLensController.PollenObservation o = HuskyLensController.solve(
                        block(1, (int) Math.round(px[0]), (int) Math.round(px[1]), 20, 20));
                assertNotNull("x=" + x + " y=" + y, o);
                // pixels are whole numbers, so allow the rounding
                assertEquals("x at " + x + "," + y, x, o.robotRelX, 0.04 * x);
                assertEquals("y at " + x + "," + y, y, o.robotRelY, 0.6);
            }
        }
    }

    @Test
    public void huskyLensFilterTakesBallsAndRejectsJunk() throws Exception {
        assertTrue(HuskyLensController.looksLikePollen(block(1, 160, 150, 30, 28)));
        assertFalse("wrong color id", HuskyLensController.looksLikePollen(block(2, 160, 150, 30, 28)));
        assertFalse("too small", HuskyLensController.looksLikePollen(block(1, 160, 150, 4, 4)));
        assertFalse("a stripe, not a ball", HuskyLensController.looksLikePollen(block(1, 160, 150, 80, 10)));
        assertFalse("above the horizon (a shirt or banner)", HuskyLensController.looksLikePollen(block(1, 160, 10, 30, 30)));
    }

    @Test
    public void huskyLensSteeringTurnsTowardTheBall() throws Exception {
        double[] left = huskySees(30, 8);
        HuskyLensController.PollenObservation o = HuskyLensController.solve(
                block(1, (int) Math.round(left[0]), (int) Math.round(left[1]), 20, 20));
        double[] s = HuskyLensController.steer(o);
        assertTrue("ball on the left -> CCW turn, got " + s[1], s[1] > 0);
        assertTrue("drives toward it", s[0] > 0);
    }
}
