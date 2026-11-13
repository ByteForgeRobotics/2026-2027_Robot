package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Test;

/** Laptop tests for {@link Field}: alliance mirroring and the auto -> TeleOp hand-off. */
public class FieldTest {

    private static final double EPS = 1e-9;

    @After
    public void restore() {
        Field.MIRROR = Field.Mirror.ROTATE;
        Field.clearSavedPose();
    }

    @Test
    public void redIsUnchanged() {
        Field.Pose p = Field.Pose.deg(10, -20, 30);
        assertEquals(p, Field.forAlliance(p, Field.Alliance.RED));
    }

    @Test
    public void reflectFlipsYAndHeading() {
        Field.MIRROR = Field.Mirror.REFLECT;
        Field.Pose blue = Field.forAlliance(Field.Pose.deg(36, -63, 90), Field.Alliance.BLUE);
        assertEquals(36, blue.x, EPS);
        assertEquals(63, blue.y, EPS);
        assertEquals(-90, Math.toDegrees(blue.heading), EPS);
    }

    @Test
    public void rotateTurnsAboutTheCenter() {
        Field.MIRROR = Field.Mirror.ROTATE;
        Field.Pose blue = Field.forAlliance(Field.Pose.deg(36, -63, 90), Field.Alliance.BLUE);
        assertEquals(-36, blue.x, EPS);
        assertEquals(63, blue.y, EPS);
        assertEquals(-90, Math.toDegrees(blue.heading), EPS);
    }

    @Test
    public void mirroringTwiceGetsBackToRed() {
        for (Field.Mirror m : Field.Mirror.values()) {
            Field.MIRROR = m;
            Field.Pose red = Field.Pose.deg(-12.5, 40, 135);
            Field.Pose back = Field.forAlliance(Field.forAlliance(red, Field.Alliance.BLUE), Field.Alliance.BLUE);
            assertEquals(m + " x", red.x, back.x, EPS);
            assertEquals(m + " y", red.y, back.y, EPS);
            assertEquals(m + " heading", 0, Field.wrap(red.heading - back.heading), EPS);
        }
    }

    @Test
    public void blueTwinSpotSeesItsGoalLikeRedDoes() {
        // BIOBUZZ is point-symmetric: red's AUDIENCE spot and blue's FAR spot are twins, and
        // each faces its own starting up cell from the same distance and bearing.
        for (Field.StartSpot spot : Field.StartSpot.values()) {
            Field.StartSpot twin = spot == Field.StartSpot.AUDIENCE ? Field.StartSpot.FAR : Field.StartSpot.AUDIENCE;
            double[] red = view(Field.startPose(Field.Alliance.RED, spot), Field.goal(Field.Alliance.RED));
            double[] blue = view(Field.startPose(Field.Alliance.BLUE, twin), Field.goal(Field.Alliance.BLUE));
            assertEquals(spot + " distance", red[0], blue[0], 1e-6);
            assertEquals(spot + " bearing", red[1], blue[1], 1e-6);
        }
    }

    @Test
    public void startSpotLabelsMatchTheSideOfTheField() {
        // +x is the audience side for both alliances.
        for (Field.Alliance a : Field.Alliance.values()) {
            assertTrue(a + " AUDIENCE", Field.startPose(a, Field.StartSpot.AUDIENCE).x > 0);
            assertTrue(a + " FAR", Field.startPose(a, Field.StartSpot.FAR).x < 0);
        }
    }

    @Test
    public void cellsSitOnEachAlliancesHive() {
        for (Field.Cell c : Field.Cell.values()) {
            Field.Pose red = Field.goal(Field.Alliance.RED, c);
            Field.Pose blue = Field.goal(Field.Alliance.BLUE, c);
            assertTrue(red.y < 0);
            assertTrue(blue.y > 0);
            assertEquals("same cell, same side", red.x, blue.x, EPS);
            // the opening faces the wall it's named after
            assertEquals(c == Field.Cell.AUDIENCE ? 1 : -1, Math.signum(red.x), 0);
            assertEquals(Math.cos(red.heading), Math.signum(red.x), EPS);
        }
        assertEquals(Field.Cell.FAR, Field.Cell.AUDIENCE.other());
        assertEquals(Field.Cell.AUDIENCE, Field.Cell.FAR.other());
    }

    /** {distance, bearing relative to the robot's heading} from robot to goal. */
    private static double[] view(Field.Pose robot, Field.Pose goal) {
        double dx = goal.x - robot.x, dy = goal.y - robot.y;
        return new double[]{Math.hypot(dx, dy), Field.wrap(Math.atan2(dy, dx) - robot.heading)};
    }

    @Test
    public void savedPoseIsHandedOffOnlyWhileFresh() {
        assertNull(Field.savedPose(120));
        Field.savePose(Field.Pose.deg(1, 2, 3));
        assertNotNull(Field.savedPose(120));
        assertEquals(1, Field.savedPose(120).x, EPS);
        assertNull("a negative max age means it's already too old", Field.savedPose(-1));
        Field.clearSavedPose();
        assertNull(Field.savedPose(120));
    }

    @Test
    public void wrapKeepsAnglesInRange() {
        assertEquals(Math.PI / 2, Field.wrap(Math.PI / 2 + 4 * Math.PI), 1e-9);
        assertEquals(-Math.PI / 2, Field.wrap(3 * Math.PI / 2), 1e-9);
        assertEquals(0, Field.wrap(0), 0);
    }
}
