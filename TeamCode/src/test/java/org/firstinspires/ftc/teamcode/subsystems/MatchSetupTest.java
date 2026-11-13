package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.sim.Sim;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/** The INIT menu: alliance, start spot, and using auto's saved pose. */
public class MatchSetupTest {

    private final Gamepad gp = new Gamepad();
    private final Sim.FakeTelemetry tel = new Sim.FakeTelemetry();

    @Before
    @After
    public void reset() {
        Field.lastAlliance = Field.Alliance.RED;
        Field.lastStartSpot = Field.StartSpot.AUDIENCE;
        Field.clearSavedPose();
    }

    private void tap(MatchSetup s, java.util.function.Consumer<Gamepad> button) {
        Sim.press(gp, button);
        s.update(gp, tel.telemetry);
        Sim.release(gp);
        s.update(gp, tel.telemetry);
    }

    @Test
    public void defaultsToWhateverWasPickedLast() {
        Field.lastAlliance = Field.Alliance.BLUE;
        Field.lastStartSpot = Field.StartSpot.FAR;
        MatchSetup s = new MatchSetup();
        assertEquals(Field.Alliance.BLUE, s.getAlliance());
        assertEquals(Field.StartSpot.FAR, s.getStartSpot());
    }

    @Test
    public void buttonsPickAllianceAndSpot() {
        MatchSetup s = new MatchSetup();
        tap(s, g -> g.x = true);
        assertEquals(Field.Alliance.BLUE, s.getAlliance());
        tap(s, g -> g.b = true);
        assertEquals(Field.Alliance.RED, s.getAlliance());
        tap(s, g -> g.dpad_right = true);
        assertEquals(Field.StartSpot.FAR, s.getStartSpot());
        tap(s, g -> g.dpad_left = true);
        assertEquals(Field.StartSpot.AUDIENCE, s.getStartSpot());
        assertEquals("choice is remembered for the next OpMode", Field.StartSpot.AUDIENCE, Field.lastStartSpot);
    }

    @Test
    public void holdingAButtonOnlyCountsOnce() {
        MatchSetup s = new MatchSetup();
        Sim.press(gp, g -> g.dpad_right = true);
        for (int i = 0; i < 5; i++) s.update(gp, tel.telemetry);
        assertEquals(Field.StartSpot.FAR, s.getStartSpot());
    }

    @Test
    public void startPoseIsTheChosenSpot() {
        MatchSetup s = new MatchSetup();
        tap(s, g -> g.x = true);
        Field.Pose p = s.startPose();
        Field.Pose expected = Field.startPose(Field.Alliance.BLUE, Field.StartSpot.AUDIENCE);
        assertEquals(expected.x, p.x, 1e-9);
        assertEquals(expected.y, p.y, 1e-9);
        assertFalse(s.usingSavedPose());
    }

    @Test
    public void autosSavedPoseIsUsedUnlessIgnored() {
        Field.savePose(Field.Pose.deg(5, -30, 45));
        MatchSetup s = new MatchSetup();
        s.update(gp, tel.telemetry);
        assertTrue(s.usingSavedPose());
        assertEquals(5, s.startPose().x, 1e-9);
        assertTrue(tel.all().contains("continuing from auto"));
        tap(s, g -> g.y = true);
        assertFalse(s.usingSavedPose());
        assertEquals(Field.startPose(Field.Alliance.RED, Field.StartSpot.AUDIENCE).x, s.startPose().x, 1e-9);
    }

    @Test
    public void telemetryShowsTheChoice() {
        MatchSetup s = new MatchSetup();
        s.update(gp, tel.telemetry);
        assertTrue(tel.all(), tel.all().contains("ALLIANCE: RED"));
        assertTrue(tel.all(), tel.all().contains("AUDIENCE"));
    }
}
