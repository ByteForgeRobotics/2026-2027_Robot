package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The INIT menu for alliance and start spot. Any OpMode (TeleOp or auto) can use it: call
 * {@link #update} every INIT loop, then {@link #startPose()} once START is pressed.
 *
 * <h2>Buttons (gamepad 1, during INIT)</h2>
 * <ul>
 *   <li>B: RED alliance. X: BLUE alliance (button colors match).</li>
 *   <li>Dpad left / right: AUDIENCE or FAR start spot.</li>
 *   <li>Y: use the pose auto left behind, or ignore it (only shown when auto saved one).</li>
 * </ul>
 *
 * <p>Starts on whatever was picked last, so if auto was set up correctly TeleOp is already
 * right and the drivers just press START.
 */
public class MatchSetup {

    /** Auto's saved pose counts if TeleOp INIT happens within this long after it. */
    public static double HANDOFF_MAX_AGE_S = 120;

    private Field.Alliance alliance = Field.lastAlliance;
    private Field.StartSpot spot = Field.lastStartSpot;
    private boolean useSaved = true;

    public void update(Gamepad gp, Telemetry telemetry) {
        if (gp.bWasPressed()) alliance = Field.Alliance.RED;
        if (gp.xWasPressed()) alliance = Field.Alliance.BLUE;
        if (gp.dpadLeftWasPressed() || gp.dpadRightWasPressed()) {
            spot = spot == Field.StartSpot.AUDIENCE ? Field.StartSpot.FAR : Field.StartSpot.AUDIENCE;
        }
        Field.Pose saved = Field.savedPose(HANDOFF_MAX_AGE_S);
        if (saved != null && gp.yWasPressed()) useSaved = !useSaved;

        Field.lastAlliance = alliance;
        Field.lastStartSpot = spot;

        telemetry.addLine(String.format("ALLIANCE: %s   (B red, X blue)", alliance));
        if (saved != null && useSaved) {
            telemetry.addLine("START: continuing from auto at " + saved + "   (Y to ignore)");
        } else {
            telemetry.addLine(String.format("START: %s side %s   (dpad to change)%s",
                    spot, Field.startPose(alliance, spot),
                    saved != null ? "   (Y: use auto's pose)" : ""));
        }
    }

    public Field.Alliance getAlliance() {
        return alliance;
    }

    public Field.StartSpot getStartSpot() {
        return spot;
    }

    /** Where the robot is at START: auto's saved pose if picked, else the chosen start spot. */
    public Field.Pose startPose() {
        Field.Pose saved = Field.savedPose(HANDOFF_MAX_AGE_S);
        return saved != null && useSaved ? saved : Field.startPose(alliance, spot);
    }

    /** True if {@link #startPose()} is auto's pose rather than a start spot. */
    public boolean usingSavedPose() {
        return useSaved && Field.savedPose(HANDOFF_MAX_AGE_S) != null;
    }
}
