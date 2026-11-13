package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

/**
 * Where things are on the field, for both alliances, plus the pose hand-off from auto to
 * TeleOp.
 *
 * <h2>Field frame (standard FTC)</h2>
 * Inches, origin at the field center. Standing at the red alliance wall: +x is to your right
 * (toward the audience this season), +y is straight out away from you toward the blue wall.
 * Heading is counter-clockwise positive, 0 = facing +x. Red's half is y &lt; 0.
 *
 * <h2>What we shoot at: the HIVE up cell</h2>
 * Both HIVEs sit on one frame at field center (red at y = -12.75, blue at +12.75). Each has an
 * AUDIENCE cell (x &gt; 0) and a FAR cell (x &lt; 0), and only the cell facing UP scores (G417).
 * The up cell's 20 x 14 in opening is on its outward face, 53.5 to 65.6 in off the tiles,
 * facing that cell's own wall. Every HIVE TIP flips which cell is up, so the target moves to
 * the other side of the HIVE (manual 9.6, 10.5.1). At the start red's AUDIENCE cell and blue's
 * FAR cell are up (10.3.1). TeleOp keeps track with a button the operator presses after a tip.
 *
 * <h2>Only measure red start spots</h2>
 * The game is point-symmetric (red's audience cell and blue's far cell start up; the loading
 * zones and gardens sit in opposite corners), so blue's start spots are red's turned 180 deg
 * about the center. That turns red's AUDIENCE spot into a spot on blue's FAR side, so blue's
 * AUDIENCE spot comes from red's FAR one and the menu labels still match where the robot
 * really sits.
 */
@Config
public class Field {

    public enum Alliance { RED, BLUE }

    public enum StartSpot { AUDIENCE, FAR }

    /** A HIVE cell, named by which wall it points at. */
    public enum Cell {
        AUDIENCE, FAR;

        public Cell other() {
            return this == AUDIENCE ? FAR : AUDIENCE;
        }
    }

    public enum Mirror {
        /** Blue = red flipped across the x axis: (x, -y, -heading). Not this season. */
        REFLECT,
        /** Blue = red turned 180 deg about the center: (-x, -y, heading + 180). BIOBUZZ. */
        ROTATE
    }

    public static Mirror MIRROR = Mirror.ROTATE;

    // ---- RED start spots. PLACEHOLDERS: check against the manual's Fig 11-1 / 9-2. ----
    // G304: fully on red's half, touching the wall, not in the LOADING ZONE (far half of the
    // red wall, about x -47.5 to -24.5) and not touching a FLOWER (one is on the audience half
    // of the red wall: make sure the AUDIENCE spot clears it).
    /** Robot center at the red audience-side start, inches / degrees. */
    public static double RED_AUDIENCE_START_X = 36;
    public static double RED_AUDIENCE_START_Y = -63;
    public static double RED_AUDIENCE_START_HEADING_DEG = 90;

    /** Between the loading zone and the center line, so the robot doesn't start in the zone. */
    public static double RED_FAR_START_X = -12;
    public static double RED_FAR_START_Y = -63;
    public static double RED_FAR_START_HEADING_DEG = 90;

    // ---- HIVE cells. From the manual figures and field CAD; check with a tape measure. ----
    /** Each HIVE's distance from the center line (red at -y, blue at +y), inches. */
    public static double HIVE_Y_IN = 12.75;
    /**
     * How far the center of an up cell's opening is from the field center along x, inches.
     * ESTIMATE (42.91 in beam tilted 30 deg, +/- 3 in): measure it on a real HIVE.
     */
    public static double CELL_OPENING_X_IN = 19;
    /** Height of the center of the up cell's opening above the tiles, meters (53.5-65.6 in). */
    public static double CELL_OPENING_HEIGHT_M = 1.51;
    /**
     * Only shoot from within this many degrees of straight in front of the opening. Further
     * round the side, the ball hits the outside of the cell (a G417.C foul).
     */
    public static double CELL_ENTRY_HALF_ANGLE_DEG = 50;

    /** Which cell is up at the start of a match (manual 10.3.1). */
    public static Cell startUpCell(Alliance alliance) {
        return alliance == Alliance.RED ? Cell.AUDIENCE : Cell.FAR;
    }

    /**
     * Center of the opening of this alliance's {@code cell} (only worth shooting at while it's
     * up). Heading is the way the opening faces: toward that cell's wall.
     */
    public static Pose goal(Alliance alliance, Cell cell) {
        double x = cell == Cell.AUDIENCE ? CELL_OPENING_X_IN : -CELL_OPENING_X_IN;
        double y = alliance == Alliance.RED ? -HIVE_Y_IN : HIVE_Y_IN;
        return new Pose(x, y, cell == Cell.AUDIENCE ? 0 : Math.PI);
    }

    /** Goal at the start of a match, before any tip. */
    public static Pose goal(Alliance alliance) {
        return goal(alliance, startUpCell(alliance));
    }

    /**
     * True if a ball launched from (x, y) would come in through the front of the opening rather
     * than hitting the side or back of the cell.
     */
    public static boolean canScoreFrom(Pose goal, double x, double y) {
        double bearing = Math.atan2(y - goal.y, x - goal.x);
        return Math.abs(wrap(bearing - goal.heading)) <= Math.toRadians(CELL_ENTRY_HALF_ANGLE_DEG);
    }

    /** A position and heading. Inches and radians. */
    public static class Pose {
        public final double x, y, heading;

        public Pose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        public static Pose deg(double x, double y, double headingDeg) {
            return new Pose(x, y, Math.toRadians(headingDeg));
        }

        @Override
        public String toString() {
            return String.format("(%.1f, %.1f) %.0f deg", x, y, Math.toDegrees(heading));
        }
    }

    /** Turns a red pose into the same spot for {@code alliance}. */
    public static Pose forAlliance(Pose red, Alliance alliance) {
        if (alliance == Alliance.RED) {
            return red;
        }
        if (MIRROR == Mirror.ROTATE) {
            return new Pose(-red.x, -red.y, wrap(red.heading + Math.PI));
        }
        return new Pose(red.x, -red.y, wrap(-red.heading));
    }

    public static Pose startPose(Alliance alliance, StartSpot spot) {
        // Rotating swaps audience and far, so use red's other spot to land on the named side.
        StartSpot redSpot = alliance == Alliance.BLUE && MIRROR == Mirror.ROTATE
                ? (spot == StartSpot.AUDIENCE ? StartSpot.FAR : StartSpot.AUDIENCE)
                : spot;
        Pose red = redSpot == StartSpot.AUDIENCE
                ? Pose.deg(RED_AUDIENCE_START_X, RED_AUDIENCE_START_Y, RED_AUDIENCE_START_HEADING_DEG)
                : Pose.deg(RED_FAR_START_X, RED_FAR_START_Y, RED_FAR_START_HEADING_DEG);
        return forAlliance(red, alliance);
    }

    // ---------------- Hand-off between OpModes ----------------
    // Static fields live as long as the Robot Controller app, so what auto (or the last INIT
    // menu) stores here is still here when TeleOp starts. A restart clears it.

    /** Last alliance and start spot picked in any INIT menu. TeleOp starts on these. */
    public static Alliance lastAlliance = Alliance.RED;
    public static StartSpot lastStartSpot = StartSpot.AUDIENCE;

    private static Pose savedPose;
    private static long savedAtMs;

    /** Call at the end of auto with where the robot is, in the field frame. */
    public static void savePose(Pose pose) {
        savedPose = pose;
        savedAtMs = System.currentTimeMillis();
    }

    /** The pose auto saved, if it's younger than {@code maxAgeS}; otherwise null. */
    public static Pose savedPose(double maxAgeS) {
        if (savedPose == null || System.currentTimeMillis() - savedAtMs > maxAgeS * 1000) {
            return null;
        }
        return savedPose;
    }

    public static void clearSavedPose() {
        savedPose = null;
    }

    /** Wraps an angle to -pi..pi. */
    public static double wrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
