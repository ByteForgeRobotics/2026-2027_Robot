package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * Does the code fit the 2026-27 BIOBUZZ field? These tests use the real field geometry, not the
 * code's placeholders, so a failure here means "this would not work in a match" rather than
 * "the math is broken".
 *
 * <p>Geometry (inches, FTC field frame: +x toward the audience, red half is y < 0), from the
 * competition manual (Sections 9.6, 10.3.1, Figs 9-9/9-10/9-16) and the team's vault note
 * "BIOBUZZ Field Localization with Vision":
 * <ul>
 *   <li>HIVE structure at field center. Red HIVE at y = -12.75, blue at y = +12.75.</li>
 *   <li>Each HIVE has an audience cell (x > 0) and a far cell (x < 0). Only the UP cell scores.
 *   Its 20 x 14 in opening is on the cell's outward face, 53.5 to 65.6 in off the tiles, facing
 *   that cell's wall and tilted up 30 deg. The opening center is roughly 19 in from the field
 *   center line along x (estimate from the 42.91 in beam at 30 deg; +/- 3 in).</li>
 *   <li>Start state: red AUDIENCE cell up, blue FAR cell up. Every tip flips it.</li>
 *   <li>Red LOADING ZONE ~23 x 11 in on the far half of the red wall, centered near (-36, -66).
 *   G304.E: a robot may not start in it.</li>
 * </ul>
 */
public class BioBuzzGameFitTest {

    static final double IN = 0.0254;
    static final double HIVE_Y = 12.75;
    static final double CELL_X = 19;
    static final double OPENING_LOW_IN = 53.5, OPENING_HIGH_IN = 65.6, OPENING_TALL_IN = 14;
    static final double OPENING_CENTER_M = (OPENING_LOW_IN + OPENING_HIGH_IN) / 2 * IN;
    static final double POLLEN_IN = 2.8;

    /**
     * How much room the ball has to get through the opening, inches, for a ball arriving at
     * {@code arrivalDeg} (negative = falling). The opening leans back 30 deg, so a ball falling
     * at 30 deg sees all 14 in; flatter or steeper sees less.
     */
    static double clearanceIn(double arrivalDeg) {
        return OPENING_TALL_IN * Math.cos(Math.toRadians(arrivalDeg + 30)) - POLLEN_IN;
    }

    /** Height (m) the ball is at when it reaches the opening, for a given rpm/angle. */
    static double heightAtOpening(double distM, double angleDeg, double rpm) {
        double[] s = ShotSolver.fly(distM, Math.toRadians(angleDeg), ShotSolver.speedForRpm(rpm));
        return s == null ? Double.NaN : s[0];
    }

    @Test
    public void solverCanHitTheUpCellFromPracticalDistances() {
        StringBuilder t = new StringBuilder(String.format(
                "Shots at the up cell (opening center %.2f m high):%n  dist   angle   rpm   arrives  room   miss@+-TOL_RPM  miss@+-2in%n", OPENING_CENTER_M));
        boolean allOk = true;
        for (double d = 0.6; d <= 3.21; d += 0.2) {
            ShotSolver.Shot s = ShotSolver.solve(d, OPENING_CENTER_M);
            if (!s.reachable) {
                t.append(String.format("  %.1f m  OUT OF RANGE%n", d));
                if (d >= 1.0 && d <= 3.0) allOk = false;
                continue;
            }
            double room = clearanceIn(s.arrivalAngleDeg);
            double rpmMiss = Math.max(
                    Math.abs(heightAtOpening(d, s.angleDeg, s.rpm + Flywheel.TOLERANCE_RPM) - OPENING_CENTER_M),
                    Math.abs(heightAtOpening(d, s.angleDeg, s.rpm - Flywheel.TOLERANCE_RPM) - OPENING_CENTER_M)) / IN;
            // Being 2 in off on distance (odometry drift) = shooting the right shot at the wrong spot.
            double distMiss = Math.max(
                    Math.abs(heightAtOpening(d + 2 * IN, s.angleDeg, s.rpm) - OPENING_CENTER_M),
                    Math.abs(heightAtOpening(d - 2 * IN, s.angleDeg, s.rpm) - OPENING_CENTER_M)) / IN;
            t.append(String.format("  %.1f m  %5.1f  %5.0f  %5.0f deg  %4.1f in   %4.1f in      %4.1f in%n",
                    d, s.angleDeg, s.rpm, s.arrivalAngleDeg, room, rpmMiss, distMiss));
        }
        System.out.println(t);
        assertTrue("some 1-3 m shots at the real cell height are out of range\n" + t, allOk);
    }

    @Test
    public void flywheelToleranceKeepsShotsInTheOpening() {
        // atSpeed() lets the ball go anywhere inside +-TOLERANCE_RPM. That error must still
        // land inside the opening at every distance the robot will shoot from.
        StringBuilder bad = new StringBuilder();
        for (double d = 1.0; d <= 3.01; d += 0.25) {
            ShotSolver.Shot s = ShotSolver.solve(d, OPENING_CENTER_M);
            if (!s.reachable) continue;
            double halfRoom = clearanceIn(s.arrivalAngleDeg) / 2;
            for (double sign : new double[]{-1, 1}) {
                double miss = Math.abs(heightAtOpening(d, s.angleDeg, s.rpm + sign * Flywheel.TOLERANCE_RPM)
                        - OPENING_CENTER_M) / IN * Math.cos(Math.toRadians(s.arrivalAngleDeg));
                if (miss > halfRoom) {
                    bad.append(String.format("%.2f m: %.0f rpm off misses by %.1f in, only %.1f in of room%n",
                            d, sign * Flywheel.TOLERANCE_RPM, miss, halfRoom));
                }
            }
        }
        assertTrue("TOLERANCE_RPM too loose for the cell opening:\n" + bad, bad.length() == 0);
    }

    @Test
    public void ballMassMattersPrintSensitivity() {
        // BALL_MASS_KG is a placeholder (0.022 kg). Show how much it moves the answer at 2.5 m.
        double saved = ShotSolver.BALL_MASS_KG;
        StringBuilder t = new StringBuilder("RPM at 2.5 m to the up cell by pollen mass:");
        try {
            for (double m : new double[]{0.022, 0.035, 0.050}) {
                ShotSolver.BALL_MASS_KG = m;
                ShotSolver.Shot s = ShotSolver.solve(2.5, OPENING_CENTER_M);
                t.append(String.format("  %.0f g: %.0f rpm @ %.1f deg", m * 1000, s.rpm, s.angleDeg));
            }
        } finally {
            ShotSolver.BALL_MASS_KG = saved;
        }
        System.out.println(t);
    }

    @Test
    public void goalIsTheRedHiveUpCell() {
        // At the start of a match red's scoring opening is the red AUDIENCE cell.
        Field.Pose g = Field.goal(Field.Alliance.RED);
        double off = Math.hypot(g.x - CELL_X, g.y + HIVE_Y);
        assertTrue(String.format("Field.goal(RED) is %s, but red's up cell is near (%.0f, %.2f) at field "
                + "center - %.0f in away. The robot would aim at an empty corner.", g, CELL_X, -HIVE_Y, off), off < 6);
    }

    @Test
    public void blueGoalIsTheBlueUpCellAtTheStart() {
        // Start state is red audience cell up, blue FAR cell up.
        Field.Pose blue = Field.goal(Field.Alliance.BLUE);
        assertTrue(String.format("Field.goal(BLUE) is %s; blue's up cell at the start is (%.0f, %.2f)",
                blue, -CELL_X, HIVE_Y), Math.hypot(blue.x + CELL_X, blue.y - HIVE_Y) < 6);
    }

    @Test
    public void startSpotsMirrorThePointSymmetricField() {
        // Red's AUDIENCE spot turned 180 deg about the center is blue's FAR spot.
        Field.Pose red = Field.startPose(Field.Alliance.RED, Field.StartSpot.AUDIENCE);
        Field.Pose blue = Field.startPose(Field.Alliance.BLUE, Field.StartSpot.FAR);
        assertTrue(red + " vs " + blue, Math.abs(red.x + blue.x) < 1e-9 && Math.abs(red.y + blue.y) < 1e-9);
    }

    @Test
    public void goalMovesToTheOtherSideAfterATip() {
        for (Field.Alliance a : Field.Alliance.values()) {
            Field.Pose before = Field.goal(a);
            Field.Pose after = Field.goal(a, Field.startUpCell(a).other());
            assertTrue(a + ": same HIVE", Math.abs(before.y - after.y) < 1e-9);
            assertTrue(a + ": other end of it", Math.abs(before.x + after.x) < 1e-9 && Math.abs(before.x) > 10);
            assertTrue(a + ": faces the other wall", Math.abs(Field.wrap(before.heading - after.heading)) > 3);
        }
    }

    @Test
    public void onlyShootsFromInFrontOfTheOpening() {
        // Red at the start: the audience cell, opening facing +x (G417.C).
        Field.Pose g = Field.goal(Field.Alliance.RED);
        assertTrue("straight in front", Field.canScoreFrom(g, 60, -HIVE_Y));
        assertTrue("in front, a bit to the side", Field.canScoreFrom(g, 55, -40));
        assertFalse("right beside the cell", Field.canScoreFrom(g, CELL_X, -60));
        assertFalse("behind it, far side of the HIVE", Field.canScoreFrom(g, -40, -HIVE_Y));
        // After a tip the far cell is the target: now the far side is right and the audience side wrong.
        Field.Pose tipped = Field.goal(Field.Alliance.RED, Field.Cell.FAR);
        assertTrue(Field.canScoreFrom(tipped, -60, -HIVE_Y));
        assertFalse(Field.canScoreFrom(tipped, 60, -HIVE_Y));
    }

    @Test
    public void shotSolverHeightPlaceholdersMatchTheCell() {
        // Full TeleOp Test and Shot Test both aim at Field.CELL_OPENING_HEIGHT_M.
        double used = Field.CELL_OPENING_HEIGHT_M;
        assertTrue(String.format("OpModes aim at %.2f m; the up-cell opening is %.2f-%.2f m (center %.2f)",
                        used, OPENING_LOW_IN * IN, OPENING_HIGH_IN * IN, OPENING_CENTER_M),
                used >= OPENING_LOW_IN * IN && used <= OPENING_HIGH_IN * IN);
    }

    // ---------------- Start positions (G304) ----------------

    /** Corners of an 18 in square robot at a pose. */
    static double[][] footprint(Field.Pose p) {
        double[][] c = new double[4][];
        int i = 0;
        for (double fx : new double[]{-9, 9}) {
            for (double fy : new double[]{-9, 9}) {
                c[i++] = new double[]{p.x + fx * Math.cos(p.heading) - fy * Math.sin(p.heading),
                        p.y + fx * Math.sin(p.heading) + fy * Math.cos(p.heading)};
            }
        }
        return c;
    }

    static boolean overlapsBox(double[][] c, double x0, double x1, double y0, double y1) {
        double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
        for (double[] p : c) {
            minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]);
            minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]);
        }
        return maxX > x0 && minX < x1 && maxY > y0 && minY < y1;
    }

    @Test
    public void startSpotsAreLegal() {
        StringBuilder bad = new StringBuilder();
        for (Field.Alliance a : Field.Alliance.values()) {
            for (Field.StartSpot s : Field.StartSpot.values()) {
                Field.Pose p = Field.startPose(a, s);
                double[][] c = footprint(p);
                double side = a == Field.Alliance.RED ? -1 : 1;
                boolean ownHalf = true, touchesWall = false, inField = true;
                for (double[] q : c) {
                    if (q[1] * side < -0.01) ownHalf = false;
                    if (Math.abs(q[0]) > 72.01 || Math.abs(q[1]) > 72.01) inField = false;
                    if (Math.abs(Math.abs(q[0]) - 72) < 0.5 || Math.abs(Math.abs(q[1]) - 72) < 0.5) touchesWall = true;
                }
                // Red loading zone near (-36, -66); blue's is red's turned 180 deg: (36, 66).
                double lzX = a == Field.Alliance.RED ? -36 : 36, lzY = a == Field.Alliance.RED ? -66 : 66;
                boolean inLz = overlapsBox(c, lzX - 11.5, lzX + 11.5, lzY - 5.5, lzY + 5.5);
                String where = a + " " + s + " " + p;
                if (!inField) bad.append(where).append(": outside the field\n");
                if (!ownHalf) bad.append(where).append(": not fully on own alliance side (G304.A)\n");
                if (!touchesWall) bad.append(where).append(": not touching the wall (G304.C)\n");
                if (inLz) bad.append(where).append(": in the LOADING ZONE (G304.E)\n");
            }
        }
        assertTrue("Illegal start spots (loading zone position is from the vault note, check Fig 9-2):\n" + bad,
                bad.length() == 0);
    }
}
