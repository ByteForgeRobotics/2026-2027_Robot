package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * HuskyLens pollen tracker.
 *
 * <p>Secondary camera, mounted low on the chassis and pointed at the floor in front of the
 * intake. The Limelight on the turret handles AprilTags; this one only answers "where is the
 * nearest yellow ball". It talks I2C, not USB, so it does not touch the Limelight's link.
 *
 * <h2>How it finds pollen</h2>
 * <p>The HuskyLens runs its Color Recognition algorithm on-device. Yellow is taught once, on the
 * camera itself (point at a POLLEN, hold the learn button), and comes back as color ID
 * {@link #POLLEN_ID}. Every {@link #update} reads the block list, keeps the ones that look like a
 * ball on the floor, and picks one target: the block nearest the previous target if there was
 * one, else the biggest (closest) block.
 *
 * <h2>Range and bearing</h2>
 * <p>Same idea as {@link LimelightController#solve}: turn the block's pixel position into a ray,
 * tilt it by the camera pitch, and stretch it until it hits the plane at ball-center height.
 * That gives forward/left inches in the robot frame without trusting the block's pixel width,
 * which shrinks whenever a ball is half hidden behind another one.
 *
 * <h2>Rules</h2>
 * <ul>
 *   <li>R702 example 5 allows the HuskyLens as a configurable, non-programmable coprocessor.</li>
 *   <li>R613: power it from the I2C port it is plugged into, nothing else.</li>
 *   <li>G407: never CONTROL more than 4 SCORING ELEMENTS. Gate the intake on a ball count;
 *       this class only says where the next ball is, not whether you may take it.</li>
 *   <li>G409: do not chase balls falling out of a tipping hive. The OpMode decides that.</li>
 * </ul>
 */
public class HuskyLensController {

    // ---------------------------------------------------------------------
    // Mounting geometry and camera model. MEASURE THESE ON YOUR ROBOT.
    // ---------------------------------------------------------------------

    /** HuskyLens image size. The device reports block coordinates in this frame. */
    public static final int IMAGE_WIDTH_PX = 320;
    public static final int IMAGE_HEIGHT_PX = 240;

    /**
     * Horizontal field of view, degrees. DFRobot does not publish a clean number, and it
     * changes if the lens has been refocused. Measure it: put a tape measure across the floor
     * at a known distance, read off how many inches span the full image width, and use
     * {@code 2 * atan((span / 2) / distance)}.
     */
    public static final double HFOV_DEG = 65.0;

    /** Height of the lens above the floor, inches. */
    public static final double CAMERA_HEIGHT_IN = 9.0;

    /** Camera pitch, degrees, positive = tilted up. Pointing at the floor makes this negative. */
    public static final double CAMERA_PITCH_DEG = -20.0;

    /** Camera yaw relative to robot forward, degrees, CCW-positive. */
    public static final double CAMERA_YAW_DEG = 0.0;

    /** Camera position forward of robot center, inches. */
    public static final double CAMERA_OFFSET_X_IN = 7.0;

    /** Camera position left of robot center, inches (negative = right). */
    public static final double CAMERA_OFFSET_Y_IN = 0.0;

    /** POLLEN is a 2.8 in ball (manual §9.8), so its center sits 1.4 in off the tiles. */
    public static final double POLLEN_DIAMETER_IN = 2.8;
    public static final double POLLEN_CENTER_HEIGHT_IN = POLLEN_DIAMETER_IN / 2.0;

    // ---------------------------------------------------------------------
    // Target selection.
    // ---------------------------------------------------------------------

    /** Color ID the HuskyLens assigned when yellow was learned. First learned color is 1. */
    public static final int POLLEN_ID = 1;

    /** Blocks narrower than this are noise or too far away to matter, pixels. */
    public static final int MIN_BLOCK_WIDTH_PX = 8;

    /** Balls are round. Blocks far from square are tape, a robot panel, or two balls merged. */
    public static final double MIN_ASPECT = 0.6;
    public static final double MAX_ASPECT = 1.7;

    /**
     * Ignore blocks whose center is above this row, pixels from the top. Anything up there is
     * not on the floor in front of the robot (the hive, a ball on another robot, a yellow shirt).
     * Depends on the pitch; with the camera tilted down 20 deg the horizon is near the top.
     */
    public static final int HORIZON_ROW_PX = 40;

    /** A block this close (pixels) to the last target is treated as the same ball. */
    public static final double STICKY_RADIUS_PX = 60.0;

    /** Solutions outside this range are rejected as implausible, inches. */
    public static final double MIN_RANGE_IN = 3.0;
    public static final double MAX_RANGE_IN = 72.0;

    // ---------------------------------------------------------------------
    // Timing.
    // ---------------------------------------------------------------------

    /**
     * Minimum time between I2C reads, milliseconds. Each {@code blocks()} call is a full
     * request/response over I2C and costs real loop time; hammering it every loop also makes
     * the HuskyLens miss commands. 50 ms (20 Hz) is plenty for steering toward a ball.
     */
    public static final long READ_PERIOD_MS = 50;

    /** How long a lost target keeps being reported, milliseconds. Short: balls do not move. */
    public static final double TARGET_HOLD_MS = 300;

    // ---------------------------------------------------------------------

    private final HuskyLens huskyLens;
    private final ElapsedTime sinceRead = new ElapsedTime();
    private final ElapsedTime sinceFresh = new ElapsedTime();

    private boolean connected = false;
    private HuskyLens.Block[] lastBlocks = new HuskyLens.Block[0];
    private int candidateCount = 0;

    private PollenObservation anchor = null;
    private PollenObservation latest = null;
    private boolean fresh = false;

    public HuskyLensController(HardwareMap hardwareMap, String deviceName) {
        huskyLens = hardwareMap.get(HuskyLens.class, deviceName);
    }

    public HuskyLensController(HardwareMap hardwareMap) {
        this(hardwareMap, "huskylens");
    }

    /**
     * Checks the link and selects the color algorithm. Call once during init.
     *
     * @return true if the HuskyLens answered. False usually means the protocol on the device
     *         is not set to I2C (General Settings, Protocol Type), or the cable is on the wrong
     *         port.
     */
    public boolean start() {
        connected = huskyLens.knock();
        if (connected) {
            // Select once. Re-selecting every loop is slow and resets the device's tracking.
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        }
        sinceRead.reset();
        sinceFresh.reset();
        return connected;
    }

    /** True if {@link #start} got a reply from the camera. */
    public boolean isConnected() {
        return connected;
    }

    /**
     * Reads the camera (rate limited) and updates the target.
     *
     * @return true if a fresh pollen target was solved on this call. False does not mean there
     *         is no estimate; see {@link #getLatest()}.
     */
    public boolean update() {
        if (!connected) {
            latest = null;
            fresh = false;
            return false;
        }

        if (sinceRead.milliseconds() < READ_PERIOD_MS) {
            // Between reads, keep reporting what we had. Age keeps counting.
            fresh = false;
            latest = holdOrExpire();
            return false;
        }
        sinceRead.reset();

        HuskyLens.Block[] blocks = huskyLens.blocks(POLLEN_ID);
        lastBlocks = (blocks == null) ? new HuskyLens.Block[0] : blocks;

        HuskyLens.Block chosen = pick(lastBlocks, anchor);
        PollenObservation solved = (chosen == null) ? null : solve(chosen);

        if (solved != null) {
            anchor = solved;
            latest = solved;
            fresh = true;
            sinceFresh.reset();
            return true;
        }

        fresh = false;
        latest = holdOrExpire();
        return false;
    }

    private PollenObservation holdOrExpire() {
        if (anchor == null) {
            return null;
        }
        double age = sinceFresh.milliseconds();
        if (age > TARGET_HOLD_MS) {
            anchor = null;
            return null;
        }
        return anchor.aged(age);
    }

    /** Best current estimate of the nearest pollen, or null. May be held; check {@code fresh}. */
    public PollenObservation getLatest() {
        return latest;
    }

    public boolean isFresh() {
        return fresh;
    }

    public boolean hasTarget() {
        return latest != null;
    }

    /** Raw block list from the last read, for telemetry. Never null. */
    public HuskyLens.Block[] getLastBlocks() {
        return lastBlocks;
    }

    /** How many blocks survived the shape and horizon filters on the last read. */
    public int getCandidateCount() {
        return candidateCount;
    }

    // ---------------------------------------------------------------------
    // Target selection
    // ---------------------------------------------------------------------

    /**
     * Picks one block to chase. Prefers the block nearest the previous target so the robot
     * does not flip between two balls that are the same size. Falls back to the largest block,
     * which is almost always the closest one.
     */
    private HuskyLens.Block pick(HuskyLens.Block[] blocks, PollenObservation previous) {
        HuskyLens.Block sticky = null;
        double stickyDist = STICKY_RADIUS_PX;
        HuskyLens.Block largest = null;
        int largestArea = 0;
        int count = 0;

        for (HuskyLens.Block b : blocks) {
            if (!looksLikePollen(b)) {
                continue;
            }
            count++;

            int area = b.width * b.height;
            if (area > largestArea) {
                largestArea = area;
                largest = b;
            }

            if (previous != null) {
                double d = Math.hypot(b.x - previous.pixelX, b.y - previous.pixelY);
                if (d < stickyDist) {
                    stickyDist = d;
                    sticky = b;
                }
            }
        }

        candidateCount = count;
        return (sticky != null) ? sticky : largest;
    }

    /** The shape and horizon filter. Public so diagnostics can show why a block was rejected. */
    public static boolean looksLikePollen(HuskyLens.Block b) {
        if (b.id != POLLEN_ID) {
            return false;
        }
        if (b.width < MIN_BLOCK_WIDTH_PX || b.height <= 0) {
            return false;
        }
        double aspect = (double) b.width / b.height;
        if (aspect < MIN_ASPECT || aspect > MAX_ASPECT) {
            return false;
        }
        return b.y >= HORIZON_ROW_PX;
    }

    // ---------------------------------------------------------------------
    // Geometry
    // ---------------------------------------------------------------------

    /** Focal length in pixels, from the horizontal FOV. */
    public static double focalPx() {
        return (IMAGE_WIDTH_PX / 2.0) / Math.tan(Math.toRadians(HFOV_DEG / 2.0));
    }

    /**
     * Projects the block's pixel center onto the plane at ball-center height.
     *
     * <p>Mirrors {@link LimelightController#solve}, except the target is below the camera, so
     * the ray has to point down to be valid. Pure function of the constants above; testable
     * off-robot.
     *
     * @return the observation, or null if the geometry is degenerate or out of range
     */
    public static PollenObservation solve(HuskyLens.Block b) {
        double f = focalPx();

        // Pixel offsets from image center. Down and right are positive in the image.
        double dx = b.x - IMAGE_WIDTH_PX / 2.0;
        double dy = b.y - IMAGE_HEIGHT_PX / 2.0;

        // Unit ray in the camera's optical frame: forward, right, up.
        double rayForward = 1.0;
        double rayRight = dx / f;
        double rayUp = -dy / f;

        // Rotate by the camera pitch about the camera's right axis.
        double cosPitch = Math.cos(Math.toRadians(CAMERA_PITCH_DEG));
        double sinPitch = Math.sin(Math.toRadians(CAMERA_PITCH_DEG));
        double forward = rayForward * cosPitch - rayUp * sinPitch;
        double up = rayForward * sinPitch + rayUp * cosPitch;
        double right = rayRight;

        // The ball is below the lens, so heightDelta is negative and the ray must point down.
        double heightDelta = POLLEN_CENTER_HEIGHT_IN - CAMERA_HEIGHT_IN;
        double scale = heightDelta / up;
        if (!Double.isFinite(scale) || scale <= 0.0 || forward <= 0.0) {
            return null;
        }

        double camForward = forward * scale;
        double camRight = right * scale;
        double camRange = Math.hypot(camForward, camRight);
        if (camRange < MIN_RANGE_IN || camRange > MAX_RANGE_IN) {
            return null;
        }

        // Camera (forward, right) -> robot (X forward, Y left). Only handedness flip.
        double camX = camForward;
        double camY = -camRight;

        double cosYaw = Math.cos(Math.toRadians(CAMERA_YAW_DEG));
        double sinYaw = Math.sin(Math.toRadians(CAMERA_YAW_DEG));
        double rotX = camX * cosYaw - camY * sinYaw;
        double rotY = camX * sinYaw + camY * cosYaw;

        double robotRelX = rotX + CAMERA_OFFSET_X_IN;
        double robotRelY = rotY + CAMERA_OFFSET_Y_IN;

        double range = Math.hypot(robotRelX, robotRelY);
        double bearing = Math.toDegrees(Math.atan2(robotRelY, robotRelX));

        // Cross-check from the ball's pixel width. Only meaningful when the ball is not
        // partly hidden, so it is reported, not used.
        double rangeFromWidth = f * POLLEN_DIAMETER_IN / b.width;

        return new PollenObservation(range, bearing, robotRelX, robotRelY,
                b.x, b.y, b.width, b.height, rangeFromWidth, 0.0, true);
    }

    // ---------------------------------------------------------------------
    // Steering helper
    // ---------------------------------------------------------------------

    /** Proportional gains for {@link #steer}. Tune on the robot, these are starting points. */
    public static final double TURN_KP = 0.02;      // motor power per degree of bearing
    public static final double FORWARD_KP = 0.03;   // motor power per inch of range error
    public static final double STOP_RANGE_IN = 8.0; // where the ball is inside the intake
    public static final double MAX_FORWARD = 0.5;
    public static final double MAX_TURN = 0.4;

    /**
     * Turns an observation into drive powers that point the intake at the ball and close in.
     *
     * <p>Returns {@code {forward, turn}} in [-1, 1]. Forward is +X, turn is CCW-positive, so a
     * ball on the left (positive bearing) produces a positive turn. Feed these into whatever
     * mecanum mixer you use. No strafe term on purpose: intakes take balls head-on.
     */
    public static double[] steer(PollenObservation obs) {
        double turn = clamp(TURN_KP * obs.bearingDeg, -MAX_TURN, MAX_TURN);

        // Do not drive forward while badly misaligned, the ball just slides off the intake.
        double alignment = Math.max(0.0, 1.0 - Math.abs(obs.bearingDeg) / 30.0);
        double forward = clamp(FORWARD_KP * (obs.rangeIn - STOP_RANGE_IN), 0.0, MAX_FORWARD)
                * alignment;

        return new double[]{forward, turn};
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // ---------------------------------------------------------------------

    /** Where the chosen pollen is, relative to the robot. */
    public static class PollenObservation {

        /** Horizontal distance from robot center to the ball, inches. */
        public final double rangeIn;

        /** Angle from robot forward to the ball, degrees, CCW-positive. */
        public final double bearingDeg;

        /** Ball position in the robot frame: +X forward, +Y left, inches. */
        public final double robotRelX;
        public final double robotRelY;

        /** Block center and size in the HuskyLens image, pixels. */
        public final int pixelX;
        public final int pixelY;
        public final int pixelWidth;
        public final int pixelHeight;

        /** Independent range estimate from the ball's pixel width, inches. For sanity checks. */
        public final double rangeFromWidthIn;

        /** Milliseconds since this was measured. 0 when {@link #fresh}. */
        public final double ageMs;

        /** True if built from live camera data; false if held through a dropout. */
        public final boolean fresh;

        PollenObservation(double rangeIn, double bearingDeg,
                          double robotRelX, double robotRelY,
                          int pixelX, int pixelY, int pixelWidth, int pixelHeight,
                          double rangeFromWidthIn, double ageMs, boolean fresh) {
            this.rangeIn = rangeIn;
            this.bearingDeg = bearingDeg;
            this.robotRelX = robotRelX;
            this.robotRelY = robotRelY;
            this.pixelX = pixelX;
            this.pixelY = pixelY;
            this.pixelWidth = pixelWidth;
            this.pixelHeight = pixelHeight;
            this.rangeFromWidthIn = rangeFromWidthIn;
            this.ageMs = ageMs;
            this.fresh = fresh;
        }

        PollenObservation aged(double ageMs) {
            return new PollenObservation(rangeIn, bearingDeg, robotRelX, robotRelY,
                    pixelX, pixelY, pixelWidth, pixelHeight, rangeFromWidthIn, ageMs, false);
        }
    }
}
