package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

/**
 * Limelight 3A vision subsystem.
 *
 * <p>Turns the camera's (tx, ty) angle readings into the target's position relative to the
 * robot, and optionally into the robot's field position when the target's field coordinates
 * are known.
 *
 * <h2>Coordinate conventions</h2>
 * <ul>
 *   <li><b>Limelight:</b> {@code tx} is positive when the target is to the <i>right</i> of the
 *       crosshair; {@code ty} is positive when it is <i>above</i>.</li>
 *   <li><b>Robot frame:</b> +X is forward, +Y is to the <i>left</i>, heading is CCW-positive.
 *       This is the standard FTC convention. Note that it disagrees with the Limelight on
 *       handedness, which is why {@link #solve} negates the lateral component exactly once.</li>
 *   <li><b>Field frame:</b> same handedness as the robot frame; heading 0 means the robot's
 *       +X points along the field's +X.</li>
 * </ul>
 *
 * <h2>Target holding</h2>
 * <p>When the target drops out, {@link #getLatest()} keeps returning the last known solution
 * for up to {@link #TARGET_HOLD_MS}, so a controller reading this does not snap to zero on a
 * one-frame glitch. Every held observation carries {@link TargetObservation#ageMs} and
 * {@link TargetObservation#fresh} — <b>check them before acting on a solution</b>, and see the
 * warning on {@link #TARGET_HOLD_MS}.
 *
 * <p>A held observation is re-projected against the current heading rather than frozen. The
 * target's position relative to robot center is invariant in the field frame under pure
 * rotation, so this is exact while the robot turns in place and degrades gracefully as it
 * translates.
 *
 * <p>This class does no I/O beyond polling the Limelight, so it can be used from any OpMode.
 * {@link #solve} is a pure static function and can be unit-tested off-robot.
 */
public class LimelightController {

    // ---------------------------------------------------------------------
    // Mounting geometry — MEASURE THESE ON YOUR ROBOT.
    // ---------------------------------------------------------------------

    /** Height of the camera lens above the floor, inches. */
    public static final double CAMERA_HEIGHT_IN = 2.5;

    /** Height of the target's center above the floor, inches. */
    public static final double TARGET_HEIGHT_IN = 56.0;

    /**
     * Camera pitch, degrees, positive = tilted up from horizontal.
     *
     * <p>With a 2.5 in camera and a 56 in target this being 0.0 is almost certainly wrong:
     * the entire ~53.5 in of elevation then has to fall inside {@code ty}, which puts the
     * target near the top of the FOV at 5 ft and out of frame any closer. Tilt the camera
     * up and put the real angle here.
     */
    public static final double CAMERA_PITCH_DEG = 0.0;

    /** Camera yaw relative to robot forward, degrees, CCW-positive. */
    public static final double CAMERA_YAW_DEG = 0.0;

    /** Camera position forward of robot center, inches (negative = behind center). */
    public static final double CAMERA_OFFSET_X_IN = 0.0;

    /** Camera position left of robot center, inches (negative = right of center). */
    public static final double CAMERA_OFFSET_Y_IN = 0.0;

    // ---------------------------------------------------------------------
    // Filtering, holding, and validity gates.
    // ---------------------------------------------------------------------

    /**
     * Low-pass time constant, seconds. The filter reaches ~63% of a step in this long,
     * independent of loop rate — unlike a fixed per-iteration alpha, whose real behavior
     * changes every time you add code to the loop.
     */
    public static final double FILTER_TAU_S = 0.08;

    /** Results older than this are treated as no target (catches a frozen/disconnected camera). */
    public static final long MAX_STALENESS_MS = 100;

    /**
     * How long a lost target keeps being reported by {@link #getLatest()}, milliseconds.
     *
     * <p><b>Warning:</b> a held target is a guess about a thing the camera can no longer see.
     * One second of coasting covers frame drops and brief occlusion during aiming. Much longer
     * than that and a robot which has driven away will keep confidently aiming at a ghost.
     * Set to {@code Double.POSITIVE_INFINITY} to hold indefinitely, but then gate every use on
     * {@link TargetObservation#ageMs} yourself.
     */
    public static final double TARGET_HOLD_MS = 1000;

    /**
     * Gaps shorter than this preserve filter state on reacquisition, milliseconds. After a
     * longer gap the filter snaps to the new reading instead of blending it with stale data.
     */
    public static final double FILTER_CONTINUITY_MS = 250;

    /** Solutions outside this horizontal range are rejected as implausible, inches. */
    public static final double MIN_RANGE_IN = 4.0;
    public static final double MAX_RANGE_IN = 300.0;

    /** Sentinel for "no fiducial in this result" (e.g. a color pipeline). */
    private static final int NO_ID = Integer.MIN_VALUE;

    // ---------------------------------------------------------------------

    private final Limelight3A limelight;

    /** Time since the last fresh, fully-solved target. Drives both holding and filter continuity. */
    private final ElapsedTime sinceFresh = new ElapsedTime();
    private final ElapsedTime filterTimer = new ElapsedTime();

    private boolean filterInitialized = false;
    private double smoothedTx = 0.0;
    private double smoothedTy = 0.0;
    private double rawTx = 0.0;
    private double rawTy = 0.0;

    private long lastStaleness = Long.MAX_VALUE;
    private int lastFiducialId = NO_ID;

    /** Last solution built from fresh camera data; the anchor that holding re-projects. */
    private TargetObservation anchor = null;

    /** What callers see: the anchor when fresh, a re-projected copy while held, else null. */
    private TargetObservation latest = null;

    private boolean fresh = false;

    public LimelightController(HardwareMap hardwareMap, String deviceName) {
        limelight = hardwareMap.get(Limelight3A.class, deviceName);
    }

    public LimelightController(HardwareMap hardwareMap) {
        this(hardwareMap, "Limelight");
    }

    /** Selects a pipeline and begins polling. Call once during init. */
    public void start(int pipeline) {
        limelight.pipelineSwitch(pipeline);
        limelight.start();
        sinceFresh.reset();
        filterTimer.reset();
    }

    /** Stops polling. Call once when the OpMode ends. */
    public void stop() {
        limelight.stop();
    }

    /**
     * Polls the camera and updates the estimate.
     *
     * @param headingDeg current robot heading in degrees, CCW-positive. Pass 0 if you have no
     *                   IMU; the robot-relative outputs stay correct either way, but holding
     *                   can then no longer compensate for rotation.
     * @return true if a fresh, valid target was solved on this call. A false return does
     *         <b>not</b> mean there is no estimate — check {@link #getLatest()}, which holds
     *         the last known target through dropouts.
     */
    public boolean update(double headingDeg) {
        LLResult result = limelight.getLatestResult();

        boolean valid = result != null
                && result.isValid()
                && result.getStaleness() <= MAX_STALENESS_MS;

        if (valid) {
            // Staleness counts up while data is stale and resets when a new frame lands, so a
            // decrease means this is genuinely new data. Without this the filter would
            // re-absorb the same frame on every loop iteration.
            long staleness = result.getStaleness();
            boolean newFrame = staleness < lastStaleness;
            lastStaleness = staleness;

            // Read the gap before resetting the timer below.
            boolean gapTooLong = sinceFresh.milliseconds() > FILTER_CONTINUITY_MS;

            // If the camera switched tags, the old filter state describes a different object;
            // blending across the switch produces a confident reading pointing at nothing.
            // Note: this tracks the primary target, which is what getTx()/getTy() refer to.
            int fiducialId = firstFiducialId(result);
            if (fiducialId != lastFiducialId || gapTooLong) {
                filterInitialized = false;
                lastFiducialId = fiducialId;
            }

            if (newFrame || !filterInitialized) {
                rawTx = result.getTx();
                rawTy = result.getTy();

                if (!filterInitialized) {
                    smoothedTx = rawTx;
                    smoothedTy = rawTy;
                    filterInitialized = true;
                } else {
                    double dt = filterTimer.seconds();
                    double alpha = 1.0 - Math.exp(-dt / FILTER_TAU_S);
                    smoothedTx += alpha * (rawTx - smoothedTx);
                    smoothedTy += alpha * (rawTy - smoothedTy);
                }
                filterTimer.reset();
            }

            TargetObservation solved = solve(smoothedTx, smoothedTy, headingDeg);
            if (solved != null) {
                anchor = solved;
                latest = solved;
                fresh = true;
                sinceFresh.reset();
                return true;
            }
            // The target is in frame but the geometry was rejected (bad mounting constants,
            // detection near the horizon). Treat it as a dropout and fall through to holding.
        }

        // A frozen camera keeps returning its last valid result forever, so staleness — not
        // isValid() — is what actually tells us the data stopped flowing.
        lastStaleness = Long.MAX_VALUE;
        fresh = false;

        if (anchor != null && sinceFresh.milliseconds() > TARGET_HOLD_MS) {
            anchor = null;
            filterInitialized = false;
            lastFiducialId = NO_ID;
        }

        latest = (anchor == null) ? null : reproject(anchor, headingDeg, sinceFresh.milliseconds());
        return false;
    }

    /**
     * Best current estimate of where the target is, or null if nothing is known.
     *
     * <p>This holds the last known target through dropouts, so a non-null return does not mean
     * the camera can currently see anything. Check {@link TargetObservation#fresh} or
     * {@link TargetObservation#ageMs} before acting on it.
     */
    public TargetObservation getLatest() {
        return latest;
    }

    /** True if the most recent {@link #update} solved a target from live camera data. */
    public boolean isFresh() {
        return fresh;
    }

    /** True if there is any usable estimate, fresh or held. */
    public boolean hasTarget() {
        return latest != null;
    }

    /** Milliseconds since the last fresh solution, or {@code Double.POSITIVE_INFINITY} if none. */
    public double getAgeMs() {
        return anchor == null ? Double.POSITIVE_INFINITY : sinceFresh.milliseconds();
    }

    public double getRawTx() {
        return rawTx;
    }

    public double getRawTy() {
        return rawTy;
    }

    public double getSmoothedTx() {
        return smoothedTx;
    }

    public double getSmoothedTy() {
        return smoothedTy;
    }

    /** Fiducial ID currently being tracked, or null when the pipeline reports no fiducials. */
    public Integer getTrackedFiducialId() {
        return lastFiducialId == NO_ID ? null : lastFiducialId;
    }

    private static int firstFiducialId(LLResult result) {
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return NO_ID;
        }
        return fiducials.get(0).getFiducialId();
    }

    /**
     * Rebuilds a held observation against the current heading.
     *
     * <p>{@code fieldRelX/fieldRelY} — the target's offset from robot center expressed in the
     * field frame — does not change when the robot rotates in place, so it is the right thing
     * to hold onto. Rotating it back by the current heading recovers robot-relative numbers
     * that stay correct through the turn. Translation is not compensated: the estimate decays
     * as the robot drives, which is what {@link TargetObservation#ageMs} is there to expose.
     */
    private static TargetObservation reproject(TargetObservation held, double headingDeg,
                                               double ageMs) {
        double cosHeading = Math.cos(Math.toRadians(headingDeg));
        double sinHeading = Math.sin(Math.toRadians(headingDeg));

        // Inverse of the robot -> field rotation applied in solve().
        double robotRelX = held.fieldRelX * cosHeading + held.fieldRelY * sinHeading;
        double robotRelY = -held.fieldRelX * sinHeading + held.fieldRelY * cosHeading;

        return new TargetObservation(
                Math.hypot(robotRelX, robotRelY),
                Math.toDegrees(Math.atan2(robotRelY, robotRelX)),
                robotRelX, robotRelY,
                held.fieldRelX, held.fieldRelY,
                ageMs, false);
    }

    // ---------------------------------------------------------------------
    // Geometry
    // ---------------------------------------------------------------------

    /**
     * Projects the camera ray onto the plane at the target's known height.
     *
     * <p>Unlike the common {@code (h2 - h1) / tan(pitch + ty)} shortcut, this builds the full
     * 3-D ray {@code (forward, right, up) = (1, tan(tx), tan(ty))} before scaling it. The
     * shortcut is only exact in the vertical plane through the target; its error grows with
     * both {@code tx} and camera pitch, and it silently mis-scales the lateral component
     * because that component was never part of the distance it computed.
     *
     * @param txDeg      horizontal angle to target, degrees, right-positive
     * @param tyDeg      vertical angle to target, degrees, up-positive
     * @param headingDeg robot heading, degrees, CCW-positive
     * @return the solution, or null if the geometry is degenerate or implausible
     */
    public static TargetObservation solve(double txDeg, double tyDeg, double headingDeg) {
        // Unit ray in the camera's optical frame.
        double rayForward = 1.0;
        double rayRight = Math.tan(Math.toRadians(txDeg));
        double rayUp = Math.tan(Math.toRadians(tyDeg));

        // Rotate the ray by the camera's pitch, about the camera's right axis.
        double cosPitch = Math.cos(Math.toRadians(CAMERA_PITCH_DEG));
        double sinPitch = Math.sin(Math.toRadians(CAMERA_PITCH_DEG));
        double forward = rayForward * cosPitch - rayUp * sinPitch;
        double up = rayForward * sinPitch + rayUp * cosPitch;
        double right = rayRight;

        double heightDelta = TARGET_HEIGHT_IN - CAMERA_HEIGHT_IN;

        // Scale the ray until it reaches the target's height. A non-positive scale means the
        // ray climbs away from the target instead of toward it — bad calibration, wrong
        // pipeline, or a false detection below the horizon. The old code let this through as
        // a negative distance, which reads as a target behind the robot.
        double scale = heightDelta / up;
        if (!Double.isFinite(scale) || scale <= 0.0) {
            return null;
        }
        if (forward <= 0.0) {
            return null; // behind the camera; not physically observable
        }

        double camForward = forward * scale;
        double camRight = right * scale;

        double camRange = Math.hypot(camForward, camRight);
        if (camRange < MIN_RANGE_IN || camRange > MAX_RANGE_IN) {
            // Also catches the near-horizon case: as `up` approaches 0 the range explodes
            // rather than hitting an exact zero an equality test could ever catch.
            return null;
        }

        // Camera frame (forward, right) -> robot-parallel frame (X forward, Y LEFT).
        // This negation is the only place handedness is converted.
        double camX = camForward;
        double camY = -camRight;

        // Apply the camera's yaw relative to robot forward.
        double cosYaw = Math.cos(Math.toRadians(CAMERA_YAW_DEG));
        double sinYaw = Math.sin(Math.toRadians(CAMERA_YAW_DEG));
        double rotX = camX * cosYaw - camY * sinYaw;
        double rotY = camX * sinYaw + camY * cosYaw;

        // Translate from the camera's mounting point to robot center. This happens in the
        // ROBOT frame, before the heading rotation — the offsets are measured on the robot,
        // so adding them after rotating into the field frame mixes two different frames and
        // is only correct at heading 0.
        double robotRelX = rotX + CAMERA_OFFSET_X_IN;
        double robotRelY = rotY + CAMERA_OFFSET_Y_IN;

        // Robot frame -> field frame.
        double cosHeading = Math.cos(Math.toRadians(headingDeg));
        double sinHeading = Math.sin(Math.toRadians(headingDeg));
        double fieldRelX = robotRelX * cosHeading - robotRelY * sinHeading;
        double fieldRelY = robotRelX * sinHeading + robotRelY * cosHeading;

        double range = Math.hypot(robotRelX, robotRelY);
        double bearing = Math.toDegrees(Math.atan2(robotRelY, robotRelX));

        return new TargetObservation(range, bearing, robotRelX, robotRelY,
                fieldRelX, fieldRelY, 0.0, true);
    }

    /**
     * Where the target is, relative to the robot.
     *
     * <p>Deliberately not called a "pose": every field here describes the <i>target</i>, not
     * the robot. To get the robot's own field position, use
     * {@link #robotFieldPosition(double, double)}.
     *
     * <p>An instance may be a held estimate rather than a live measurement — see
     * {@link #fresh} and {@link #ageMs}.
     */
    public static class TargetObservation {

        /** Horizontal distance from robot center to the target, inches. */
        public final double rangeIn;

        /** Angle from robot forward to the target, degrees, CCW-positive. */
        public final double bearingDeg;

        /** Target position in the robot frame: +X forward, +Y left, inches. */
        public final double robotRelX;
        public final double robotRelY;

        /** Same vector rotated into the field frame, inches. */
        public final double fieldRelX;
        public final double fieldRelY;

        /** Milliseconds since this was measured. 0 when {@link #fresh}. */
        public final double ageMs;

        /** True if built from live camera data; false if held through a dropout. */
        public final boolean fresh;

        TargetObservation(double rangeIn, double bearingDeg,
                          double robotRelX, double robotRelY,
                          double fieldRelX, double fieldRelY,
                          double ageMs, boolean fresh) {
            this.rangeIn = rangeIn;
            this.bearingDeg = bearingDeg;
            this.robotRelX = robotRelX;
            this.robotRelY = robotRelY;
            this.fieldRelX = fieldRelX;
            this.fieldRelY = fieldRelY;
            this.ageMs = ageMs;
            this.fresh = fresh;
        }

        /**
         * The robot's field position, given the target's known field position.
         *
         * @return {@code {x, y}} in inches
         */
        public double[] robotFieldPosition(double targetFieldX, double targetFieldY) {
            return new double[]{targetFieldX - fieldRelX, targetFieldY - fieldRelY};
        }
    }
}