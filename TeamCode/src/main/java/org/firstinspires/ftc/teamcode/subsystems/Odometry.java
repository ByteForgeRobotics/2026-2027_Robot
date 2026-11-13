package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Robot position from the two dead wheels, read through the goBILDA Pinpoint.
 *
 * <p>The Pinpoint does the odometry math on its own chip and fuses in its IMU for heading, so
 * this class just configures it and hands out the result. Call {@link #update()} once per
 * loop; every getter returns what that read saw. One update is one I2C read, because
 * everything here is in the Pinpoint's default bulk read.
 *
 * <h2>Coordinates</h2>
 * <p>Same as the Pinpoint and Road Runner: +x is forward, +y is left, heading is
 * counter-clockwise positive in radians. Zero is wherever the robot sat at the last
 * {@link #reset()}, unless {@link #setPose} moved it (TeleOp sets the field start pose, see
 * {@link Field}).
 */
@Config
public class Odometry {

    // ======================================================================
    // DO NOT TOUCH
    // Pinpoint settings. The name must match the Driver Station config, and
    // the pod type and directions were checked on the robot. Wrong values
    // make auto drive the wrong way.
    // ======================================================================
    public static String PINPOINT_NAME = "pinpoint";
    public static GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    /** How far left of the robot's center the forward (X) pod sits. Right is negative. */
    public static double X_POD_OFFSET_MM = 0.0;
    /** How far forward of the robot's center the sideways (Y) pod sits. Back is negative. */
    public static double Y_POD_OFFSET_MM = 0.0;
    /** Flip if pushing the robot forward makes x go down. */
    public static boolean REVERSE_X_POD = false;
    /** Flip if pushing the robot left makes y go down. */
    public static boolean REVERSE_Y_POD = false;

    private final GoBildaPinpointDriver pinpoint;

    public Odometry(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, PINPOINT_NAME);
    }

    public boolean isConnected() {
        return pinpoint != null;
    }

    /**
     * Pushes the dashboard pod settings to the Pinpoint and zeroes position and heading. The
     * IMU recalibrates for about a quarter second, so the robot has to be still; wait for
     * {@link #isReady()} before moving.
     */
    public void reset() {
        if (pinpoint == null) {
            return;
        }
        pinpoint.setEncoderResolution(POD_TYPE);
        pinpoint.setOffsets(X_POD_OFFSET_MM, Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderDirections(
                REVERSE_X_POD ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                REVERSE_Y_POD ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                        : GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }

    /**
     * Tells the Pinpoint where the robot is right now (inches, radians), e.g. its start spot in
     * the field frame. Call after the robot is still and {@link #reset()} has finished
     * calibrating (at START is a good time), or the reset will zero it again.
     */
    public void setPose(double xIn, double yIn, double headingRad) {
        if (pinpoint != null) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, xIn, yIn, AngleUnit.RADIANS, headingRad));
        }
    }

    /** Reads the Pinpoint. Call once per loop, before any getter. */
    public void update() {
        if (pinpoint != null) {
            pinpoint.update();
        }
    }

    public boolean isReady() {
        return pinpoint != null
                && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    /** READY, CALIBRATING, a fault name, or NOT FOUND. */
    public String getStatus() {
        return pinpoint == null ? "NOT FOUND" : pinpoint.getDeviceStatus().toString();
    }

    /** Inches. */
    public double getX() {
        return pinpoint == null ? 0.0 : pinpoint.getPosX(DistanceUnit.INCH);
    }

    /** Inches. */
    public double getY() {
        return pinpoint == null ? 0.0 : pinpoint.getPosY(DistanceUnit.INCH);
    }

    /** Radians, -pi to pi. */
    public double getHeading() {
        return pinpoint == null ? 0.0 : pinpoint.getHeading(AngleUnit.RADIANS);
    }

    /** Raw forward-pod count. Goes up when the robot is pushed forward. */
    public int getEncoderX() {
        return pinpoint == null ? 0 : pinpoint.getEncoderX();
    }

    /** Raw sideways-pod count. Goes up when the robot is pushed left. */
    public int getEncoderY() {
        return pinpoint == null ? 0 : pinpoint.getEncoderY();
    }

    /** Field-frame velocity, inches per second. */
    public double getVelX() {
        return pinpoint == null ? 0.0 : pinpoint.getVelX(DistanceUnit.INCH);
    }

    /** Field-frame velocity, inches per second. */
    public double getVelY() {
        return pinpoint == null ? 0.0 : pinpoint.getVelY(DistanceUnit.INCH);
    }
}
