package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

/**
 * Battery voltage and current draw, for watching a drive test brown out.
 *
 * <p>Answers three questions while you drive: what is the battery doing right now, how much
 * current is the robot pulling, and how far did the voltage dip at the worst moment. The last
 * one is the useful one — a battery that reads 13 V sitting still and sags to 10 V the instant
 * four wheels spin is a tired battery, and that shows up as erratic motors and dropped encoder
 * counts long before anything looks obviously broken.
 *
 * <h2>Why this is rate limited</h2>
 * <p>Hub voltage and current are not part of the bulk read. Every
 * {@link LynxModule#getInputVoltage} and {@link LynxModule#getCurrent} is its own ADC request
 * over the RS485 link, costing a couple of milliseconds each, and the same is true of
 * {@link DcMotorEx#getCurrent}. Reading all of them every loop would noticeably slow a drive
 * OpMode, so {@link #update()} refreshes on a fixed cadence ({@link #READ_PERIOD_MS}) and the
 * getters return the last reading in between. 10 Hz is far faster than anyone can read a
 * number off the Driver Station, and peaks are tracked across reads so a dip between two reads
 * still lands in {@link #getMinVolts()}.
 *
 * <h2>What the numbers mean</h2>
 * <ul>
 *   <li>Voltage is the lowest input voltage reported by any hub — the Control Hub reads the
 *       battery directly, so with one hub this is just the battery.</li>
 *   <li>Current is the sum over hubs of total module current: motors, servos, and the hub's
 *       own logic. It is the whole robot's draw, not just the drivetrain.</li>
 *   <li>Watts is voltage times current. Handy for comparing drivetrain configurations.</li>
 * </ul>
 *
 * <p>The thresholds below are practical starting points for spotting a bad battery, not
 * official limits.
 */
public class PowerMonitor {

    /**
     * Milliseconds between readings. Each read is several ADC round trips, so this trades
     * display freshness against loop time. 100 ms (10 Hz) is cheap and plenty to read.
     */
    public static final long READ_PERIOD_MS = 100;

    /**
     * Below this, under load, the battery is getting tired, inches toward unpredictable motor
     * behavior, and should be swapped before a match. Volts.
     */
    public static final double SAG_WARN_V = 11.0;

    /**
     * Below this the hub is close to resetting and anything the robot does is suspect. Volts.
     * A REV hub can hold on lower than this, but nothing above it is still trustworthy.
     */
    public static final double BROWNOUT_V = 9.0;

    private final List<LynxModule> hubs = new ArrayList<>();
    private final List<VoltageSensor> voltageSensors = new ArrayList<>();
    private final ElapsedTime sinceRead = new ElapsedTime();

    private String[] motorNames = new String[0];
    private DcMotorEx[] motors = new DcMotorEx[0];
    private double[] motorAmps = new double[0];
    private double[] motorPeakAmps = new double[0];

    /** True when {@link #update()} reads the motors itself; false when the caller feeds them. */
    private boolean selfReadMotors = true;

    private double volts = 0.0;
    private double amps = 0.0;
    private double minVolts = Double.POSITIVE_INFINITY;
    private double peakAmps = 0.0;
    private double peakWatts = 0.0;
    private boolean read = false;

    public PowerMonitor(HardwareMap hardwareMap) {
        hubs.addAll(hardwareMap.getAll(LynxModule.class));
        // Fallback voltage source. A hub is the only thing that reports current, so if there
        // is no hub the current readings stay at zero rather than pretending.
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltageSensors.add(sensor);
        }
        // Force the first update() to actually read.
        sinceRead.reset();
        read = false;
    }

    /**
     * Registers drive motors and reads their current here, on the same cadence as the hub, so
     * adding four motors costs four more ADC reads every {@link #READ_PERIOD_MS} rather than
     * four more per loop. Nulls are allowed and reported as missing.
     *
     * <p>Use this in an OpMode that does not already read motor current — the loop stays fast,
     * at the cost of sampling peaks only 10 times a second. For the highest peak accuracy use
     * {@link #trackMotors} instead.
     *
     * @param names configuration names, same order as {@code motors}
     */
    public void watchMotors(String[] names, DcMotorEx[] motors) {
        registerMotors(names, motors);
        this.selfReadMotors = true;
    }

    /**
     * Registers drive motors whose current the caller reads itself and hands over with
     * {@link #recordMotorAmps}. Nothing here reads them, so there is no extra ADC traffic.
     *
     * <p>Use this in an OpMode that already reads motor current every loop: feeding those
     * values in catches peaks at the full loop rate instead of at
     * {@link #READ_PERIOD_MS}, which matters because a stall spike can be shorter than one
     * read period. {@link #report} then leaves out the live amps lines, assuming the caller
     * already shows them, and prints only the peaks.
     */
    public void trackMotors(String[] names, DcMotorEx[] motors) {
        registerMotors(names, motors);
        this.selfReadMotors = false;
    }

    private void registerMotors(String[] names, DcMotorEx[] motors) {
        this.motorNames = names.clone();
        this.motors = motors.clone();
        this.motorAmps = new double[motors.length];
        this.motorPeakAmps = new double[motors.length];
    }

    /**
     * Hands over a motor current the caller just read, for peak tracking. Only meaningful
     * after {@link #trackMotors}. Out-of-range indices are ignored.
     *
     * @param index position in the arrays passed to {@link #trackMotors}
     * @param amps  current in amps
     */
    public void recordMotorAmps(int index, double amps) {
        if (index < 0 || index >= motorAmps.length) {
            return;
        }
        motorAmps[index] = amps;
        motorPeakAmps[index] = Math.max(motorPeakAmps[index], amps);
    }

    /** True if anything can report voltage. False means nothing here will show real numbers. */
    public boolean isAvailable() {
        return !hubs.isEmpty() || !voltageSensors.isEmpty();
    }

    /** True if at least one hub is present, which is what current readings need. */
    public boolean hasCurrentSource() {
        return !hubs.isEmpty();
    }

    /**
     * Refreshes the readings if {@link #READ_PERIOD_MS} has elapsed. Call every loop; it
     * decides for itself whether to spend the time.
     *
     * @return true if this call took a fresh reading
     */
    public boolean update() {
        if (read && sinceRead.milliseconds() < READ_PERIOD_MS) {
            return false;
        }
        sinceRead.reset();
        read = true;

        volts = readVolts();
        amps = readAmps();

        if (volts > 0.0) {
            minVolts = Math.min(minVolts, volts);
        }
        peakAmps = Math.max(peakAmps, amps);
        peakWatts = Math.max(peakWatts, volts * amps);

        if (selfReadMotors) {
            for (int i = 0; i < motors.length; i++) {
                if (motors[i] == null) {
                    continue;
                }
                recordMotorAmps(i, motors[i].getCurrent(CurrentUnit.AMPS));
            }
        }
        return true;
    }

    /**
     * Lowest input voltage across hubs, else whatever a voltage sensor says. Volts.
     *
     * <p>The voltage sensors are a fallback, not an addition: a hub's voltage sensor is a
     * separate device object that answers with its own ADC request for the same battery, so
     * reading both would double the cost for no new information.
     */
    private double readVolts() {
        double lowest = Double.POSITIVE_INFINITY;
        if (!hubs.isEmpty()) {
            for (LynxModule hub : hubs) {
                lowest = Math.min(lowest, hub.getInputVoltage(VoltageUnit.VOLTS));
            }
        } else {
            for (VoltageSensor sensor : voltageSensors) {
                lowest = Math.min(lowest, sensor.getVoltage());
            }
        }
        // A disconnected hub can report 0; that is not a real "lowest" reading.
        return Double.isFinite(lowest) && lowest > 0.0 ? lowest : 0.0;
    }

    /** Total current across hubs, amps. Zero if there is no hub to ask. */
    private double readAmps() {
        double total = 0.0;
        for (LynxModule hub : hubs) {
            total += hub.getCurrent(CurrentUnit.AMPS);
        }
        return total;
    }

    public double getVolts() {
        return volts;
    }

    public double getAmps() {
        return amps;
    }

    public double getWatts() {
        return volts * amps;
    }

    /** Lowest voltage seen since the OpMode started, or since {@link #resetPeaks()}. */
    public double getMinVolts() {
        return Double.isFinite(minVolts) ? minVolts : 0.0;
    }

    public double getPeakAmps() {
        return peakAmps;
    }

    public double getPeakWatts() {
        return peakWatts;
    }

    /** Per-motor current from the last read, amps. Empty unless motors were registered. */
    public double[] getMotorAmps() {
        return motorAmps.clone();
    }

    /**
     * Highest current each motor has reached, amps, in the order the motors were registered.
     * Empty unless motors were registered. Only as good as the sampling: see
     * {@link #watchMotors} and {@link #trackMotors}.
     */
    public double[] getMotorPeakAmps() {
        return motorPeakAmps.clone();
    }

    /** Highest current one motor has reached, amps, or 0 for an unregistered index. */
    public double getMotorPeakAmps(int index) {
        return (index < 0 || index >= motorPeakAmps.length) ? 0.0 : motorPeakAmps[index];
    }

    /** Clears the min/peak history, motor peaks included. Useful before a specific maneuver. */
    public void resetPeaks() {
        minVolts = Double.POSITIVE_INFINITY;
        peakAmps = 0.0;
        peakWatts = 0.0;
        for (int i = 0; i < motorPeakAmps.length; i++) {
            motorPeakAmps[i] = 0.0;
        }
    }

    /** OK, SAG, or BROWNOUT, based on the lowest voltage seen so far. */
    public String getStatus() {
        if (!isAvailable()) {
            return "NO SENSOR";
        }
        double low = getMinVolts();
        if (low <= 0.0) {
            return "no reading";
        }
        if (low < BROWNOUT_V) {
            return "BROWNOUT";
        }
        if (low < SAG_WARN_V) {
            return "SAG";
        }
        return "OK";
    }

    /** True if the voltage is below the sag threshold right now. */
    public boolean isSagging() {
        return volts > 0.0 && volts < SAG_WARN_V;
    }

    /**
     * Writes the standard power block through {@code put}, so the caller decides where it
     * goes. Pass {@code telemetry::addData} for the Driver Station, or a lambda that also
     * stuffs it into a {@link com.acmerobotics.dashboard.telemetry.TelemetryPacket} to make it
     * graphable. Values stay numbers so the dashboard can graph them.
     */
    public void report(BiConsumer<String, Object> put) {
        if (!isAvailable()) {
            put.accept("power", "NO SENSOR");
            return;
        }

        put.accept("battery (V)", round2(volts));
        put.accept("battery min (V)", round2(getMinVolts()));
        put.accept("power status", getStatus());

        if (hasCurrentSource()) {
            put.accept("draw (A)", round2(amps));
            put.accept("draw peak (A)", round2(peakAmps));
            put.accept("draw (W)", round1(getWatts()));
            put.accept("draw peak (W)", round1(peakWatts));
        } else {
            put.accept("draw (A)", "no hub");
        }

        for (int i = 0; i < motorNames.length && i < motorAmps.length; i++) {
            if (motors[i] == null) {
                // In trackMotors mode the caller prints its own missing-motor line, so adding
                // another one here would just duplicate it.
                if (selfReadMotors) {
                    put.accept(motorNames[i] + " amps", "NOT FOUND");
                }
                continue;
            }
            if (selfReadMotors) {
                put.accept(motorNames[i] + " amps", round2(motorAmps[i]));
            }
            put.accept(motorNames[i] + " amps peak", round2(motorPeakAmps[i]));
        }
    }

    private static double round1(double v) {
        return Math.round(v * 10.0) / 10.0;
    }

    private static double round2(double v) {
        return Math.round(v * 100.0) / 100.0;
    }
}
