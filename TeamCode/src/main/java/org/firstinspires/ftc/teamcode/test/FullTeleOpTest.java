package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Field;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MatchSetup;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.PowerMonitor;
import org.firstinspires.ftc.teamcode.subsystems.ShotSolver;

/**
 * The whole robot in one TeleOp: drive, intake, and the shooter aimed by {@link ShotSolver}.
 * Practice only. When it works, the good parts move into Comp TeleOp.
 *
 * <h2>INIT (gamepad 1)</h2>
 * <ul>
 *   <li>B: RED, X: BLUE. Dpad left / right: AUDIENCE or FAR start spot.</li>
 *   <li>Y: use the pose auto saved, or ignore it (only if auto saved one).</li>
 * </ul>
 * Put the robot on the chosen start spot exactly as {@link Field} describes it.
 *
 * <h2>Gamepad 1 - driver</h2>
 * <ul>
 *   <li>Left stick: drive and strafe. Right stick X: turn.</li>
 *   <li>Hold left bumper or right trigger: slow mode. Dpad up / down: normal speed +/- 0.1.</li>
 *   <li>Hold left trigger: auto-turn to face the goal (ODOMETRY mode only).</li>
 * </ul>
 *
 * <h2>Gamepad 2 - operator</h2>
 * <ul>
 *   <li>Hold right trigger: intake in. Hold left trigger: intake out.</li>
 *   <li>A: spin up and aim on / off. Both gamepads rumble when it's ready to shoot.</li>
 *   <li>Hold right bumper: shoot (feeds only while ready and aimed).</li>
 *   <li>Left bumper: the HIVE just tipped, switch the target to the other cell. Telemetry shows
 *   which cell it's aiming at. Starts on red AUDIENCE / blue FAR, the cells that start up.</li>
 *   <li>B: stop shooter and intake.</li>
 *   <li>X: switch distance mode, MANUAL or ODOMETRY.</li>
 *   <li>Dpad up / down: MANUAL distance +/- 0.10 m.</li>
 *   <li>Dpad right / left: RPM trim +/- 50 (both modes, for fixing short or long shots live).</li>
 *   <li>Y: RPM trim back to 0.</li>
 *   <li>Back: call the hood's current spot zero.</li>
 * </ul>
 *
 * <h2>Distance modes</h2>
 * <ul>
 *   <li><b>MANUAL</b>: the operator dials the distance to the goal. Works with no odometry.</li>
 *   <li><b>ODOMETRY</b>: distance comes from the Pinpoint. At START the robot's position is set
 *   to the chosen start spot in field coordinates, and the goal is the up cell of the chosen
 *   alliance's HIVE (see {@link Field}). It won't let you shoot from beside or behind the cell,
 *   where the ball would hit the outside of it (G417.C).</li>
 * </ul>
 *
 * <p>Only 4 game pieces may be held at once (G407) and the robot starts with 4. Nothing counts
 * them yet, so don't intake a 5th.
 *
 * <p>Don't touch the robot after INIT: the Pinpoint calibrates then. The hood must also be at
 * its zero spot.
 */
@Config
@TeleOp(name = "Full TeleOp Test", group = "Tests")
public class FullTeleOpTest extends LinearOpMode {

    // ---- Driving ----
    public static double NORMAL_SPEED = 0.8;
    public static double SLOW_SPEED = 0.35;
    public static double DEADZONE = 0.05;

    // Goal position and height live in Field (HIVE cell geometry).

    // ---- Shooter on the robot (PLACEHOLDERS: measure on the robot) ----
    /** Where the ball leaves the hood, from the robot's center. +x forward, +y left, inches. */
    public static double SHOOTER_X_IN = 0;
    public static double SHOOTER_Y_IN = 0;
    /** Which way the shooter fires, from robot forward, CCW positive. 180 = out the back. */
    public static double SHOOTER_FACING_DEG = 0;

    // ---- Auto-aim ----
    public static double AIM_KP = 2.0;
    public static double AIM_MAX_TURN = 0.5;
    /** Counts as aimed within this many degrees. */
    public static double AIM_TOLERANCE_DEG = 2.0;

    public static double START_MANUAL_DISTANCE_M = 1.5;
    public static double RPM_TRIM_STEP = 50;

    private enum Mode { MANUAL, ODOMETRY }

    @Override
    public void runOpMode() {
        Drivetrain drive = new Drivetrain(hardwareMap);
        Odometry odometry = new Odometry(hardwareMap);
        Hood hood = new Hood(hardwareMap);
        Flywheel flywheel = new Flywheel(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        PowerMonitor power = new PowerMonitor(hardwareMap);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry.setMsTransmissionInterval(50);

        odometry.reset(); // robot must be still now
        MatchSetup setup = new MatchSetup();

        while (opModeInInit()) {
            odometry.update();
            power.update();
            telemetry.addLine("FULL TELEOP TEST - hands off the robot, hood at zero");
            setup.update(gamepad1, telemetry);
            telemetry.addData("pinpoint", odometry.getStatus());
            telemetry.addData("battery (V)", "%.2f", power.getVolts());
            missing("hood", hood.isConnected());
            missing("flywheel", flywheel.isConnected());
            missing("intake", intake.isConnected());
            for (int i = 0; i < Drivetrain.NAMES.length; i++) {
                missing(Drivetrain.NAMES[i], drive.hasMotor(i));
            }
            telemetry.update();
        }
        power.resetPeaks();

        // Put odometry in field coordinates: chosen start spot, or where auto left off.
        Field.Alliance alliance = setup.getAlliance();
        Field.Pose start = setup.startPose();
        boolean fromAuto = setup.usingSavedPose();
        odometry.setPose(start.x, start.y, start.heading);
        Field.clearSavedPose(); // used once; a later TeleOp run starts from its start spot
        Field.Cell upCell = Field.startUpCell(alliance);

        Mode mode = odometry.isReady() ? Mode.ODOMETRY : Mode.MANUAL;
        double speed = NORMAL_SPEED;
        double manualDistance = START_MANUAL_DISTANCE_M;
        double rpmTrim = 0;
        boolean armed = false;
        boolean wasReady = false;
        double lastSolvedDistance = -1, lastSolvedHeight = -1;
        ShotSolver.Shot shot = ShotSolver.solve(manualDistance, Field.CELL_OPENING_HEIGHT_M);
        ElapsedTime loopTimer = new ElapsedTime();

        try {
            while (opModeIsActive()) {
                odometry.update();
                power.update();

                // ---------------- Operator buttons ----------------
                if (gamepad2.xWasPressed()) {
                    mode = mode == Mode.MANUAL ? Mode.ODOMETRY : Mode.MANUAL;
                }
                if (gamepad2.aWasPressed()) armed = !armed;
                if (gamepad2.bWasPressed()) armed = false;
                if (gamepad2.dpadUpWasPressed()) manualDistance += 0.10;
                if (gamepad2.dpadDownWasPressed()) manualDistance = Math.max(0.3, manualDistance - 0.10);
                if (gamepad2.dpadRightWasPressed()) rpmTrim += RPM_TRIM_STEP;
                if (gamepad2.dpadLeftWasPressed()) rpmTrim -= RPM_TRIM_STEP;
                if (gamepad2.yWasPressed()) rpmTrim = 0;
                if (gamepad2.backWasPressed()) hood.zeroHere();
                if (gamepad2.leftBumperWasPressed()) upCell = upCell.other(); // HIVE tipped
                manualDistance = Math.round(manualDistance * 100) / 100.0;

                // ---------------- Where is the goal? ----------------
                Field.Pose goal = Field.goal(alliance, upCell);
                double goalHeight = Field.CELL_OPENING_HEIGHT_M;
                double heading = odometry.getHeading();
                double exitX = odometry.getX()
                        + SHOOTER_X_IN * Math.cos(heading) - SHOOTER_Y_IN * Math.sin(heading);
                double exitY = odometry.getY()
                        + SHOOTER_X_IN * Math.sin(heading) + SHOOTER_Y_IN * Math.cos(heading);
                double toGoalX = goal.x - exitX, toGoalY = goal.y - exitY;
                double odoDistanceM = Math.hypot(toGoalX, toGoalY) * 0.0254;
                double aimHeading = Math.atan2(toGoalY, toGoalX) - Math.toRadians(SHOOTER_FACING_DEG);
                double aimErrorDeg = Math.toDegrees(Field.wrap(aimHeading - heading));
                boolean odoUsable = odometry.isReady();
                if (mode == Mode.ODOMETRY && !odoUsable) {
                    mode = Mode.MANUAL; // Pinpoint dropped out: fall back so the shooter still works
                }
                double distance = mode == Mode.ODOMETRY ? odoDistanceM : manualDistance;
                // In MANUAL mode the robot can't know where it is, so that's on the drivers.
                boolean rightSide = mode == Mode.MANUAL || Field.canScoreFrom(goal, exitX, exitY);

                // Re-solve only when the answer could have changed by a useful amount.
                if (Math.abs(distance - lastSolvedDistance) > 0.01 || goalHeight != lastSolvedHeight) {
                    shot = ShotSolver.solve(distance, goalHeight);
                    lastSolvedDistance = distance;
                    lastSolvedHeight = goalHeight;
                }

                // ---------------- Drive ----------------
                if (gamepad1.dpadUpWasPressed()) speed = Math.min(1.0, speed + 0.1);
                if (gamepad1.dpadDownWasPressed()) speed = Math.max(0.2, speed - 0.1);
                speed = Math.round(speed * 10.0) / 10.0;
                boolean slow = gamepad1.left_bumper || gamepad1.right_trigger > 0.5;
                double max = slow ? SLOW_SPEED : speed;
                double forward = -deadzone(gamepad1.left_stick_y) * max;
                double left = -deadzone(gamepad1.left_stick_x) * max;
                double turn = -deadzone(gamepad1.right_stick_x) * max;
                boolean autoAim = gamepad1.left_trigger > 0.5 && mode == Mode.ODOMETRY;
                if (autoAim) {
                    turn = clip(AIM_KP * Math.toRadians(aimErrorDeg), AIM_MAX_TURN);
                }
                drive.drive(forward, left, turn);

                // ---------------- Shooter ----------------
                if (armed && shot.reachable) {
                    flywheel.setRpm(shot.rpm + rpmTrim);
                    hood.setAngleDeg(shot.angleDeg);
                } else {
                    flywheel.stop();
                }
                boolean aimed = mode == Mode.MANUAL || Math.abs(aimErrorDeg) <= AIM_TOLERANCE_DEG;
                boolean ready = armed && shot.reachable && rightSide
                        && flywheel.atSpeed() && hood.atTarget();
                if (ready && !wasReady) {
                    gamepad1.rumble(200);
                    gamepad2.rumble(200);
                }
                wasReady = ready;

                // ---------------- Intake / feed ----------------
                if (gamepad2.right_bumper && ready && aimed) {
                    intake.in(); // shoot
                } else if (gamepad2.right_trigger > 0.3) {
                    intake.in();
                } else if (gamepad2.left_trigger > 0.3) {
                    intake.out();
                } else {
                    intake.stop();
                }

                hood.update();
                flywheel.update();
                intake.update();

                // ---------------- Telemetry ----------------
                String state;
                if (!armed) state = "shooter off (gp2 A)";
                else if (!rightSide) state = "WRONG SIDE - go to the " + upCell + " side of the HIVE";
                else if (!shot.reachable) state = "OUT OF RANGE - move closer/farther";
                else if (ready && aimed) state = "READY - gp2 RB to shoot";
                else if (ready) state = "READY but not aimed (gp1 LT)";
                else state = "spinning up / aiming...";
                telemetry.addData("STATE", state);
                telemetry.addData("target", "%s cell %s  (gp2 LB after a tip)", upCell, goal);
                telemetry.addData("alliance", "%s  started %s", alliance,
                        fromAuto ? "from auto's pose" : "at " + setup.getStartSpot());
                telemetry.addData("mode", "%s  distance %.2f m%s", mode, distance,
                        mode == Mode.ODOMETRY ? String.format("  aim err %.1f deg", aimErrorDeg) : "");
                telemetry.addData("shot", shot.toString());
                telemetry.addData("flywheel", "%.0f / %.0f rpm  (trim %+.0f)",
                        flywheel.getRpm(), flywheel.getTargetRpm(), rpmTrim);
                if (hood.isFaulted()) {
                    telemetry.addLine("!! HOOD RUNAWAY - power cut. Flip Hood.ENCODER_REVERSED, push the hood down, then zero it (Back).");
                }
                telemetry.addData("hood", "%.1f / %.1f deg", hood.getAngleDeg(), shot.angleDeg);
                telemetry.addData("intake", "%s%.0f rpm", intake.isJammed() ? "JAMMED  " : "", intake.getRpm());
                telemetry.addData("drive speed", slow ? "SLOW" : String.valueOf(speed));
                telemetry.addData("pose (in, deg)", "%.1f, %.1f, %.1f  pinpoint %s",
                        odometry.getX(), odometry.getY(), Math.toDegrees(heading), odometry.getStatus());
                telemetry.addData("battery (V)", "%.2f   low %.2f", power.getVolts(), power.getMinVolts());
                telemetry.addData("loop (ms)", "%.1f", loopTimer.milliseconds());
                loopTimer.reset();
                missing("hood", hood.isConnected());
                missing("flywheel", flywheel.isConnected());
                missing("intake", intake.isConnected());
                telemetry.update();

                TelemetryPacket packet = new TelemetryPacket();
                packet.put("flywheel rpm", flywheel.getRpm());
                packet.put("flywheel target", flywheel.getTargetRpm());
                packet.put("hood deg", hood.getAngleDeg());
                packet.put("hood target deg", shot.angleDeg);
                packet.put("distance m", distance);
                packet.put("aim error deg", aimErrorDeg);
                packet.put("battery V", power.getVolts());
                dashboard.sendTelemetryPacket(packet);
            }
        } finally {
            drive.stop();
            flywheel.stop();
            flywheel.update();
            intake.stop();
            intake.update();
            hood.stop();
        }
    }

    private void missing(String name, boolean connected) {
        if (!connected) {
            telemetry.addData("!! " + name, "NOT FOUND - check config");
        }
    }

    private static double deadzone(double v) {
        return Math.abs(v) < DEADZONE ? 0.0 : v;
    }

    private static double clip(double v, double limit) {
        return Math.max(-limit, Math.min(limit, v));
    }
}
