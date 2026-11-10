# FTC Robotics — Java + Road Runner

> **Running the robot at a match?** Read [COMPETITION.md](COMPETITION.md): how to install,
> config names (`FL` `FR` `BL` `BR` `pinpoint`), which OpModes to pick, and a pre-match checklist.

ByteForge Robotics' code for the 2026-27 FTC season, **BIOBUZZ presented by RTX**.
Built on the FTC SDK with Road Runner 1.0 and FTC Dashboard, plus our own drive,
odometry, vision and power subsystems. Part 1 and Part 2 below are a reference for
the FTC SDK and Road Runner.

| Component | Version |
|---|---|
| FTC SDK | 12.0.0 |
| Road Runner | `core`/`actions` 1.0.1, `ftc` 0.1.25 |
| FTC Dashboard | 0.6.0 |
| Android Gradle Plugin | 9.3.2 |
| Gradle | 9.5.0 |

## Branches

| Branch | Use it for |
|---|---|
| `competition` | What goes on the robot at matches. Only tested, working code. |
| `test` | Day-to-day work. New code lands here first. |
| `main` | Stable history. `test` merges in once it has run on the robot. |

Every push is built by GitHub Actions (the **Build** check on GitHub). A red X means
that commit doesn't compile, so don't deploy it.

---

## Contents

**Getting started**
- [Setup](#setup)
- [Project layout](#project-layout)
- [Hardware configuration names](#hardware-configuration-names)
- [OpModes](#opmodes)
- [Road Runner status](#road-runner-status)

**Part 1 — FTC SDK reference**
- [How an OpMode runs](#how-an-opmode-runs)
- [OpMode annotations](#opmode-annotations)
- [The hardware map](#the-hardware-map)
- [Motors](#motors)
- [Servos](#servos)
- [The gamepad](#the-gamepad)
- [Telemetry](#telemetry)
- [The IMU](#the-imu)
- [Utility classes](#utility-classes)
- [Common FTC mistakes](#common-ftc-mistakes)

**Part 2 — Road Runner reference**
- [The coordinate system](#the-coordinate-system)
- [Core geometry types](#core-geometry-types)
- [The drive class](#the-drive-class)
- [Building trajectories](#building-trajectories)
- [Heading modes — the thing everyone gets wrong](#heading-modes--the-thing-everyone-gets-wrong)
- [Actions](#actions)
- [Writing your own Action](#writing-your-own-action)
- [Markers — doing things mid-path](#markers--doing-things-mid-path)
- [Speed limits on one trajectory](#speed-limits-on-one-trajectory)
- [Chaining trajectories with fresh()](#chaining-trajectories-with-fresh)
- [Tuning, start to finish](#tuning-start-to-finish)
- [FTC Dashboard](#ftc-dashboard)
- [Road Runner troubleshooting](#road-runner-troubleshooting)

**Reference**
- [Competition day notes](#competition-day-notes)
- [Updating this project](#updating-this-project)

---

## Setup

1. **Install Android Studio** (Ladybug or newer). It brings its own JDK and the
   Android SDK, which is what actually builds this. If you build from the
   terminal, use JDK 17 or 21. The Android plugin doesn't support every newer Java.
2. Clone the repo and open the top folder in Android Studio (`File > Open`).
3. Switch to the branch you want (see [Branches](#branches)).
4. Wait for the first Gradle sync. It downloads ~1 GB and takes a while.
5. Plug in the Control Hub over USB-C (or join its Wi-Fi), pick it in the
   device dropdown, and press Run.

Everything we write goes in `TeamCode`. Leave `FtcRobotController` alone. It's
FIRST's code, and editing it makes SDK updates painful.

## Project layout

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── competition/                  match code (see COMPETITION.md)
│   ├── CompTeleOp.java           mecanum TeleOp with slow mode
│   └── CompAutoLeave.java        drives off the wall for LEAVE
├── subsystems/                   our robot code, shared by every OpMode
│   ├── Drivetrain.java           the four mecanum wheels; motor directions live here
│   ├── Odometry.java             Pinpoint dead wheels: x, y, heading
│   ├── DriveToPoint.java         drive in a straight line to a field point
│   ├── PowerMonitor.java         battery voltage, current draw, sag warnings
│   ├── LimelightController.java  Limelight 3A AprilTag tracking
│   └── HuskyLensController.java  HuskyLens POLLEN tracking
├── test/                         pit and tuning OpModes (never in a match)
├── tuning/                       Road Runner tuning OpModes
├── MecanumDrive.java             Road Runner mecanum drive (not tuned yet)
├── PinpointLocalizer.java        Road Runner localizers: pick one in MecanumDrive
├── TwoDeadWheelLocalizer.java
├── ThreeDeadWheelLocalizer.java
├── OTOSLocalizer.java
├── TankDrive.java                Road Runner tank drive (unused, we're mecanum)
├── Drawing.java                  draws the robot on the Dashboard field
└── messages/                     log records for Dashboard flight recording
```

**Where to put new code:** a mechanism (intake, launcher...) gets a class in
`subsystems/` that takes a `HardwareMap` in its constructor, like `Drivetrain`.
OpModes stay thin and just call subsystems. That way TeleOp and auto share the
same code, and a fix in one place fixes both.

**`DO NOT TOUCH` comments** mark code the robot depends on in matches: config
names, motor and pod directions, tuned gains, the match OpModes and the SDK
versions. Don't change those without testing on the real robot, and ask Ryan
first.

## Hardware configuration names

The code looks hardware up **by name**, so the configuration on the Driver
Station has to match these strings exactly — case and all.

| Name | What | Port | Used by |
|---|---|---|---|
| `FL` `FR` `BL` `BR` | drive motors | motor | everything that drives |
| `pinpoint` | goBILDA Pinpoint, two dead wheels | I2C | `Odometry`, auto, drive tests |
| `Limelight` | Limelight 3A | USB | Limelight Test, Drive Test |
| `huskylens` | DFRobot HuskyLens | I2C | HuskyLens Test |
| `imu` | Control Hub built-in IMU | built in | Limelight Test, Road Runner |

Motor directions are set in **one place for our code**: the `REVERSE_*` flags in
`subsystems/Drivetrain.java`. (`DriveTest` and `Drive Motor Test Final` still keep
their own copies so they can be flipped live from the Dashboard. If you change one,
change them all.)

## OpModes

| Name on Driver Station | Type | What it's for |
|---|---|---|
| **Comp TeleOp** | TeleOp | Match driving. |
| **Comp Auto: Leave** | Auto | Match auto: LEAVE, with a timer fallback. |
| Drive Test | TeleOp | Drive plus Pinpoint, Limelight and battery readouts. |
| Drive Motor Test | TeleOp | One motor at a time: wiring, direction, encoders, current. |
| Drive Motor Test Final | TeleOp | Lean drive test with Dashboard graphs. |
| Auto Test | Auto | Checks the dead wheels, then drives to a point. |
| Limelight Test | TeleOp | Calibrating the Limelight mount. |
| HuskyLens Test | TeleOp | HuskyLens POLLEN tracking readout. |
| Road Runner tuning (several) | both | Only once you're tuning Road Runner. See below. |

## Road Runner status

**Not tuned yet.** `MecanumDrive` still has the quickstart defaults: no motor
directions set, `inPerTick = 1`, `kS`/`kV` = 0, and it uses the drive encoders
(`DriveLocalizer`) instead of the Pinpoint. Road Runner paths and the tuning
OpModes **won't drive correctly** until that's done. Our own code (`Drivetrain`,
`DriveToPoint`) doesn't use Road Runner and works now.

To switch Road Runner over: in `MecanumDrive`, set the motor directions to match
`Drivetrain`, change the localizer to
`new PinpointLocalizer(hardwareMap, PARAMS.inPerTick, pose)`, then follow
[Tuning, start to finish](#tuning-start-to-finish).

---

# Part 1 — FTC SDK reference

## How an OpMode runs

There are two base classes. **Use `LinearOpMode`** — it reads top to bottom and
is far easier to reason about.

### LinearOpMode

Your code is one method. It runs on its own thread, start to finish.

```java
@TeleOp(name = "My OpMode", group = "drive")
public class MyOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // ── INIT ── runs when the driver presses INIT
        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");

        waitForStart();           // blocks until PLAY is pressed
        if (isStopRequested()) return;

        // ── RUNNING ── after PLAY
        while (opModeIsActive()) {
            arm.setPower(gamepad1.left_stick_y);
            telemetry.addData("power", arm.getPower());
            telemetry.update();
        }
        // falls out of the loop when STOP is pressed
    }
}
```

| Method | Does |
|---|---|
| `waitForStart()` | Blocks until PLAY |
| `opModeIsActive()` | `true` while running; `false` after STOP |
| `isStopRequested()` | `true` once STOP is pressed |
| `opModeInInit()` | `true` during init, before PLAY — loop on this to show sensor readings while waiting |
| `idle()` | Yields to other threads inside a tight loop |
| `sleep(ms)` | Blocking wait. Avoid in TeleOp; it makes the robot unresponsive |
| `terminateOpModeNow()` | Stops the OpMode immediately |

### OpMode (iterative)

The SDK calls your methods for you. You'll see this in older sample code.

| Method | When |
|---|---|
| `init()` | Once, on INIT |
| `init_loop()` | Repeatedly, between INIT and PLAY |
| `start()` | Once, on PLAY |
| `loop()` | Repeatedly, after PLAY |
| `stop()` | Once, on STOP |

The catch: you can't use local variables across calls, so every piece of state
becomes a field. That's why `LinearOpMode` is the better default.

## OpMode annotations

An OpMode only shows up on the Driver Station if it's annotated.

```java
@TeleOp(name = "Mecanum TeleOp", group = "drive")
@Autonomous(name = "Blue Left", group = "auto", preselectTeleOp = "Mecanum TeleOp")
@Disabled   // hides it without deleting it
```

- `name` — what shows on the Driver Station. Defaults to the class name.
- `group` — sorts OpModes into headings in the dropdown. Group your autos by
  alliance (`"blue"`, `"red"`) and it saves fumbling between matches.
- `preselectTeleOp` — auto-selects that TeleOp when this auto finishes. Small
  thing, genuinely useful during a match.

## The hardware map

`hardwareMap` turns a configuration name into an object you can control.

```java
DcMotorEx motor  = hardwareMap.get(DcMotorEx.class, "leftFront");
Servo claw       = hardwareMap.get(Servo.class, "claw");
IMU imu          = hardwareMap.get(IMU.class, "imu");
ColorSensor color = hardwareMap.get(ColorSensor.class, "color");
TouchSensor limit = hardwareMap.get(TouchSensor.class, "limit");
```

Ask for `DcMotorEx` rather than `DcMotor` — it's the same motor plus velocity
control, current draw, and PIDF access. There's no downside.

Look everything up **during init**, never inside the loop. A `hardwareMap.get()`
per iteration is slow and pointless.

## Motors

```java
DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "arm");

motor.setDirection(DcMotorSimple.Direction.REVERSE);
motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

motor.setPower(0.5);                    // -1.0 to 1.0
int ticks = motor.getCurrentPosition();
double vel = motor.getVelocity();       // ticks/sec, DcMotorEx only
double amps = motor.getCurrent(CurrentUnit.AMPS);
```

### Run modes

| Mode | Behavior |
|---|---|
| `RUN_WITHOUT_ENCODER` | `setPower` is raw voltage. What Road Runner uses for the drive |
| `RUN_USING_ENCODER` | `setPower` is a fraction of max *speed*, held by a velocity PID |
| `RUN_TO_POSITION` | Drives to `setTargetPosition()` and holds. Set a target and a power first |
| `STOP_AND_RESET_ENCODER` | Zeroes the count. Must set another mode afterward |

`RUN_TO_POSITION` in full:

```java
motor.setTargetPosition(1500);
motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
motor.setPower(0.8);                    // speed cap, not direction
while (opModeIsActive() && motor.isBusy()) {
    telemetry.addData("pos", motor.getCurrentPosition());
    telemetry.update();
}
motor.setPower(0);
```

### Zero power behavior

- `BRAKE` — shorts the motor so it resists movement. Use for lifts and arms.
- `FLOAT` — coasts. Use where a hard stop would rack the mechanism.

### Encoder counts

Ticks per revolution depend on the gearbox, not the motor:

| Motor | Ticks/rev at the output |
|---|---|
| goBILDA 5202/5203 435 RPM | 384.5 |
| goBILDA 5202/5203 312 RPM | 537.7 |
| goBILDA 5202/5203 223 RPM | 751.8 |
| REV HD Hex 40:1 | 1120 |
| REV Core Hex | 288 |

Check the spec sheet for your exact motor — mixing these up produces a mechanism
that overshoots by a factor of two and nobody can see why.

## Servos

```java
Servo claw = hardwareMap.get(Servo.class, "claw");
claw.setPosition(0.5);              // 0.0 to 1.0, an absolute angle
claw.setDirection(Servo.Direction.REVERSE);
claw.scaleRange(0.2, 0.8);          // remap 0..1 onto a safe sub-range
```

`scaleRange` is worth knowing: it stops the code from ever commanding a position
that would grind the servo against a hard stop.

A **continuous rotation servo** is a different type — it takes a *speed*, not a
position:

```java
CRServo intake = hardwareMap.get(CRServo.class, "intake");
intake.setPower(1.0);               // -1.0 to 1.0, 0 is stop
```

## The gamepad

### Sticks and triggers

| Field | Range | Note |
|---|---|---|
| `left_stick_x` `left_stick_y` | -1.0 to 1.0 | **y is negative when pushed forward** |
| `right_stick_x` `right_stick_y` | -1.0 to 1.0 | same |
| `left_trigger` `right_trigger` | 0.0 to 1.0 | analog |

The inverted y-axis catches everyone once. Negate it: `-gamepad1.left_stick_y`.

### Buttons

`a` `b` `x` `y` · `dpad_up` `dpad_down` `dpad_left` `dpad_right` ·
`left_bumper` `right_bumper` · `left_stick_button` `right_stick_button` ·
`start` `back` `guide` `options` `share` · `touchpad`

PlayStation aliases for the same four face buttons: `cross`(a) `circle`(b)
`square`(x) `triangle`(y).

### Rising-edge detection

A button field is `true` for *every loop* it's held — so `if (gamepad1.a) count++`
counts to 50 in one press. For "did this just get pressed", the SDK gives you:

```java
if (gamepad1.aWasPressed()) { ... }         // first loop the button goes down
if (gamepad1.rightBumperWasReleased()) { }  // first loop it comes back up
```

Available for every button: `aWasPressed()`, `bWasPressed()`, `xWasPressed()`,
`yWasPressed()`, `dpadUpWasPressed()`, `leftBumperWasPressed()`,
`optionsWasPressed()`, `backWasPressed()`, `startWasPressed()`,
`leftTriggerWasPressed()`, and each one's `WasReleased()` twin.

> Note that `start`+`a` / `start`+`b` are how drivers bind their controllers, so
> avoid `start` combinations in your own bindings.

### Rumble

```java
gamepad1.rumble(500);               // milliseconds
gamepad1.rumbleBlips(3);
```

Good for endgame warnings — the driver feels it without looking away.

## Telemetry

```java
telemetry.addData("heading", "%.1f deg", heading);   // printf-style
telemetry.addData("position", pos);                  // plain value
telemetry.addLine("ready to score");
telemetry.update();                                  // nothing shows without this
```

`update()` is what actually sends it. Call it once at the end of your loop —
calling it per line just wastes bandwidth.

To mirror telemetry to both the Driver Station and FTC Dashboard:

```java
telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
```

## The IMU

The Control Hub has a built-in IMU. Road Runner's drive class sets it up for you,
but if you need it directly:

```java
IMU imu = hardwareMap.get(IMU.class, "imu");
imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
        RevHubOrientationOnRobot.LogoFacingDirection.UP,
        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
imu.resetYaw();
```

Those two direction constants must describe how the hub is **physically bolted
to the robot** — which way the REV logo points, and which way the USB port
faces. Get them wrong and every heading is wrong. The same values go in
`MecanumDrive.Params`.

## Utility classes

```java
ElapsedTime timer = new ElapsedTime();
timer.reset();
if (timer.seconds() > 2.0) { ... }
timer.milliseconds();

double safe = Range.clip(value, -1.0, 1.0);
double mapped = Range.scale(input, 0, 1, -100, 100);
```

## Common FTC mistakes

| Symptom | Cause |
|---|---|
| Crash the instant you press INIT | Config name doesn't match `hardwareMap.get()` |
| Robot drives backward / spins | Motor direction not reversed on one side |
| A button "fires" dozens of times | Used `gamepad1.a` where you wanted `aWasPressed()` |
| Robot drifts forward on its own | Stick deadzone — apply a small threshold |
| Telemetry blank | Forgot `telemetry.update()` |
| TeleOp feels laggy | `sleep()` inside the loop, or telemetry spam |
| Encoder reads 0 forever | Encoder cable in the wrong port, or mode never left `STOP_AND_RESET_ENCODER` |
| Mechanism overshoots ~2x | Wrong ticks-per-revolution for that gearbox |

---

# Part 2 — Road Runner reference

Road Runner drives the robot along a **planned path** instead of "forward for 1.4
seconds". It knows where the robot is (localization), where it should be
(trajectory), and corrects the difference (feedback).

Official docs: <https://rr.brott.dev/docs/v1-0/>

## The coordinate system

- Units are **inches**. Angles are **radians** — use `Math.toRadians(90)`.
- The origin is the **center of the field**. The field is 144" × 144", so every
  coordinate is between -72 and +72.
- **+x** points away from the audience, **+y** points to the left.
- Heading 0 faces +x, and increases **counter-clockwise**. So 90° faces +y.

```
        +y (left)
         │
         │
─────────┼───────── +x  (away from audience, heading = 0)
         │
         │
```

A robot at `(-36, -60, toRadians(90))` sits back-right, pointing across the field.

## Core geometry types

```java
new Vector2d(x, y)                    // a position
new Rotation2d(cos, sin)              // a heading
Rotation2d.exp(radians)               // build one from an angle — use this
new Pose2d(x, y, headingRadians)      // position + heading
new Pose2d(vector, headingRadians)
new PoseVelocity2d(vector, angVel)    // a velocity command
```

Reading them back:

```java
pose.position.x
pose.position.y
pose.heading.toDouble()               // radians
Math.toDegrees(pose.heading.toDouble())
```

`Rotation2d` also does rotation maths, which is how field-centric driving works:

```java
Vector2d robotFrame = pose.heading.inverse().times(fieldFrameInput);
```

## The drive class

`MecanumDrive` is yours to edit — it's in `TeamCode`, not the SDK.

```java
MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

drive.updatePoseEstimate();               // call every loop in TeleOp
drive.localizer.getPose();                // where the robot thinks it is
drive.localizer.setPose(newPose);         // override the estimate
drive.setDrivePowers(new PoseVelocity2d(new Vector2d(fwd, strafe), turn));
drive.actionBuilder(startPose);           // start building a trajectory
MecanumDrive.PARAMS                       // the tuned constants
```

### Carrying the pose from auto into TeleOp

The estimate resets when an OpMode ends. To keep it, stash it in a static:

```java
public class PoseStore { public static Pose2d pose = new Pose2d(0, 0, 0); }

// end of auto:
PoseStore.pose = drive.localizer.getPose();
// start of TeleOp:
MecanumDrive drive = new MecanumDrive(hardwareMap, PoseStore.pose);
```

## Building trajectories

`actionBuilder(startPose)` returns a builder. Chain segments onto it, then
`.build()` to get an `Action`.

```java
Action traj = drive.actionBuilder(new Pose2d(-36, -60, Math.toRadians(90)))
        .lineToY(-30)
        .turn(Math.toRadians(90))
        .splineTo(new Vector2d(-12, -12), Math.toRadians(45))
        .waitSeconds(0.5)
        .build();

Actions.runBlocking(traj);
```

**Start the builder at the pose where the robot actually is.** The builder plans
from that assumption; if it's wrong, the whole path is wrong.

### Every builder method

**Straight lines** — along one axis, heading unchanged:

| Method | |
|---|---|
| `lineToX(x)` | drive to an x coordinate |
| `lineToY(y)` | drive to a y coordinate |

**Straight lines with a heading change:**

| Method | |
|---|---|
| `lineToXConstantHeading(x)` / `lineToYConstantHeading(y)` | hold current heading |
| `lineToXLinearHeading(x, h)` / `lineToYLinearHeading(y, h)` | rotate steadily to `h` |
| `lineToXSplineHeading(x, h)` / `lineToYSplineHeading(y, h)` | rotate smoothly to `h` |

**Strafes** — straight to any point:

| Method | |
|---|---|
| `strafeTo(new Vector2d(x, y))` | hold heading |
| `strafeToConstantHeading(vec)` | same, explicit |
| `strafeToLinearHeading(vec, h)` | rotate steadily to `h` on the way |
| `strafeToSplineHeading(vec, h)` | rotate smoothly to `h` on the way |

**Splines** — curved paths:

| Method | |
|---|---|
| `splineTo(vec, tangent)` | curve to a point, arriving along `tangent` |
| `splineToConstantHeading(vec, tangent)` | curve, robot keeps facing one way |
| `splineToLinearHeading(pose, tangent)` | curve, heading rotates steadily |
| `splineToSplineHeading(pose, tangent)` | curve, heading eases smoothly |

**Turns and waits:**

| Method | |
|---|---|
| `turn(radians)` | turn *by* an amount |
| `turnTo(radians)` | turn *to* an absolute heading |
| `waitSeconds(s)` | sit still |

**Control:**

| Method | |
|---|---|
| `setTangent(radians)` | set the direction the next spline leaves at |
| `setReversed(true)` | drive the following segment backwards |
| `stopAndAdd(action)` | stop, run an action to completion, continue |
| `afterTime(s, action)` | fire an action `s` seconds in, without stopping |
| `afterDisp(inches, action)` | fire an action after `inches` travelled |
| `endTrajectory()` | close the current segment |
| `fresh()` | new builder starting at this end pose |
| `build()` | produce the `Action` |

## Heading modes — the thing everyone gets wrong

Path (where the robot goes) and heading (which way it points) are **separate**.
That's the whole point of a mecanum drive, and the naming trips people up:

- **Tangent** (the plain `splineTo`, `lineToX`) — the robot faces along its
  direction of travel, like a car. The default.
- **ConstantHeading** — the robot holds whatever heading it started with and
  crab-walks the path. Use when a camera or intake has to keep facing one way.
- **LinearHeading** — heading interpolates at a constant rate from start to end.
  Predictable; can look mechanical.
- **SplineHeading** — heading eases in and out smoothly. Nicer, slightly less
  predictable about *when* it gets there.

The `tangent` argument on a spline is **not the robot's heading** — it's the
direction the *path* enters and leaves at. Getting this wrong produces a path
that loops back on itself, which is confusing until you see it drawn in the
Dashboard field view.

## Actions

An `Action` is anything with a `run()` that returns "call me again?". Trajectories
are Actions, so are subsystem movements, and they compose.

```java
import com.acmerobotics.roadrunner.ftc.Actions;   // note: the .ftc package

Actions.runBlocking(action);
```

| Type | Behavior |
|---|---|
| `SequentialAction(a, b, c)` | one after another |
| `ParallelAction(a, b, c)` | all at once; finishes when **all** are done |
| `RaceAction(a, b)` | all at once; finishes when the **first** is done |
| `SleepAction(seconds)` | wait |
| `InstantAction(() -> ...)` | run a lambda once, immediately |
| `NullAction()` | do nothing (a placeholder) |

Nest them freely:

```java
Actions.runBlocking(new SequentialAction(
        new ParallelAction(
                driveToScore,           // drive and lift at the same time
                lift.toHigh()),
        claw.open(),
        new SleepAction(0.3),
        new ParallelAction(
                driveToPark,
                lift.toDown())
));
```

`RaceAction` is the timeout pattern — whichever finishes first wins:

```java
new RaceAction(intake.runUntilFull(), new SleepAction(3.0))
```

## Writing your own Action

Return `true` to be called again next loop, `false` when finished. `run()` must
**not** block — it does one slice of work and returns. A loop or `sleep()` inside
`run()` freezes everything running in parallel with it.

```java
public class ClawOpen implements Action {
    private final Servo claw;
    private boolean initialized = false;
    private final ElapsedTime timer = new ElapsedTime();

    public ClawOpen(Servo claw) { this.claw = claw; }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (!initialized) {
            claw.setPosition(0.6);
            timer.reset();
            initialized = true;
        }
        packet.put("claw", claw.getPosition());
        return timer.seconds() < 0.4;      // true = still going
    }
}
```

For something instant, skip the class:

```java
new InstantAction(() -> claw.setPosition(0.6))
```

A fuller version would be a lift `Action`: a proportional controller that reports
position through the packet and finishes inside a tolerance.

## Markers — doing things mid-path

Start a mechanism partway through a drive without stopping:

```java
drive.actionBuilder(start)
        .afterTime(0.5, lift.toHigh())        // 0.5s after this segment starts
        .afterDisp(20, claw.open())           // after 20 inches travelled
        .splineTo(new Vector2d(0, -12), 0)
        .stopAndAdd(claw.close())             // fully stop, then run it
        .build();
```

`afterTime`/`afterDisp` overlap with the driving — that's where match time comes
from. `stopAndAdd` halts until the action finishes; use it only when the robot
genuinely must be still.

## Speed limits on one trajectory

Every drive method takes optional constraints, for when one segment needs to be
gentler than the rest:

```java
.splineTo(new Vector2d(0, -12), 0,
        new TranslationalVelConstraint(20),      // in/s
        new ProfileAccelConstraint(-20, 20))     // min, max in/s²
```

| Constraint | |
|---|---|
| `TranslationalVelConstraint(inPerSec)` | cap linear speed |
| `AngularVelConstraint(radPerSec)` | cap turn speed |
| `MinVelConstraint(List.of(a, b))` | apply the tightest of several |
| `ProfileAccelConstraint(min, max)` | cap acceleration |

Global defaults live in `MecanumDrive.PARAMS` (`maxWheelVel`, `maxProfileAccel`,
`minProfileAccel`, `maxAngVel`, `maxAngAccel`).

## Chaining trajectories with fresh()

`build()` ends a builder. To continue from where it stopped, use `fresh()` —
it hands you a new builder already at the right pose:

```java
TrajectoryActionBuilder tab = drive.actionBuilder(start)
        .splineTo(new Vector2d(0, -36), Math.toRadians(90));

Action first  = tab.build();
Action second = tab.fresh()
        .strafeTo(new Vector2d(24, -36))
        .build();
```

Useful when a decision happens between the two — reading a randomization,
picking a branch — because you build both branches during init and only choose
at run time.

## Tuning, start to finish

**Nothing follows a path until this is done.** An untuned drive will wander,
oscillate, or not move. Budget a couple of hours with a clear stretch of floor
and the field tiles if you have them.

Full guide with photos: <https://rr.brott.dev/docs/v1-0/tuning/>

**Step 0 — before anything.** In `MecanumDrive.Params`, set
`logoFacingDirection` and `usbFacingDirection` to match how the Control Hub is
mounted. Then run `MecanumMotorDirectionDebugger` and confirm all four motors
drive forward on positive power; reverse the ones that don't.

| # | OpMode | Produces | What you do |
|---|---|---|---|
| 1 | `ForwardPushTest` | `inPerTick` | Push the robot straight forward by hand, exactly 4 tiles (96"). Divide inches by reported ticks |
| 2 | `LateralPushTest` | `lateralInPerTick` | Same, pushing sideways. Mecanums strafe less efficiently, so this is smaller |
| 3 | `ForwardRampLogger` | `kS`, `kV` | Robot accelerates forward on its own. Read the fit off the Dashboard |
| 4 | `LateralRampLogger` | check | Verifies the lateral number |
| 5 | `AngularRampLogger` | `trackWidthTicks` | Robot spins up in place |
| 6 | `ManualFeedforwardTuner` | refine `kV`, set `kA` | Robot drives back and forth. Adjust live until measured tracks target |
| 7 | `ManualFeedbackTuner` | the gains | Set `axialGain`, `lateralGain`, `headingGain`. Raise until it corrects crisply, back off before it oscillates |
| 8 | `SplineTest` | confidence | Should drive a clean curve |
| 9 | `LocalizationTest` | confidence | Drive by hand; the Dashboard robot should track the real one |

The tuning OpModes appear under the **quickstart** group on the Driver Station.

Feedforward before feedback — that's the order that matters. Feedforward is the
robot's model of itself (what power produces what speed); feedback only cleans up
the leftover error. Tuning gains on top of a bad model means chasing your tail.

**Write the numbers down somewhere outside the code.** They're the most expensive
thing on the robot and a bad merge can lose them.

### Choosing a localizer

| Option | Set `MecanumDrive` to use |
|---|---|
| Drive motor encoders only | the built-in `DriveLocalizer` (default) |
| Two dead wheels + IMU | `TwoDeadWheelLocalizer` |
| Three dead wheels | `ThreeDeadWheelLocalizer` |
| goBILDA Pinpoint | `PinpointLocalizer` |
| SparkFun OTOS | `OTOSLocalizer` |

Drive encoders are free but drift when wheels slip. Dead wheels or a Pinpoint are
a large accuracy upgrade and worth it if the budget is there.

## FTC Dashboard

With the robot on, connect to its WiFi and open:

- Control Hub — <http://192.168.43.1:8080/dash>
- Phone — <http://192.168.49.1:8080/dash>

What it gives you:

- **Field view** — the robot's estimated position, live, plus the planned path.
  The fastest way to see that a trajectory is wrong.
- **Live config** — anything annotated `@Config` is editable while the robot
  runs. `MecanumDrive.PARAMS`, `Drivetrain`, `Odometry` and `DriveToPoint` are
  all exposed, so you can tune without a rebuild cycle.
- **Graphs** — plot any telemetry value over time.
- **Camera stream**.

**Not during matches.** Rule R704.D limits streaming over the robot's Wi-Fi to
the Driver Station app, and laptops must be off the robot network during a
MATCH (R704.C). The Dashboard is for the pits and practice.

Values you change in the Dashboard are **lost on restart**. Once you're happy,
copy them back into the source.

To expose your own class:

```java
@Config
public class Lift {
    public static double KP = 0.005;    // must be public static
}
```

## Road Runner troubleshooting

| Symptom | Likely cause |
|---|---|
| Robot doesn't move on a trajectory | Not tuned — `kV`/`kS` still 0 |
| Drives roughly right, drifts off | Feedback gains too low |
| Oscillates / shakes along the path | Feedback gains too high |
| Curves into the wrong place | `tangent` misunderstood — it's the path direction, not the heading |
| Path is mirrored or rotated | Wrong start pose, or IMU orientation constants wrong |
| Strafes badly, forward is fine | `lateralInPerTick` untuned |
| Position drifts over a match | Wheel slip — move to dead wheels or a Pinpoint |
| Heading right, position wrong | Localizer encoder directions reversed |
| Parallel action never finishes | An Action's `run()` always returns `true` |
| Everything freezes mid-auto | A blocking loop or `sleep()` inside `run()` |

---

# Reference

## Competition day notes

- Follow [COMPETITION.md](COMPETITION.md) for the match OpModes and checklist.
- Build and deploy **before** you're queued, never in the queue line.
- Keep a known-good commit tagged. `git tag qualifier-1` costs nothing and gives
  you something to fall back to.
- Set `preselectTeleOp` on your autos so the right TeleOp is already loaded.
- The Dashboard needs the robot's WiFi — you can't use it while the field
  controls the robot. Tune between matches.
- If an OpMode crashes on init at a meet, it's a config name mismatch nine times
  out of ten. Check the Driver Station config first.

## Updating this project

**New FTC SDK release:** bump the versions in `build.dependencies.gradle`, and
copy the new `FtcRobotController` module in from the release so the app version
matches the libraries.

**New Road Runner release:** versions are in `TeamCode/build.gradle`. Check
<https://github.com/acmerobotics/road-runner-quickstart> for changes to the drive
classes that go with it.

**Useful links**

- Road Runner docs — <https://rr.brott.dev/docs/v1-0/>
- Road Runner tuning — <https://rr.brott.dev/docs/v1-0/tuning/>
- FTC SDK docs — <https://ftc-docs.firstinspires.org/>
- SDK javadoc — <https://javadoc.io/doc/org.firstinspires.ftc>
- FTC Dashboard — <https://acmerobotics.github.io/ftc-dashboard/>
- `README_FTC_SDK.md` — FIRST's original README and release notes
