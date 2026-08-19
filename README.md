# FTC Robotics — Java + Road Runner

An FTC robot controller project with Road Runner 1.0 and FTC Dashboard already
wired in, plus a working TeleOp, autonomous, and subsystem to build from.

## What's in here

| Version | |
|---|---|
| FTC SDK | 11.2.1 (July 2026) |
| Road Runner | core/actions 1.0.1, ftc 0.1.25 |
| FTC Dashboard | 0.5.1 |
| Android Gradle Plugin | 8.13.2 |
| Gradle | 9.1.0 |

Built from the FTC SDK v11.2.1 release with the Road Runner quickstart's drive
code layered on top, so it starts on the current SDK rather than the older one
the quickstart ships against.

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── MecanumDrive.java          Road Runner mecanum drive — tune the constants here
├── TankDrive.java             tank alternative (delete if you're on mecanum)
├── *Localizer.java            dead wheel / Pinpoint / OTOS options
├── tuning/                    the tuning OpModes — run these first
├── teleop/MecanumTeleOp.java  field-centric driving, slow mode
├── auto/SampleAutonomous.java trajectory + subsystem, running in parallel
└── subsystems/Lift.java       how to write a Road Runner Action subsystem
```

## Setup

1. **Install Android Studio** (Ladybug or newer). It brings its own JDK and the
   Android SDK, which is what actually builds this. Don't build from the
   terminal — the system JDK here is Java 26, and the Android plugin needs 17 or
   21.
2. Open this folder in Android Studio: `File > Open`, pick `ftc-robotics`.
3. Wait for the first Gradle sync. It downloads ~1 GB and takes a while.
4. Plug in the Control Hub or a phone over USB, pick it in the device dropdown,
   and press Run.

## Hardware configuration names

The drive code looks these up by name, so the config on the Driver Station has
to match exactly:

| Name | What |
|---|---|
| `leftFront` `leftBack` `rightFront` `rightBack` | drive motors |
| `imu` | Control Hub IMU |
| `lift` | example subsystem motor (`subsystems/Lift.java`) |

Dead wheel encoders plug into motor ports and are named in the localizer file
you choose.

## Tuning, in order

Nothing follows a path correctly until this is done — an untuned drive will
wander or not move at all. Full guide: <https://rr.brott.dev/docs/v1-0/tuning/>

1. Set `logoFacingDirection` / `usbFacingDirection` in `MecanumDrive.Params` to
   match how the Control Hub is mounted.
2. `ForwardPushTest` → gives you `inPerTick`
3. `LateralPushTest` → `lateralInPerTick`
4. `ForwardRampLogger` → `kS`, `kV`
5. `LateralRampLogger`
6. `AngularRampLogger` → `trackWidthTicks`
7. `ManualFeedforwardTuner` → refine `kV`, set `kA`
8. `ManualFeedbackTuner` → the gains (`axialGain`, `lateralGain`, `headingGain`)
9. `SplineTest` and `LocalizationTest` to confirm it all works

The tuning OpModes show up on the Driver Station under the **quickstart** group.

## FTC Dashboard

While the robot is on, connect to its WiFi and open <http://192.168.43.1:8080/dash>
(Control Hub) or <http://192.168.49.1:8080/dash> (phone). You get a live field
view of the robot's position and can edit anything marked `@Config` — including
`MecanumDrive.PARAMS` and `Lift`'s gains — without redeploying.

## Notes

- Field coordinates are **inches**, headings are **radians**, origin is the
  center of the field.
- `SampleAutonomous` has placeholder coordinates. Set `START_POSE` to where the
  robot actually starts before you run it.
- `README_FTC_SDK.md` is FIRST's original README — release notes and SDK docs.

## Updating later

New FTC SDK release: bump the version numbers in `build.dependencies.gradle`,
and copy the new `FtcRobotController` module in from the release so the app
version matches the libraries.

New Road Runner release: the versions are in `TeamCode/build.gradle`. Check
<https://github.com/acmerobotics/road-runner-quickstart> for changes to the
drive classes that go with it.
