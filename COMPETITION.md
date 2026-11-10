# Competition Guide (BIOBUZZ 2026-27)

This is the `competition` branch: the code to run at scrimmages and meets. If you just need
the robot driving, everything you need is on this page.

## 1. Put the code on the robot

1. Open this project in **Android Studio** (the top folder, not `TeamCode`).
2. Make sure the branch is `competition` (bottom right of Android Studio, or `git switch competition`).
3. Plug the Control Hub into the laptop with USB-C, or connect to the robot's Wi-Fi.
4. Pick **TeamCode** in the run menu at the top and press the green **Run ▶** button.
5. Wait for "Install successfully finished". The Driver Station will reconnect on its own.

## 2. Robot configuration names

The code finds hardware **by name**. These have to match the active configuration on the
Driver Station exactly, capital letters included:

| Name | Device | Port |
|---|---|---|
| `FL` | front left drive motor | motor |
| `FR` | front right drive motor | motor |
| `BL` | back left drive motor | motor |
| `BR` | back right drive motor | motor |
| `pinpoint` | goBILDA Pinpoint (dead wheels) | I2C |

If a motor name is wrong, the OpMode doesn't crash. It shows `!! FL NOT FOUND - check config`
and drives on the other wheels. Fix the name before the match.

## 3. Which OpModes to run

| When | OpMode | What it does |
|---|---|---|
| AUTO | **Comp Auto: Leave** | Drives off the wall for LEAVE (3 pts). Loads Comp TeleOp for you afterward. |
| TELEOP | **Comp TeleOp** | Mecanum drive on gamepad 1. |

Rule G305: **every match you must select and INIT an OpMode**, even if you aren't running auto.
For AUTO, make sure the 30 second timer is turned on.

### Comp Auto: Leave, set up during INIT (gamepad 1)

| Button | Changes | Default |
|---|---|---|
| Y | FORWARD or BACKWARD | FORWARD |
| Dpad up / down | distance (2 in steps) | 12 in |
| Dpad right / left | start delay (1 s steps) | 0 s |

- Point the robot so the direction you picked moves it **away from the wall** and doesn't hit anything.
- Use the delay if your alliance partner drives through the same space.
- **Don't touch the robot after pressing INIT.** The Pinpoint calibrates while it sits still,
  and the robot has to be motionless after INIT anyway (G304.H).
- If the Pinpoint isn't working, it says `(will use timer instead)` and still drives using a timer.

### Comp TeleOp controls (gamepad 1)

| Control | Action |
|---|---|
| Left stick | drive forward/back and strafe |
| Right stick left/right | turn |
| Hold left bumper **or** right trigger | slow mode (for lining up) |
| Dpad up / down | normal speed +/- 0.1 (starts at 0.8) |

## 4. Before every match

- [ ] Battery reads **12.5 V or more** on the INIT screen. If not, swap it. (The screen warns you.)
- [ ] No `NOT FOUND` lines on the INIT screen.
- [ ] Correct configuration is active on the Driver Station.
- [ ] Robot fits in the 18 in cube and has its 4 pre-load POLLEN.
- [ ] Driver Station and gamepads are charged.
- [ ] Auto is selected with the 30 s timer on, direction and delay are set.

## 5. If something goes wrong

| Problem | Fix |
|---|---|
| A wheel spins the wrong way | In `subsystems/Drivetrain.java`, flip that wheel's `REVERSE_FL/FR/BL/BR` (true ↔ false), then rebuild. Push the left stick straight up: all four wheels should roll forward. |
| Robot drives sideways or spins when it should go straight | Same as above: one or two directions are flipped. |
| Strafing is slower than driving forward | Raise `STRAFE_MULTIPLIER` in `Drivetrain.java` a little (1.1 → 1.2). |
| Auto says `WRONG WAY` or `NOT MOVING` | The dead wheels are backwards or unplugged. The timer fallback still runs. Check with the **Auto Test** OpMode (push the robot forward and x should go up). |
| Robot moves at random / resets mid-match | Usually the battery. Look at `battery low` in TeleOp; under 11 V means swap it. |
| Robot too fast to control | Dpad down during TeleOp, or lower `NORMAL_SPEED` in `CompTeleOp.java`. |

### Pit / test OpModes

These are still on the list for diagnosing problems. **Don't run them in a match.**

- **Drive Motor Test**: spins and checks each motor, shows current draw per wheel.
- **Drive Test**: drive plus dead wheel, Limelight and battery readouts.
- **Auto Test**: checks the dead wheels in INIT, then drives to a point.
- **Limelight Test** / **HuskyLens Test**: camera checks.

## Where the code lives

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
  competition/   CompTeleOp, CompAutoLeave       <- match code
  subsystems/    Drivetrain, Odometry, DriveToPoint, PowerMonitor, cameras
  test/          pit and test OpModes
```
