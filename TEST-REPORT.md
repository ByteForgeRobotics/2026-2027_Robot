# Robot Code Test Report (2026-11-13)

## Update: game-fit fixes (same day)

After the first run, these were fixed on the `test` branch working tree (still not committed).
**92 tests, all passing**, and the app builds.

| # | Fix |
|---|---|
| 1, 4 | `Field.goal()` is now your HIVE's up cell: about (+/-19, -/+12.75) at field center, 1.51 m high (`Field.CELL_OPENING_HEIGHT_M`). Full TeleOp Test and Shot Test both use it. |
| 2 | **Tip button:** gamepad 2 **left bumper** switches the target to the other cell after a tip. It starts on red AUDIENCE / blue FAR, and telemetry shows which cell it's aiming at. |
| 3 | **Side check:** in ODOMETRY mode the robot won't report READY or feed unless it's within 50 deg of straight in front of the opening (`Field.CELL_ENTRY_HALF_ANGLE_DEG`). The screen says "WRONG SIDE" otherwise. RB also needs the robot aimed now, not just READY. |
| 5 | `Field.MIRROR` is `ROTATE`. Blue's AUDIENCE and FAR spots still land on the audience and far sides. |
| 6 | RED FAR start moved to (-12, -63), clear of the loading zone. Blue gets the rotated spots, which are clear too. |
| 8 | COMPETITION.md checklist: turn FTC Dashboard off with **Enable/Disable Dashboard** at events. |
| 9 | Limelight `TARGET_HEIGHT_IN` is now 51. |

**Still open:**
- **#7, the 4-ball limit.** This needs a ball sensor or counter on the robot, so it's a hardware call. For now there's a reminder in the Full TeleOp Test notes.
- **#10, auto points.** HIVE TIP and PARK autos, plus the pose hand-off, are new features. Comp Auto is marked DO NOT TOUCH, so it wasn't changed.
- **Tuning warnings** (flywheel KF/KI, hood KG/tolerance) are left for the real robot.
- **Measure on a real field:** the cell x position (19 in is an estimate), both start spots against Fig 11-1, and the red-wall flower near the AUDIENCE spot.

---

## First run (before the fixes)

Branch `test`, working tree, including the uncommitted shooter code. Game rules checked against the
BIOBUZZ Competition Manual, Team Update 09 (Nov 12), plus the field geometry in the vault note
"BIOBUZZ Field Localization with Vision".

## Bottom line

- **The app builds** (`assembleDebug`, 54 MB APK).
- **87 laptop tests, 82 pass.** All 5 failures are game-fit problems. None are crashes or math bugs.
- **Every subsystem works in simulation:** drive, drive-to-point, hood, flywheel, intake, PID,
  INIT menu, battery monitor, shot solver, vision math.
- **The shooter is aimed at the wrong place for BIOBUZZ.** It aims at a corner, 1.0 m high. The real
  target is your HIVE's up cell at field center, about 1.5 m high, and it moves to the other side
  every time the HIVE tips. The good news: the solver and flywheel can make that shot.

## Will it work with this year's game? (ranked)

| # | Problem | Rule | Test |
|---|---|---|---|
| 1 | `Field.RED_GOAL` is (-60, 60), a corner on **blue's** half. Red's scoring opening at match start is the red audience cell, about (19, -12.75), at field center, 107 in away. | 10.5.1, G417 | `goalIsTheRedHiveUpCell` FAIL |
| 2 | **The target moves after every tip.** Only the up cell scores. Each tip flips it from the audience side (x ≈ +19) to the far side (x ≈ -19). The code has one fixed goal. It needs at least an operator button for "which cell is up". | 9.6.2, 10.5.1 | not testable yet (feature missing) |
| 3 | **You have to shoot from the opening's side.** The opening faces that cell's own wall. A shot from behind or beside it hits the outside of the cell, which is a STRATEGIC foul. Auto-aim and READY never check which side of the HIVE the robot is on. | G417.C | static review |
| 4 | Goal height is 1.0 m in Full TeleOp Test and Shot Test. The up-cell opening runs 1.36–1.67 m (center 1.51). | 9.6.2, Fig 9-9 | `shotSolverHeightPlaceholdersMatchTheCell` FAIL |
| 5 | **Blue is mirrored the wrong way.** The start state (red audience cell up, blue far cell up), the flowers and the loading zones are all point-symmetric. `Field.MIRROR` should be `ROTATE`, not `REFLECT`. Right now blue aims at its own down cell. The existing `FieldTest` passes either way, so it couldn't catch this. | 10.3.1 | `blueGoalIsTheBlueUpCellAtTheStart` FAIL |
| 6 | Two start spots land in a LOADING ZONE: RED FAR (-36, -63) and BLUE AUDIENCE (36, 63). Robots can't start there. Also check that RED AUDIENCE (36, -63) isn't touching the red-wall flower on the audience half. *The loading zone position comes from the vault note, so confirm it on Fig 9-2.* | G304.D, G304.E | `startSpotsAreLegal` FAIL |
| 7 | **Max 4 game pieces at once.** You start with 4 pre-loads, and nothing counts balls or stops the intake, so grabbing a 5th is a foul. | G407 | static review |
| 8 | **FTC Dashboard is banned during matches.** Comp TeleOp and Comp Auto don't use it, but the library is in the app and its server starts with the app. At events, run Dashboard's "Enable/Disable Dashboard" OpMode to turn it off, or take it out of the competition build. Ask the inspector. | R704.D | static review |
| 9 | Limelight `TARGET_HEIGHT_IN` = 56. The up-cell AprilTags sit at about 51 in, so every range reading is off. | 9.9 | `limelightTargetHeightMatchesTheHiveTags` FAIL |
| 10 | Points left on the table: Comp Auto only scores LEAVE (3 pts). A HIVE TIP in auto is 20 and AUTO PARK is 5. Nothing parks in the loading zone at the end (PARK, 5). `Field.savePose()` is never called, so the auto-to-TeleOp pose hand-off never happens. | 10.5 | static review |

### The shot itself works

Shot table for the real cell height (1.51 m). "Room" is how much space the 2.8 in ball has in the
opening. The miss columns show how far an allowed flywheel error, or 2 in of odometry error, moves
the ball:

| Distance | Hood | RPM | Arrives | Room | Miss at ±75 rpm | Miss at ±2 in |
|---|---|---|---|---|---|---|
| 0.6 m | 75.9° | 2547 | -14° | 10.6 in | 3.0 in | 0.9 in |
| 1.0 m | 68.9° | 2675 | -20° | 11.0 in | 3.3 in | 0.9 in |
| 2.0 m | 59.0° | 3092 | -31° | 11.2 in | 4.2 in | 1.3 in |
| 3.0 m | 54.5° | 3541 | -37° | 11.1 in | 5.1 in | 1.6 in |

- Every shot from 0.6 to 3.2 m can be made, at well under the 5400 rpm cap. Every one comes down into the opening.
- At 3 m, the flywheel's 75 rpm tolerance uses most of the ±5.5 in of room. If far shots are
  inconsistent, tighten `Flywheel.TOLERANCE_RPM` to about 40.
- Ball mass barely matters (3316 rpm at 22 g vs 3198 rpm at 50 g, at 2.5 m), so the placeholder is fine for now.

## Subsystem results (simulated motors and robot)

| Subsystem | Result | What it showed |
|---|---|---|
| PIDFController | 7/7 | F/S/P math right; D and I skipped on the first loop and after a stall; integral capped |
| Flywheel | 9/9 | 0.45 s to 3600 rpm. Back to speed 0.17 s after a shot. Same speed on 11.5 V and 13.2 V. Never drives backwards. |
| Hood | 9/9 | 0.39 s to a target with no overshoot. A wrong `ENCODER_REVERSED` trips the runaway cutoff like it should. Soft limit sits below the top stop. |
| Intake | 6/6 | Jam detected after 0.4 s, no false jam on spin-up, clears when unstuck |
| Drivetrain | 7/7 | Forward, strafe and turn all go the right way; scaling keeps the direction; a missing wheel doesn't crash |
| DriveToPoint | 5/5 | Arrives within 0.2 in in 1.1–2.4 s from 4 different headings and never heads the wrong way. Comp Auto's 12 in LEAVE move takes 1.1 s. |
| MatchSetup | 6/6 | Buttons, held-button debounce, saved-pose use/ignore, telemetry |
| PowerMonitor | 5/5 | OK / SAG / BROWNOUT, rate limit, worst-dip memory |
| Field, ShotSolver | 19/19 | Your existing tests, still passing |
| Vision math | 6/7 | Limelight and HuskyLens geometry recover positions exactly. Pollen filter works. Fails only on the 56 in height (#9). |

### Tuning warnings from the sims (not failures)

- **Flywheel settles off target.** The loop is feedforward + P only. If the real motor makes ≥~98%
  or ≤~85% of 6000 rpm, `atSpeed()` never comes true. In the sim that's +79 rpm at 100% and
  -132 rpm at 80%. Tune KF on the robot first. A small KI fixes it for good.
- **Hood sag vs tolerance.** With a little gravity load and `KG = 0`, the hood holds 5–6 ticks low.
  That's right at the 5-tick `TOLERANCE_TICKS`, so READY can flicker and block shots. Set KG,
  or raise the tolerance to 8. Each tick is only 0.04°, so 8 is harmless.
- Intake has no voltage compensation: 336 rpm on 11.5 V (target 350). Fine.

## Other code notes

- **None of the shooter work is committed.** `main`, `test` and `competition` are all on 713b423.
  Hood, Flywheel, Intake, ShotSolver, Field, MatchSetup, the three test OpModes and all the tests
  exist only on this laptop. Commit them to `test`.
- Every target change resets the PID. In ODOMETRY mode the distance changes almost every loop
  while driving, so the hood's KD rarely gets to act. Minor for now.
- Full TeleOp Test fires on RB when READY even if the robot isn't aimed yet. Consider requiring `aimed`.
- Shots don't account for the robot moving. Shoot while stopped, or add this later.
- `DriveToPoint` assumes the Pinpoint's velocities are in the field frame. Worth confirming on the robot.
- `LimelightController` and `HuskyLensController` use `hardwareMap.get`, which crashes if the camera
  isn't in the config. That's OK for test OpModes; don't use them in Comp code as-is.
- `ShotTest.java` javadoc has a `<ul\` typo (harmless).

## What could not be tested off the robot

- Real motor directions, encoder flags and gains
- Pinpoint hardware and cameras
- The OpModes themselves (a LinearOpMode can't run on a laptop). Comp TeleOp and Comp Auto were reviewed by reading them.
- The exact x position of the cell opening (±3 in estimate)

## Running the tests

Use Android Studio's JDK; the Homebrew JDK 26 is too new for Gradle here:

```bash
JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home" ./gradlew :TeamCode:testDebugUnitTest
```

Or right-click `TeamCode/src/test` in Android Studio and choose Run Tests. Each run takes about 45 s,
because the sims run in real time.

New test files (laptop only, nothing goes on the robot):
- `sim/Sim.java`: fake motors, hardware map, telemetry and gamepad
- `sim/SimRobot.java`: fake mecanum robot and Pinpoint
- `subsystems/`: `PIDFControllerTest`, `FlywheelTest`, `HoodTest`, `IntakeTest`, `DrivetrainTest`,
  `DriveToPointTest`, `MatchSetupTest`, `PowerMonitorTest`, `VisionMathTest`, `BioBuzzGameFitTest`

No robot code was changed.
