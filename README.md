# mainbot-4143-2026

FRC Team 4143 (MARS/WARS) robot code for the 2026 **REBUILT** season.

Built on [MW-Lib](https://github.com/FRC-Team-4143/MW-Lib), our shared subsystem framework, with
[AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageLog) logging/replay, Phoenix 6, PhotonLib
vision, and Choreo-planned autonomous paths.

## Game Summary

Each match the robot cycles **fuel** game pieces through an intake → shooter loop:

- **Intake** fuel from the field or the depot.
- **Score** into the alliance **hub**, or **pass** fuel to a teammate — which is legal depends on where the
  robot is standing on the field.
- **Climb** the tower during the end game.

Only one alliance's hub is "active" (scorable) at a time, and the active alliance alternates on a fixed
shift timer for the back half of the match. [`HubMonitor`](src/main/java/frc/robot/lib2026/HubMonitor.java)
tracks whose hub is currently live from the FMS game-specific message (with a dashboard chooser fallback for
practice) so the rest of the robot code can reason about it.

## Robot state machine (RobotContainer)

[`RobotContainer`](src/main/java/frc/robot/RobotContainer.java) is the robot's **super state machine**.
It extends MW-Lib's `MwSuperStateMachine<RobotStates>`, and it is the only place that coordinates
subsystems:
- Buttons (`OI`) and autos ask for a robot state with `setWantedState(RobotStates.X)`.
- RobotContainer then requests the matching subsystem states.
- It runs every loop **before** the subsystems.

There are no command files.

| State | Meaning |
| ----- | ------- |
| `IDLE` | Disabled: shooter idle |
| `DRIVE` | Teleop driving with no button held, shooter idle |
| `TRACK` | Driving with the flywheel kept at speed (after releasing aim), so the next shot is quick |
| `AUTO` | Autonomous: the robot asks for nothing; autos set subsystem states themselves |
| `AIM` | Aim at the target without shooting (driver LT) |
| `SHOOT` | Spin up and aim. The intake feeds (`SQUEEZE`) only once the shooter is ready, and the robot brakes once stopped and ready (driver RT; autos' `ShootFor`) |
| `MANUAL_HUB` / `MANUAL_PASS` | Fixed flywheel speed shot or pass, for when vision is down (driver Y / B) |
| `INTAKE` / `OUTTAKE` | Run the intake in / out (driver RB / A) |
| `BRAKE` | Cross the wheels (driver X) |
| `MAINTENANCE` | Test mode: the robot asks for nothing, so dashboard maintenance buttons (homing, tuning) drive the subsystems |

RobotContainer is written like a subsystem state machine: `updateLogic` is a `switch` on the robot
state that asks every subsystem for its state, every loop. In `AUTO` and `MAINTENANCE` it asks for
nothing, so autos and maintenance tasks (homing, tuning) can set subsystem states directly. Driver
toggles (field centric, intake stowed, able to rack) are flags the states read.

Autos use the same states as teleop through the `SetWantedState` behavior-tree node. It waits until
the robot is in the state (SUCCESS) or the state fails (FAILURE). A scoring sequence is written once,
in RobotContainer, and used by both teleop and autos.

**Scoring mode.** Every loop RobotContainer also works out a **scoring mode** from the robot's pose:

| Mode | Meaning |
| ---- | ------- |
| `HOLD` | Holding fuel, not committed to scoring or passing yet |
| `SCORE` | In the alliance zone: aim at and shoot into the hub |
| `PASS` | In a neutral/opponent-side pass region: aim at a pass target instead of the hub |
| `AUTO` | Autonomous period: the auto picks the target |

The scoring mode picks the shooter target and decides when `SHOOT` feeds. Mode changes are driven by
[`FieldRegions`](src/main/java/frc/robot/lib2026/FieldRegions.java) polygons (alliance zone, left/right
pass regions, hold zones, tower, depot/outpost, neutral zone), which are alliance-flipped at match
start. The operator can also hold a dead-man override (left bumper) to force a pass target while
still standing in the alliance zone.

## Subsystems

| Subsystem | Responsibility |
| --------- | --------------- |
| `SwerveSubsystem` | Drivebase. States include `FIELD_CENTRIC`/`ROBOT_CENTRIC` driving, `*_ROTATION_LOCK` variants for heading-locked driving, `CHOREO_PATH` for trajectory following, `TRACTOR_BEAM` for pose-seeking, `CRAWL_*` for slow POV nudging, and `BRAKE`/`TUNING`/`IDLE`. |
| `ShooterSubsystem` | Flywheel + hood + indexer/accelerator. States: `IDLE`, `TRACKING`, `AIMING`, `SHOOT_WAIT`, `SHOOT`, `MANUAL_HUB`, `MANUAL_PASS`, `DUMP`, `TUNING`. Targets either `HUB` or `PASS`. |
| `IntakeSubsystem` | Pivoting roller intake. States: `STORE`, `DEPLOYED`, `PIVOT_HOMING`, `INTAKE`, `OUTTAKE`, `SQUEEZE` (feeds fuel into the shooter), `IDLE`. |
| `LocalizationSubsystem` | Pose estimation. States: `FULL` (normal), `SHOOTING_FOCUS` / `CLIMBING_FOCUS` (weight specific AprilTag sets while aiming or climbing). |
| `SimulationSubsystem` | Only registered in simulation (and never during log replay) — drives physics sim of game pieces/mechanisms. |

All subsystems are registered in [`RobotContainer`](src/main/java/frc/robot/RobotContainer.java), which
extends MW-Lib's `MwSuperStateMachine` (a `SubsystemManager`).

## Controller Bindings

Two Xbox controllers: Driver (port 0) and Operator (port 1). See
[`OI`](src/main/java/frc/robot/OI.java). Held buttons ask for a robot state, and releasing them goes back to
`DRIVE`.

**Driver**

| Button | Action |
| ------ | ------ |
| Right Trigger | `SHOOT`: shoot fuel at the current target (hub or pass, per scoring mode) |
| Left Trigger | `AIM`: aim at target without shooting (lines up shooter/rotation/localization) |
| Right Bumper | `INTAKE`: intake fuel |
| Left Bumper (on release) | Toggle intake stow/deploy |
| Right Stick (click) | Toggle field-centric / robot-centric driving |
| Y | `MANUAL_HUB`: manual shoot at hub (fixed flywheel speed, for when vision tracking is unavailable) |
| B | `MANUAL_PASS`: manual pass (fixed flywheel speed + hood, for when vision tracking is unavailable) |
| A | `OUTTAKE`: outtake fuel |
| X | `BRAKE`: cross the wheels |

**Operator**

| Button | Action |
| ------ | ------ |
| Left Bumper (hold) | Dead-man pass override — force a pass target while inside the alliance zone |
| POV Up / Down | Nudge flywheel speed +/- |
| POV Left / Right | Nudge hood angle +/- |
| X | Toggle "able to rack" (allows/blocks auto-squeeze into the shooter while shooting) |

**Dashboard (SmartDashboard)**

- `Zero Gyro Yaw`, `Set Start Pose`, `Zero Wheel Offsets` — field setup/calibration, work while disabled.
- `Spin Down Flywheel` — manual flywheel stop.
- `Hub - First Alliance` chooser — manually seed which alliance's hub goes active first (auto-detected from
  FMS game data in a real match; useful in practice/sim).

## Robot Configs

Hardware constants (swerve module gearing/wheel radius, motor gains, shooter geometry, etc.) are loaded at
runtime from JSON in [`src/main/deploy/robots/`](src/main/deploy/robots) rather than hardcoded, so the same
code runs on multiple physical robots:

- [`AlphaBot.json`](src/main/deploy/robots/AlphaBot.json) / [`BetaBot.json`](src/main/deploy/robots/BetaBot.json) — practice vs. competition chassis (differ in wheel radius and module gearing).
- [`SimBot.json`](src/main/deploy/robots/SimBot.json) — used automatically in simulation.

The active config is chosen by `RobotName` (MW-Lib `ConstantsLoader`): defaults to `AlphaBot`, falls back to
`SimBot` in simulation (overridable with the `ROBOT_NAME` environment variable), and can otherwise be set
persistently via a `RobotName` robot preference from the dashboard.

## Autonomous

Auto routines live in [`src/main/java/frc/robot/autos/`](src/main/java/frc/robot/autos) and are registered
with MW-Lib's `AutoManager` in [`Robot.java`](src/main/java/frc/robot/Robot.java). Paths are authored in
[Choreo](https://sleipnirgroup.github.io/Choreo/) (`src/main/deploy/choreo/*.traj`) and referenced from Java
via the generated `ChoreoTraj`/`ChoreoVars` (excluded from Spotless since they're generated code, not hand
written).

## Field Layouts

We compete on the welded AprilTag field for all regional events; the AndyMark field only shows up at
offseason competitions. See [`FieldLayouts.md`](src/main/deploy/apriltag_layouts/FieldLayouts.md).

## Local MWLib development

By default the project builds against the **published** `mw-lib-java` artifact
(`com.github.frc-team-4143:mw-lib-java`, from GitHub Packages / jitpack).

To iterate on [MW-Lib](https://github.com/FRC-Team-4143/MW-Lib) locally without publishing, opt into a
Gradle composite build that substitutes the published jar for a live compile of your local MWLib checkout.
Edit MWLib source, then `./gradlew build` / `deploy` here recompiles it and bundles it automatically — no
publish, no version bump.

- **Persistent** (per developer, this project only, never committed — safe for CI): create a
  `local.properties` file in the project root:
  ```properties
  mwlibLocal=true
  # mwlibPath=/absolute/path/to/MW-Lib   # optional; defaults to ../MW-Lib
  ```
  `local.properties` is git-ignored, so it can't leak into CI or teammates' checkouts.
- **One-off** (no file needed):
  ```bash
  ./gradlew deploy -PmwlibLocal
  ./gradlew build -PmwlibLocal -PmwlibPath=/absolute/path/to/MW-Lib   # custom location
  ```
- **Disable** for a single run (overrides `local.properties`):
  ```bash
  ./gradlew build -PmwlibLocal=false
  ```

Command-line `-P` flags always override `local.properties`.

While local mode is active, a **bold-yellow banner** prints at the start and end of every
build/deploy/simulate so it's obvious you are not running the published library. The wiring lives in
[settings.gradle](settings.gradle).
