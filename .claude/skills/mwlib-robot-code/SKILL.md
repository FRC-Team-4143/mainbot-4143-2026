---
name: mwlib-robot-code
description: >-
  Team 4143's conventions and MW-Lib framework for writing this robot's Java code. Use whenever
  creating or editing anything under src/main/java/frc/robot — subsystems, constants classes, state
  machines, mechanisms (ArmMech/ElevatorMech/FlywheelMech/RollerMech/TurretMech), commands,
  OI/controller bindings, autonomous routines, Choreo paths, or MwLog logging/telemetry/tunables.
  MW-Lib is a custom state-machine framework, NOT stock WPILib command-based, so generic FRC/WPILib
  answers usually do not apply here.
---

# MW-Lib Robot Code

Team 4143 (MARS/WARS) robot code is built on **MW-Lib** (`com.marswars.*`), an in-house framework
that replaces WPILib's command-based subsystem model with **singleton state machines + a hardware IO
layer + AdvantageKit logging**. Match the existing code in `src/main/java/frc/robot/`; do not assume
stock WPILib patterns.

## The mental model

Data flows in one direction: **OI → Commands → Subsystems → Mechanisms**.

```
 OI.java            Reads operator input, binds buttons/axes to Commands. Bindings only.
   │
 ControlCommands    Command factories. Coordinate subsystems by requesting states. No hardware.
   │  setWantedState(...)
 Subsystems         Singleton state machines: MwSubsystem<States, Constants>.
   │                Each loop: readInputs → updateLogic (switch on state) → writeOutputs → logData.
 Mechanisms         ArmMech / ElevatorMech / FlywheelMech / RollerMech / TurretMech.
                    The hardware IO layer. Sim vs. real is transparent here.
```

Two schedulers run each 20 ms in `Robot.robotPeriodic()`:
- `CommandScheduler.getInstance().run()` — WPILib, drives OI bindings + autos.
- `robot_container_.doControlLoop()` — MW-Lib, ticks every registered subsystem's IO + state machine.

`RobotContainer extends SubsystemManager`; `Robot extends LoggedRobot` (AdvantageKit).

## The subsystem control loop (per tick, per subsystem)

`SubsystemManager.doControlLoop()` runs, in strict order:
1. `MwLog.periodic()` — poll live tunables.
2. `io.readInputs(timestamp)` for each mechanism (sensor reads, captured for replay).
3. `subsystem.update(timestamp)` — base method (DO NOT OVERRIDE); auto-logs `WantedState`/`State`,
   then calls your `handleStateTransition()` and `updateLogic()`.
4. `io.writeOutputs(timestamp)` then `io.logData()` for each mechanism.

`timestamp` is log-sourced (`MwLog.timestampSeconds()`); use it, never `Timer.getFPGATimestamp()`.

## Non-negotiables

- State changes only through `setWantedState(...)`; never assign `system_state_` from outside.
- Every subsystem is a singleton and must be `registerSubsystem(...)`-ed in `RobotContainer`.
- Never override `MwSubsystem.update()`.
- Log with `MwLog` (DogLog-compatible facade over AdvantageKit), keyed off `getSubsystemKey()` —
  never `DogLog` or raw `SmartDashboard`/`Logger`.
- Commands never `require()` a subsystem; joysticks are read only in `OI`.
- `*Subsystem` / `*Constants` / `*Mech` name suffixes are reflected into logging keys.

## Formatting

Spotless + google-java-format `.aosp()`: 4-space indent, 100 cols, explicit imports, no wildcards.
Fields are `snake_case_` (trailing underscore); constants `UPPER_SNAKE_CASE` via `CONSTANTS.`; methods
`camelCase`. Run `./gradlew spotlessApply` (also runs automatically after a build).

## Which reference to read

| Task | Read |
| --- | --- |
| Create/edit a subsystem or its `*Constants` (state machine, singleton skeleton, registration, per-robot variants) | `references/subsystems.md` |
| Use a mechanism (`ArmMech`/`ElevatorMech`/`FlywheelMech`/`RollerMech`/`TurretMech`) — constructors, setters, getters | `references/mechanisms.md` |
| Add a command or a controller/dashboard binding | `references/commands-and-oi.md` |
| Write an autonomous routine (Choreo paths, event triggers, swerve states) | `references/autonomous.md` |
| Log data, add a live tunable, or handle replay | `references/logging.md` |

Canonical code examples to imitate: `subsystems/intake/IntakeSubsystem.java` +
`IntakeConstants.java` (mechanism-driven), `subsystems/gamestates/GameStatesSubsystem.java`
(logic-only), `ControlCommands.java` + `OI.java`, `autos/Right_Trench_Trench_Swipe.java`.
