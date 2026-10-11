# Team 4143 Robot Code — Working Agreement

This robot runs on **MW-Lib** (`com.marswars.*`), our in-house framework. It is **not** a stock
WPILib command-based project — generic FRC/WPILib patterns and ChatGPT/Google answers usually do
**not** match how we write code. When in doubt, mirror the existing code in `src/main/java/frc/robot/`.

For anything beyond the rules below, use the **`mwlib-robot-code`** skill in
`.claude/skills/mwlib-robot-code/` — its `references/` files cover subsystems, mechanisms, commands/OI,
autonomous, and logging in depth.

## Architecture in one breath

Data flows **OI / Autos → RobotContainer (robot state machine) → Subsystems → Mechanisms**:

- **Subsystems** are singleton state machines that extend `MwSubsystem<StatesEnum, ConstantsClass>`.
  They are ticked by `RobotContainer.doControlLoop()` (our scheduler), **not** the WPILib
  `CommandScheduler`. Every loop each subsystem runs `readInputs → updateLogic → writeOutputs → logData`.
- **Mechanisms** (`ArmMech`, `ElevatorMech`, `FlywheelMech`, `RollerMech`, `TurretMech`) are the
  hardware IO layer. Subsystems **compose** them (has-a), never extend them. They abstract the motors
  and make sim vs. real transparent.
- **`RobotContainer` is the robot's super state machine** (`MwSuperStateMachine<RobotStates>`). OI
  buttons and BT autos ask for a robot state with `RobotContainer.getInstance().setWantedState(...)`;
  RobotContainer's `updateLogic` is a `switch` on the robot state that asks every subsystem for its
  state every loop, just like a subsystem's state machine (AUTO and MAINTENANCE ask for nothing, so
  autos and maintenance tasks like homing set subsystems directly). It runs before the subsystems every loop. **There are no command files**:
  anything that coordinates more than one subsystem is a robot state in RobotContainer, shared by
  teleop and autos.
- **OI** bindings are plain `Commands.startEnd/runOnce` lambdas around `setWantedState`, run by the
  WPILib `CommandScheduler`. `Robot.robotPeriodic()` calls **both** `CommandScheduler.getInstance().run()`
  **and** `robot_container_.doControlLoop()`.
- `Robot extends LoggedRobot` (AdvantageKit), not `TimedRobot`.

## Hard rules (do not violate)

- **Change subsystem state only via `setWantedState(SomeState)`.** Never assign `system_state_` from
  outside a subsystem. Inside a subsystem, transitions live in `handleStateTransition()`.
- **Conditions only in `handleStateTransition()`.** `updateLogic()` is a `switch` on the state with
  no `if`/`?:` deciding behaviour. If a state behaves differently depending on a condition, split it
  into states and pick between them in `handleStateTransition()` (see RobotContainer's `SHOOT_`
  steps). Same rule for subsystems and RobotContainer.
- **Never override `MwSubsystem.update()`.** It auto-logs state and drives the tick; override
  `updateLogic()` and `handleStateTransition()` instead.
- **Every subsystem is a singleton** (`private static instance_` + `getInstance()`) and **must be
  registered** in `RobotContainer`'s constructor with `registerSubsystem(X.getInstance())`, or it
  never ticks.
- **Log with `MwLog`, never `DogLog` or raw `SmartDashboard`/`Logger`.** `MwLog` is our facade over
  AdvantageKit with a DogLog-compatible API. Build keys off `getSubsystemKey()` /
  `getLoggingKey()` (they already produce `Subsystem/<Name>/…`).
- **Use the `timestamp` passed into `updateLogic(double timestamp)`.** Never call
  `Timer.getFPGATimestamp()` in the control loop — the passed timestamp is log-sourced and replays
  deterministically.
- **Don't add command files.** Coordination between subsystems belongs in `RobotContainer` as a robot
  state; buttons and autos only call `setWantedState(...)` on the robot. Commands never `require()` a
  subsystem. **Read joysticks only in `OI`**, never in subsystems or RobotContainer (the one exception
  is piping drive-axis values into the swerve's driver inputs).
- **Class name suffixes are load-bearing.** `*Subsystem`, `*Constants`, `*Mech` names are reflected to
  derive logging keys — don't drop or rename them.

## Naming & formatting

Enforced by **Spotless + google-java-format `.aosp()`** (`build.gradle`); `spotlessApply` runs after
every build. Do not fight the formatter.

- **4-space indentation**, 100-column limit, explicit imports only (no wildcards), one import block.
- **Member/instance fields:** `snake_case_` with a **trailing underscore** — `system_state_`,
  `roller_0_`, `driver_controller_`. (No WPILib `m_` prefix.)
- **Constants:** `UPPER_SNAKE_CASE`, declared `public final` in the `*Constants` class, accessed via
  the injected `CONSTANTS` field: `CONSTANTS.PIVOT_GEAR_RATIO`.
- **Local variables & parameters:** `snake_case` (no trailing underscore).
- **Methods:** `camelCase`; getters/setters prefixed `get`/`set`, booleans `is`.
- **Classes:** `UpperCamelCase`. **Enum values & state names:** `UPPER_SNAKE_CASE`.
- **Javadoc** on public methods with `@param`/`@return`; per-value Javadoc on state enums; avoid `while`
  loops in control code.

## Where things live

```
src/main/java/frc/robot/
├── Robot.java, Main.java                         # wiring
├── RobotContainer.java                           # robot super state machine, with its RobotStates and
│                                                 #   ScoringModes enums nested inside, and the robot ID
├── OI.java                                       # controller/dashboard bindings only
├── subsystems/<name>/
│   ├── <Name>Subsystem.java                      # singleton state machine
│   └── <Name>Constants.java                      # public final constants + state enum + MotorConfigs
│       (per-robot variants: Beta<Name>Constants/Sim<Name>Constants reassign fields in their constructor)
├── autos/bt/AutoNodes.java                       # robot BT nodes; the autos themselves are XML in deploy/autos/
└── lib2026/                                       # season field/game helpers
```

Per-robot constants are pure Java (no JSON): the base `*Constants` class is AlphaBot, with the
values that differ between robots as its only non-final fields. A robot that differs is a small
subclass whose constructor reassigns just those fields (see
`subsystems/swerve/BetaSwerveConstants.java`), selected by a static `create()` switching on
`RobotContainer.ROBOT`. Anything derived from those fields must be computed after construction (e.g.
in a method the subsystem calls), never in the base constructor, or variant values are missed.
