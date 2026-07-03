# Team 4143 Robot Code — Working Agreement

This robot runs on **MW-Lib** (`com.marswars.*`), our in-house framework. It is **not** a stock
WPILib command-based project — generic FRC/WPILib patterns and ChatGPT/Google answers usually do
**not** match how we write code. When in doubt, mirror the existing code in `src/main/java/frc/robot/`.

For anything beyond the rules below, use the **`mwlib-robot-code`** skill in
`.claude/skills/mwlib-robot-code/` — its `references/` files cover subsystems, mechanisms, commands/OI,
autonomous, and logging in depth.

## Architecture in one breath

Data flows **OI → Commands → Subsystems → Mechanisms**:

- **Subsystems** are singleton state machines that extend `MwSubsystem<StatesEnum, ConstantsClass>`.
  They are ticked by `RobotContainer.doControlLoop()` (our scheduler), **not** the WPILib
  `CommandScheduler`. Every loop each subsystem runs `readInputs → updateLogic → writeOutputs → logData`.
- **Mechanisms** (`ArmMech`, `ElevatorMech`, `FlywheelMech`, `RollerMech`, `TurretMech`) are the
  hardware IO layer. Subsystems **compose** them (has-a), never extend them. They abstract the motors
  and make sim vs. real transparent.
- **Commands** (in `ControlCommands`) and **OI** (bindings) run through the normal WPILib
  `CommandScheduler`. `Robot.robotPeriodic()` calls **both** `CommandScheduler.getInstance().run()`
  **and** `robot_container_.doControlLoop()`.
- `Robot extends LoggedRobot` (AdvantageKit), not `TimedRobot`.

## Hard rules (do not violate)

- **Change subsystem state only via `setWantedState(SomeState)`.** Never assign `system_state_` from
  outside a subsystem. Inside a subsystem, transitions live in `handleStateTransition()`.
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
- **Commands never `require()` a subsystem** (deadlock/preemption risk). They coordinate subsystems by
  calling `setWantedState(...)` / public mechanism helpers. **Read joysticks only in `OI`**, never in
  subsystems or commands (the one exception is piping drive-axis values to the teleop drive command).
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
├── Robot.java, RobotContainer.java, Main.java   # wiring; RobotContainer extends SubsystemManager
├── RobotId.java                                  # which robot the code runs on (burned RobotName)
├── OI.java                                       # controller/dashboard bindings only
├── ControlCommands.java                          # command factory methods
├── subsystems/<name>/
│   ├── <Name>Subsystem.java                      # singleton state machine
│   └── <Name>Constants.java                      # public final constants + state enum + MotorConfigs
│       (per-robot variants: Beta<Name>Constants/Sim<Name>Constants override configure())
├── autos/                                         # one class per auto, each extends Auto
└── lib2026/                                       # season field/game helpers
```

Per-robot constants are pure Java (no JSON): a base `*Constants` class holds every value with
AlphaBot defaults; robots that differ get a small subclass overriding `configure()` (see
`subsystems/swerve/BetaSwerveConstants.java`), selected by a static `create()` switching on
`RobotId.current()`.
