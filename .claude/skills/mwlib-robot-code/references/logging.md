# Logging, Tunables & Replay

All telemetry goes through **`com.marswars.logging.MwLog`**. `MwLog` is our facade over **AdvantageKit**
(`org.littletonrobotics.junction`) with a **DogLog-compatible API** — so call sites read like DogLog,
but the data goes through AdvantageKit for deterministic log replay.

- **Use `MwLog`. Never `DogLog`, and never raw `SmartDashboard.putNumber(...)` / `Logger.recordOutput`
  for telemetry.** (`SmartDashboard.putData(...)` for operator *buttons/choosers* is still fine.)
- If you find `DogLog.log(...)` anywhere, it's stale — the migration replaced it with `MwLog.log(...)`
  one-for-one.

## Logging values

Build every key off the subsystem/mechanism prefix so logs stay grouped:

```java
// getSubsystemKey() -> "Subsystem/<Name>/"  (e.g. Subsystem/Shooter/)
MwLog.log(getSubsystemKey() + "LaunchCalculator/Hood/Angle", hood_angle, Radians);
MwLog.log(getSubsystemKey() + "IsReady", is_ready_);
MwLog.log(getSubsystemKey() + "State", system_state_);   // enums log fine
```

Overloads (mirror the DogLog signatures): `double`, `double` + `Unit`, `double[]`,
`boolean`, `long`, `String`, `String[]`, any `Enum`, any single WPI struct
(`Pose2d`, `Pose3d`, `ChassisSpeeds`, `Rotation2d`, …) and struct arrays
(`SwerveModuleState[]`, `Pose2d[]`, …).

Prefer the unit-typed overload with statically-imported units:

```java
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
MwLog.log(getSubsystemKey() + "Flywheel/Velocity", velocity, RadiansPerSecond);
```

**You don't log state yourself** — the base `MwSubsystem.update()` already logs
`<key>WantedState` and `<key>State` every tick, and each mechanism logs its own targets/actuals and
motor voltage/current/temp. Only add logs for your own derived values (setpoints, tolerances,
readiness booleans, computed poses).

Keys:
- `getSubsystemKey()` → `Subsystem/<Name>/…` — AdvantageKit outputs (the normal one to use).
- `getNtKey()` → `Robot/Subsystem/<Name>/…` — NetworkTables-facing values.
- `getLoggingKey()` (inside a mechanism) → the mechanism's own prefix.
- Group related values with compound keys (`"Choreo/…"`, `"LaunchCalculator/Hood/…"`).

## Live tunables

`MwLog.tunable(key, defaultValue, onChange)` publishes a `double` under `/Tuning/<key>` in
NetworkTables and fires the consumer immediately with the default and again on every change. Tunable
changes are captured in the log, so replays behave identically.

```java
MwLog.tunable(getSubsystemKey() + "Hood/kV", CONSTANTS.HOOD_KV, (new_kv) -> hood_kv_ = new_kv);
```

- Polled by `MwLog.periodic()`, which `SubsystemManager` calls at the top of every control loop.
- Tunables **don't persist** — once you settle on a value, copy it back into the `*Constants` class.
- Mechanisms already register tunable PID gains + setpoints under `/Tuning/…` automatically (via
  `TunablePid`); you rarely need to add tunables for a mechanism's built-in control.

## Timing / profiling

```java
MwLog.time(getSubsystemKey() + "expensiveStep");
// ... work ...
MwLog.timeEnd(getSubsystemKey() + "expensiveStep");   // logs elapsed seconds
```

## Replay & sim gating

- `MwLog.isReplay()` is true when replaying a log (`AKIT_LOG_PATH` env var set). Use it to skip
  hardware reads and to gate sim-only subsystems:
  ```java
  if (RobotBase.isSimulation() && !MwLog.isReplay()) {
      registerSubsystem(SimulationSubsystem.getInstance());
  }
  ```
- `MwLog.timestampSeconds()` is the deterministic, log-sourced loop time. `SubsystemManager` passes it
  into `update`/`readInputs`/`writeOutputs` as `timestamp` — always use that parameter rather than
  `Timer.getFPGATimestamp()`, or replay diverges.
- `MwLog.init(BuildConstants.class)` is called once by `SubsystemManager`; you don't call it yourself.
- **Replay picks the right robot automatically.** Every log records the robot's name as
  `RobotName` metadata; during replay `RobotIdentity` reads it back, so the matching per-robot
  constants variant loads without any setup (`AKIT_LOG_PATH=logs/foo.wpilog ./gradlew simulateJava`
  just works). Setting `ROBOT_NAME` explicitly still overrides it (a warning is printed if it
  disagrees with the log). Logs from before this metadata existed fall back to SimBot — set
  `ROBOT_NAME` manually for those.

## Viewing

Live NT data and logs are viewed in **AdvantageScope** and **Elastic** (the driver-station dashboard).
`Robot` switches Elastic tabs with `Elastic.selectTab("Autonomous" / "Teleoperated")`.
