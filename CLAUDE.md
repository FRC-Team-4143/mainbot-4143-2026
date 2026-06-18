# CLAUDE.md — FRC Robot 4143 (2026, Alpha-5)

## Project Overview
FRC robot code for team 4143, migrated from WPILib 2027 alpha-1 to alpha-5. Uses a local copy of MW-Lib (the team's private library) compiled directly into the robot build.

## Build
```bash
JAVA_HOME=/home/ubuntu/wpilib/2027_alpha5/jdk ./gradlew compileJava
```
JDK is at `/home/ubuntu/wpilib/2027_alpha5/jdk`. Do not use the system JDK.

## MW-Lib
- Source lives at `/home/ubuntu/MW-Lib-Alpha5/src/main/java`
- Included in the mainbot build via `sourceSets` in `build.gradle` — no separate compile step
- All MW-Lib edits go to that directory, not the original

## Dependencies
- WPILib 2027 alpha-5 (`org.wpilib.GradleRIO` version `2027.0.0-alpha-6`)
- CTRE Phoenix 6 `26.1.1` — compiled against old `edu.wpi.first.*` types (incompatible with alpha-5 at runtime but works with compileOnly shim)
- ChoreoLib `2027.0.0-alpha-1` — also uses old types; access sample fields directly (`sample.x`, `sample.y`, `sample.heading`, `sample.vx`, `sample.vy`, `sample.omega`) instead of `.getPose()` / `.getChassisSpeeds()`
- MapleSim `0.4.0-beta` — uses old `edu.wpi.first.units.measure.*` types in `SimulatedMotorController` interface
- PhotonVision `v2026.2.2` — uses old geometry types

## Old-Type Compatibility (compileOnly shim)
CTRE, MapleSim, and PhotonVision reference `edu.wpi.first.*` types from the old WPILib. To let the compiler find them, `build.gradle` adds alpha-1 jars as `compileOnly`:
- Key jar: `wpiunits-java-2027.0.0-alpha-2.jar` — contains `edu.wpi.first.units.measure.*` (Angle, Voltage, etc.)
- All jars are under `/home/ubuntu/wpilib/2027_alpha1/maven/edu/wpi/first/`

When touching CTRE `StatusSignal` fields, use raw `StatusSignal` (no generic) with `@SuppressWarnings("rawtypes")` to avoid type-mismatch errors. Access values via `.getValueAsDouble()` — CTRE default units are rotations, amps, and Celsius.

## Package Rename (alpha-1 → alpha-5)
All `edu.wpi.first.*` → `org.wpilib.*`. Key renames beyond that:
- `ChassisSpeeds` → `ChassisVelocities`
- `SwerveModuleState` → `SwerveModuleVelocity`
- `LinearSystemId` removed → `org.wpilib.math.system.Models`
- `Timer.getFPGATimestamp()` → `Timer.getTimestamp()`
- `RobotController.getFPGATime()` → `RobotController.getTime()`
- `SwerveModulePosition.distanceMeters` → `.distance`
- `DCMotor.KtNMPerAmp` → `.Kt`, `.rOhms` → `.R`
- Test mode renamed: `RobotState.isTest()` → `isUtility()`, lifecycle methods → `utilityInit/Periodic/Exit()`
- `TimedRobot.robotInit()` removed — put init code in the constructor
- `PubSubOption.sendAll(true)` → `PubSubOption.SEND_ALL`
- `Pose2d.exp(Twist2d)` removed → `pose.plus(twist.exp())`
- `DriverStation.isDisabled/isEnabled` → `RobotState.isDisabled/isEnabled`
- `Alliance.Blue/Red` → `Alliance.BLUE/RED`
- `SwerveModuleVelocity.optimize()` and `cosineScale()` are `@NoDiscard` — must assign the return value

## DogLog Stub
A compile stub lives at `src/main/java/dev/doglog/DogLog.java`. It has no-op implementations of all log methods so the real DogLog dependency isn't needed to compile.
