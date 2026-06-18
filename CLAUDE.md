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
- MW-Lib has its own standalone build: `cd /home/ubuntu/MW-Lib-Alpha5 && JAVA_HOME=/home/ubuntu/wpilib/2027_alpha5/jdk ./gradlew build` — all 50 tests pass

## Dependencies
- WPILib 2027 alpha-5 (`org.wpilib.GradleRIO` version `2027.0.0-alpha-6`)
- CTRE Phoenix 6 `26.50.0-alpha-1` — alpha-5 native (uses `org.wpilib.*` types); no longer needs a compileOnly shim
- DogLog `2027.1.0` — alpha-5 compatible real library (stub removed); resolved via JitPack (`com.github.jonahsnider:doglog:2027.1.0`)
- ChoreoLib `2027.0.0-alpha-1` — still uses old `edu.wpi.first.*` types; access sample fields directly (`sample.x`, `sample.y`, `sample.heading`, `sample.vx`, `sample.vy`, `sample.omega`) instead of `.getPose()` / `.getChassisSpeeds()`; do NOT log `getPoses()` via DogLog (returns old `edu.wpi.first.math.geometry.Pose2d[]`, incompatible with DogLog's `T extends StructSerializable` bound)
- MapleSim `0.4.0-beta` — uses old `edu.wpi.first.units.measure.*` types; all MapleSim integration is commented out in `PhoenixUtil.java`; `TalonFXMotorControllerSim` no longer implements `SimulatedMotorController`
- PhotonVision `v2026.2.2` — uses old geometry types

## Old-Type Compatibility (compileOnly shim)
MapleSim, ChoreoLib, and PhotonVision reference `edu.wpi.first.*` types from the old WPILib. To let the compiler find them, `build.gradle` adds alpha-1 jars as `compileOnly`:
- Key jar: `wpiunits-java-2027.0.0-alpha-2.jar` — contains `edu.wpi.first.units.measure.*` (Angle, Voltage, etc.)
- All jars are under `/home/ubuntu/wpilib/2027_alpha1/maven/edu/wpi/first/`

Note: Phoenix 6 `26.50.0-alpha-1` no longer requires this shim — it is fully alpha-5 compatible.

## Phoenix 6 API Changes (26.1.1 → 26.50.0-alpha-1)
- `TalonFX`, `TalonFXS`, `CANcoder`, `Pigeon2` constructors now require a `CANBus` object instead of a plain `String`: `new TalonFX(id, new CANBus(canbus_name))`
- `setNeutralMode(...)` renamed to `configNeutralMode(...)`
- When touching CTRE `StatusSignal` fields, use raw `StatusSignal` (no generic) with `@SuppressWarnings("rawtypes")` to avoid type-mismatch errors. Access values via `.getValueAsDouble()` — CTRE default units are rotations, amps, and Celsius.

## DogLog 2027.1.0 API Changes
The following `DogLogOptions` builder methods were removed — do not use them:
- `withNtPublish(boolean)`
- `withCaptureNt(boolean)`
- `withLogEntryQueueCapacity(int)`

## PhotonVision Maven Timeout Workaround
Both `mainbot/build.gradle` and `MW-Lib-Alpha5/build.gradle` use content filters on PhotonVision repos to prevent Gradle from timing out while querying them for unrelated artifacts:
```groovy
maven {
    url = "https://maven.photonvision.org/repository/internal"
    content { includeGroup "org.photonvision" }
}
maven {
    url = "https://maven.photonvision.org/repository/snapshots"
    content { includeGroup "org.photonvision" }
}
```

## MW-Lib Test Runtime Dependency
WPILib geometry classes require `quickbuf-runtime` at test runtime. It must be added explicitly to `MW-Lib-Alpha5/build.gradle`:
```groovy
testRuntimeOnly 'us.hebi.quickbuf:quickbuf-runtime:1.4'
```

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
The compile stubs (`dev/doglog/DogLog.java` and `DogLogOptions.java`) have been **removed** from both mainbot and MW-Lib. The real DogLog `2027.1.0` library is used instead.
