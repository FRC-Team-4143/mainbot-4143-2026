# Autonomous

Autos are **Choreo-first**. Each routine is one class in `autos/` that extends `com.marswars.auto.Auto`
(itself a `SequentialCommandGroup`). Autos are registered with `AutoManager` and selected on the
dashboard. The swerve subsystem executes trajectories; the auto drives it by requesting swerve states.

Examples: `autos/Shoot.java` (trivial, no path) and `autos/Right_Trench_Trench_Swipe.java`
(full path + event triggers).

## Anatomy of an auto

All work happens in the constructor, in three banner-separated sections: load trajectories → bind
event triggers → `addCommands(...)`.

```java
public class Right_Trench_Trench_Swipe extends Auto {
    public Right_Trench_Trench_Swipe() {
        // 1. TRAJECTORY LOADING — load in the order they'll run (fixes start poses)
        loadTrajectory(ChoreoTraj.RTrenchStartTrenchReturn.name());
        loadTrajectory(ChoreoTraj.RTrenchSwipeTrenchReturn.name());

        // 2. EVENT TRIGGER BINDING — bind Choreo event markers BEFORE addCommands
        SwerveSubsystem.getInstance()
                .getChoreoEventTimeTrigger("Intake Out")
                .onTrue(Commands.runOnce(
                        () -> IntakeSubsystem.getInstance().setWantedState(IntakeStates.INTAKE)));

        // 3. AUTO COMMAND SEQUENCE
        addCommands(
                Commands.runOnce(() -> {
                    ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.HUB);
                    ShooterSubsystem.getInstance().setWantedState(ShooterStates.TRACKING);
                }),
                // point swerve at the first trajectory
                SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.RTrenchStartTrenchReturn.name())),
                // follow it until the setpoint is reached, then revert the swerve state
                Commands.startEnd(
                                () -> SwerveSubsystem.getInstance()
                                        .setWantedState(SwerveStates.CHOREO_PATH),
                                () -> SwerveSubsystem.getInstance()
                                        .setWantedState(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK))
                        .until(SwerveSubsystem.getInstance()::isAtChoreoSetpoint),
                Commands.runOnce(() -> {
                    ShooterSubsystem.getInstance().setWantedState(ShooterStates.SHOOT);
                    IntakeSubsystem.getInstance().setWantedState(IntakeStates.SQUEEZE);
                }),
                new WaitCommand(2.5));
    }
}
```

### Key pieces

- **`loadTrajectory(name)` / `getTrajectory(name)`** — trajectory names come from the generated
  `ChoreoTraj` enum (`ChoreoTraj.X.name()`). Load first, in run order, so start poses line up.
- **Event markers** — `SwerveSubsystem.getInstance().getChoreoEventTimeTrigger("Marker Name")`
  returns a `Trigger` you bind with `.onTrue(...)`. There is also a pose-based variant. **Bind these
  before `addCommands`.**
- **Driving the path** — `setDesiredChoreoTrajectoryCommand(getTrajectory(...))` selects the path,
  then request swerve `CHOREO_PATH` (or `CHOREO_PATH_ROTATION_LOCK` to hold a heading, e.g. aiming
  while moving). Wrap in `Commands.startEnd(...).until(...)` and revert to a field-centric state on end.
- **Progress conditions** — `SwerveSubsystem.getInstance()::isAtChoreoSetpoint` and
  `hasChoreoTimeElapsed(seconds)` are the standard `.until(...)` conditions.

## Swerve states you'll request

- `FIELD_CENTRIC` — normal teleop driving.
- `FIELD_CENTRIC_ROTATION_LOCK` — field-centric translation, heading locked (e.g. aiming).
- `CHOREO_PATH` — follow the selected trajectory.
- `CHOREO_PATH_ROTATION_LOCK` — follow the path while holding a locked heading.

Alliance flipping is handled by the swerve/localization layer
(`SwerveConstants.FLIP_TRAJECTORY_ON_RED` + `AllianceFlipUtil`), so author paths for one alliance.

## Registering an auto

Register instances with `AutoManager` in `Robot`'s constructor; it builds the dashboard chooser and
`Field2d` preview:

```java
AutoManager.getInstance().registerAutos(
        new Left_Trench_Bump_Swipe(),
        new Right_Trench_Trench_Swipe(),
        new Shoot());
```

`autonomousInit()` runs `AutoManager.getInstance().getSelectedAuto()`. An auto that isn't registered
won't appear or run.

> Generated files `ChoreoTraj.java` / `ChoreoVars.java` are Spotless-excluded — don't hand-edit them.
