# Commands & OI

Two files split the responsibility:

- **`OI.java`** — bindings only. Maps controllers/dashboard inputs to commands. The only place
  joysticks/buttons are read.
- **`ControlCommands.java`** — command factories. Each returns a WPILib `Command` that coordinates
  subsystems by requesting states. No hardware access, no joystick reads.

These run through the normal WPILib `CommandScheduler` (unlike subsystems). **Commands never
`require()` a subsystem** — requiring one can preempt/deadlock the state machine. They influence
subsystems purely via `setWantedState(...)` and public mechanism/subsystem helpers.

## Command factory pattern (`ControlCommands.java`)

Factories are `static Command xxxCommand()` methods built from `Commands.startEnd`,
`Commands.runOnce`, or `new FunctionalCommand(...)`. They:
- coordinate multiple subsystems by setting wanted states in the init/end lambdas,
- end with `.withName("...")`, and usually `.ignoringDisable(true)`,
- carry **structured Javadoc** listing the state effects (a strong house idiom — keep it).

```java
/**
 * Aims the robot at the target without shooting (for lining up shots / teleop aiming).
 *
 * <p>On Initialize:
 *
 * <ul>
 *   <li>Shooter: AIMING
 *   <li>Swerve: FIELD_CENTRIC_ROTATION_LOCK
 *   <li>Localization: SHOOTING_FOCUS
 * </ul>
 *
 * <p>On End:
 *
 * <ul>
 *   <li>Shooter: TRACKING
 *   <li>Swerve: FIELD_CENTRIC
 *   <li>Localization: FULL
 * </ul>
 */
static Command aimAtTargetCommand() {
    return Commands.startEnd(
                    () -> {
                        ShooterSubsystem.getInstance().setWantedState(ShooterStates.AIMING);
                        SwerveSubsystem.getInstance()
                                .setWantedState(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
                        LocalizationSubsystem.getInstance()
                                .setWantedState(LocalizationStates.SHOOTING_FOCUS);
                    },
                    () -> {
                        ShooterSubsystem.getInstance().setWantedState(ShooterStates.TRACKING);
                        SwerveSubsystem.getInstance().setWantedState(SwerveStates.FIELD_CENTRIC);
                        LocalizationSubsystem.getInstance()
                                .setWantedState(LocalizationStates.FULL);
                    })
            .withName("Aim At Target")
            .ignoringDisable(true);
}
```

Guidelines:
- **A command does one coherent action.** Little to no logic in the command itself — the logic lives
  in the subsystem state machines it requests.
- Prefer `Commands.startEnd(onStart, onEnd)` for "hold to do X, release to revert", `Commands.runOnce`
  for fire-and-forget state requests, and `FunctionalCommand` when you need an `isFinished` condition.
- The Javadoc "On Initialize / On End" `<ul><li>Subsystem: STATE</li></ul>` block is expected on every
  factory that changes states.

## OI: bindings (`OI.java`)

`public abstract class OI` with two static `CommandXboxController`s and a static
`configureBindings()` called once from `Robot`'s constructor. Sections are separated with banner
comments (`SMARTDASHBOARD COMMANDS`, `DRIVER CONTROLLER BINDINGS`, `OPERATOR CONTROLLER BINDINGS`).

```java
public abstract class OI {
    private static final CommandXboxController driver_controller_ = new CommandXboxController(0);
    private static final CommandXboxController operator_controller_ = new CommandXboxController(1);

    public static void configureBindings() {
        // Dashboard buttons
        SmartDashboard.putData(
                "Zero Gyro Yaw", SwerveSubsystem.getInstance().zeroGyroYaw().ignoringDisable(true));

        // Driver
        driver_controller_.rightTrigger()
                .whileTrue(ControlCommands.shootFuelCommand().ignoringDisable(true));
        driver_controller_.leftTrigger().whileTrue(ControlCommands.aimAtTargetCommand());
        driver_controller_.rightStick()
                .onTrue(SwerveSubsystem.getInstance().toggleFieldCentric().ignoringDisable(true));

        // Operator: simple inline requests are fine for one-liners
        operator_controller_.povUp()
                .onTrue(Commands.runOnce(() -> ShooterSubsystem.getInstance().adjustFlywheel(2)));
    }

    /** @return driver controller left joystick x axis */
    public static double getDriverJoystickLeftX() {
        return driver_controller_.getLeftX();
    }
    // ...more static axis accessors (getDriverJoystickPOV returns Optional<Rotation2d>, etc.)
}
```

Rules:
- **Only `OI` reads inputs.** Subsystems/commands never touch a controller. The lone exception is
  piping drive-axis values into the teleop drive command via the static accessors
  (`OI.getDriverJoystickLeftX()`, `getDriverJoystickPOV()`, …).
- Trigger verbs: `.whileTrue(...)` for hold-to-run, `.onTrue(...)` / `.onFalse(...)` for edges.
- Add `.ignoringDisable(true)` on bindings/commands that should still work while the robot is disabled
  (calibration, zeroing, toggles).
- Bind to a `ControlCommands.xCommand()` factory when a command touches multiple subsystems; an inline
  `Commands.runOnce(...)` is acceptable for a single trivial state request.

## Where each lives / gets called

- `OI.configureBindings()` is invoked once in `Robot`'s constructor.
- `Robot` mode inits push wanted states directly, e.g.
  `teleopInit()` → `SwerveSubsystem.getInstance().setWantedState(SwerveStates.FIELD_CENTRIC)`.
