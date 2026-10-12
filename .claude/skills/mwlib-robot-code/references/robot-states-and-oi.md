# Robot states (RobotContainer) & OI

There are **no command files**:
- **`RobotContainer.java`** is the robot's **super state machine**
  (`MwSuperStateMachine<RobotStates, RobotConstants>`), written like a big subsystem: `system_state_`,
  `CONSTANTS` (robot-level values such as the drive speed scales, plus the static robot identity),
  `handleStateTransition`, `updateLogic`. It is the only place that coordinates more than one
  subsystem.
- **`OI.java`** only binds buttons to robot states.
- **Autos** ask for the same robot states as teleop.

```
 OI / BT autos        robot.setWantedState(RobotStates.SHOOT)
   │
 RobotContainer       handleStateTransition: guards / sequence steps (default: go to wanted)
   │                  updateLogic: switch on the robot state, ask every subsystem for a state
   │  setWantedState(...)
 Subsystems           each subsystem's own state machine
```

`doControlLoop()` runs RobotContainer **before** the subsystems, so they see its requests in the
same loop. It logs `Robot/WantedState`, `Robot/State` and `Robot/ScoringMode`.

## Adding a robot behaviour

1. Add a value to the `RobotStates` enum at the top of `RobotContainer` (per-value Javadoc).
2. In `RobotContainer.updateLogic`, add a `case X ->` to the switch that asks **every** subsystem for
   the state it needs (the same shape as a subsystem's `updateLogic`). **No conditions in
   `updateLogic`:** if X behaves differently depending on something (feed only when the shooter is
   ready), split it into step states and pick the step in `handleStateTransition`, like `SHOOT` →
   `SHOOT_WAIT` / `SHOOT_FEED` / `SHOOT_FEED_BRAKED` / `SHOOT_PASS_INTAKE`.
3. Bind it in `OI` (below). An auto uses it with `<SetWantedState state="X"/>`.

**Multi-step sequences** (aim, then spin up, then feed) are written **once**, as a robot state:
- Add step states to `RobotStates`.
- While the wanted state is the sequence, `handleStateTransition` walks through the step states and
  lands in the sequence state itself when it is done.
- If it can't finish (timeout, no game piece), call `failWantedState()`.

`getStateResult()` reports RUNNING / SUCCESS / FAILURE. The BT `SetWantedState` node waits on it, so
an auto can wrap the sequence in `Timeout` / `Fallback` / `Retry` for auto-only fail handling, while
teleop reuses the same sequence.

**Hands-off states.** In `AUTO` and `MAINTENANCE` (Test mode) the switch asks for nothing, so autos'
subsystem nodes (`SetIntakeState` on a path event) and maintenance dashboard buttons (homing,
tuning) set subsystem states directly. Maintenance tasks live outside the robot state machine.
Driver toggles (field centric, intake stowed) are flags on RobotContainer that the states read.

## OI: bindings (`OI.java`)

`public abstract class OI` has two static `CommandXboxController`s and a static `configureBindings()`,
which `Robot`'s constructor calls once. Banner comments separate the sections
(`SMARTDASHBOARD COMMANDS`, `DRIVER CONTROLLER BINDINGS`, `OPERATOR CONTROLLER BINDINGS`).

Buttons use plain lambdas around `setWantedState`, the same syntax as subsystems:

```java
RobotContainer robot = RobotContainer.getInstance();

// hold: the robot wants SHOOT; release: back to DRIVE (whileHeld wraps Commands.startEnd)
whileHeld(driver_controller_.rightTrigger(), RobotStates.SHOOT, RobotStates.DRIVE);
// release to TRACK instead, so the flywheel stays at speed after aiming
whileHeld(driver_controller_.leftTrigger(), RobotStates.AIM, RobotStates.TRACK);

// one-shot
driver_controller_.leftBumper()
        .onFalse(Commands.runOnce(robot::toggleStoreIntake).ignoringDisable(true));

// tuning knobs that are not states can still poke a subsystem helper
operator_controller_.povUp()
        .onTrue(Commands.runOnce(() -> ShooterSubsystem.getInstance().adjustFlywheel(2)));
```

`whileHeld(button, state, release)` is `Commands.startEnd(() -> robot.setWantedState(state), ...)`.
On release it asks for `release`, **but only if `state` is still the wanted state**, so letting go
of one button never cancels a state another button asked for after it.

Rules:
- **Only `OI` reads inputs.** Subsystems and RobotContainer never touch a controller. The one
  exception is piping drive-axis values into the swerve's driver inputs through the static accessors
  (`OI.getDriverJoystickLeftX()`, `getDriverJoystickPOV()`, …).
- Bindings never `require()` a subsystem. MW-Lib subsystems are not WPILib `Subsystem`s.
- Add `.ignoringDisable(true)` to bindings that should still work while the robot is disabled
  (calibration, zeroing, toggles).
- If a binding would set more than one subsystem, make it a robot state instead.

## Robot mode inits

`Robot` sets robot states, never subsystem states:
- `disabledInit()` → `IDLE`
- `autonomousInit()` → `AUTO`
- `teleopInit()` → `DRIVE`
- `testInit()` → `MAINTENANCE` (hands off; it also puts the shooter and intake in `TUNING` directly)
