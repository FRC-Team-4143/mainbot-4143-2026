# Robot states (RobotContainer) & OI

There are **no command files**:
- **`RobotContainer.java`** is the robot's **super state machine**
  (`MwSuperStateMachine<RobotStates>`). It is the only place that coordinates more than one
  subsystem.
- **`OI.java`** only binds buttons to robot states.
- **Autos** ask for the same robot states as teleop.

```
 OI / BT autos        robot.setWantedState(RobotStates.SHOOT)
   │
 RobotContainer       handleStateTransition: exit old state / enter new state (subsystem requests)
   │                  updateLogic: scoring mode, shooter target, per-loop gating (SHOOT feed)
   │  setWantedState(...)
 Subsystems           each subsystem's own state machine
```

`doControlLoop()` runs RobotContainer **before** the subsystems, so they see its requests in the
same loop. It logs `Robot/WantedState`, `Robot/State` and `Robot/ScoringMode`.

## Adding a robot behaviour

1. Add a value to `RobotStates` (per-value Javadoc).
2. In `RobotContainer`, in `enterState`, request the subsystem states it needs. In `exitState`,
   undo them.
3. If part of it must be checked every loop (gating, like feeding only when the shooter is ready),
   put that in `updateLogic` under `if (system_state_ == RobotStates.X)`.
4. Bind it in `OI` (below). An auto uses it with `<SetWantedState state="X"/>`.

**Multi-step sequences** (aim, then spin up, then feed) are written **once**, as a robot state:
- Add step states to `RobotStates`.
- While the wanted state is the sequence, `handleStateTransition` walks through the step states and
  lands in the sequence state itself when it is done.
- If it can't finish (timeout, no game piece), call `failWantedState()`.

`getStateResult()` reports RUNNING / SUCCESS / FAILURE. The BT `SetWantedState` node waits on it, so
an auto can wrap the sequence in `Timeout` / `Fallback` / `Retry` for auto-only fail handling, while
teleop reuses the same sequence.

**Requests on change, not every loop.** RobotContainer asks for subsystem states only when the
robot state changes, plus any per-loop gating. That leaves room for dashboard buttons (homing, spin
down) and small one-off auto moves (`SetIntakeState` on a path event) to set subsystems in between.

## OI: bindings (`OI.java`)

`public abstract class OI` has two static `CommandXboxController`s and a static `configureBindings()`,
which `Robot`'s constructor calls once. Banner comments separate the sections
(`SMARTDASHBOARD COMMANDS`, `DRIVER CONTROLLER BINDINGS`, `OPERATOR CONTROLLER BINDINGS`).

Buttons use plain lambdas around `setWantedState`, the same syntax as subsystems:

```java
RobotContainer robot = RobotContainer.getInstance();

// hold: the robot is in SHOOT; release: back to DRIVE (whileHeld wraps Commands.startEnd)
whileHeld(driver_controller_.rightTrigger(), RobotStates.SHOOT);

// one-shot
driver_controller_.leftBumper()
        .onFalse(Commands.runOnce(robot::toggleStoreIntake).ignoringDisable(true));

// tuning knobs that are not states can still poke a subsystem helper
operator_controller_.povUp()
        .onTrue(Commands.runOnce(() -> ShooterSubsystem.getInstance().adjustFlywheel(2)));
```

`whileHeld(button, state)` is `Commands.startEnd(() -> robot.setWantedState(state), ...)`. On release
it goes back to `DRIVE`, **but only if `state` is still the wanted state**, so letting go of one
button never cancels a state another button asked for after it.

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
- `teleopInit()` → `DRIVE` (from `IDLE`/`AUTO`, this starts field-centric driving)
- `testInit()` → `TUNING`
