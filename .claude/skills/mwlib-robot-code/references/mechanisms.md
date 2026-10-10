# Mechanisms (the hardware IO layer)

Mechanisms live in `com.marswars.mechanisms` and wrap CTRE Phoenix 6 motors (TalonFX / TalonFXS).
A subsystem **composes** mechanisms (has-a) and commands them from `updateLogic()`. Mechanisms handle
motor construction, leader/follower setup, unit conversions, alerts, live-tunable PID, **and physics
simulation** — so subsystem logic is identical in sim and on the real robot (`IS_SIM` flips the
internals only).

Available: `ArmMech`, `ElevatorMech`, `FlywheelMech`, `RollerMech`, `TurretMech`.

## Construction

All constructors share the same leading shape:

```
new XMech(logging_prefix, "InstanceName", List.of(motorConfigs...), gear_ratio, ...physical params)
```

- **`logging_prefix`** — always pass `getSubsystemKey()` from the owning subsystem.
- **`"InstanceName"`** — a short name, required when a subsystem has more than one of the same mech
  (e.g. `"Roller0"`, `"Roller1"`). The single-arg overload without a name also exists.
- **motor list** — `List.of(...)`. **The first motor is the leader; the rest auto-follow** it
  (`StrictFollower`). Build the `MotorConfig`s in the `*Constants` class.
- **`gear_ratio`** — motor rotations per mechanism rotation.
- **physical params** — simulation/limits, per mechanism (below).

```java
// Roller (velocity/duty intake-style): gear ratio only, optional roller_inertia
roller_0_ = new RollerMech(getSubsystemKey(), "Roller0",
        List.of(CONSTANTS.ROLLER_MOTOR_CONFIG), CONSTANTS.ROLLER_GEAR_RATIO);

// Arm (pivot): length(m), mass(kg), min/max angle(rad); gravity comp on by default
pivot_ = new ArmMech(getSubsystemKey(), "Pivot",
        List.of(CONSTANTS.PIVOT_MOTOR_CONFIG), CONSTANTS.PIVOT_GEAR_RATIO,
        CONSTANTS.PIVOT_LENGTH, CONSTANTS.PIVOT_MASS, CONSTANTS.PIVOT_MIN, CONSTANTS.PIVOT_MAX);
```

Per-mechanism physical params (see each class's Javadoc for exact overloads — several exist):
- **`ArmMech`** — `length`, `mass_kg`, `min_angle`, `max_angle`, optional `gravity_compensate` (default
  true). Positions/velocities are in **radians** / rad·s⁻¹.
- **`ElevatorMech`** — drum radius, carriage mass, min/max height, `is_vertical` (gravity comp).
  Positions in **meters**.
- **`FlywheelMech`** — wheel moment of inertia / radius. Velocity-focused (no position control).
- **`RollerMech`** — gear ratio, optional `roller_inertia`. Velocity / duty / position / current.
- **`TurretMech`** — rotational, position-focused.

## Commanding a mechanism (setters)

Call these from `updateLogic()`. The setter you call also selects the control mode.

| Method | Available on | Notes |
| --- | --- | --- |
| `setTargetPosition(pos)` | Arm, Elevator, Roller, Turret | Slot0 position control. rad (arm/turret), m (elevator) |
| `setTargetPositionWithFF(pos, ff)` | same | position + arbitrary feedforward |
| `setTargetPositionMotionProfile(pos)` | same (**TalonFX only**) | Motion Magic profile |
| `setTargetPositionMotionProfileWithFF(pos, ff)` | same | profiled + feedforward |
| `setTargetVelocity(vel)` | all | Slot1 velocity control |
| `setTargetVelocityWithFF(vel, ff)` | Flywheel | velocity + feedforward |
| `setTargetVelocityMotionProfile(vel)` | Roller, Flywheel (**TalonFX only**) | profiled velocity |
| `setTargetDutyCycle(dc)` | all | open loop, `[-1, 1]` |
| `setTargetCurrent(amps)` | all | closed-loop **stator current** control (see below) |
| `setCurrentPosition(pos)` | Arm, Elevator, Roller, Turret | **seed/zero** the encoder to a known position |
| `setCurrentLimits(CurrentLimitsConfigs)` | all | change limits at runtime |

> Motion-profile variants exist only on TalonFX (not TalonFXS). If unsure, use the plain
> `setTargetPosition` / `setTargetVelocity`.

## Reading a mechanism (getters)

```java
double pos  = pivot_.getCurrentPosition();      // rad (arm/turret) or m (elevator)
double vel  = pivot_.getCurrentVelocity();       // rad/s or m/s
double supA = pivot_.getLeaderSupplyCurrent();   // battery-side amps (e.g. homing current-spike checks)
double statA = pivot_.getLeaderStatorCurrent();  // motor-winding amps
```

`getLeaderSupplyCurrent()` is the one used for current-spike homing (see `IntakeSubsystem`
`PIVOT_HOMING`). There is no generic `getLeaderCurrent()` — pick supply or stator explicitly.

## Current control (a distinctive feature)

Phoenix 6 has no native current-PID mode, so mechanisms implement one: **Slot2** gains configure a
`PIDController` on **stator current**, and `setTargetCurrent(amps)` drives it. Configure Slot2 in the
`*Constants` motor config (e.g. `PIVOT_CURRENT_GAINS = new Slot2Configs().withKP(...).withKI(...)`) and
call `setTargetCurrent(...)` from a state (e.g. a squeeze/hold that pushes at constant torque).

## Zeroing / homing

- Seed a known position with `setCurrentPosition(pos)` (e.g. on construction, or after a homing move).
- A typical homing sequence: a `*_HOMING` state drives a small `setTargetDutyCycle(...)` until
  `getLeaderSupplyCurrent()` exceeds a threshold (debounced), then `setCurrentPosition(HOME)` and
  transition on — done in `handleStateTransition()`.

## Logging & tunables (automatic)

Each mechanism logs its control mode, targets vs. actuals, and per-motor voltage/current/temp under
`<subsystemKey>/<MechName>/...` every loop, and registers live-tunable PID gains + setpoints under
`/Tuning/...` (via `TunablePid` and `MwLog.tunable`). You don't add logging for a mechanism yourself —
just add subsystem-level logs for your own derived values (see `references/logging.md`).

## Do / don't

- **Do** compose mechanisms as fields; command them only through these setters/getters.
- **Do** put all motor config (CAN id, bus, inversion, current limits, gains) in the `*Constants` class.
- **Don't** extend a `*Mech`, subclass motors, or call `motor.setControl(...)` from a subsystem — go
  through the mechanism API.
- **Don't** read `Timer.getFPGATimestamp()`; mechanisms and subsystems use the passed `timestamp`.
