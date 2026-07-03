# Subsystems & Constants

A subsystem is a **singleton state machine** extending `MwSubsystem<StatesEnum, ConstantsClass>`.
Each subsystem folder holds exactly two files:

```
subsystems/<name>/
├── <Name>Subsystem.java   // the state machine + mechanisms
└── <Name>Constants.java   // public final constants + the state enum + MotorConfigs
```

Canonical examples: `subsystems/intake/` (mechanism-driven) and
`subsystems/gamestates/GameStatesSubsystem.java` (logic-only, no motors).

## Subsystem skeleton (keep methods in this order)

```java
public class IntakeSubsystem extends MwSubsystem<IntakeStates, IntakeConstants> {
    private static IntakeSubsystem instance_ = null;

    // Mechanism (IO) fields + any filters/timers/debouncers
    private RollerMech roller_0_;
    private ArmMech pivot_;
    private Timer timer = new Timer();

    // 1. getInstance — lazy singleton
    public static IntakeSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new IntakeSubsystem();
        }
        return instance_;
    }

    // 2. Constructor — super(DEFAULT_STATE, new XConstants()); then build mechanisms
    public IntakeSubsystem() {
        super(IntakeStates.STORE, new IntakeConstants());
        roller_0_ =
                new RollerMech(
                        getSubsystemKey(),                    // logging prefix
                        "Roller0",                            // instance name
                        List.of(CONSTANTS.ROLLER_MOTOR_CONFIG),
                        CONSTANTS.ROLLER_GEAR_RATIO);
        pivot_ =
                new ArmMech(
                        getSubsystemKey(), "Pivot",
                        List.of(CONSTANTS.PIVOT_MOTOR_CONFIG),
                        CONSTANTS.PIVOT_GEAR_RATIO,
                        CONSTANTS.PIVOT_LENGTH, CONSTANTS.PIVOT_MASS,
                        CONSTANTS.PIVOT_MIN, CONSTANTS.PIVOT_MAX);
    }

    // 3. reset — put the subsystem in a safe state
    @Override
    public void reset() {
        system_state_ = IntakeStates.IDLE;
    }

    // 4. getIos — every mechanism the framework should tick (read/write/log)
    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(roller_0_, pivot_);
    }

    // 5. handleStateTransition — the transition table (only override if you need guards)
    @Override
    protected void handleStateTransition(IntakeStates wantedState) {
        if (wantedState == IntakeStates.SQUEEZE && system_state_ == IntakeStates.DEPLOYED) {
            system_state_ = IntakeStates.SQUEEZE_WAIT;   // insert an intermediate/timed state
            timer.reset();
            timer.start();
        } else {
            system_state_ = wantedState;                 // default: accept the requested state
        }
    }

    // 6. updateLogic — switch on the CURRENT state, command mechanisms
    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case INTAKE:
                roller_0_.setTargetDutyCycle(CONSTANTS.INTAKE_DUTY_CYCLE);
                pivot_.setTargetPosition(CONSTANTS.PIVOT_DEPLOY_POSITION);
                break;
            // ... one case per state ...
            default:
            case IDLE:
                roller_0_.setTargetDutyCycle(0.0);
                pivot_.setTargetDutyCycle(0.0);
                break;
        }
    }

    // 7. Public helpers (getters / adjusters), then private helpers, under banner comments
    // =============================================================================
    // PUBLIC HELPER METHODS
    // =============================================================================
    public double getPivotAngle() {
        return pivot_.getCurrentPosition();
    }
}
```

### Rules that matter

- **`getInstance()` is a lazy singleton** (`private static instance_`). Constructor is either public
  (as in `IntakeSubsystem`) or `private` (as in `GameStatesSubsystem`) — both patterns exist; prefer
  `private` for new code so nothing constructs a second instance.
- **`super(DEFAULT_STATE, new XConstants())`** sets the initial state and injects the constants (the
  base stores them in the `CONSTANTS` field).
- **`updateLogic()` switches on `system_state_` (the current state)** and commands mechanisms. Always
  end with `default:` falling into a safe/`IDLE` case.
- **`handleStateTransition(wanted)` decides the next state.** The base default just does
  `system_state_ = wanted`. Override it only to insert guards, timed waits, or intermediate states
  (e.g. current-spike homing, `SQUEEZE_WAIT`). Inside it you assign `system_state_` directly — this is
  the *only* place that's allowed.
- **`getIos()` returns the mechanisms** so the framework ticks their hardware IO. A **logic-only
  subsystem returns `new ArrayList<SubsystemIoBase>()`** and commands other subsystems via their
  singletons instead of driving motors.
- **Never override `update()`** — it's the base tick and auto-logs `WantedState`/`State`.
- **Cross-subsystem interaction is direct singleton calls** inside `updateLogic`/`handleStateTransition`,
  e.g. `LocalizationSubsystem.getInstance().getFieldPose()`.

## The matching `<Name>Constants` class

Extends `MwConstants`. Fields are **`public final` instance fields** (not `static`); only values
that differ between physical robots are non-final (see the per-robot variants section below).
Organize with banner comments in this order:

```java
public class IntakeConstants extends MwConstants {

    // =============================================================================
    // ENUMS AND STATE DEFINITIONS
    // =============================================================================
    public enum IntakeStates {
        /** Intake stowed in robot */
        STORE,
        /** Homing the pivot by driving until a current spike */
        PIVOT_HOMING,
        // ... every state gets a one-line Javadoc ...
        TUNING
    }

    // =============================================================================
    // CAN IDS AND HARDWARE CONFIGURATION
    // =============================================================================
    public final int PIVOT_MOTOR_ID = 31;

    // =============================================================================
    // MECHANICAL CONSTANTS - PIVOT
    // =============================================================================
    public final double PIVOT_GEAR_RATIO = (50.0 / 12.0) * (50.0 / 24.0) * (32.0 / 14.0);
    public final double PIVOT_LENGTH = Units.inchesToMeters(13.0);
    public final double PIVOT_MASS = Units.lbsToKilograms(7.875);
    public final double PIVOT_MIN = Units.degreesToRadians(-10);
    public final double PIVOT_MAX = Units.degreesToRadians(140);
    public final Slot0Configs PIVOT_POSITION_GAINS =
            new Slot0Configs().withKG(0.45).withKP(15.0).withKD(0.0);

    // =============================================================================
    // MOTOR CONFIGURATION OBJECTS
    // =============================================================================
    public final MotorConfig PIVOT_MOTOR_CONFIG = new MotorConfig();

    // =============================================================================
    // CONSTRUCTOR - MOTOR CONFIGURATION INITIALIZATION
    // =============================================================================
    public IntakeConstants() {
        PIVOT_MOTOR_CONFIG.can_id = PIVOT_MOTOR_ID;
        PIVOT_MOTOR_CONFIG.motor_type = TalonMotorType.X60;   // X60, X44, FALCON500, MINION, NEO_550
        PIVOT_MOTOR_CONFIG.canbus_name = "CANivore";          // "rio" or a named CAN bus
        TalonFXConfiguration pivot_config = new TalonFXConfiguration();
        pivot_config.MotorOutput.Inverted = PhoenixUtil.toInvertedValue(PIVOT_MOTOR_INVERTED);
        pivot_config.CurrentLimits.StatorCurrentLimit = PIVOT_STATOR_CURRENT_LIMIT;
        pivot_config.CurrentLimits.StatorCurrentLimitEnable = true;
        pivot_config.Slot0 = PIVOT_POSITION_GAINS;             // position/velocity gains
        pivot_config.Slot2 = PIVOT_CURRENT_GAINS;              // Slot2 = current-control PID (see mechanisms)
        PIVOT_MOTOR_CONFIG.apply(pivot_config);
    }
}
```

Conventions:
- Use WPILib unit helpers for readability: `Units.degreesToRadians(...)`, `Units.inchesToMeters(...)`,
  `Units.lbsToKilograms(...)`.
- Gains are Phoenix 6 slot config builders (`new Slot0Configs().withKP(...).withKG(...)`).
  **Slot0** = primary position, **Slot1** = velocity, **Slot2** = current-control PID.
- **The state enum lives here**, in the Constants class, and the subsystem imports it
  (`import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;`).

### Per-robot values (multi-robot support)

Everything is Java — there is no JSON config. The base `*Constants` class holds every value with
**AlphaBot defaults**; values that differ between physical robots are the class's only
**non-final** fields, grouped under a `PER-ROBOT CONSTANTS` banner. A robot that differs gets a
small subclass overriding a `configure()` hook, and a static `create()` picks the variant
(see `subsystems/swerve/` for the canonical example):

```java
// SwerveConstants.java (base = AlphaBot)
public double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.8);   // non-final knob

public SwerveConstants() {
    configure();                 // per-robot overrides run first
    // ...then derive motor/module configs from the (possibly overridden) fields
}

protected void configure() {}    // base robot = no-op

public static SwerveConstants create() {
    return switch (RobotId.current()) {
        case BETA_BOT -> new BetaSwerveConstants();
        case SIM_BOT -> new SimSwerveConstants();
        default -> new SwerveConstants();
    };
}

// BetaSwerveConstants.java — ONLY what's unique to BetaBot
@Override
protected void configure() {
    WHEEL_RADIUS_METERS = Units.inchesToMeters(1.978);
}
```

The subsystem constructs via the factory: `super(SwerveStates.IDLE, SwerveConstants.create())`.
`configure()` runs during the base constructor — overrides must only assign self-contained values
to base fields, never read subclass state. The active robot comes from `frc.robot.RobotId.current()`
(MW-Lib `RobotIdentity`: persistent `MWPreferences` "RobotName" burned via the Test-mode
`Config/Burn RobotName` dashboard button, or `SimBot`/`ROBOT_NAME` env var in sim). Subsystems with
no per-robot variance need none of this — plain `new XConstants()` stays.

## Register the subsystem (required)

A subsystem that isn't registered never ticks. Add it in `RobotContainer`'s constructor:

```java
public RobotContainer() {
    super(BuildConstants.class);
    // !!!!!! ALL SUBSYSTEMS MUST BE REGISTERED HERE TO RUN !!!!!!!
    registerSubsystem(SwerveSubsystem.getInstance());
    registerSubsystem(IntakeSubsystem.getInstance());
    // sim-only subsystems are gated:
    if (RobotBase.isSimulation() && !MwLog.isReplay()) {
        registerSubsystem(SimulationSubsystem.getInstance());
    }
    // !!!!! LEAVE reset() AS THE LAST LINE IN THE CONSTRUCTOR !!!!!!
    reset();
}
```

## Requesting states from elsewhere

External code (commands, `Robot` mode inits, other subsystems) drives a subsystem only by requesting
a state:

```java
IntakeSubsystem.getInstance().setWantedState(IntakeStates.INTAKE);
```

Never touch `system_state_` from outside. The requested state is "sticky" — `handleStateTransition`
re-evaluates it every loop until something requests a different one.
