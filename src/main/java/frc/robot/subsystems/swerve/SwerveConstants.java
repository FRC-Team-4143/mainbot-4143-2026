package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.marswars.mechanisms.MotorConfig;
import com.marswars.mechanisms.MotorConfig.TalonMotorType;
import com.marswars.swerve_lib.MwSwerveConstants;
import com.marswars.swerve_lib.SwerveDriveConfig;
import com.marswars.swerve_lib.module.ModuleType;
import com.marswars.swerve_lib.module.SwerveModuleConfig;
import com.marswars.util.PhoenixUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotId;

public class SwerveConstants extends MwSwerveConstants {

    // =============================================================================
    // PER-ROBOT CONSTANTS (defaults = AlphaBot; variants reassign in configure())
    // =============================================================================
    // These (plus the inherited MwSwerveConstants tunables) are the only non-final fields. They
    // may only be reassigned inside a configure() override, which runs before any derived config
    // is built.
    // Overrides must assign self-contained values and never read subclass state (the
    // subclass is not initialized yet when configure() runs).

    public String CANBUS_NAME = "CANivore";
    public double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.8);
    public ModuleType MODULE_TYPE = ModuleType.getModuleType("TSN-P13-S18");
    public TalonMotorType STEER_MOTOR_TYPE = TalonMotorType.X44;
    public Slot0Configs DRIVE_GAINS_SLOT0 = new Slot0Configs();
    public Slot1Configs DRIVE_GAINS_SLOT1 =
            new Slot1Configs().withKS(0.2).withKV(0.1).withKA(0.02).withKG(0.3).withKP(8.0);
    public Slot0Configs STEER_GAINS_SLOT0 = new Slot0Configs().withKP(80.0);
    public Slot1Configs STEER_GAINS_SLOT1 = new Slot1Configs();
    public double DRIVE_SUPPLY_CURRENT_LIMIT = 40.0; // Amps
    public double DRIVE_STATOR_CURRENT_LIMIT = 40.0; // Amps
    public double STEER_SUPPLY_CURRENT_LIMIT = 40.0; // Amps
    public double STEER_STATOR_CURRENT_LIMIT = 40.0; // Amps

    /**
     * Per-robot overrides land here; the base class is the AlphaBot configuration. Runs at the
     * start of the constructor, before module/motor configs are derived from these fields.
     */
    protected void configure() {}

    /** Builds the constants variant for the robot the code is running on. */
    public static SwerveConstants create() {
        return switch (RobotId.current()) {
            case BETA_BOT -> new BetaSwerveConstants();
            case SIM_BOT -> new SimSwerveConstants();
            default -> new SwerveConstants();
        };
    }

    // =============================================================================
    // CAN IDS AND HARDWARE CONFIGURATION
    // =============================================================================

    public final int PIGEON2_ID = 0;
    public final String PIGEON2_CANBUS_NAME;

    // =============================================================================
    // PHYSICAL ROBOT CONSTANTS
    // =============================================================================

    public final double ROBOT_MASS_KG = Units.lbsToKilograms(140.0); // Mass of the robot in pounds
    public final double BUMPER_WIDTH_METERS =
            Units.inchesToMeters(34.75); // Width of the bumpers in meters (y axis : left -> right)
    public final double BUMPER_LENGTH_METERS =
            Units.inchesToMeters(34.75); // Length of the bumpers in meters (x axis : front -> back)
    public final double BUMPER_THICKNESS_METERS =
            Units.inchesToMeters(3.0); // Thickness of the bumpers in meters

    // =============================================================================
    // MOTOR AND MECHANICAL CONSTANTS
    // =============================================================================

    public final double DRIVE_INERTIA = 0.025; // kg*m^2, inertia of the drive motor
    public final double STEER_INERTIA = 0.004; // kg*m^2, inertia of the steer motor
    public final double DRIVE_FRICTION_VOLTAGE = 0.2;
    public final double STEER_FRICTION_VOLTAGE = 0.2;

    public final double SLIP_CURRENT_AMPS = 50.0;
    public final double SPEED_AT_12V_MPS = 5.0;
    public final double COUPLE_RATIO = 3.5;

    public final double DRIVE_SUPPLY_CURRENT_TRIGGER_TIME = 0.1; // Seconds
    public final double STEER_SUPPLY_CURRENT_TRIGGER_TIME = 0.1; // Seconds

    // =============================================================================
    // CONTROL, CHOREO AND TRACTOR BEAM TUNABLES
    // =============================================================================
    // Inherited from MwSwerveConstants (MAX_TRANSLATION_RATE, CHOREO_*, TRACTOR_BEAM_*,
    // HEADING_CONTROLLER_*, ...). The library defaults are this robot's values; reassign
    // any of them in configure() to override per robot.

    // =============================================================================
    // SWERVE MODULE CONFIGURATION OBJECTS
    // =============================================================================

    public final SwerveModuleConfig FL_MODULE_CONFIG = new SwerveModuleConfig();
    public final SwerveModuleConfig FR_MODULE_CONFIG = new SwerveModuleConfig();
    public final SwerveModuleConfig BL_MODULE_CONFIG = new SwerveModuleConfig();
    public final SwerveModuleConfig BR_MODULE_CONFIG = new SwerveModuleConfig();

    public final Translation2d FL_MODULE_TRANSLATION;
    public final Translation2d FR_MODULE_TRANSLATION;
    public final Translation2d BL_MODULE_TRANSLATION;
    public final Translation2d BR_MODULE_TRANSLATION;

    public final SwerveDriveConfig SWERVE_DRIVE_CONFIG;

    // Skid-detection threshold (max allowed spread of per-module velocities).
    public final double SKID_DETECTION_RANGE = 0.3;

    // =============================================================================
    // CONSTRUCTOR - SWERVE CONFIGURATION INITIALIZATION
    // =============================================================================

    public SwerveConstants() {
        // Apply per-robot overrides before deriving any configuration from them
        configure();

        // The pigeon rides the same bus as the swerve modules
        PIGEON2_CANBUS_NAME = CANBUS_NAME;

        // Module hardware is identical on every robot: drive_id, steer_id, encoder_id,
        // and module position (inches, x front / y left)
        configureModule(FL_MODULE_CONFIG, 1, 2, 0, 7.5, 7.5);
        configureModule(FR_MODULE_CONFIG, 3, 4, 1, 7.5, -7.5);
        configureModule(BL_MODULE_CONFIG, 5, 6, 2, -7.5, 7.5);
        configureModule(BR_MODULE_CONFIG, 7, 8, 3, -7.5, -7.5);

        FL_MODULE_TRANSLATION =
                new Translation2d(FL_MODULE_CONFIG.location_x, FL_MODULE_CONFIG.location_y);
        FR_MODULE_TRANSLATION =
                new Translation2d(FR_MODULE_CONFIG.location_x, FR_MODULE_CONFIG.location_y);
        BL_MODULE_TRANSLATION =
                new Translation2d(BL_MODULE_CONFIG.location_x, BL_MODULE_CONFIG.location_y);
        BR_MODULE_TRANSLATION =
                new Translation2d(BR_MODULE_CONFIG.location_x, BR_MODULE_CONFIG.location_y);

        SWERVE_DRIVE_CONFIG =
                new SwerveDriveConfig(
                        FL_MODULE_CONFIG,
                        FR_MODULE_CONFIG,
                        BL_MODULE_CONFIG,
                        BR_MODULE_CONFIG,
                        PIGEON2_ID,
                        PIGEON2_CANBUS_NAME,
                        SKID_DETECTION_RANGE);
    }

    @Override
    public SwerveDriveConfig getDriveConfig() {
        return SWERVE_DRIVE_CONFIG;
    }

    // =============================================================================
    // PRIVATE HELPER METHODS
    // =============================================================================

    private void configureModule(
            SwerveModuleConfig module,
            int drive_id,
            int steer_id,
            int encoder_id,
            double x_position_inches,
            double y_position_inches) {
        module.module_type = MODULE_TYPE;
        module.encoder_type = SwerveModuleConfig.EncoderType.ANALOG_ENCODER;
        module.encoder_id = encoder_id;
        module.wheel_radius_m = WHEEL_RADIUS_METERS;
        module.speed_at_12_volts = SPEED_AT_12V_MPS;
        module.location_x = Units.inchesToMeters(x_position_inches);
        module.location_y = Units.inchesToMeters(y_position_inches);
        module.drive_motor_config = buildDriveMotor(drive_id, false);
        module.steer_motor_config = buildSteerMotor(steer_id);
    }

    private MotorConfig buildDriveMotor(int can_id, boolean inverted) {
        MotorConfig motor = new MotorConfig();
        motor.canbus_name = CANBUS_NAME;
        motor.can_id = can_id;
        motor.motor_type = TalonMotorType.X60;
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = PhoenixUtil.toInvertedValue(inverted);
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        // All four modules share the gain config objects; downstream only reads them
        config.Slot0 = DRIVE_GAINS_SLOT0;
        config.Slot1 = DRIVE_GAINS_SLOT1;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = DRIVE_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerTime = DRIVE_SUPPLY_CURRENT_TRIGGER_TIME;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = DRIVE_STATOR_CURRENT_LIMIT;
        motor.apply(config);
        return motor;
    }

    private MotorConfig buildSteerMotor(int can_id) {
        MotorConfig motor = new MotorConfig();
        motor.canbus_name = CANBUS_NAME;
        motor.can_id = can_id;
        motor.motor_type = STEER_MOTOR_TYPE;
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = PhoenixUtil.toInvertedValue(false);
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Slot0 = STEER_GAINS_SLOT0;
        config.Slot1 = STEER_GAINS_SLOT1;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = STEER_SUPPLY_CURRENT_LIMIT;
        config.CurrentLimits.SupplyCurrentLowerTime = STEER_SUPPLY_CURRENT_TRIGGER_TIME;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = STEER_STATOR_CURRENT_LIMIT;
        motor.apply(config);
        return motor;
    }
}
