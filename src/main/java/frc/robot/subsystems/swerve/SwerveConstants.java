package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.marswars.mechanisms.MotorConfig;
import com.marswars.subsystem.MwConstants;
import com.marswars.swerve_lib.SwerveDriveConfig;
import com.marswars.swerve_lib.module.ModuleType;
import com.marswars.swerve_lib.module.SwerveModuleConfig;
import com.marswars.util.PhoenixUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class SwerveConstants extends MwConstants {

    // =============================================================================
    // ENUMS AND STATE DEFINITIONS
    // =============================================================================

    /** Swerve drive operational states defining different control modes. */
    public enum SwerveStates {
        /** Drive relative to the robot's orientation using joystick inputs. */
        ROBOT_CENTRIC,
        /** Drive relative to the field's orientation using joystick inputs. */
        FIELD_CENTRIC,
        /** Follow a pre-planned Choreo trajectory path with Choreo rotation control. */
        CHOREO_PATH,
        /** Drive towards a target pose using feedback control. */
        TRACTOR_BEAM,
        /** Drive using raw ChassisSpeeds including rotation. */
        CHASSIS_SPEEDS,
        /** Slow precision movement relative to robot orientation using POV/D-pad. */
        CRAWL_ROBOT_CENTRIC,
        /** Slow precision movement relative to field orientation using POV/D-pad. */
        CRAWL_FIELD_CENTRIC,
        /** Drive relative to robot orientation with rotation locked to a target heading. */
        ROBOT_CENTRIC_ROTATION_LOCK,
        /** Drive relative to field orientation with rotation locked to a target heading. */
        FIELD_CENTRIC_ROTATION_LOCK,
        /** Follow a Choreo trajectory path with rotation locked to a target heading. */
        CHOREO_PATH_ROTATION_LOCK,
        /** Drive using raw ChassisSpeeds with rotation locked to a target heading. */
        CHASSIS_SPEEDS_ROTATION_LOCK,
        /** Slow precision movement relative to robot with rotation locked to a target heading. */
        CRAWL_ROBOT_CENTRIC_ROTATION_LOCK,
        /** Slow precision movement relative to field with rotation locked to a target heading. */
        CRAWL_FIELD_CENTRIC_ROTATION_LOCK,
        /** Brake mode locking the wheels in an x pattern */
        BRAKE,
        /** Manual tuning mode for testing chassis speeds. */
        TUNING,
        /** Idle state with no movement commands. */
        IDLE
    }

    public enum OperatorPerspective {
        BLUE_ALLIANCE(Rotation2d.fromDegrees(0.0)),
        RED_ALLIANCE(Rotation2d.fromDegrees(180.0));

        private OperatorPerspective(Rotation2d heading) {
            this.heading = heading;
        }

        public final Rotation2d heading;
    }

    // =============================================================================
    // CAN IDS AND HARDWARE CONFIGURATION
    // =============================================================================

    public final int PIGEON2_ID = 0;
    public final String PIGEON2_CANBUS_NAME = "CANivore";
    public final String MODULE_CANBUS_NAME = "CANivore";

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

    public final double SLIP_CURRENT_AMPS = getDoubleConstant("com", "slip_current");
    public final double SPEED_AT_12V_MPS = getDoubleConstant("com", "speed_at_12v");
    public final double COUPLE_RATIO = getDoubleConstant("com", "couple_ratio");
    public final double WHEEL_RADIUS_METERS =
            Units.inchesToMeters(getDoubleConstant("com", "wheel_radius_inches"));

    // =============================================================================
    // CONTROL AND OPERATIONAL CONSTANTS
    // =============================================================================

    public final double CONTROLLER_DEADBAND = 0.1; // Deadband for joystick inputs to prevent drift
    public final double MAX_TRANSLATION_RATE = getDoubleConstant("com", "max_translation_rate");
    public final double MAX_TRANSLATION_ACCEL =
            40.0; // Meters per second squared (Used for slew rate limiters)
    public final double MAX_CRAWL_RATE = 0.5; // Meters per second, max speed during crawl mode
    public final double MAX_ANGULAR_RATE = getDoubleConstant("com", "max_angular_rate");
    public final PhoenixPIDController HEADING_CONTROLLER = new PhoenixPIDController(12, 0.0, 1);

    // Thresholds for determining when the chassis is stationary
    public final double STATIONARY_TRANSLATION_VELOCITY_THRESHOLD =
            0.1; // Meters per second, max translation velocity to be considered stationary
    public final double STATIONARY_ANGULAR_VELOCITY_THRESHOLD =
            0.2; // Radians per second, max angular velocity to be considered stationary

    // =============================================================================
    // CHOREO PATH FOLLOWING CONSTANTS
    // =============================================================================

    public boolean FLIP_TRAJECTORY_ON_RED = false;
    public final double CHOREO_TRANSLATION_ERROR_MARGIN = Units.inchesToMeters(1.0);
    public final double CHOREO_VELOCITY_ERROR_MARGIN = 0.2;
    public final double CHOREO_TRANSLATION_CONTROLLER_KP = 7.0;
    public final double CHOREO_TRANSLATION_CONTROLLER_KI = 0.0;
    public final double CHOREO_TRANSLATION_CONTROLLER_KD = 0.0;
    public final double CHOREO_THETA_CONTROLLER_KP = 12.0; // 7.3;
    public final double CHOREO_THETA_CONTROLLER_KI = 0.0;
    public final double CHOREO_THETA_CONTROLLER_KD = 1.0; // 0.07;

    // =============================================================================
    // TRACTOR BEAM CONSTANTS
    // =============================================================================

    public final double TRACTOR_BEAM_TRANSLATION_ERROR_MARGIN = Units.inchesToMeters(0.5);
    public final double TRACTOR_BEAM_STATIC_FRICTION_CONSTANT = 0.1;
    public final double TRACTOR_BEAM_CONTROLLER_KP = 0.0;
    public final double TRACTOR_BEAM_CONTROLLER_KI = 0.0;
    public final double TRACTOR_BEAM_CONTROLLER_KD = 0.0;

    // =============================================================================
    // SWERVE MODULE CONFIGURATION OBJECTS
    // =============================================================================

    public final MotorConfig DRIVE_MOTOR_CONFIG = new MotorConfig();
    public final MotorConfig STEER_MOTOR_CONFIG = new MotorConfig();

    public final SwerveModuleConfig FL_MODULE_CONFIG = new SwerveModuleConfig();
    public final SwerveModuleConfig FR_MODULE_CONFIG = new SwerveModuleConfig();
    public final SwerveModuleConfig BL_MODULE_CONFIG = new SwerveModuleConfig();
    public final SwerveModuleConfig BR_MODULE_CONFIG = new SwerveModuleConfig();

    public final Translation2d FL_MODULE_TRANSLATION;
    public final Translation2d FR_MODULE_TRANSLATION;
    public final Translation2d BL_MODULE_TRANSLATION;
    public final Translation2d BR_MODULE_TRANSLATION;

    public final SwerveDriveConfig SWERVE_DRIVE_CONFIG;

    // =============================================================================
    // CONSTRUCTOR - SWERVE CONFIGURATION INITIALIZATION
    // =============================================================================

    public SwerveConstants() {

        // Load base motor configurations from config files
        DRIVE_MOTOR_CONFIG.loadFromConfig("swerve", "com", "drive_motor");
        STEER_MOTOR_CONFIG.loadFromConfig("swerve", "com", "steer_motor");

        // ---------------------------------
        // FL Swerve Module Configuration
        // ---------------------------------
        FL_MODULE_CONFIG.module_type = ModuleType.getModuleTypeFromJSON("fl");
        FL_MODULE_CONFIG.encoder_type = SwerveModuleConfig.EncoderType.ANALOG_ENCODER;
        FL_MODULE_CONFIG.encoder_id = getIntConstant("fl", "encoder_id");
        FL_MODULE_CONFIG.wheel_radius_m = WHEEL_RADIUS_METERS;
        FL_MODULE_CONFIG.speed_at_12_volts = SPEED_AT_12V_MPS;
        FL_MODULE_CONFIG.location_x = Units.inchesToMeters(getDoubleConstant("fl", "x_position"));
        FL_MODULE_CONFIG.location_y = Units.inchesToMeters(getDoubleConstant("fl", "y_position"));
        FL_MODULE_TRANSLATION =
                new Translation2d(FL_MODULE_CONFIG.location_x, FL_MODULE_CONFIG.location_y);

        // FL Drive Motor Configuration
        final MotorConfig FL_DRIVE_MOTOR_CONFIG = new MotorConfig(DRIVE_MOTOR_CONFIG);
        FL_DRIVE_MOTOR_CONFIG.can_id = getIntConstant("fl", "drive_id");
        FL_DRIVE_MOTOR_CONFIG.getAsFXConfig().MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(getBoolConstant("fl", "invert_drive"));
        FL_MODULE_CONFIG.drive_motor_config = FL_DRIVE_MOTOR_CONFIG;

        // FL Steer Motor Configuration
        final MotorConfig FL_STEER_MOTOR_CONFIG = new MotorConfig(STEER_MOTOR_CONFIG);
        FL_STEER_MOTOR_CONFIG.can_id = getIntConstant("fl", "steer_id");
        FL_MODULE_CONFIG.steer_motor_config = FL_STEER_MOTOR_CONFIG;

        // ---------------------------------
        // FR Swerve Module Configuration
        // ---------------------------------
        FR_MODULE_CONFIG.module_type = ModuleType.getModuleTypeFromJSON("fr");
        FR_MODULE_CONFIG.encoder_type = SwerveModuleConfig.EncoderType.ANALOG_ENCODER;
        FR_MODULE_CONFIG.encoder_id = getIntConstant("fr", "encoder_id");
        FR_MODULE_CONFIG.wheel_radius_m = WHEEL_RADIUS_METERS;
        FR_MODULE_CONFIG.speed_at_12_volts = SPEED_AT_12V_MPS;
        FR_MODULE_CONFIG.location_x = Units.inchesToMeters(getDoubleConstant("fr", "x_position"));
        FR_MODULE_CONFIG.location_y = Units.inchesToMeters(getDoubleConstant("fr", "y_position"));
        FR_MODULE_TRANSLATION =
                new Translation2d(FR_MODULE_CONFIG.location_x, FR_MODULE_CONFIG.location_y);

        // FR Drive Motor Configuration
        final MotorConfig FR_DRIVE_MOTOR_CONFIG = new MotorConfig(DRIVE_MOTOR_CONFIG);
        FR_DRIVE_MOTOR_CONFIG.can_id = getIntConstant("fr", "drive_id");
        FR_DRIVE_MOTOR_CONFIG.getAsFXConfig().MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(getBoolConstant("fr", "invert_drive"));
        FR_MODULE_CONFIG.drive_motor_config = FR_DRIVE_MOTOR_CONFIG;

        // FR Steer Motor Configuration
        final MotorConfig FR_STEER_MOTOR_CONFIG = new MotorConfig(STEER_MOTOR_CONFIG);
        FR_STEER_MOTOR_CONFIG.can_id = getIntConstant("fr", "steer_id");
        FR_MODULE_CONFIG.steer_motor_config = FR_STEER_MOTOR_CONFIG;

        // ---------------------------------
        // BL Swerve Module Configuration
        // ---------------------------------
        BL_MODULE_CONFIG.module_type = ModuleType.getModuleTypeFromJSON("bl");
        BL_MODULE_CONFIG.encoder_type = SwerveModuleConfig.EncoderType.ANALOG_ENCODER;
        BL_MODULE_CONFIG.encoder_id = getIntConstant("bl", "encoder_id");
        BL_MODULE_CONFIG.wheel_radius_m = WHEEL_RADIUS_METERS;
        BL_MODULE_CONFIG.speed_at_12_volts = SPEED_AT_12V_MPS;
        BL_MODULE_CONFIG.location_x = Units.inchesToMeters(getDoubleConstant("bl", "x_position"));
        BL_MODULE_CONFIG.location_y = Units.inchesToMeters(getDoubleConstant("bl", "y_position"));
        BL_MODULE_TRANSLATION =
                new Translation2d(BL_MODULE_CONFIG.location_x, BL_MODULE_CONFIG.location_y);

        // BL Drive Motor Configuration
        final MotorConfig BL_DRIVE_MOTOR_CONFIG = new MotorConfig(DRIVE_MOTOR_CONFIG);
        BL_DRIVE_MOTOR_CONFIG.can_id = getIntConstant("bl", "drive_id");
        BL_DRIVE_MOTOR_CONFIG.getAsFXConfig().MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(getBoolConstant("bl", "invert_drive"));
        BL_MODULE_CONFIG.drive_motor_config = BL_DRIVE_MOTOR_CONFIG;

        // BL Steer Motor Configuration
        final MotorConfig BL_STEER_MOTOR_CONFIG = new MotorConfig(STEER_MOTOR_CONFIG);
        BL_STEER_MOTOR_CONFIG.can_id = getIntConstant("bl", "steer_id");
        BL_MODULE_CONFIG.steer_motor_config = BL_STEER_MOTOR_CONFIG;

        // ---------------------------------
        // BR Swerve Module Configuration
        // ---------------------------------
        BR_MODULE_CONFIG.module_type = ModuleType.getModuleTypeFromJSON("br");
        BR_MODULE_CONFIG.encoder_type = SwerveModuleConfig.EncoderType.ANALOG_ENCODER;
        BR_MODULE_CONFIG.encoder_id = getIntConstant("br", "encoder_id");
        BR_MODULE_CONFIG.wheel_radius_m = WHEEL_RADIUS_METERS;
        BR_MODULE_CONFIG.speed_at_12_volts = SPEED_AT_12V_MPS;
        BR_MODULE_CONFIG.location_x = Units.inchesToMeters(getDoubleConstant("br", "x_position"));
        BR_MODULE_CONFIG.location_y = Units.inchesToMeters(getDoubleConstant("br", "y_position"));
        BR_MODULE_TRANSLATION =
                new Translation2d(BR_MODULE_CONFIG.location_x, BR_MODULE_CONFIG.location_y);

        // BR Drive Motor Configuration
        final MotorConfig BR_DRIVE_MOTOR_CONFIG = new MotorConfig(DRIVE_MOTOR_CONFIG);
        BR_DRIVE_MOTOR_CONFIG.can_id = getIntConstant("br", "drive_id");
        BR_DRIVE_MOTOR_CONFIG.getAsFXConfig().MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(getBoolConstant("br", "invert_drive"));
        BR_MODULE_CONFIG.drive_motor_config = BR_DRIVE_MOTOR_CONFIG;

        // BR Steer Motor Configuration
        final MotorConfig BR_STEER_MOTOR_CONFIG = new MotorConfig(STEER_MOTOR_CONFIG);
        BR_STEER_MOTOR_CONFIG.can_id = getIntConstant("br", "steer_id");
        BR_MODULE_CONFIG.steer_motor_config = BR_STEER_MOTOR_CONFIG;

        // ---------------------------------
        // Swerve Drive Configuration
        // ---------------------------------
        SWERVE_DRIVE_CONFIG =
                new SwerveDriveConfig(
                        FL_MODULE_CONFIG,
                        FR_MODULE_CONFIG,
                        BL_MODULE_CONFIG,
                        BR_MODULE_CONFIG,
                        PIGEON2_ID,
                        PIGEON2_CANBUS_NAME);
    }
}
