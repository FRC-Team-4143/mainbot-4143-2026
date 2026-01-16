package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.mw_lib.subsystem.MwConstants;
import frc.mw_lib.swerve_lib.SwerveDriveConfig;
import frc.mw_lib.swerve_lib.module.ModuleType;
import frc.mw_lib.swerve_lib.module.SwerveModuleConfig;
import frc.mw_lib.util.FxMotorConfig;
import frc.mw_lib.util.PhoenixUtil;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class SwerveConstants extends MwConstants {
    // Current system states for the swerve drive
    public enum SwerveStates {
        FIELD_CENTRIC,
        ROBOT_CENTRIC,
        CHOREO_PATH,
        ROTATION_LOCK,
        TRACTOR_BEAM,
        IDLE
    }

    // Deadband for joystick axis in swerve
    public final double CONTROLLER_DEADBAND = 0.05;

    // CAN bus name and CAN ID for the pigeon2
    public final int PIGEON2_ID = 0;
    public final String PIGEON2_CANBUS_NAME = "rio";

    // CAN bus names for each of the swerve modules
    public final String MODULE_CANBUS_NAME = "rio";

    public final double DRIVE_INERTIA = 0.025; // kg*m^2, inertia of the drive motor
    public final double STEER_INERTIA = 0.004; // kg*m^2, inertia of the steer motor
    public final double DRIVE_FRICTION_VOLTAGE = 0.2;
    public final double STEER_FRICTION_VOLTAGE = 0.2;

    public final double ROBOT_MASS_KG = Units.lbsToKilograms(140.0); // Mass of the robot in pounds
    public final double BUMPER_WIDTH_METERS =
            Units.inchesToMeters(34.75); // Width of the bumpers in meters (y
    // axis : left -> right)
    public final double BUMPER_LENGTH_METERS =
            Units.inchesToMeters(34.75); // Length of the bumpers in meters (x
    // axis : front -> back)
    public final double BUMPER_THICKNESS_METERS =
            Units.inchesToMeters(3.0); // Thickness of the bumpers in meters

    // Forward Reference rotation constants
    public enum OperatorPerspective {
        BLUE_ALLIANCE(Rotation2d.fromDegrees(0.0)),
        RED_ALLIANCE(Rotation2d.fromDegrees(180.0));

        private OperatorPerspective(Rotation2d heading) {
            this.heading = heading;
        }

        public final Rotation2d heading;
    }

    // Control Constants for the swerve modules
    public final double SLIP_CURRENT_AMPS = getDoubleConstant("com", "slip_current");
    public final double SPEED_AT_12V_MPS = getDoubleConstant("com", "speed_at_12v");
    public final double COUPLE_RATIO = getDoubleConstant("com", "couple_ratio");
    public final double WHEEL_RADIUS_METERS =
            Units.inchesToMeters(getDoubleConstant("com", "wheel_radius_inches"));
    public final double MAX_TRANSLATION_RATE = getDoubleConstant("com", "max_translation_rate");
    public final double MAX_ANGULAR_RATE = getDoubleConstant("com", "max_angular_rate");

    // Swerve Module Constants
    public final FxMotorConfig DRIVE_MOTOR_CONFIG = new FxMotorConfig();
    public final FxMotorConfig STEER_MOTOR_CONFIG = new FxMotorConfig();

    public final SwerveModuleConfig FL_MODULE_CONFIG = new SwerveModuleConfig();
    public final SwerveModuleConfig FR_MODULE_CONFIG = new SwerveModuleConfig();
    public final SwerveModuleConfig BL_MODULE_CONFIG = new SwerveModuleConfig();
    public final SwerveModuleConfig BR_MODULE_CONFIG = new SwerveModuleConfig();

    public final Translation2d FL_MODULE_TRANSLATION;
    public final Translation2d FR_MODULE_TRANSLATION;
    public final Translation2d BL_MODULE_TRANSLATION;
    public final Translation2d BR_MODULE_TRANSLATION;

    public final SwerveDriveConfig SWERVE_DRIVE_CONFIG;
    public final DriveTrainSimulationConfig SIM_SWERVE_DRIVE_CONFIG;

    // Choreo Constants
    public final boolean FLIP_TRAJECTORY_ON_RED = true;
    public final double CHOREO_TRANSLATION_ERROR_MARGIN = Units.inchesToMeters(1.0);
    public final double CHOREO_X_CONTROLLER_KP = 0.0;
    public final double CHOREO_X_CONTROLLER_KI = 0.0;
    public final double CHOREO_X_CONTROLLER_KD = 0.0;
    public final double CHOREO_Y_CONTROLLER_KP = 0.0;
    public final double CHOREO_Y_CONTROLLER_KI = 0.0;
    public final double CHOREO_Y_CONTROLLER_KD = 0.0;
    public final double CHOREO_THETA_CONTROLLER_KP = 0.0;
    public final double CHOREO_THETA_CONTROLLER_KI = 0.0;
    public final double CHOREO_THETA_CONTROLLER_KD = 0.0;

    // Tractor Beam Constants
    public final double TRACTOR_BEAM_TRANSLATION_ERROR_MARGIN = Units.inchesToMeters(0.5);
    public final double TRACTOR_BEAM_STATIC_FRICTION_CONSTANT = 0.1;
    public final double TRACTOR_BEAM_CONTROLLER_KP = 0.0;
    public final double TRACTOR_BEAM_CONTROLLER_KI = 0.0;
    public final double TRACTOR_BEAM_CONTROLLER_KD = 0.0;

    @SuppressWarnings("unchecked")
    public SwerveConstants() {

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
        // Drive Motor Configuration
        final FxMotorConfig FL_DRIVE_MOTOR_CONFIG = new FxMotorConfig(DRIVE_MOTOR_CONFIG);
        FL_DRIVE_MOTOR_CONFIG.can_id = getIntConstant("fl", "drive_id");
        FL_DRIVE_MOTOR_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(getBoolConstant("fl", "invert_drive"));
        FL_MODULE_CONFIG.drive_motor_config = FL_DRIVE_MOTOR_CONFIG;
        // Steer Motor Configuration
        final FxMotorConfig FL_STEER_MOTOR_CONFIG = new FxMotorConfig(STEER_MOTOR_CONFIG);
        FL_STEER_MOTOR_CONFIG.can_id = getIntConstant("fl", "steer_id");
        FL_MODULE_CONFIG.steer_motor_config = FL_STEER_MOTOR_CONFIG;
        // TODO: Encoder offset

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
        final FxMotorConfig FR_DRIVE_MOTOR_CONFIG = new FxMotorConfig(DRIVE_MOTOR_CONFIG);
        FR_DRIVE_MOTOR_CONFIG.can_id = getIntConstant("fr", "drive_id");
        FR_DRIVE_MOTOR_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(getBoolConstant("fr", "invert_drive"));
        FR_MODULE_CONFIG.drive_motor_config = FR_DRIVE_MOTOR_CONFIG;
        final FxMotorConfig FR_STEER_MOTOR_CONFIG = new FxMotorConfig(STEER_MOTOR_CONFIG);
        FR_STEER_MOTOR_CONFIG.can_id = getIntConstant("fr", "steer_id");
        FR_MODULE_CONFIG.steer_motor_config = FR_STEER_MOTOR_CONFIG;
        // TODO: Encoder offset

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
        final FxMotorConfig BL_DRIVE_MOTOR_CONFIG = new FxMotorConfig(DRIVE_MOTOR_CONFIG);
        BL_DRIVE_MOTOR_CONFIG.can_id = getIntConstant("bl", "drive_id");
        BL_DRIVE_MOTOR_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(getBoolConstant("bl", "invert_drive"));
        BL_MODULE_CONFIG.drive_motor_config = BL_DRIVE_MOTOR_CONFIG;
        final FxMotorConfig BL_STEER_MOTOR_CONFIG = new FxMotorConfig(STEER_MOTOR_CONFIG);
        BL_STEER_MOTOR_CONFIG.can_id = getIntConstant("bl", "steer_id");
        BL_MODULE_CONFIG.steer_motor_config = BL_STEER_MOTOR_CONFIG;
        // TODO: Encoder offset

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
        final FxMotorConfig BR_DRIVE_MOTOR_CONFIG = new FxMotorConfig(DRIVE_MOTOR_CONFIG);
        BR_DRIVE_MOTOR_CONFIG.can_id = getIntConstant("br", "drive_id");
        BR_DRIVE_MOTOR_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(getBoolConstant("br", "invert_drive"));
        BR_MODULE_CONFIG.drive_motor_config = BR_DRIVE_MOTOR_CONFIG;
        final FxMotorConfig BR_STEER_MOTOR_CONFIG = new FxMotorConfig(STEER_MOTOR_CONFIG);
        BR_STEER_MOTOR_CONFIG.can_id = getIntConstant("br", "steer_id");
        BR_MODULE_CONFIG.steer_motor_config = BR_STEER_MOTOR_CONFIG;
        // TODO: Encoder offset

        // Swerve Drive Configuration
        SWERVE_DRIVE_CONFIG =
                new SwerveDriveConfig(
                        FL_MODULE_CONFIG,
                        FR_MODULE_CONFIG,
                        BL_MODULE_CONFIG,
                        BR_MODULE_CONFIG,
                        PIGEON2_ID,
                        PIGEON2_CANBUS_NAME);

        // Swerve Drive Simulation Configuration
        SIM_SWERVE_DRIVE_CONFIG =
                DriveTrainSimulationConfig.Default()
                        .withBumperSize(
                                Meters.of(BUMPER_LENGTH_METERS), Meters.of(BUMPER_WIDTH_METERS))
                        .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
                        .withCustomModuleTranslations(new Translation2d[]{
                            FL_MODULE_TRANSLATION,
                            FR_MODULE_TRANSLATION,
                            BL_MODULE_TRANSLATION,
                            BR_MODULE_TRANSLATION})
                        .withGyro(COTS.ofPigeon2())
                        .withSwerveModules(
                                COTS.ofMark4i(
                                        DCMotor.getKrakenX60(1),
                                        DCMotor.getKrakenX60(1),
                                        COTS.WHEELS.VEX_GRIP_V2.cof,
                                        2));
    }
}
