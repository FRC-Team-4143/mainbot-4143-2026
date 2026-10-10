package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.marswars.mechanisms.MotorConfig.TalonMotorType;
import com.marswars.swerve_lib.MwSwerveConstants;
import com.marswars.swerve_lib.SwerveDriveConfig;
import com.marswars.swerve_lib.module.ModuleType;
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

    public final String CANBUS_NAME = "CANivore";
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

    // =============================================================================
    // CONTROL, CHOREO AND TRACTOR BEAM TUNABLES
    // =============================================================================
    // Inherited from MwSwerveConstants (MAX_TRANSLATION_RATE, CHOREO_*, TRACTOR_BEAM_*,
    // HEADING_CONTROLLER_*, ...). The library defaults are this robot's values; reassign
    // any of them in configure() to override per robot.

    // =============================================================================
    // DRIVETRAIN HARDWARE
    // =============================================================================

    public final SwerveDriveConfig SWERVE_DRIVE_CONFIG;

    public SwerveConstants() {
        // Apply per-robot overrides before deriving any configuration from them
        configure();

        // Module CAN IDs (drive, steer, encoder) and positions are identical on every robot
        SWERVE_DRIVE_CONFIG =
                SwerveDriveConfig.builder()
                        .canbus(CANBUS_NAME)
                        .pigeon2Id(PIGEON2_ID)
                        .moduleType(MODULE_TYPE)
                        .wheelRadius(WHEEL_RADIUS_METERS)
                        .speedAt12V(SPEED_AT_12V_MPS)
                        .driveGains(DRIVE_GAINS_SLOT0, DRIVE_GAINS_SLOT1)
                        .driveCurrentLimits(DRIVE_SUPPLY_CURRENT_LIMIT, DRIVE_STATOR_CURRENT_LIMIT)
                        .steerMotor(STEER_MOTOR_TYPE)
                        .steerGains(STEER_GAINS_SLOT0, STEER_GAINS_SLOT1)
                        .steerCurrentLimits(STEER_SUPPLY_CURRENT_LIMIT, STEER_STATOR_CURRENT_LIMIT)
                        .frontLeft(1, 2, 0, moduleLocation(7.5, 7.5))
                        .frontRight(3, 4, 1, moduleLocation(7.5, -7.5))
                        .backLeft(5, 6, 2, moduleLocation(-7.5, 7.5))
                        .backRight(7, 8, 3, moduleLocation(-7.5, -7.5))
                        .build();
    }

    @Override
    public SwerveDriveConfig getDriveConfig() {
        return SWERVE_DRIVE_CONFIG;
    }

    private static Translation2d moduleLocation(double x_inches, double y_inches) {
        return new Translation2d(Units.inchesToMeters(x_inches), Units.inchesToMeters(y_inches));
    }
}
