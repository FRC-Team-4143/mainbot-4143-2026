package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.marswars.geometry.LaunchTrajectory;
import com.marswars.subsystem.MwConstants;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.FxMotorConfig.FxMotorType;
import com.marswars.util.PhoenixUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class ShooterConstants extends MwConstants {

    // =============================================================================
    // ENUMS AND STATE DEFINITIONS
    // =============================================================================

    public enum ShooterStates {
        AIMING,
        DUMP,
        SHOOT,
        IDLE,
        TRACKING,
        TUNING,
        MANUAL
    }

    // =============================================================================
    // CAN IDS AND HARDWARE CONFIGURATION
    // =============================================================================

    // Motor CAN IDs
    public final int SHOOTER_LEADER_ID = 10;
    public final int SHOOTER_FOLLOWER_1_ID = 11;
    public final int SHOOTER_FOLLOWER_2_ID = 12;
    public final int SHOOTER_FOLLOWER_3_ID = 13;
    public final int INDEXER_LEADER_ID = 14;
    public final int INDEXER_FOLLOWER_ID = 15;
    public final int HOOD_ID = 16;
    public final int TURRET_ID = 19; // bookmark

    // =============================================================================
    // MECHANICAL CONSTANTS - FLYWHEEL
    // =============================================================================

    public final boolean FLYWHEEL_LEADER_INVERTED = false;
    public final boolean FLYWHEEL_FOLLOWER_1_INVERTED = false;
    public final boolean FLYWHEEL_FOLLOWER_2_INVERTED = false;
    public final boolean FLYWHEEL_FOLLOWER_3_INVERTED = false;
    public final double FLYWHEEL_GEAR_RATIO = 1.0;
    public final double FLYWHEEL_WHEEL_RADIUS_METERS = Units.inchesToMeters(2);
    public final double FLYWHEEL_MASS_KG = Units.lbsToKilograms(3.5);
    public final double FLYWHEEL_INERTIA =
            0.5
                    * FLYWHEEL_MASS_KG
                    * Math.pow(FLYWHEEL_WHEEL_RADIUS_METERS, 2.0); // kg m^2, approximate
    public final double FLYWHEEL_EFF_FACTOR = 1.0;
    public final Slot1Configs FLYWHEEL_VELOCITY_GAINS =
            new Slot1Configs().withKP(0.5).withKV(0.117);

    // =============================================================================
    // MECHANICAL CONSTANTS - INDEXER
    // =============================================================================

    public final boolean INDEXER_LEADER_INVERTED = false;
    public final boolean INDEXER_FOLLOWER_INVERTED = false;
    public final double INDEXER_GEAR_RATIO = 24.0 / 18.0;

    // =============================================================================
    // MECHANICAL CONSTANTS - HOOD
    // =============================================================================

    public final boolean HOOD_INVERTED = false;
    public final double HOOD_GEAR_RATIO =
            (5.0 * (372.0 / 40.0)); // motor rotations / output mechanism rotations
    // Min/max physical hood angles (radians). Configure to match the mechanical limits
    public final double HOOD_MIN_ANGLE = Units.degreesToRadians(45);
    public final double HOOD_MAX_ANGLE = Units.degreesToRadians(83.673);
    public final double HOOD_HOME_POSITION = Units.degreesToRadians(83.673);
    public final Slot0Configs HOOD_POSITION_GAINS = new Slot0Configs().withKP(30).withKD(0.15);

    // =============================================================================
    // MECHANICAL CONSTANTS - TURRET (needs actual values)
    // =============================================================================

    public final double TURRET_GEAR_RATIO = 1.0;
    public final double TURRET_MOI = 0.001;
    public final Slot0Configs TURRET_POSITION_GAINS = new Slot0Configs().withKP(10);

    // =============================================================================
    // CONTROL AND OPERATIONAL CONSTANTS
    // =============================================================================
    public final double INDEXER_DUTY_CYCLE = 0.3; // 30% power for indexing
    public final double LAUNCH_HEIGHT = Units.inchesToMeters(getDoubleConstant("translation", "z"));
    public final LaunchTrajectory SOLVER =
            new LaunchTrajectory(new Translation3d(), LAUNCH_HEIGHT, true);
    public final double MAX_TURRET_WRAP = Units.degreesToRadians(190);
    public final Transform2d SHOOTER_CENTER =
            new Transform2d(
                    new Translation2d(
                            getDoubleConstant("translation", "x"),
                            getDoubleConstant("translation", "y")),
                    new Rotation2d());

    // =============================================================================
    // MOTOR CONFIGURATION OBJECTS
    // =============================================================================
    public final FxMotorConfig SHOOTER_LEADER_MOTOR_CONFIG = new FxMotorConfig();
    public final FxMotorConfig SHOOTER_FOLLOWER_MOTOR_1_CONFIG = new FxMotorConfig();
    public final FxMotorConfig SHOOTER_FOLLOWER_MOTOR_2_CONFIG = new FxMotorConfig();
    public final FxMotorConfig SHOOTER_FOLLOWER_MOTOR_3_CONFIG = new FxMotorConfig();
    public final FxMotorConfig INDEXER_LEADER_MOTOR_CONFIG = new FxMotorConfig();
    public final FxMotorConfig INDEXER_FOLLOWER_MOTOR_CONFIG = new FxMotorConfig();
    public final FxMotorConfig HOOD_MOTOR_CONFIGS = new FxMotorConfig();
    public final FxMotorConfig TURRET_MOTOR_CONFIGS = new FxMotorConfig();
    public final boolean TURRET_ENABLED = getBoolConstant("turret_enabled");

    // =============================================================================
    // CONSTRUCTOR - MOTOR CONFIGURATION INITIALIZATION
    // =============================================================================

    public ShooterConstants() {
        // Configure Indexer Leader Motor
        INDEXER_LEADER_MOTOR_CONFIG.can_id = INDEXER_LEADER_ID;
        INDEXER_LEADER_MOTOR_CONFIG.motor_type = FxMotorType.X44;
        INDEXER_LEADER_MOTOR_CONFIG.canbus_name = "rio";
        INDEXER_LEADER_MOTOR_CONFIG.config = new TalonFXConfiguration();
        INDEXER_LEADER_MOTOR_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(INDEXER_LEADER_INVERTED);

        // Configure Indexer FOLLOWER Motor
        INDEXER_FOLLOWER_MOTOR_CONFIG.can_id = INDEXER_FOLLOWER_ID;
        INDEXER_FOLLOWER_MOTOR_CONFIG.motor_type = FxMotorType.X44;
        INDEXER_FOLLOWER_MOTOR_CONFIG.canbus_name = "rio";
        INDEXER_FOLLOWER_MOTOR_CONFIG.config = new TalonFXConfiguration();
        INDEXER_FOLLOWER_MOTOR_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(INDEXER_FOLLOWER_INVERTED);

        // Configure Shooter Leader Motor
        SHOOTER_LEADER_MOTOR_CONFIG.can_id = SHOOTER_LEADER_ID;
        SHOOTER_LEADER_MOTOR_CONFIG.motor_type = FxMotorType.X60;
        SHOOTER_LEADER_MOTOR_CONFIG.canbus_name = "rio";
        SHOOTER_LEADER_MOTOR_CONFIG.config = new TalonFXConfiguration();
        SHOOTER_LEADER_MOTOR_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(FLYWHEEL_LEADER_INVERTED);
        SHOOTER_LEADER_MOTOR_CONFIG.config.Slot1 = FLYWHEEL_VELOCITY_GAINS;

        // Configure Shooter Follower 1 Motor
        SHOOTER_FOLLOWER_MOTOR_1_CONFIG.can_id = SHOOTER_FOLLOWER_1_ID;
        SHOOTER_FOLLOWER_MOTOR_1_CONFIG.motor_type = FxMotorType.X60;
        SHOOTER_FOLLOWER_MOTOR_1_CONFIG.canbus_name = "rio";
        SHOOTER_FOLLOWER_MOTOR_1_CONFIG.config = new TalonFXConfiguration();
        SHOOTER_FOLLOWER_MOTOR_1_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(FLYWHEEL_FOLLOWER_1_INVERTED);

        // Configure Shooter Follower 2 Motor
        SHOOTER_FOLLOWER_MOTOR_2_CONFIG.can_id = SHOOTER_FOLLOWER_2_ID;
        SHOOTER_FOLLOWER_MOTOR_2_CONFIG.motor_type = FxMotorType.X60;
        SHOOTER_FOLLOWER_MOTOR_2_CONFIG.canbus_name = "rio";
        SHOOTER_FOLLOWER_MOTOR_2_CONFIG.config = new TalonFXConfiguration();
        SHOOTER_FOLLOWER_MOTOR_2_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(FLYWHEEL_FOLLOWER_2_INVERTED);

        // Configure Shooter Follower 3 Motor
        SHOOTER_FOLLOWER_MOTOR_3_CONFIG.can_id = SHOOTER_FOLLOWER_3_ID;
        SHOOTER_FOLLOWER_MOTOR_3_CONFIG.motor_type = FxMotorType.X60;
        SHOOTER_FOLLOWER_MOTOR_3_CONFIG.canbus_name = "rio";
        SHOOTER_FOLLOWER_MOTOR_3_CONFIG.config = new TalonFXConfiguration();
        SHOOTER_FOLLOWER_MOTOR_3_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(FLYWHEEL_FOLLOWER_3_INVERTED);

        // Configure Hood Motor
        HOOD_MOTOR_CONFIGS.can_id = HOOD_ID;
        HOOD_MOTOR_CONFIGS.motor_type = FxMotorType.X44;
        HOOD_MOTOR_CONFIGS.canbus_name = "rio";
        HOOD_MOTOR_CONFIGS.config = new TalonFXConfiguration();
        HOOD_MOTOR_CONFIGS.config.MotorOutput.Inverted = PhoenixUtil.toInvertedValue(HOOD_INVERTED);
        HOOD_MOTOR_CONFIGS.config.Slot0 = HOOD_POSITION_GAINS;

        // Configure Turret Motor
        TURRET_MOTOR_CONFIGS.can_id = TURRET_ID;
        TURRET_MOTOR_CONFIGS.motor_type = FxMotorType.X44;
        TURRET_MOTOR_CONFIGS.canbus_name = "rio";
        TURRET_MOTOR_CONFIGS.config = new TalonFXConfiguration();
        TURRET_MOTOR_CONFIGS.config.Slot0 = TURRET_POSITION_GAINS;

        // Solver Map Population
        SOLVER.addVelocityPoint(0.0, 6.283);
        SOLVER.addVelocityPoint(0.5, 6.382);
        SOLVER.addVelocityPoint(1.0, 6.635);
        SOLVER.addVelocityPoint(1.5, 6.977);
        SOLVER.addVelocityPoint(2.0, 7.367);
        SOLVER.addVelocityPoint(2.5, 7.764);
        SOLVER.addVelocityPoint(3.0, 8.160);
        SOLVER.addVelocityPoint(3.5, 8.544);
        SOLVER.addVelocityPoint(4.0, 8.928);
        SOLVER.addVelocityPoint(4.5, 9.288);
        SOLVER.addVelocityPoint(5.0, 9.648);
        SOLVER.addVelocityPoint(5.5, 9.996);
        SOLVER.addVelocityPoint(6.0, 10.332);
        SOLVER.addVelocityPoint(6.5, 10.656);
        SOLVER.addVelocityPoint(7.0, 10.980);
        SOLVER.addVelocityPoint(7.5, 11.292);
        SOLVER.addVelocityPoint(8.0, 11.592);
        SOLVER.addVelocityPoint(8.5, 11.892);
        SOLVER.addVelocityPoint(9.0, 12.180);
        SOLVER.addVelocityPoint(9.5, 12.456);
        SOLVER.addVelocityPoint(10.0, 12.744);
    }
}
