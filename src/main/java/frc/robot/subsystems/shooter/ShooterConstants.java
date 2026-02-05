package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.marswars.geometry.LaunchTrajectory;
import com.marswars.subsystem.MwConstants;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.FxMotorConfig.FxMotorType;
import com.marswars.util.PhoenixUtil;
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
        MANUAL,
        PROFILE
    }

    // =============================================================================
    // CAN IDS AND HARDWARE CONFIGURATION
    // =============================================================================

    // Motor CAN IDs
    public final int SHOOTER_LEADER_ID = 10;
    public final int SHOOTER_FOLLOWER_ID = 11;
    public final int INDEXER_ID = 12;
    public final int HOOD_ID = 13;
    public final int TURRET_ID = 15;

    // =============================================================================
    // MECHANICAL CONSTANTS - FLYWHEEL
    // =============================================================================

    public final boolean FLYWHEEL_LEADER_INVERTED = true;
    public final boolean FLYWHEEL_FOLLOWER_INVERTED = false;
    public final double FLYWHEEL_GEAR_RATIO = 1.0;
    public final double FLYWHEEL_WHEEL_RADIUS_METERS = Units.inchesToMeters(3);
    public final double FLYWHEEL_MASS_KG = 2.3; // kg, approximate
    public final double FLYWHEEL_INERTIA =
            0.5
                    * FLYWHEEL_MASS_KG
                    * Math.pow(FLYWHEEL_WHEEL_RADIUS_METERS, 2.0); // kg m^2, approximate
    public final double FLYWHEEL_EFF_FACTOR = 1.0;
    public final double FLYWHEEL_SPEED_TOLERANCE = 1.0;
    public final Slot1Configs FLYWHEEL_VELOCITY_GAINS =
            new Slot1Configs().withKP(0.5).withKV(0.117);

    // =============================================================================
    // MECHANICAL CONSTANTS - INDEXER
    // =============================================================================

    public final boolean INDEXER_INVERTED = false;
    public final double INDEXER_GEAR_RATIO = 1.0;

    // =============================================================================
    // MECHANICAL CONSTANTS - HOOD
    // =============================================================================

    public final boolean HOOD_INVERTED = true;
    public final double HOOD_GEAR_RATIO = (5.0 * (372.0 / 40.0)); // motor rotations / output mechanism rotations
    // Min/max physical hood angles (radians). Configure to match the mechanical limits
    public final double HOOD_MIN_ANGLE = Units.degreesToRadians(45);
    public final double HOOD_MAX_ANGLE = Units.degreesToRadians(83.673);
    public final double HOOD_HOME_POSITION = Units.degreesToRadians(83.673);

    // Tolerances are expressed in hood position units (radians) for comparing against
    // the mech's current position/readback.
    public final double HOOD_POSITION_TOLERANCE =
            Units.degreesToRadians(2.0); // 2 degrees tolerance
    public final Slot0Configs HOOD_POSITION_GAINS = new Slot0Configs().withKP(30).withKD(0.15);

    // =============================================================================
    // MECHANICAL CONSTANTS - TURRET (needs actual values)
    // =============================================================================

    public final double TURRET_GEAR_RATIO = 1.0;
    public final double TURRET_MOI = 0.001;
    public final double TURRET_ANGLE_TOLERANCE = 1.0;
    public final Slot0Configs TURRET_POSITION_GAINS = new Slot0Configs().withKP(10);

    // =============================================================================
    // CONTROL AND OPERATIONAL CONSTANTS
    // =============================================================================
    public final double INDEXER_DUTY_CYCLE = 0.3; // 30% power for indexing
    public final Translation3d HUB_TRANSLATION =
            new Translation3d(4.611624, 4.021328, 1.397); // where the hub is
    public final double LAUNCH_HEIGHT = Units.inchesToMeters(getDoubleConstant("translation", "z"));
    public final LaunchTrajectory SOLVER =
            new LaunchTrajectory(HUB_TRANSLATION, LAUNCH_HEIGHT, true);
    public final double MAX_TURRET_WRAP = Units.degreesToRadians(190);
    public final Translation2d SHOOTER_CENTER = new Translation2d(0.171, 0.079);

    // =============================================================================
    // MOTOR CONFIGURATION OBJECTS
    // =============================================================================

    public final FxMotorConfig SHOOTER_LEADER_MOTOR_CONFIG = new FxMotorConfig();
    public final FxMotorConfig SHOOTER_FOLLOWER_MOTOR_CONFIG = new FxMotorConfig();
    public final FxMotorConfig INDEX_MOTOR_CONFIG = new FxMotorConfig();
    public final FxMotorConfig HOOD_MOTOR_CONFIGS = new FxMotorConfig();
    public final FxMotorConfig TURRET_MOTOR_CONFIGS = new FxMotorConfig();
    public final boolean TURRET_ENABLED = false;

    // =============================================================================
    // CONSTRUCTOR - MOTOR CONFIGURATION INITIALIZATION
    // =============================================================================

    public ShooterConstants() {
        // Configure Indexer Motor
        INDEX_MOTOR_CONFIG.can_id = INDEXER_ID;
        INDEX_MOTOR_CONFIG.motor_type = FxMotorType.X44;
        INDEX_MOTOR_CONFIG.canbus_name = "CANivore";
        INDEX_MOTOR_CONFIG.config = new TalonFXConfiguration();

        // Configure Shooter Leader Motor
        SHOOTER_LEADER_MOTOR_CONFIG.can_id = SHOOTER_LEADER_ID;
        SHOOTER_LEADER_MOTOR_CONFIG.motor_type = FxMotorType.X60;
        SHOOTER_LEADER_MOTOR_CONFIG.canbus_name = "CANivore";
        SHOOTER_LEADER_MOTOR_CONFIG.config = new TalonFXConfiguration();
        SHOOTER_LEADER_MOTOR_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(FLYWHEEL_LEADER_INVERTED);
        SHOOTER_LEADER_MOTOR_CONFIG.config.Slot1 = FLYWHEEL_VELOCITY_GAINS;

        // Configure Shooter Follower Motor
        SHOOTER_FOLLOWER_MOTOR_CONFIG.can_id = SHOOTER_FOLLOWER_ID;
        SHOOTER_FOLLOWER_MOTOR_CONFIG.motor_type = FxMotorType.X60;
        SHOOTER_FOLLOWER_MOTOR_CONFIG.canbus_name = "CANivore";
        SHOOTER_FOLLOWER_MOTOR_CONFIG.config = new TalonFXConfiguration();
        SHOOTER_FOLLOWER_MOTOR_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(FLYWHEEL_FOLLOWER_INVERTED);

        // Configure Hood Motor
        HOOD_MOTOR_CONFIGS.can_id = HOOD_ID;
        HOOD_MOTOR_CONFIGS.motor_type = FxMotorType.X60;
        HOOD_MOTOR_CONFIGS.canbus_name = "CANivore";
        HOOD_MOTOR_CONFIGS.config = new TalonFXConfiguration();
        HOOD_MOTOR_CONFIGS.config.MotorOutput.Inverted = PhoenixUtil.toInvertedValue(HOOD_INVERTED);
        HOOD_MOTOR_CONFIGS.config.Slot0 = HOOD_POSITION_GAINS;

        // Configure Turret Motor
        TURRET_MOTOR_CONFIGS.can_id = TURRET_ID;
        TURRET_MOTOR_CONFIGS.motor_type = FxMotorType.X44;
        TURRET_MOTOR_CONFIGS.canbus_name = "CANivore";
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
