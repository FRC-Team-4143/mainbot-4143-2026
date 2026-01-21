package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.marswars.subsystem.MwConstants;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.FxMotorConfig.FxMotorType;
import com.marswars.util.PhoenixUtil;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class ShooterConstants extends MwConstants {

    // =============================================================================
    // ENUMS AND STATE DEFINITIONS
    // =============================================================================
    
    public enum ShooterStates {
        UNWIND,
        AIMING,
        DUMP,
        SHOOT,
        IDLE,
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
    public final int TOP_SPIN_ID = 14;
    public final int TURRET_ID = 15;

    // =============================================================================
    // MECHANICAL CONSTANTS - SHOOTER
    // =============================================================================
    
    public final boolean SHOOTER_LEADER_INVERTED = false;
    public final boolean SHOOTER_FOLLOWER_INVERTED = true;
    public final double SHOOTER_GEAR_RATIO = 1.0;
    public final double SHOOTER_WHEEL_RADIUS_METERS = Units.inchesToMeters(3);
    public final double SHOOTER_WHEEL_MASS_KG = 2.3; // kg, approximate
    public final double SHOOTER_WHEEL_INERTIA =
            0.5
                    * SHOOTER_WHEEL_MASS_KG
                    * Math.pow(SHOOTER_WHEEL_RADIUS_METERS, 2.0); // kg m^2, approximate

    // =============================================================================
    // MECHANICAL CONSTANTS - INDEXER
    // =============================================================================
    
    public final boolean INDEXER_INVERTED = false;
    public final double INDEXER_GEAR_RATIO = 1.0;

    // =============================================================================
    // MECHANICAL CONSTANTS - HOOD (needs actual values)
    // =============================================================================
    
    public final boolean HOOD_INVERTED = false;
    public final double HOOD_GEAR_RATIO = 1.0;
    public final double HOOD_LENGTH = Units.inchesToMeters(8.25);
    public final double HOOD_MASS_KG = Units.lbsToKilograms(1);
    public final double HOOD_MIN_ANGLE = 0;
    public final double HOOD_MAX_ANGLE = 0;

    // =============================================================================
    // MECHANICAL CONSTANTS - TOP SPIN (needs actual values)
    // =============================================================================
    
    public final boolean TOP_SPIN_INVERTED = false;
    public final double TOP_SPIN_GEAR_RATIO = 1.0;
    public final double TOP_SPIN_RADIUS_METERS = Units.inchesToMeters(1);

    // =============================================================================
    // MECHANICAL CONSTANTS - TURRET (needs actual values)
    // =============================================================================
    
    public final double TURRET_GEAR_RATIO = 1.0;
    public final double TURRET_MOI = 0.001;

    // =============================================================================
    // CONTROL AND OPERATIONAL CONSTANTS
    // =============================================================================
    
    public final double INDEXER_DUTY_CYCLE = 0.3; // 30% power for indexing
    public final Translation3d HUB_TRANSLATION =
            new Translation3d(4.611624, 4.021328, 1.397); // where the hub is
    public final double LAUNCH_HIGHT = 0.613;

    // =============================================================================
    // MOTOR CONFIGURATION OBJECTS
    // =============================================================================
    
    public final FxMotorConfig SHOOTER_LEADER_MOTOR_CONFIG = new FxMotorConfig();
    public final FxMotorConfig SHOOTER_FOLLOWER_MOTOR_CONFIG = new FxMotorConfig();
    public final FxMotorConfig INDEX_MOTOR_CONFIG = new FxMotorConfig();
    public final FxMotorConfig HOOD_MOTOR_CONFIGS = new FxMotorConfig();
    public final FxMotorConfig TOP_SPIN_CONFIG = new FxMotorConfig();
    public final FxMotorConfig TURRET_MOTOR_CONFIGS = new FxMotorConfig();

    
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
                PhoenixUtil.toInvertedValue(SHOOTER_LEADER_INVERTED);

        // Configure Shooter Follower Motor
        SHOOTER_FOLLOWER_MOTOR_CONFIG.can_id = SHOOTER_FOLLOWER_ID;
        SHOOTER_FOLLOWER_MOTOR_CONFIG.motor_type = FxMotorType.X60;
        SHOOTER_FOLLOWER_MOTOR_CONFIG.canbus_name = "CANivore";
        SHOOTER_FOLLOWER_MOTOR_CONFIG.config = new TalonFXConfiguration();
        SHOOTER_FOLLOWER_MOTOR_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(SHOOTER_FOLLOWER_INVERTED);

        // Configure Hood Motor
        HOOD_MOTOR_CONFIGS.can_id = HOOD_ID;
        HOOD_MOTOR_CONFIGS.motor_type = FxMotorType.X60;
        HOOD_MOTOR_CONFIGS.canbus_name = "CANivore";
        HOOD_MOTOR_CONFIGS.config = new TalonFXConfiguration();

        // Configure Top Spin Motor
        TOP_SPIN_CONFIG.can_id = TOP_SPIN_ID;
        TOP_SPIN_CONFIG.motor_type = FxMotorType.X44;
        TOP_SPIN_CONFIG.canbus_name = "CANivore";
        TOP_SPIN_CONFIG.config = new TalonFXConfiguration();
        TOP_SPIN_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(TOP_SPIN_INVERTED);

        // Configure Turret Motor
        TURRET_MOTOR_CONFIGS.can_id = TURRET_ID;
        TURRET_MOTOR_CONFIGS.motor_type = FxMotorType.X44;
        TURRET_MOTOR_CONFIGS.canbus_name = "CANivore";
        TURRET_MOTOR_CONFIGS.config = new TalonFXConfiguration();
    }
}
