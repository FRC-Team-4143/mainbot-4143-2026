package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.marswars.mechanisms.MotorConfig;
import com.marswars.mechanisms.MotorConfig.TalonMotorType;
import com.marswars.subsystem.MwConstants;
import edu.wpi.first.math.util.Units;

public class ClimberConstants extends MwConstants {

    // =============================================================================
    // ENUMS AND STATE DEFINITIONS
    // =============================================================================

    public enum ClimberStates {
        /** Climber fully stowed in robot */
        STOWED,
        /** Deploying climber mechanisms */
        DEPLOY,
        /** Climbing to Level 1 */
        L1_CLIMB,
        /** Moving down from Level 1 */
        L1_DOWN,
        /** Climbing to Level 3 */
        L3_CLIMB,
        TUNNING,
        MANUAL
    }

    // =============================================================================
    // CAN IDS AND HARDWARE CONFIGURATION
    // =============================================================================

    // Motor CAN IDs
    public final int DEPLOY_MOTOR_ID = 40;
    public final int FLIP_MOTOR_ID = 41;

    // =============================================================================
    // MECHANICAL CONSTANTS - DEPLOY
    // =============================================================================

    public final boolean DEPLOY_MOTOR_INVERTED = false;
    public final double DEPLOY_GEAR_RATIO = 25.0;
    public final double DEPLOY_DEPLOYED_ANGLE = (1 * Math.PI) / 2; // angle of DEPLOY being out
    public final double DEPLOY_STOWED_ANGLE = 0.0; // angle of DEPLOY being stowed
    public final double DEPLOY_TOLERANCE_ANGLE = Units.degreesToRadians(0.5);
    public final double CLIMBER_HOME_POSITION = 0.0;

    // =============================================================================
    // MECHANICAL CONSTANTS - FLIP
    // =============================================================================

    public final boolean FLIP_MOTOR_INVERTED = false;
    public final double FLIP_GEAR_RATIO = 585;
    public final double FLIP_LENGTH = 1.0; // Meters
    public final double FLIP_MASS = 1.0; // KG
    public final double FLIP_MIN_ANGLE = -2.0 * Math.PI; // Radians, min rotation angle
    public final double FLIP_MAX_ANGLE = 2.0 * Math.PI; // Radians, max rotation angle
    public final double FLIP_L1_CLIMB = (1 * Math.PI) / 6; // Radians, auto climb climb angle
    public final double FLIP_L3_CLIMB = 1 * Math.PI; // radians, teleop climb height
    public final double FLIP_L0_POSITION = 0; // angle for the flip to return to

    // =============================================================================
    // MOTOR CONFIGURATION OBJECTS
    // =============================================================================

    public final MotorConfig DEPLOY_MOTOR_CONFIG = new MotorConfig();
    public final MotorConfig FLIP_MOTOR_CONFIG = new MotorConfig();

    // =============================================================================
    // CONSTRUCTOR - MOTOR CONFIGURATION INITIALIZATION
    // =============================================================================

    public ClimberConstants() {

        // Configure DEPLOY Motor
        DEPLOY_MOTOR_CONFIG.can_id = DEPLOY_MOTOR_ID;
        DEPLOY_MOTOR_CONFIG.motor_type = TalonMotorType.NEO_550;
        DEPLOY_MOTOR_CONFIG.canbus_name = "rio";
        TalonFXSConfiguration deploy_config = new TalonFXSConfiguration();
        deploy_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        deploy_config.CurrentLimits.StatorCurrentLimit = 30;
        deploy_config.CurrentLimits.StatorCurrentLimitEnable = true;
        deploy_config.CurrentLimits.SupplyCurrentLimit = 5;
        deploy_config.CurrentLimits.SupplyCurrentLimitEnable = true;
        deploy_config.Slot0 = new Slot0Configs().withKP(600);
        DEPLOY_MOTOR_CONFIG.apply(deploy_config);

        // Configure FLIP Motor
        FLIP_MOTOR_CONFIG.can_id = FLIP_MOTOR_ID;
        FLIP_MOTOR_CONFIG.motor_type = TalonMotorType.X60;
        FLIP_MOTOR_CONFIG.canbus_name = "rio";
        
        TalonFXConfiguration flip_config = new TalonFXConfiguration();
        flip_config.Slot0 = new Slot0Configs().withKP(100);
        FLIP_MOTOR_CONFIG.apply(flip_config);
    }
}
