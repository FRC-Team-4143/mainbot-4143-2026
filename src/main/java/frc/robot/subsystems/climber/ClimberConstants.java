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
        L1,
        /** Climbing to Level 2 */
        L2,
        /** Moving down from Level 1 */
        GROUND,
        /** Climbing to Level 3 */
        L3,
        /** Stops flip after climb 3 is done */
        CLIMB_HOLD,
        /** Disables state machine control for tuning */
        TUNNING
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
    public final double DEPLOY_MOI = 25.0; // kg*m^2, moment of inertia of the deploy mechanism
    public final double DEPLOY_DEPLOYED_ANGLE = (1 * Math.PI) / 2; // angle of DEPLOY being out
    public final double DEPLOY_STOWED_ANGLE = 0; // angle of DEPLOY being stowed
    public final double DEPLOY_TOLERANCE_ANGLE = Units.degreesToRadians(5);

    // =============================================================================
    // MECHANICAL CONSTANTS - FLIP
    // =============================================================================

    public final boolean FLIP_MOTOR_INVERTED = false;
    public final double FLIP_GEAR_RATIO = 585;
    public final double FLIP_LENGTH = 0.1; // Meters
    public final double FLIP_MASS = 0.1; // KG
    public final double FLIP_MIN_ANGLE = -2.0 * Math.PI; // Radians, min rotation angle
    public final double FLIP_MAX_ANGLE = 2.0 * Math.PI; // Radians, max rotation angle
    public final double FLIP_L1_CLIMB = Units.degreesToRadians(90); // Radians, auto climb climb angle
    public final double FLIP_L2_CLIMB = Units.degreesToRadians(125); // Radians, L2 climb angle
    public final double FLIP_L3_CLIMB_ANGLE = Units.degreesToRadians(180); // radians, teleop climb height
    public final double FLIP_GROUND_ANGLE = Units.degreesToRadians(0); // angle for the flip to return to
    public final double FLIP_CLIMB_UP_DUTY_CYCLE = 1.0; //percent power for moving up
    public final double FLIP_DOWN_UP_DUTY_CYCLE = -1.0; // percent power for moving down
    public final double FLIP_BUMP_DUTY_CYCLE = 1.0; // percent power for small bump adjustments
    public final double FLIP_ADJUSTMENT_INCREMENT = Units.degreesToRadians(4); // radians to adjust per bump
    public final double FLIP_ANGLE_TOLERANCE = Units.degreesToRadians(2);


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
        deploy_config.Slot0.kP = 100;
        DEPLOY_MOTOR_CONFIG.apply(deploy_config);

        // Configure FLIP Motor
        FLIP_MOTOR_CONFIG.can_id = FLIP_MOTOR_ID;
        FLIP_MOTOR_CONFIG.motor_type = TalonMotorType.X60;
        FLIP_MOTOR_CONFIG.canbus_name = "rio";

        TalonFXConfiguration flip_config = new TalonFXConfiguration();
        flip_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        flip_config.CurrentLimits.StatorCurrentLimitEnable = false;
        flip_config.CurrentLimits.SupplyCurrentLimitEnable = false;
        FLIP_MOTOR_CONFIG.apply(flip_config);
    }
}
