package frc.robot.subsystems.climber;

import com.marswars.subsystem.MwConstants;
import com.marswars.util.NovaMotorConfig;
import com.marswars.util.NovaMotorConfig.NovaMotorType;
import com.thethriftybot.devices.ThriftyNova.ThriftyNovaConfig;
import edu.wpi.first.math.util.Units;

// IMPORTANT
// Change ALL placeholders BEFORE branch merge
// Delete all Place Holders

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
        L3_CLIMB
    }

    // =============================================================================
    // CAN IDS AND HARDWARE CONFIGURATION
    // =============================================================================

    // Motor CAN IDs
    public final int DEPLOY_MOTOR_ID = 21;
    public final int FLIP_MOTOR_ID = 22;

    // =============================================================================
    // MECHANICAL CONSTANTS - DEPLOY
    // =============================================================================

    public final boolean DEPLOY_MOTOR_INVERTED = false;
    public final double DEPLOY_GEAR_RATIO = 1.0;
    public final double DEPLOY_DEPLOYED_ANGLE = (1 * Math.PI) / 2; // angle of DEPLOY being out
    public final double DEPLOY_STOWED_ANGLE = 0; // angle of DEPLOY being stowed
    public final double DEPLOY_TOLERANCE_ANGLE = Units.degreesToRadians(0.5);

    // =============================================================================
    // MECHANICAL CONSTANTS - FLIP
    // =============================================================================

    public final boolean FLIP_MOTOR_INVERTED = false;
    public final double FLIP_GEAR_RATIO = 1.0;
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

    public final NovaMotorConfig DEPLOY_MOTOR_CONFIG = new NovaMotorConfig();
    public final NovaMotorConfig FLIP_MOTOR_CONFIG = new NovaMotorConfig();

    // =============================================================================
    // CONSTRUCTOR - MOTOR CONFIGURATION INITIALIZATION
    // =============================================================================

    public ClimberConstants() {
        // Configure DEPLOY Motor
        DEPLOY_MOTOR_CONFIG.can_id = DEPLOY_MOTOR_ID;
        DEPLOY_MOTOR_CONFIG.motor_type = NovaMotorType.NEO_550;
        DEPLOY_MOTOR_CONFIG.canbus_name = "CANivore";
        DEPLOY_MOTOR_CONFIG.config = new ThriftyNovaConfig();

        // Configure FLIP Motor
        FLIP_MOTOR_CONFIG.can_id = FLIP_MOTOR_ID;
        FLIP_MOTOR_CONFIG.motor_type = NovaMotorType.VORTEX;
        FLIP_MOTOR_CONFIG.canbus_name = "rio";
        FLIP_MOTOR_CONFIG.config = new ThriftyNovaConfig();
    }
}
