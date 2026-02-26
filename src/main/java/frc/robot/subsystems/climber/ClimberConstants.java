package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.marswars.subsystem.MwConstants;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.FxMotorConfig.FxMotorType;
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
    public final int EXTENDERMOTER_ID = 21; // may change
    public final int ARM_MOTER_ID = 22; // may change

    // =============================================================================
    // MECHANICAL CONSTANTS - EXTENDER
    // =============================================================================

    public final boolean EXTENDERMOTER_INVERTED = false; // Place Holder
    public final double EXTENDER_GEAR_RATIO = 1.0; // place holder
    public final double EXTENDER_DEPLOYED_ANGLE = (1 * Math.PI) / 2; // angle of extender being out
    public final double EXTENDER_STOWED_ANGLE = 0; // angle of extender being stowed
    public final double EXTENDER_TOLERANCE_ANGLE = Units.degreesToRadians(0.5);

    // =============================================================================
    // MECHANICAL CONSTANTS - ARM
    // =============================================================================

    public final boolean ARM_MOTER_INVERTED = false; // Place Holder
    public final double ARM_GEAR_RATIO = 1.0; // place holder
    public final double ARM_LENGTH = 1.0; // Meters, Place Holder
    public final double ARM_MASS = 1.0; // KG, Place Holder
    public final double ARM_MIN_ANGLE = -2.0 * Math.PI; // Radians, min rotation angle
    public final double ARM_MAX_ANGLE = 2.0 * Math.PI; // Radians, max rotation angle
    public final double ARM_L1_CLIMB = (1 * Math.PI) / 6; // Radians, auto climb climb angle
    public final double ARM_L3_CLIMB = 1 * Math.PI; // radians, teleop climb height
    public final double ARM_L0_POSITION = 0; // angle for the arm to return to

    // =============================================================================
    // MOTOR CONFIGURATION OBJECTS
    // =============================================================================

    public final FxMotorConfig EXTENDER_MOTOR_CONFIG = new FxMotorConfig();
    public final FxMotorConfig ARM_MOTOR_CONFIG = new FxMotorConfig();

    // =============================================================================
    // CONSTRUCTOR - MOTOR CONFIGURATION INITIALIZATION
    // =============================================================================

    public ClimberConstants() {
        // Configure Extender Motor
        EXTENDER_MOTOR_CONFIG.can_id = EXTENDERMOTER_ID;
        EXTENDER_MOTOR_CONFIG.motor_type = FxMotorType.FALCON500; // Place Holder
        EXTENDER_MOTOR_CONFIG.canbus_name = "CANivore"; // Place Holder
        EXTENDER_MOTOR_CONFIG.config = new TalonFXConfiguration(); // Place Holder

        // Configure Arm Motor
        ARM_MOTOR_CONFIG.can_id = ARM_MOTER_ID; // Place Holder
        ARM_MOTOR_CONFIG.motor_type = FxMotorType.X60; // Place Holder
        ARM_MOTOR_CONFIG.canbus_name = "rio"; // Place Holder
        ARM_MOTOR_CONFIG.config = new TalonFXConfiguration(); // Place Holder
    }
}
