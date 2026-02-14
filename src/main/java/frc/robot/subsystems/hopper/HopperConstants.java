package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.marswars.subsystem.MwConstants;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.FxMotorConfig.FxMotorType;

public class HopperConstants extends MwConstants {

    // =============================================================================
    // ENUMS AND STATE DEFINITIONS
    // =============================================================================

    public enum HopperStates {
        IDLE,
        SHOOTING,
        UNJAM_REVERSE,
        UNJAM_FORWARD,
        TUNING,
        MANUAL
    }

    // =============================================================================
    // CAN IDS AND HARDWARE CONFIGURATION
    // =============================================================================

    // Motor CAN IDs
    public final int HOPPER_MOTOR_ID = 20;
    public final int FEED_MOTOR_ID = 21;

    // =============================================================================
    // MECHANICAL CONSTANTS - HOPPER
    // =============================================================================

    public final boolean HOPPER_MOTOR_INVERTED = false;
    public final double HOPPER_GEAR_RATIO = 1.0;
    public final double HOPPER_DANGER_CURRENT = 20;
    public final double HOPPER_DUTY_CYCLE = 0.5;

    // =============================================================================
    // MECHANICAL CONSTANTS - FEED
    // =============================================================================

    public final boolean FEED_MOTOR_INVERTED = false;
    public final double FEED_GEAR_RATIO = 1.0;
    public final double FEED_DANGER_CURRENT = 0.0;
    public final double FEED_DUTY_CYCLE = 0.5;

    // =============================================================================
    // CONTROL AND OPERATIONAL CONSTANTS
    // =============================================================================

    public final double DEBOUNCE_TIME = 0.1;
    public final double UNJAMM_TIMER = 0.5;

    // =============================================================================
    // MOTOR CONFIGURATION OBJECTS
    // =============================================================================

    public final FxMotorConfig HOPPER_MOTOR_CONFIG = new FxMotorConfig();
    public final FxMotorConfig FEED_MOTOR_CONFIG = new FxMotorConfig();

    // =============================================================================
    // CONSTRUCTOR - MOTOR CONFIGURATION INITIALIZATION
    // =============================================================================

    public HopperConstants() {
        // Configure Hopper Motor
        HOPPER_MOTOR_CONFIG.can_id = HOPPER_MOTOR_ID;
        HOPPER_MOTOR_CONFIG.motor_type = FxMotorType.FALCON500;
        HOPPER_MOTOR_CONFIG.canbus_name = "CANivore";
        HOPPER_MOTOR_CONFIG.config = new TalonFXConfiguration();

        // Configure Feed Motor
        FEED_MOTOR_CONFIG.can_id = FEED_MOTOR_ID;
        FEED_MOTOR_CONFIG.motor_type = FxMotorType.X60;
        FEED_MOTOR_CONFIG.canbus_name = "rio";
        FEED_MOTOR_CONFIG.config = new TalonFXConfiguration();
    }
}
