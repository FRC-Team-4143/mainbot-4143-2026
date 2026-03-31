package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.marswars.mechanisms.MotorConfig;
import com.marswars.mechanisms.MotorConfig.TalonMotorType;
import com.marswars.subsystem.MwConstants;

public class HopperConstants extends MwConstants {

    // =============================================================================
    // ENUMS AND STATE DEFINITIONS
    // =============================================================================

    public enum HopperStates {
        /** Idle state with hopper stopped */
        IDLE,
        /** Actively intaking game pieces */
        INTAKE,
        /** Actively feeding game pieces to shooter */
        SHOOTING,
        /** Unjamming by reversing hopper */
        UNJAM_REVERSE,
        /** Unjamming by running hopper forward */
        UNJAM_FORWARD,
        /** Running hopper in reverse */
        REVERSE,
        /** Manual tuning mode for testing and calibration */
        TUNING,
    }

    // =============================================================================
    // CAN IDS AND HARDWARE CONFIGURATION
    // =============================================================================

    // Motor CAN IDs
    public final int HOPPER_MOTOR_ID = 20;

    // =============================================================================
    // MECHANICAL CONSTANTS - HOPPER
    // =============================================================================

    public final boolean HOPPER_MOTOR_INVERTED = false;
    public final double HOPPER_GEAR_RATIO = 1.0;
    public final double HOPPER_DANGER_CURRENT = 40;
    public final double HOPPER_VELOCITY_TARGET = 200;
    public final Slot1Configs HOPPER_VELOCITY_GAINS = new Slot1Configs().withKV(.122).withKP(0.5);

    // =============================================================================
    // CONTROL AND OPERATIONAL CONSTANTS
    // =============================================================================

    public final double DEBOUNCE_TIME = 0.1;
    public final double UNJAMM_TIMER = 0.5;

    // =============================================================================
    // MOTOR CONFIGURATION OBJECTS
    // =============================================================================

    public final MotorConfig HOPPER_MOTOR_CONFIG = new MotorConfig();

    // public final MotorConfig FEED_MOTOR_CONFIG = new MotorConfig();

    // =============================================================================
    // CONSTRUCTOR - MOTOR CONFIGURATION INITIALIZATION
    // =============================================================================

    public HopperConstants() {
        // Configure Hopper Motor
        HOPPER_MOTOR_CONFIG.can_id = HOPPER_MOTOR_ID;
        HOPPER_MOTOR_CONFIG.motor_type = TalonMotorType.X44;
        HOPPER_MOTOR_CONFIG.canbus_name = "rio";
        TalonFXConfiguration hopper_config = new TalonFXConfiguration();
        hopper_config.Slot1 = HOPPER_VELOCITY_GAINS;
        //hopper_config.CurrentLimits.StatorCurrentLimit = 50;
        HOPPER_MOTOR_CONFIG.apply(hopper_config);
    }
}
