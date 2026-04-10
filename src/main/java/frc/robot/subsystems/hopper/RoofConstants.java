package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.marswars.mechanisms.MotorConfig;
import com.marswars.mechanisms.MotorConfig.TalonMotorType;
import com.marswars.subsystem.MwConstants;
import edu.wpi.first.math.util.Units;

public class RoofConstants extends MwConstants {

    // =============================================================================
    // ENUMS AND STATE DEFINITIONS
    // =============================================================================

    public enum RoofStates {
        /** Elevator is at the top */
        UP,
        /** Elevator is at the bottom */
        DOWN,
        /** Elevator is climbing */
        CLIMB,
        TUNING
    }

    // =============================================================================
    // CAN IDS AND HARDWARE CONFIGURATION
    // =============================================================================

    // Motor CAN IDs
    public final int ELEVATOR_MOTOR_ID = 21;

    // =============================================================================
    // MECHANICAL CONSTANTS - ELEVATOR
    // =============================================================================

    public final boolean ELEVATOR_MOTOR_INVERTED = false;
    public final double ELEVATOR_GEAR_RATIO = 3.0 * (22.0 / 48.0) * (48.0 / 32.0);
    public final double ELEVATOR_DRUM_RADIUS = Units.inchesToMeters(0.878);
    public final double ELEVATOR_CARRIAGE_MASS_KG = Units.lbsToKilograms(3.44);
    public final double ELEVATOR_MAX_EXTENSION_METERS =
            Units.inchesToMeters(8.25); // Maximum extension of the elevator in meters
    public final Slot0Configs ELEVATOR_POSITION_GAINS =
            new Slot0Configs().withKV(0.0).withKP(3.0).withKI(0.0).withKD(0.0).withKG(0.2895);
    public final double ELEVATOR_RIGGING_RATIO =
            1.0; // Ratio of motor rotation to elevator extension (depends on pulley system)
    public final double ELEVATOR_UP_POSITION_METERS = 0.95*ELEVATOR_MAX_EXTENSION_METERS; // Target position for the elevator when in the UP state
    public final double ELEVATOR_DOWN_POSITION_METERS = 0.05*ELEVATOR_MAX_EXTENSION_METERS; // Target position for the elevator when in the DOWN state
    // =============================================================================
    // MOTOR CONFIGURATION OBJECTS
    // =============================================================================

    public final MotorConfig ELEVATOR_MOTOR_CONFIG = new MotorConfig();


    // =============================================================================
    // CONSTRUCTOR - MOTOR CONFIGURATION INITIALIZATION
    // =============================================================================

    public RoofConstants() {

        // Configure Elevator Motor
        ELEVATOR_MOTOR_CONFIG.can_id = ELEVATOR_MOTOR_ID;
        ELEVATOR_MOTOR_CONFIG.motor_type = TalonMotorType.X60;
        ELEVATOR_MOTOR_CONFIG.canbus_name = "rio";
        TalonFXConfiguration elevator_config = new TalonFXConfiguration();
        elevator_config.Slot0 = ELEVATOR_POSITION_GAINS;
        // elevator_config.CurrentLimits.StatorCurrentLimit = 50;
        ELEVATOR_MOTOR_CONFIG.apply(elevator_config);
    }
}
