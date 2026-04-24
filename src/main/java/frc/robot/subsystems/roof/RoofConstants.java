package frc.robot.subsystems.roof;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.marswars.mechanisms.MotorConfig;
import com.marswars.mechanisms.MotorConfig.TalonMotorType;
import com.marswars.subsystem.MwConstants;
import com.marswars.util.PhoenixUtil;

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
        /** Elevator is in a safety-down position which is farther than the hard stop */
        SAFETY_DOWN,
        /** Elevator is climbing */
        CLIMB,
        /**Elevator homing */
        ROOF_HOMING,
        /** */
        SQUEEZE,
        /** Elevator is in a tuning mode (e.g., for testing or calibration) */
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

    public final boolean ELEVATOR_MOTOR_INVERTED = true;
    public final double ELEVATOR_GEAR_RATIO =
            5.0 * (32.0 / 22.0); // Total gear ratio from motor to elevator extension
    public final double ELEVATOR_DRUM_RADIUS = Units.inchesToMeters(0.878);
    public final double ELEVATOR_CARRIAGE_MASS_KG = Units.lbsToKilograms(3.44);
    public final double ELEVATOR_MAX_EXTENSION_METERS =
            Units.inchesToMeters(11.4); // Maximum extension of the elevator in meters
    public final double ELEVATOR_HOME_POSITION = 0.0;
    public final SlotConfigs ELEVATOR_POSITION_GAINS =
            new SlotConfigs().withKV(0.0).withKP(4.8).withKI(0.0).withKD(0.0).withKG(0.0);
    public final SlotConfigs ELEVATOR_CURRENT_GAINS = new SlotConfigs().withKI(0.01666);
    public final SlotConfigs ELEVATOR_CLIMB_POSITION_GAINS = new SlotConfigs().withKP(4.8).withKI(0.0).withKD(0.0);
    public final double ELEVATOR_RIGGING_RATIO =
            1.0; // Ratio of motor rotation to elevator extension (depends on pulley system)
    public final double ELEVATOR_UP_POSITION_METERS = ELEVATOR_MAX_EXTENSION_METERS; // Target position for the elevator when in the
    // UP state
    public final double ELEVATOR_DOWN_POSITION_METERS = 0.005; // Target position for the elevator when in the
    // DOWN state


    public final double SAFETY_DEBOUNCER_TIME_SECONDS = 0.25; // Time in seconds to confirm that the roof is safely down before allowing certain actions

    //Squeeze Constants
    public final double ELEVATOR_SQUEEZE_CURRENT = -10.0;
    public final double ELEVATOR_SQUEEZE_MIN_POSITION = ELEVATOR_DOWN_POSITION_METERS;


    // Homing for elevator - drive with a small duty cycle until the motor current spikes
    public final double ELEVATOR_HOMING_DUTY_CYCLE = -0.08;
    public final double ELEVATOR_HOMING_CURRENT_THRESHOLD =
            3.0; // Amps, threshold for detecting stall during homing
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
        elevator_config.MotorOutput.Inverted = PhoenixUtil.toInvertedValue(ELEVATOR_MOTOR_INVERTED);
        elevator_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevator_config.Slot0 = Slot0Configs.from(ELEVATOR_CLIMB_POSITION_GAINS);
        elevator_config.Slot2 = Slot2Configs.from(ELEVATOR_CURRENT_GAINS);
        // elevator_config.CurrentLimits.StatorCurrentLimit = 50;
        ELEVATOR_MOTOR_CONFIG.apply(elevator_config);
    }
}
