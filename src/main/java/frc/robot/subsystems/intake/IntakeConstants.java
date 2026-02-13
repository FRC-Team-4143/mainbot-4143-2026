package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.marswars.subsystem.MwConstants;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.FxMotorConfig.FxMotorType;
import com.marswars.util.PhoenixUtil;

public class IntakeConstants extends MwConstants {

    // =============================================================================
    // ENUMS AND STATE DEFINITIONS
    // =============================================================================

    public enum IntakeStates {
        STORE,
        DEPLOY,
        INTAKE,
        OUTTAKE,
        IDLE,
        TUNING,
        MANUAL;
    }

    // =============================================================================
    // CAN IDS AND HARDWARE CONFIGURATION
    // =============================================================================

    // Motor CAN IDs
    public final int ROLLER_MOTOR_ID = 30;
    public final int PIVOT_MOTOR_ID = 31;

    // =============================================================================
    // MECHANICAL CONSTANTS - ROLLER
    // =============================================================================

    public final boolean ROLLER_MOTOR_INVERTED = false;
    public final double ROLLER_GEAR_RATIO = 1.0;

    // =============================================================================
    // MECHANICAL CONSTANTS - PIVOT
    // =============================================================================

    public final boolean PIVOT_MOTOR_INVERTED = false;
    public final double PIVOT_GEAR_RATIO = (20.0 / 12.0) * (56.0 / 24.0) * (32.0 / 14.0);
    public final SlotConfigs PIVOT_POSITION_SLOT_CONFIG = new SlotConfigs();

    // =============================================================================
    // MOTOR CONFIGURATION OBJECTS
    // =============================================================================

    public final FxMotorConfig ROLLER_MOTOR_CONFIG = new FxMotorConfig();
    public final FxMotorConfig PIVOT_MOTOR_CONFIG = new FxMotorConfig();

    // =============================================================================
    // CONSTRUCTOR - MOTOR CONFIGURATION INITIALIZATION
    // =============================================================================

    public IntakeConstants() {
        // Configure Roller Motor
        ROLLER_MOTOR_CONFIG.can_id = ROLLER_MOTOR_ID;
        ROLLER_MOTOR_CONFIG.motor_type = FxMotorType.X60;
        ROLLER_MOTOR_CONFIG.canbus_name = "rio";
        ROLLER_MOTOR_CONFIG.config = new TalonFXConfiguration();
        ROLLER_MOTOR_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(ROLLER_MOTOR_INVERTED);

        // Configure Pivot Motor
        PIVOT_MOTOR_CONFIG.can_id = PIVOT_MOTOR_ID;
        PIVOT_MOTOR_CONFIG.motor_type = FxMotorType.X44;
        PIVOT_MOTOR_CONFIG.canbus_name = "CANivore";
        PIVOT_MOTOR_CONFIG.config = new TalonFXConfiguration();
        PIVOT_MOTOR_CONFIG.config.Slot0 = Slot0Configs.from(PIVOT_POSITION_SLOT_CONFIG);
        PIVOT_MOTOR_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(PIVOT_MOTOR_INVERTED);
    }
}
