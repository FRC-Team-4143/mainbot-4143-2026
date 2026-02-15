package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.marswars.subsystem.MwConstants;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.FxMotorConfig.FxMotorType;

import edu.wpi.first.math.util.Units;

import com.marswars.util.PhoenixUtil;

public class IntakeConstants extends MwConstants {

    // =============================================================================
    // ENUMS AND STATE DEFINITIONS
    // =============================================================================

    public enum IntakeStates {
        STORE,
        DEPLOYING,
        DEPLOYED,
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
    public final double PIVOT_GEAR_RATIO = (40.0 / 12.0) * (56.0 / 24.0) * (32.0 / 14.0);
    public final SlotConfigs PIVOT_POSITION_SLOT_CONFIG = new SlotConfigs();
    public final double PIVOT_LENGTH = Units.inchesToMeters(11.5);
    public final double PIVOT_MASS = Units.lbsToKilograms(8.38);
    public final double PIVOT_MIN = Units.degreesToRadians(41);
    public final double PIVOT_MAX = Units.degreesToRadians(86);
    public final double PIVOT_HOME_POSITION = Units.degreesToRadians(43);
    public final double PIVOT_STATOR_CURRENT_LIMIT = 60;
    public final double PIVOT_DEPLOY_POSITION = Units.degreesToRadians(43);
    public final double PIVOT_STORE_POSITION = Units.degreesToRadians(80);
    public final double PIVOT_TOLERANCE = Units.degreesToRadians(5);
    public final Slot0Configs PIVOT_POSITION_GAINS = new Slot0Configs().withKG(0.12).withKP(250.0);

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
        PIVOT_MOTOR_CONFIG.config.CurrentLimits.StatorCurrentLimit = PIVOT_STATOR_CURRENT_LIMIT;
        PIVOT_MOTOR_CONFIG.config.CurrentLimits.StatorCurrentLimitEnable = true;
        PIVOT_MOTOR_CONFIG.config.Slot0 = PIVOT_POSITION_GAINS;
    }
}
