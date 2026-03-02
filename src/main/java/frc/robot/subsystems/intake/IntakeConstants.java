package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.marswars.mechanisms.MotorConfig;
import com.marswars.mechanisms.MotorConfig.TalonMotorType;
import com.marswars.subsystem.MwConstants;
import com.marswars.util.PhoenixUtil;
import edu.wpi.first.math.util.Units;

public class IntakeConstants extends MwConstants {

    // =============================================================================
    // ENUMS AND STATE DEFINITIONS
    // =============================================================================

    public enum IntakeStates {
        /** Intake stowed in robot */
        STORE,
        /** Intake actively deploying to ground */
        DEPLOYING,
        /** Intake deployed and ready */
        DEPLOYED,
        /** Actively intaking game pieces */
        INTAKE,
        /** Ejecting game pieces out of intake */
        OUTTAKE,
        /** Idle state with intake mechanisms stopped */
        IDLE,
        /** Manual tuning mode for testing and calibration */
        TUNING
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
    public final double INTAKE_DUTY_CYCLE = 1.0;
    // =============================================================================
    // MECHANICAL CONSTANTS - PIVOT
    // =============================================================================

    public final boolean PIVOT_MOTOR_INVERTED = false;
    public final double PIVOT_GEAR_RATIO = (40.0 / 12.0) * (56.0 / 24.0) * (32.0 / 14.0);
    public final SlotConfigs PIVOT_POSITION_SLOT_CONFIG = new SlotConfigs();
    public final double PIVOT_LENGTH = Units.inchesToMeters(11.5);
    public final double PIVOT_MASS = Units.lbsToKilograms(8.38);
    public final double PIVOT_MIN = Units.degreesToRadians(11);
    public final double PIVOT_MAX = Units.degreesToRadians(98);
    public final double PIVOT_HOME_POSITION = Units.degreesToRadians(11);
    public final double PIVOT_STATOR_CURRENT_LIMIT = 60;
    public final double PIVOT_DEPLOY_POSITION = Units.degreesToRadians(11);
    public final double PIVOT_RACKING_POSITION = Units.degreesToRadians(60);
    public final double PIVOT_STORE_POSITION = Units.degreesToRadians(98);
    public final double DEPLOY_PIVOT_TOLERANCE = Units.degreesToRadians(20);
    public final Slot0Configs PIVOT_POSITION_GAINS = new Slot0Configs().withKG(2).withKP(50);

    // Time to wait between cycling SHOOTING and RACKING modes (seconds)
    public final double SHOOTING_CYCLE_TIME = 1.0;

    // =============================================================================
    // MOTOR CONFIGURATION OBJECTS
    // =============================================================================

    public final MotorConfig ROLLER_MOTOR_CONFIG = new MotorConfig();
    public final MotorConfig PIVOT_MOTOR_CONFIG = new MotorConfig();

    // =============================================================================
    // CONSTRUCTOR - MOTOR CONFIGURATION INITIALIZATION
    // =============================================================================

    public IntakeConstants() {
        // Configure Roller Motor
        ROLLER_MOTOR_CONFIG.can_id = ROLLER_MOTOR_ID;
        ROLLER_MOTOR_CONFIG.motor_type = TalonMotorType.X60;
        ROLLER_MOTOR_CONFIG.canbus_name = "rio";
        TalonFXConfiguration roller_config = new TalonFXConfiguration();
        roller_config.MotorOutput.Inverted = PhoenixUtil.toInvertedValue(ROLLER_MOTOR_INVERTED);
        ROLLER_MOTOR_CONFIG.apply(roller_config);

        // Configure Pivot Motor
        PIVOT_MOTOR_CONFIG.can_id = PIVOT_MOTOR_ID;
        PIVOT_MOTOR_CONFIG.motor_type = TalonMotorType.X44;
        PIVOT_MOTOR_CONFIG.canbus_name = "CANivore";
        TalonFXConfiguration pivot_config = new TalonFXConfiguration();
        pivot_config.Slot0 = Slot0Configs.from(PIVOT_POSITION_SLOT_CONFIG);
        pivot_config.MotorOutput.Inverted = PhoenixUtil.toInvertedValue(PIVOT_MOTOR_INVERTED);
        pivot_config.CurrentLimits.StatorCurrentLimit = PIVOT_STATOR_CURRENT_LIMIT;
        pivot_config.CurrentLimits.StatorCurrentLimitEnable = true;
        pivot_config.Slot0 = PIVOT_POSITION_GAINS;
        PIVOT_MOTOR_CONFIG.apply(pivot_config);
    }
}
