package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.marswars.subsystem.MwConstants;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.FxMotorConfig.FxMotorType;
import com.marswars.util.PhoenixUtil;

public class IntakeConstants extends MwConstants {

    public enum IntakeStates {
        CLOSED,
        DEPLOYED,
        ROLLING;
    }

    // Intake Configs
    public final int INTAKEMOTOR_ID = 20; // Place holder
    public final boolean INTAKEMOTOR_INVERTED = false;
    public final double INTAKE_GEAR_RATIO = 1.0; // Place Holder
    public final FxMotorConfig INTAKE_MOTOR_CONFIG = new FxMotorConfig();

    // Arm Configs
    public final int ARMMOTOR_ID = 21; // Place holder
    public final boolean ARMMOTOR_INVERTED = false;
    public final double ARM_GEAR_RATIO = (20.0/12.0)*(56.0/24.0)*(32.0/14.0);
    public final FxMotorConfig ARM_MOTOR_CONFIG = new FxMotorConfig();

    public IntakeConstants() {
        // Configure Indexer Motor
        INTAKE_MOTOR_CONFIG.can_id = INTAKEMOTOR_ID;
        INTAKE_MOTOR_CONFIG.motor_type = FxMotorType.X60;
        INTAKE_MOTOR_CONFIG.canbus_name = "rio";
        INTAKE_MOTOR_CONFIG.config = new TalonFXConfiguration();

        ARM_MOTOR_CONFIG.can_id = ARMMOTOR_ID;
        ARM_MOTOR_CONFIG.motor_type = FxMotorType.X44;
        ARM_MOTOR_CONFIG.canbus_name = "rio";
        ARM_MOTOR_CONFIG.config = new TalonFXConfiguration();
        ARM_MOTOR_CONFIG.config.Slot0.kP = 0.0; // placeholder
        ARM_MOTOR_CONFIG.config.MotorOutput.Inverted = PhoenixUtil.toInvertedValue(ARMMOTOR_INVERTED);
    }
}
