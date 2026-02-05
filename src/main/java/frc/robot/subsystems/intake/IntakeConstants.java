package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.marswars.subsystem.MwConstants;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.FxMotorConfig.FxMotorType;

public class IntakeConstants extends MwConstants {

    public enum IntakeStates {
        CLOSED,
        DEPLOYED,
        ROLLING;
    }

    // Intake Configs
    public final int INTAKEMOTOR_ID = 20; // Place holder
    public final boolean INTAKEMOTOR_INVERTED = false; // Place Holder
    public final double INTAKE_GEAR_RATIO = 1.0; // Place Holder
    public final FxMotorConfig INTAKE_MOTOR_CONFIG = new FxMotorConfig();

    // Arm Configs
    public final int ARMMOTOR_ID = 21; // Place holder
    public final boolean ARMMOTOR_INVERTED = false; // Place Holder
    public final double ARM_GEAR_RATIO = 1.0; // Place Holder
    public final FxMotorConfig ARM_MOTOR_CONFIG = new FxMotorConfig();

    public IntakeConstants() {
        // Configure Indexer Motor
        INTAKE_MOTOR_CONFIG.can_id = INTAKEMOTOR_ID;
        INTAKE_MOTOR_CONFIG.motor_type = FxMotorType.FALCON500; // placeholder
        INTAKE_MOTOR_CONFIG.canbus_name = "CANivore"; // placeholder
        INTAKE_MOTOR_CONFIG.config = new TalonFXConfiguration();

        ARM_MOTOR_CONFIG.can_id = ARMMOTOR_ID;
        ARM_MOTOR_CONFIG.motor_type = FxMotorType.FALCON500; // placeholder
        ARM_MOTOR_CONFIG.canbus_name = "CANivore"; // placeholder
        ARM_MOTOR_CONFIG.config = new TalonFXConfiguration();
        ARM_MOTOR_CONFIG.config.Slot0.kP = 5.0;
    }
}
