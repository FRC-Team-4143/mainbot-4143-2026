package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.marswars.subsystem.MwConstants;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.FxMotorConfig.FxMotorType;

public class HopperConstants extends MwConstants {

    public enum HopperStates {
        IDLE,
        STIRRING,
        SHOOTING,
        PROFILE
    }

    // Indexer Configs
    public final int INDEXER_ID = 20;
    public final boolean INDEXER_INVERTED = false;
    public final double INDEXER_GEAR_RATIO = 1.0;
    public final FxMotorConfig INDEXER_MOTOR_CONFIG = new FxMotorConfig();

    // Feed Configs
    public final int FEEDER_ID = 21;
    public final boolean FEEDER_INVERTED = false;
    public final double FEEDER_GEAR_RATIO = 1.0;
    public final FxMotorConfig FEEDER_MOTOR_CONFIG = new FxMotorConfig();

    // Control Setpoints
    public final double INDEXER_DUTY_CYCLE_SHOOT = 0.5;
    public final double FEEDER_DUTY_CYCLE_SHOOT = 0.5;

    public HopperConstants() {
        // Configure Indexer Motor
        INDEXER_MOTOR_CONFIG.can_id = INDEXER_ID;
        INDEXER_MOTOR_CONFIG.motor_type = FxMotorType.FALCON500;
        INDEXER_MOTOR_CONFIG.canbus_name = "CANivore";
        INDEXER_MOTOR_CONFIG.config = new TalonFXConfiguration();

        FEEDER_MOTOR_CONFIG.can_id = FEEDER_ID;
        FEEDER_MOTOR_CONFIG.motor_type = FxMotorType.X60;
        FEEDER_MOTOR_CONFIG.canbus_name = "rio";
        FEEDER_MOTOR_CONFIG.config = new TalonFXConfiguration();
    }
}
