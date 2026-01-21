package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.marswars.subsystem.MwConstants;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.FxMotorConfig.FxMotorType;

import edu.wpi.first.math.util.Units;

public class HopperContstants extends MwConstants {

    public enum HopperStates {
        IDLE,
        STIRRING,
        SHOOTING,
        PROFILE
    }

    // Hopper Configs
    public final int HOPPERMOTOR_ID = 0; // Place holder
    public final boolean HOPPERMOTOR_INVERTED = false; // Place Holder
    public final double HOPPER_GEAR_RATIO = 1.0; // Place Holder
    public final FxMotorConfig HOPPER_MOTOR_CONFIG = new FxMotorConfig();
    // Feed Configs
    public final int FEED_ID = 0; // Place holder
    public final boolean FEED_INVERTED = false; // Place Holder
    public final double FEED_GEAR_RATIO = 1.0; // Place Holder
    public final FxMotorConfig FEED_MOTOR_CONFIG = new FxMotorConfig();

    public HopperContstants() {
        // Configure Indexer Motor
        HOPPER_MOTOR_CONFIG.can_id = HOPPERMOTOR_ID;
        HOPPER_MOTOR_CONFIG.motor_type = FxMotorType.FALCON500;
        HOPPER_MOTOR_CONFIG.canbus_name = "CANivore";
        HOPPER_MOTOR_CONFIG.config = new TalonFXConfiguration();

        FEED_MOTOR_CONFIG.can_id = FEED_ID;
        FEED_MOTOR_CONFIG.motor_type = FxMotorType.X60;
        FEED_MOTOR_CONFIG.canbus_name = "rio";
        FEED_MOTOR_CONFIG.config = new TalonFXConfiguration();
    }

}
