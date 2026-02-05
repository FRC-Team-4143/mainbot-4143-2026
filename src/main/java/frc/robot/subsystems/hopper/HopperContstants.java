package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.marswars.subsystem.MwConstants;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.FxMotorConfig.FxMotorType;

public class HopperContstants extends MwConstants {

    public enum HopperStates {
        IDLE,
        SHOOTING,
        UNJAM_REVERSE,
        UNJAM_FORWARD,
        PROFILE
    }

    public final double DEBOUNCE_TIME = 0.1;
    // Hopper Configs
    public final int HOPPERMOTOR_ID = 20; // Placeholder
    public final boolean HOPPERMOTOR_INVERTED = false; // Placeholder
    public final double HOPPER_GEAR_RATIO = 1.0; // Placeholder
    public final FxMotorConfig HOPPER_MOTOR_CONFIG = new FxMotorConfig();
    public final double HOPPER_DANGER_CURRENT = 20; // placeholder, test robot for current
    public final double HOPPER_DUTY_CYCLE = 0.5; // placehoder
    public final double UNJAMM_TIMMER = 0.5;
    // Feed Configs
    public final int FEED_ID = 21; // Place holder
    public final boolean FEED_INVERTED = false; // Placeholder
    public final double FEED_GEAR_RATIO = 1.0; // Placeholder
    public final FxMotorConfig FEED_MOTOR_CONFIG = new FxMotorConfig();
    public final double FEED_DANGER_CURRENT = 0.0; // placeholder, test robot for current
    public final double FEED_DUTY_CYCLE = 0.5; // placeholder

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
