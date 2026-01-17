package frc.robot.subsystems.hopper;

import com.marswars.subsystem.MwConstants;
import com.marswars.util.FxMotorConfig;
import edu.wpi.first.math.util.Units;

public class HopperContstants extends MwConstants {

    public enum HopperStates {
        IDLE,
        STIRRING,
        SHOOTING
    }

    // Hopper Configs

    public final int HOPPERMOTOR_ID = 0; // Place holder
    public final boolean HOPPERMOTOR_INVERTED = false; // Place Holder
    public final double HOPPER_GEAR_RATIO = 1.0; // Place Holder
    public final double HOPPER_WHEEL_RADIUS_METERS = Units.inchesToMeters(1); // Place Holder
    public final double HOPPER_WHEEL_MASS_KG = 0.05; // Place Holder
    public final double HOPPER_WHEEL_INTERIA = 0.5; // Place Holder
    public final FxMotorConfig HOPPER_MOTOR_CONFIG = new FxMotorConfig();
    // Feed Configs
    public final int FEED_ID = 0; // Place holder
    public final boolean FEED_INVERTED = false; // Place Holder
    public final double FEED_GEAR_RATIO = 1.0; // Place Holder
    public final double FEED_WHEEL_RADIUS_METERS = Units.inchesToMeters(1); // Place Holder
    public final double FEED_WHEEL_MASS_KG = 0.05; // Place Holder
    public final double FEED_WHEEL_INERTIA = 0.5; // Place Holder
    public final FxMotorConfig FEED_MOTOR_CONFIG = new FxMotorConfig();
}
