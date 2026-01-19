package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.marswars.subsystem.MwConstants;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.FxMotorConfig.FxMotorType;
import edu.wpi.first.math.util.Units;

public class ShooterConstants extends MwConstants {

    public enum ShooterStates {
        UNWIND,
        AIMING,
        DUMP,
        SHOOT,
        IDLE,
        PROFILE
    }

    // Shooter Mech Configs
    public final int SHOOTER_ID = 10;
    public final boolean SHOOTER_INVERTED = false;
    public final double SHOOTER_GEAR_RATIO = 1.0;
    public final double SHOOTER_WHEEL_RADIUS_METERS = Units.inchesToMeters(3);
    public final double SHOOTER_WHEEL_MASS_KG = 2.3; // kg, approximate
    public final double SHOOTER_WHEEL_INERTIA =
            0.5
                    * SHOOTER_WHEEL_MASS_KG
                    * Math.pow(SHOOTER_WHEEL_RADIUS_METERS, 2.0); // kg m^2, approximate
    public final FxMotorConfig SHOOTER_MOTOR_CONFIGS = new FxMotorConfig();

    // Indexer Mech Config
    public final int INDEXER_ID = 11;
    public final boolean INDEXER_INVERTED = false;
    public final double INDEXER_GEAR_RATIO = 1.0;
    public final FxMotorConfig INDEX_MOTOR_CONFIG = new FxMotorConfig();

    // Hood Mech Config
    public final int HOOD_ID = 0; // Place Holder
    public final boolean HOOD_INVERTED = false;
    public final double HOOD_GEAR_RATIO = 1.0; // Place Holder
    public final double HOOD_LENGTH = 2; // Place Holder
    public final double HOOD_MASS_KG = 2; // Place Holder
    public final double HOOD_MIN_ANGLE = 0; // Place Holder
    public final double HOOD_MAX_ANGLE = 3; // Place Holder
    public final FxMotorConfig HOOD_MOTOR_CONFIG = new FxMotorConfig();

    // Control Setpoints
    public final double SHOOT_DUTY_CYCLE = 0.5; // 50% power for shooting
    public final double INDEXER_DUTY_CYCLE = 0.3; // 30% power for indexing

    public ShooterConstants() {
        INDEX_MOTOR_CONFIG.can_id = INDEXER_ID;
        INDEX_MOTOR_CONFIG.motor_type = FxMotorType.X60;
        INDEX_MOTOR_CONFIG.canbus_name = "rio";
        INDEX_MOTOR_CONFIG.config = new TalonFXConfiguration();
    }
}
