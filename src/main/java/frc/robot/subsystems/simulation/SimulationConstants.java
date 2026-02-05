package frc.robot.subsystems.simulation;

import com.marswars.subsystem.MwConstants;
import com.marswars.util.ConstantsLoader;
import edu.wpi.first.math.util.Units;

public class SimulationConstants extends MwConstants {

    private final ConstantsLoader LOADER = ConstantsLoader.getInstance();

    // =============================================================================
    // ENUMS AND STATE DEFINITIONS
    // =============================================================================

    public enum SimulationStates {
        ACTIVE
    }

    // =============================================================================
    // POSE ESTIMATION SIMULATION
    // =============================================================================
    public final boolean SIM_VISION_ENABLED = false;
    public final double GYRO_NOISE_STD_DEV = Math.toRadians(0.5); // radians (0.5 degrees)
    public final double MODULE_POSITION_NOISE_STD_DEV = 0.01; // meters (1cm per reading)

    // =============================================================================
    // SHOOTER SIMULATION
    // =============================================================================
    public final double SHOOTER_LAUNCH_HEIGHT =
            Units.inchesToMeters(LOADER.getDoubleValue("shooter", "translation", "z"));

    // =============================================================================
    // INTAKE SIMULATION
    // =============================================================================
    public final boolean SIM_FUEL_ENABLED = true;
    public final double BASE_LENGTH =
            Units.inchesToMeters(LOADER.getDoubleValue("swerve", "com", "base_length"));
    public final double BASE_WIDTH =
            Units.inchesToMeters(LOADER.getDoubleValue("swerve", "com", "base_width"));
    public final double BUMPER_HEIGHT =
            Units.inchesToMeters(LOADER.getDoubleValue("swerve", "com", "bumper_height"));
    public final double INTAKE_MAX_EXTENSION =
            Units.inchesToMeters(LOADER.getDoubleValue("intake", "max_extension"));
    public final int HOPPER_CAPACITY = 75; // number of game pieces

    // =============================================================================
    // CONSTRUCTOR
    // =============================================================================

    public SimulationConstants() {}
}
