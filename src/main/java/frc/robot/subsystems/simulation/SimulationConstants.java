package frc.robot.subsystems.simulation;

import com.marswars.subsystem.MwConstants;

public class SimulationConstants extends MwConstants {

    public enum SimulationStates {
        ACTIVE
    }

    public final boolean SIM_VISION_ENABLED = true;

    // Odometry Noise Parameters
    public final boolean ENABLE_ODOMETRY_NOISE = true;
    public final double GYRO_NOISE_STD_DEV = Math.toRadians(0.5); // radians (0.5 degrees)
    public final double MODULE_POSITION_NOISE_STD_DEV = 0.01; // meters (1cm per reading)

    public SimulationConstants() {}
}
