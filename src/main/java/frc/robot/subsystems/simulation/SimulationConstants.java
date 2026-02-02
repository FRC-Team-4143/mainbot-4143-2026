package frc.robot.subsystems.simulation;

import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import com.marswars.subsystem.MwConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SimulationConstants extends MwConstants {

    public enum SimulationStates {
        ACTIVE
    }

    // Pose Estimation Simulation
    public final boolean SIM_VISION_ENABLED = false;
    public final double GYRO_NOISE_STD_DEV = Math.toRadians(0.5); // radians (0.5 degrees)
    public final double MODULE_POSITION_NOISE_STD_DEV = 0.01; // meters (1cm per reading)
    public final DriveTrainSimulationConfig SWERVE_DRIVE_SIM_CONFIG = DriveTrainSimulationConfig.Default();
    public final Pose2d INITIAL_ROBOT_POSE = new Pose2d(3.4671716690063477, 5.603157997131348,  Rotation2d.fromRadians(2.211905120806451));

    public SimulationConstants() {}
}
