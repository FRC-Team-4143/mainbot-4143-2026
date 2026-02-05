package frc.robot.subsystems.simulation;

import com.marswars.subsystem.MwConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.autos.ChoreoTraj;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public class SimulationConstants extends MwConstants {

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
    public final DriveTrainSimulationConfig SWERVE_DRIVE_SIM_CONFIG =
            DriveTrainSimulationConfig.Default();
    public final Pose2d INITIAL_ROBOT_POSE = ChoreoTraj.LeftStartNeutralOutpost.initialPoseBlue();

    // =============================================================================
    // INTAKE SIMULATION
    // =============================================================================

    public final double INTAKE_WIDTH = Units.inchesToMeters(20);
    public final double INTAKE_LENGTH = Units.inchesToMeters(12);
    public final IntakeSide INTAKE_SIDE = IntakeSide.BACK;
    public final int HOPPER_CAPACITY = 100; // number of game pieces

    // =============================================================================
    // CONSTRUCTOR
    // =============================================================================

    public SimulationConstants() {}
}
