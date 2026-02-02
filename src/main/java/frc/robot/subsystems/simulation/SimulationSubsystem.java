package frc.robot.subsystems.simulation;

import com.marswars.proxy_server.ProxyServerThread;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import com.marswars.swerve_lib.SwerveMeasurements.SwerveMeasurement;
import com.marswars.vision.MwVisionSim;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.localization.LocalizationConstants.LocalizationStates;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.simulation.SimulationConstants.SimulationStates;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

public class SimulationSubsystem extends MwSubsystem<SimulationStates, SimulationConstants> {
    private static SimulationSubsystem instance_ = null;

    public static SimulationSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new SimulationSubsystem();
        }
        return instance_;
    }

    private MwVisionSim vision_sim_;
    private SwerveDriveSimulation swerve_drive_sim_;
    private final Random noise_generator_ = new Random();

    public SimulationSubsystem() {
        super(SimulationStates.ACTIVE, new SimulationConstants());

        if (CONSTANTS.SIM_VISION_ENABLED) {
            vision_sim_ =
                    ProxyServerThread.getInstance()
                            .initializeVisionSimulation(
                                    LocalizationSubsystem.getInstance().getAprilTagLayout());
            vision_sim_.addDefaultCameras();
            LocalizationSubsystem.getInstance().setWantedState(LocalizationStates.VISION_SIM);
        }

        // Initialize the swerve drive simulation
        swerve_drive_sim_ = new SwerveDriveSimulation(CONSTANTS.SWERVE_DRIVE_SIM_CONFIG, LocalizationSubsystem.getInstance().getFieldPose());
        SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
        SimulatedArena.getInstance().addDriveTrainSimulation(swerve_drive_sim_);

        // Setup autonomous reset
        resetForAuto();
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::resetForAuto));
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(
            () -> LocalizationSubsystem.getInstance().resetPoseEstimator(CONSTANTS.INITIAL_ROBOT_POSE)));
    }

    // @Override
    // public void handleStateTransition(SimulationStates wanted) {
    // }

    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case ACTIVE:
            default:
                // No unique behavior currently implemented
                break;
        }

        // Always update the vision simulation if active
        if (vision_sim_ != null) {
            Pose2d robot_pose = LocalizationSubsystem.getInstance().getSmoothPose();
            ProxyServerThread.getInstance()
                    .updateVisionSimulation(robot_pose);
            swerve_drive_sim_.setSimulationWorldPose(robot_pose);
        } else {
            swerve_drive_sim_.setSimulationWorldPose(LocalizationSubsystem.getInstance().getFieldPose());
        }

        // Always update the swerve drive simulation
        SimulatedArena.getInstance().simulationPeriodic();

        // Log MapleSim related object
        DogLog.log(getSubsystemKey() + "MapleSim/RobotPose", swerve_drive_sim_.getSimulatedDriveTrainPose());
        DogLog.log(getSubsystemKey() + "MapleSim/Fuel", SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        // This subsystem has no IOs
        return Arrays.asList();
    }

    @Override
    public void reset() {
        system_state_ = SimulationStates.ACTIVE;
    }

    /** Resets the simulation for autonomous mode. */
    public void resetForAuto(){
        SimulatedArena.getInstance().clearGamePieces();
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    /**
     * Adds Gaussian noise to a complete swerve measurement. Applies noise to both gyro and module
     * position measurements.
     *
     * @param clean_measurement The clean swerve measurement
     * @return Swerve measurement with added noise
     */
    public SwerveMeasurement addNoise(SwerveMeasurement clean_measurement) {
        SwerveMeasurement noisy_measurement = clean_measurement;
        noisy_measurement.gyro_yaw = addGyroNoise(clean_measurement.gyro_yaw);
        noisy_measurement.module_positions =
                addModulePositionNoise(clean_measurement.module_positions);

        return noisy_measurement;
    }

    /**
     * Adds Gaussian noise to gyro measurement for simulation.
     *
     * @param clean_gyro The clean gyro measurement
     * @return Gyro measurement with added noise
     */
    private Rotation2d addGyroNoise(Rotation2d clean_gyro) {
        double noise = noise_generator_.nextGaussian() * CONSTANTS.GYRO_NOISE_STD_DEV;
        return clean_gyro.plus(Rotation2d.fromRadians(noise));
    }

    /**
     * Adds Gaussian noise to module positions for simulation.
     *
     * @param clean_positions The clean module position measurements
     * @return Module positions with added noise
     */
    private SwerveModulePosition[] addModulePositionNoise(SwerveModulePosition[] clean_positions) {
        SwerveModulePosition[] noisy_positions = new SwerveModulePosition[clean_positions.length];
        for (int i = 0; i < clean_positions.length; i++) {
            double distance_noise =
                    noise_generator_.nextGaussian() * CONSTANTS.MODULE_POSITION_NOISE_STD_DEV;
            noisy_positions[i] =
                    new SwerveModulePosition(
                            clean_positions[i].distanceMeters + distance_noise,
                            clean_positions[i].angle);
        }
        return noisy_positions;
    }
}
