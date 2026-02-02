package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.Meters;

import com.marswars.auto.AutoManager;
import com.marswars.geometry.AllianceFlipUtil;
import com.marswars.proxy_server.ProxyServerThread;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import com.marswars.swerve_lib.SwerveMeasurements.SwerveMeasurement;
import com.marswars.vision.MwVisionSim;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.localization.LocalizationConstants.LocalizationStates;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.simulation.SimulationConstants.SimulationStates;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import org.ironmaple.simulation.IntakeSimulation;
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
    private IntakeSimulation intake_sim_;
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
        SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
        swerve_drive_sim_ =
                new SwerveDriveSimulation(
                        CONSTANTS.SWERVE_DRIVE_SIM_CONFIG,
                        LocalizationSubsystem.getInstance().getFieldPose());
        intake_sim_ =
                IntakeSimulation.OverTheBumperIntake(
                        "Fuel",
                        swerve_drive_sim_,
                        Meters.of(CONSTANTS.INTAKE_WIDTH),
                        Meters.of(CONSTANTS.INTAKE_LENGTH),
                        CONSTANTS.INTAKE_SIDE,
                        CONSTANTS.HOPPER_CAPACITY);
        SimulatedArena.getInstance().addDriveTrainSimulation(swerve_drive_sim_);

        // Setup autonomous reset
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::resetForAuto));
    }

    // @Override
    // public void handleStateTransition(SimulationStates wanted) {
    // }

    @Override
    public void updateLogic(double timestamp) {
        // Always update the vision simulation if active
        if (vision_sim_ != null) {
            Pose2d robot_pose = LocalizationSubsystem.getInstance().getSmoothPose();
            ProxyServerThread.getInstance().updateVisionSimulation(robot_pose);
            swerve_drive_sim_.setSimulationWorldPose(robot_pose);
        } else {
            swerve_drive_sim_.setSimulationWorldPose(
                    LocalizationSubsystem.getInstance().getFieldPose());
        }

        // Update intake simulation
        intake_sim_.startIntake();

        // Always update the swerve drive simulation
        SimulatedArena.getInstance().simulationPeriodic();

        // Log MapleSim related object
        DogLog.log(
                getSubsystemKey() + "MapleSim/RobotPose",
                swerve_drive_sim_.getSimulatedDriveTrainPose());
        DogLog.log(
                getSubsystemKey() + "MapleSim/Fuel",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        DogLog.log(getSubsystemKey() + "MapleSim/Hopper Count", intake_sim_.getGamePiecesAmount());
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
    public void resetForAuto() {
        SimulatedArena.getInstance().clearGamePieces();
        SimulatedArena.getInstance().resetFieldForAuto();
        intake_sim_.setGamePiecesCount(0);

        Pose2d start_pose = AutoManager.getInstance().getSelectedAuto().getStartPose();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            start_pose = AllianceFlipUtil.apply(start_pose);
        }
        LocalizationSubsystem.getInstance().resetPoseEstimator(start_pose);
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
