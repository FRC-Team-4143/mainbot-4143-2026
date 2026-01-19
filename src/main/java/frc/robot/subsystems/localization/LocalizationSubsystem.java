package frc.robot.subsystems.localization;

import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import com.marswars.swerve_lib.PhoenixOdometryThread;
import com.marswars.swerve_lib.SwerveMeasurements.SwerveMeasurement;
import dev.doglog.DogLog;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Robot;
import frc.robot.subsystems.localization.LocalizationConstants.LocalizationStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.Arrays;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class LocalizationSubsystem extends MwSubsystem<LocalizationStates, LocalizationConstants> {
    private static LocalizationSubsystem instance_ = null;

    // Singleton Accessor
    public static LocalizationSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new LocalizationSubsystem();
        }
        return instance_;
    }

    private SwerveDrivePoseEstimator smooth_pose_estimator_;
    private SwerveDrivePoseEstimator field_pose_estimator_;
    private List<SwerveMeasurement> swerve_measurements_;

    private final boolean IS_SIM = Robot.isSimulation();
    private SwerveDriveSimulation swerve_sim_;

    public LocalizationSubsystem() {
        // (Default State, Constants Class)
        super(LocalizationStates.ACTIVE, new LocalizationConstants());

        SwerveDriveKinematics kinematics = SwerveSubsystem.getInstance().getKinematics();
        Rotation2d gyro_angle = SwerveSubsystem.getInstance().getGyroRotation();
        SwerveModulePosition[] module_positions =
                SwerveSubsystem.getInstance().getModulePositions();

        smooth_pose_estimator_ =
                new SwerveDrivePoseEstimator(
                        kinematics, gyro_angle, module_positions, CONSTANTS.START_POSE);
        field_pose_estimator_ =
                new SwerveDrivePoseEstimator(
                        kinematics, gyro_angle, module_positions, CONSTANTS.START_POSE);

        if (IS_SIM) {
            swerve_sim_ = SwerveSubsystem.getInstance().getSwerveSimulation();
            swerve_sim_.setSimulationWorldPose(CONSTANTS.START_POSE);
        }
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList();
    }

    @Override
    public void reset() {
        system_state_ = LocalizationStates.ACTIVE;
    }

    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case ACTIVE:
                swerve_measurements_ = PhoenixOdometryThread.getInstance().getSwerveSamples();
                for (int i = 0; i < swerve_measurements_.size(); i++) {
                    // Update Smooth Pose Estimator
                    smooth_pose_estimator_.updateWithTime(
                            swerve_measurements_.get(i).timestamp,
                            swerve_measurements_.get(i).gyro_yaw,
                            swerve_measurements_.get(i).module_positions);
                    // DO NOT ADD ANY VISION MEASUREMENTS TO THIS ESTIMATOR

                    // Update Field Post Estimator
                    field_pose_estimator_.updateWithTime(
                            swerve_measurements_.get(i).timestamp,
                            swerve_measurements_.get(i).gyro_yaw,
                            swerve_measurements_.get(i).module_positions);
                    // This pose estimator will later have vision measurements added to it
                }
                break;
            case SIMPLE_SIM_CONTROL:
                // Directly override pose estimators with simulation pose
                if (IS_SIM && swerve_sim_ != null) {
                    Pose2d simPose = swerve_sim_.getSimulatedDriveTrainPose();
                    Rotation2d gyroRotation = SwerveSubsystem.getInstance().getGyroRotation();
                    SwerveModulePosition[] modulePositions =
                            SwerveSubsystem.getInstance().getModulePositions();

                    // Reset both pose estimators to the simulation pose
                    smooth_pose_estimator_.resetPosition(gyroRotation, modulePositions, simPose);
                    field_pose_estimator_.resetPosition(gyroRotation, modulePositions, simPose);
                }
                break;
        }

        if (IS_SIM) {
            simulateArena();
        }
        DogLog.log(getSubsystemKey() + "CurrentState", system_state_.toString());
        DogLog.log(getSubsystemKey() + "SmoothPose", getSmoothPose());
        DogLog.log(getSubsystemKey() + "FieldPose", getFieldPose());
    }

    // Only override if you need custom state transitions!
    // @Override
    // public void handleStateTransition(LocalizationStates wanted) {
    // }

    /**
     * @return The smoothed pose estimate of the robot.
     */
    public Pose2d getSmoothPose() {
        return smooth_pose_estimator_.getEstimatedPosition();
    }

    /**
     * @return The field-relative pose estimate of the robot.
     */
    public Pose2d getFieldPose() {
        return field_pose_estimator_.getEstimatedPosition();
    }

    // Private Helper Methods

    /** Simulate the Arena and log game pieces */
    private void simulateArena() {
        DogLog.log(
                getSubsystemKey() + "FieldSimulation/Fuel",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
        DogLog.log(
                getSubsystemKey() + "FieldSimulation/RobotPose",
                swerve_sim_.getSimulatedDriveTrainPose());
    }
}
