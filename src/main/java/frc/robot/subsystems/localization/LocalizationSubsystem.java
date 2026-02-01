package frc.robot.subsystems.localization;

import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import com.marswars.swerve_lib.PhoenixOdometryThread;
import com.marswars.swerve_lib.SwerveMeasurements.SwerveMeasurement;
import dev.doglog.DogLog;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.localization.LocalizationConstants.LocalizationStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.Arrays;
import java.util.List;

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

                    // Update Field Pose Estimator
                    field_pose_estimator_.updateWithTime(
                            swerve_measurements_.get(i).timestamp,
                            swerve_measurements_.get(i).gyro_yaw,
                            swerve_measurements_.get(i).module_positions);
                    // This pose estimator will later have vision measurements added to it
                }
                break;
        }

        DogLog.log(getSubsystemKey() + "SmoothPose", getSmoothPose());
        DogLog.log(getSubsystemKey() + "FieldPose", getFieldPose());
    }

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

    /**
     * Resets both pose estimators to a new pose.
     * @param new_pose The new pose to reset to
     */
    public void resetPoseEstimator(Pose2d new_pose) {
        smooth_pose_estimator_.resetPose(new_pose);
        field_pose_estimator_.resetPose(new_pose);
    }

    /**
     * @return The chassis speeds in field-relative vectors
     */
    public ChassisSpeeds getChassisSpeedsFieldRelative() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                SwerveSubsystem.getInstance().getChassisSpeeds(), getFieldPose().getRotation());
    }
}
