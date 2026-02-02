package frc.robot.subsystems.localization;

import com.marswars.proxy_server.ProxyServerThread;
import com.marswars.proxy_server.TagSolutionPacket.TagSolutionData;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import com.marswars.swerve_lib.PhoenixOdometryThread;
import com.marswars.swerve_lib.SwerveMeasurements.SwerveMeasurement;
import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.localization.LocalizationConstants.LocalizationStates;
import frc.robot.subsystems.simulation.SimulationSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

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

    // vision detection logging
    private ArrayList<Pose3d> detected_tag_poses_ = new ArrayList<Pose3d>();
    private ArrayList<Pose2d> estimated_vision_poses_ = new ArrayList<Pose2d>();

    public LocalizationSubsystem() {
        // (Default State, Constants Class)
        super(LocalizationStates.FULL, new LocalizationConstants());

        SwerveDriveKinematics kinematics = SwerveSubsystem.getInstance().getKinematics();
        Rotation2d gyro_angle = SwerveSubsystem.getInstance().getGyroRotation();
        SwerveModulePosition[] module_positions = SwerveSubsystem.getInstance().getModulePositions();

        smooth_pose_estimator_ =
                new SwerveDrivePoseEstimator(
                        kinematics, gyro_angle, module_positions, Pose2d.kZero);
        field_pose_estimator_ =
                new SwerveDrivePoseEstimator(
                        kinematics, gyro_angle, module_positions, Pose2d.kZero);
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList();
    }

    @Override
    public void reset() {
        system_state_ = LocalizationStates.FULL;
    }

    @Override
    public void updateLogic(double timestamp) {
        // Get latest swerve measurements from odometry thread
        List<SwerveMeasurement> swerve_measurements_ =
                PhoenixOdometryThread.getInstance().getSwerveSamples();
        // Get latest vision measurements from proxy server
        List<TagSolutionData> vision_measurements =
                ProxyServerThread.getInstance().getLatestTagSolutions();

        switch (system_state_) {
            case ODOM_ONLY: // This state uses only odometry data
                applySwerveMeasurements(smooth_pose_estimator_, swerve_measurements_);
                applySwerveMeasurements(field_pose_estimator_, swerve_measurements_);
                break;
            case VISION_SIM: // This state uses noisy odometry + simulated vision data
                applySwerveMeasurements(smooth_pose_estimator_, swerve_measurements_);
                applyNoisySwerveMeasurements(field_pose_estimator_, swerve_measurements_);
                applyVisionMeasurements(field_pose_estimator_, vision_measurements);
                break;
            case FULL:
            default:
                applySwerveMeasurements(smooth_pose_estimator_, swerve_measurements_);
                applySwerveMeasurements(field_pose_estimator_, swerve_measurements_);
                applyVisionMeasurements(field_pose_estimator_, vision_measurements);
                break;
        }

        // Log the pose estimates
        DogLog.log(getSubsystemKey() + "SmoothPose", getSmoothPose());
        DogLog.log(getSubsystemKey() + "FieldPose", getFieldPose());

        // Log vision detections
        DogLog.log(
                getSubsystemKey() + "DetectedTagPoses",
                detected_tag_poses_.toArray(new Pose3d[detected_tag_poses_.size()]));
        DogLog.log(
                getSubsystemKey() + "EstimatedVisionPoses",
                estimated_vision_poses_.toArray(new Pose2d[estimated_vision_poses_.size()]));
        // Clear logged lists for next cycle
        detected_tag_poses_.clear();
        estimated_vision_poses_.clear();
    }

    /**
     * Applies vision measurements from the proxy server to the field pose estimator.
     *
     * @param pose_estimator The pose estimator to update
     * @param vision_measurements The list of vision measurements to apply
     */
    private void applyVisionMeasurements(
            SwerveDrivePoseEstimator pose_estimator, List<TagSolutionData> vision_measurements) {
        for (TagSolutionData vision_data : vision_measurements) {
            // Add vision measurement to field pose estimator
            pose_estimator.addVisionMeasurement(
                    vision_data.pose, vision_data.timestamp.getSeconds());

            // Log detected tag poses and estimated vision poses
            for (int tag_pose : vision_data.detectedIds) {
                Optional<Pose3d> tag_layout_pose = CONSTANTS.APRIL_TAG_LAYOUT.getTagPose(tag_pose);
                if (tag_layout_pose.isPresent()) detected_tag_poses_.add(tag_layout_pose.get());
            }
            estimated_vision_poses_.add(vision_data.pose);
        }
    }

    /**
     * Applies swerve measurements to the given pose estimator.
     *
     * @param pose_estimator The pose estimator to update
     * @param swerve_measurements The list of swerve measurements to apply
     */
    private void applySwerveMeasurements(
            SwerveDrivePoseEstimator pose_estimator, List<SwerveMeasurement> swerve_measurements) {
        for (int i = 0; i < swerve_measurements.size(); i++) {
            SwerveMeasurement measurement = swerve_measurements.get(i);

            // Update the given Pose Estimator
            pose_estimator.updateWithTime(
                    measurement.timestamp, measurement.gyro_yaw, measurement.module_positions);
        }
    }

    /**
     * Applies swerve measurements to the given pose estimator with noise.
     *
     * @param pose_estimator The pose estimator to update
     * @param swerve_measurements The list of swerve measurements to apply (noise will be added)
     */
    private void applyNoisySwerveMeasurements(
            SwerveDrivePoseEstimator pose_estimator, List<SwerveMeasurement> swerve_measurements) {
        for (int i = 0; i < swerve_measurements.size(); i++) {
            SwerveMeasurement measurement =
                    SimulationSubsystem.getInstance().addNoise(swerve_measurements.get(i));

            // Update the given Pose Estimator
            pose_estimator.updateWithTime(
                    measurement.timestamp, measurement.gyro_yaw, measurement.module_positions);
        }
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
     *
     * @param new_pose The new pose to reset to
     */
    public void resetPoseEstimator(Pose2d new_pose) {
        smooth_pose_estimator_.resetPose(new_pose);
        field_pose_estimator_.resetPose(new_pose);
    }

    public AprilTagFieldLayout getAprilTagLayout() {
        return CONSTANTS.APRIL_TAG_LAYOUT;
    }

    /**
     * @return The chassis speeds in field-relative vectors
     */
    public ChassisSpeeds getChassisSpeedsFieldRelative() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                SwerveSubsystem.getInstance().getChassisSpeeds(), getFieldPose().getRotation());
    }
}
