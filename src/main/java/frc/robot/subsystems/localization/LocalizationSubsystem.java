package frc.robot.subsystems.localization;

import com.marswars.proxy_server.ProxyServerThread;
import com.marswars.proxy_server.TagSolutionPacket.TagSolutionData;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import com.marswars.swerve_lib.PhoenixOdometryThread;
import com.marswars.swerve_lib.SwerveMeasurements.SwerveMeasurement;
import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.localization.LocalizationConstants.LocalizationStates;
import frc.robot.subsystems.simulation.SimulationSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Set;

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
    private Field2d field_visualizer_ = new Field2d();
    private Set<Integer> shooting_focus_tags_ = CONSTANTS.SHOOTING_FOCUS_TAG_IDS_BLUE;
    private Set<Integer> climbing_focus_tags_ = CONSTANTS.CLIMBING_FOCUS_TAG_IDS_BLUE;

    // vision detection logging
    private ArrayList<Pose3d> detected_tag_poses_ = new ArrayList<Pose3d>();
    private ArrayList<Pose2d> estimated_vision_poses_ = new ArrayList<Pose2d>();
    private boolean swerve_noise_enabled_ = false;

    public LocalizationSubsystem() {
        // (Default State, Constants Class)
        super(LocalizationStates.FULL, new LocalizationConstants());

        SwerveDriveKinematics kinematics = SwerveSubsystem.getInstance().getKinematics();
        Rotation2d gyro_angle = SwerveSubsystem.getInstance().getGyroYaw();
        SwerveModulePosition[] module_positions =
                SwerveSubsystem.getInstance().getModulePositions();

        // No covariance on the smooth estimator since it's only used for short-term smoothing and
        // shouldn't be fed any vision measurements that could cause large jumps in the pose
        // estimate
        smooth_pose_estimator_ =
                new SwerveDrivePoseEstimator(
                        kinematics, gyro_angle, module_positions, Pose2d.kZero);
        // Adjusted covariance on the field estimator to better reflect the expected accuracy of the
        // odometry and vision measurements, which should improve the Kalman filter
        field_pose_estimator_ =
                new SwerveDrivePoseEstimator(
                        kinematics,
                        gyro_angle,
                        module_positions,
                        Pose2d.kZero,
                        CONSTANTS.DEFAULT_ODOM_COVARIANCE,
                        CONSTANTS.DEFAULT_VISION_COVARIANCE);
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
            case SHOOTING_FOCUS: // This state uses the same swerve measurements but different
                // vision covariances based on shooting-focused tags
                applySwerveMeasurements(smooth_pose_estimator_, swerve_measurements_);
                applySwerveMeasurements(field_pose_estimator_, swerve_measurements_);
                applyFilteredVisionMeasurements(
                        field_pose_estimator_,
                        vision_measurements,
                        shooting_focus_tags_,
                        CONSTANTS.SHOOTING_FOCUSED_COVARIANCE,
                        CONSTANTS.SHOOTING_NOT_FOCUSED_COVARIANCE);
                break;
            case CLIMBING_FOCUS: // This state uses the same swerve measurements but different
                // vision covariances based on climbing-focused tags
                applySwerveMeasurements(smooth_pose_estimator_, swerve_measurements_);
                applySwerveMeasurements(field_pose_estimator_, swerve_measurements_);
                applyFilteredVisionMeasurements(
                        field_pose_estimator_,
                        vision_measurements,
                        climbing_focus_tags_,
                        CONSTANTS.CLIMBING_FOCUSED_COVARIANCE,
                        CONSTANTS.CLIMBING_NOT_FOCUSED_COVARIANCE);
                break;
            case FULL: // This state uses full odometry + vision data
            default:
                applySwerveMeasurements(smooth_pose_estimator_, swerve_measurements_);
                applySwerveMeasurements(field_pose_estimator_, swerve_measurements_);
                applyVisionMeasurements(field_pose_estimator_, vision_measurements);
                break;
        }

        // Update field visualizer with the latest field-relative pose
        field_visualizer_.setRobotPose(getFieldPose());

        // Log the pose estimates
        DogLog.log(getSubsystemKey() + "SmoothPose", getSmoothPose());
        DogLog.log(getSubsystemKey() + "FieldPose", getFieldPose());
        SmartDashboard.putData("Field", field_visualizer_);

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
     * Applies vision measurements from the proxy server to the field pose estimator. Uses default
     * covariance for all measurements.
     *
     * @param pose_estimator The pose estimator to update
     * @param vision_measurements The list of vision measurements to apply
     */
    private void applyVisionMeasurements(
            SwerveDrivePoseEstimator pose_estimator, List<TagSolutionData> vision_measurements) {
        // Use filtered method with empty filter set - all measurements use default covariance
        applyFilteredVisionMeasurements(
                pose_estimator,
                vision_measurements,
                Set.of(), // Empty filter set
                CONSTANTS.DEFAULT_VISION_COVARIANCE,
                CONSTANTS.DEFAULT_VISION_COVARIANCE);
    }

    /**
     * Applies vision measurements with different covariances based on detected tag IDs.
     * Measurements containing any tag ID in the filtered set use the "included" covariance, while
     * all others use the "excluded" covariance.
     *
     * @param pose_estimator The pose estimator to update
     * @param vision_measurements The list of vision measurements to apply
     * @param filtered_ids Set of tag IDs to filter for (empty set means all use excluded
     *     covariance)
     * @param included_covariance Covariance matrix for measurements containing filtered tags
     * @param excluded_covariance Covariance matrix for measurements not containing filtered tags
     */
    private void applyFilteredVisionMeasurements(
            SwerveDrivePoseEstimator pose_estimator,
            List<TagSolutionData> vision_measurements,
            Set<Integer> filtered_ids,
            Matrix<N3, N1> included_covariance,
            Matrix<N3, N1> excluded_covariance) {

        if (SwerveSubsystem.getInstance().getGyroYawRate() > CONSTANTS.YAW_RATE_DISCARD) {
            // If the robot is spinning too fast, discard all vision measurements to prevent
            // localization errors from blurred vision data
            return;
        }

        for (TagSolutionData vision_data : vision_measurements) {
            // Skip measurement with no detected tags
            if (vision_data.detectedIds.isEmpty()) {
                continue;
            }

            // Skip measurement if rotation difference is too large
            double rotation_difference = Math.abs(getFieldPose().getRotation().minus(vision_data.pose.getRotation()).getRadians());
            if (rotation_difference > CONSTANTS.MAX_ROTATION_DIFFERENCE) {
                continue;
            }

            // Check if any detected tag ID is in the filtered set (O(1) lookup)
            boolean contains_filtered_id = false;
            for (int detected_id : vision_data.detectedIds) {
                if (filtered_ids.contains(detected_id)) {
                    contains_filtered_id = true;
                    break;
                }
            }

            // Apply the appropriate standard deviation matrix
            Matrix<N3, N1> covariance =
                    contains_filtered_id ? included_covariance : excluded_covariance;
            pose_estimator.addVisionMeasurement(
                    vision_data.pose, vision_data.timestamp.getSeconds(), covariance);

            // Log detected tag poses and estimated vision poses
            for (int tag_id : vision_data.detectedIds) {
                Optional<Pose3d> tag_layout_pose = CONSTANTS.APRIL_TAG_LAYOUT.getTagPose(tag_id);
                if (tag_layout_pose.isPresent()) detected_tag_poses_.add(tag_layout_pose.get());
            }
            estimated_vision_poses_.add(vision_data.pose);
        }
    }

    /**
     * Applies swerve measurements to the given pose estimator with optional noise.
     *
     * @param pose_estimator The pose estimator to update
     * @param swerve_measurements The list of swerve measurements to apply
     */
    private void applySwerveMeasurements(
            SwerveDrivePoseEstimator pose_estimator, List<SwerveMeasurement> swerve_measurements) {
        for (SwerveMeasurement measurement : swerve_measurements) {
            // Optionally add noise for simulation
            if (swerve_noise_enabled_) {
                measurement = SimulationSubsystem.getInstance().addNoise(measurement);
            }

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

    /**
     * @return The AprilTag field layout
     */
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

    /**
     * Enables noise on swerve measurements for testing purposes. This will add random noise to all
     * swerve measurements before they are applied to the pose estimators, simulating real-world
     * sensor imperfections and allowing testing of the localization system's robustness to noisy
     * data.
     */
    public void enableSwerveMeasurementNoise() {
        swerve_noise_enabled_ = true;
    }

    /**
     * Sets the tag focus for vision measurements based on the alliance color. This updates the sets
     * of tag IDs
     *
     * @param alliance The alliance color (Red or Blue)
     */
    public void setTagFocus(Alliance alliance) {
        if (alliance == Alliance.Blue) {
            shooting_focus_tags_ = CONSTANTS.SHOOTING_FOCUS_TAG_IDS_BLUE;
            climbing_focus_tags_ = CONSTANTS.CLIMBING_FOCUS_TAG_IDS_BLUE;
        } else {
            shooting_focus_tags_ = CONSTANTS.SHOOTING_FOCUS_TAG_IDS_RED;
            climbing_focus_tags_ = CONSTANTS.CLIMBING_FOCUS_TAG_IDS_RED;
        }
    }
    ;
}
