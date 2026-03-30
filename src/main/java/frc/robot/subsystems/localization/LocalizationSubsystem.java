package frc.robot.subsystems.localization;

import com.marswars.auto.AutoManager;
import com.marswars.geometry.AllianceFlipUtil;
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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.lib2026.FieldConstants;
import frc.robot.lib2026.FieldRegions;
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

    private SwerveDrivePoseEstimator smooth_pose_estimator_;
    private SwerveDrivePoseEstimator field_pose_estimator_;
    private Field2d field_visualizer_ = new Field2d();
    private Set<Integer> shooting_focus_tags_ = CONSTANTS.SHOOTING_FOCUS_TAG_IDS_BLUE;
    private Set<Integer> climbing_focus_tags_ = CONSTANTS.CLIMBING_FOCUS_TAG_IDS_BLUE;

    // vision detection logging
    private ArrayList<Pose3d> detected_tag_poses_ = new ArrayList<Pose3d>();
    private ArrayList<Pose2d> estimated_vision_poses_ = new ArrayList<Pose2d>();
    private boolean swerve_noise_enabled_ = false;

    // Timer to prevent continuous gyro updates while disabled
    private final Timer disabled_gyro_update_timer_ = new Timer();

    // getInstance
    // Singleton Accessor
    public static LocalizationSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new LocalizationSubsystem();
        }
        return instance_;
    }

    // Constructor
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
                        CONSTANTS.DEFAULT_VISION_STD_DEV);

        // Put the field visualizer on SmartDashboard once during initialization
        SmartDashboard.putData("Field", field_visualizer_);
        DogLog.log(getSubsystemKey() + "SwerveNoise", false);

        // Start the timer for disabled gyro updates
        disabled_gyro_update_timer_.start();
    }

    // reset
    @Override
    public void reset() {
        system_state_ = LocalizationStates.FULL;
    }

    // getIos
    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList();
    }

    // updateLogic
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
                        CONSTANTS.SHOOTING_FOCUSED_STD_DEV,
                        CONSTANTS.SHOOTING_NOT_FOCUSED_STD_DEV);
                break;
            case CLIMBING_FOCUS: // This state uses the same swerve measurements but different
                // vision covariances based on climbing-focused tags
                applySwerveMeasurements(smooth_pose_estimator_, swerve_measurements_);
                applySwerveMeasurements(field_pose_estimator_, swerve_measurements_);
                applyFilteredVisionMeasurements(
                        field_pose_estimator_,
                        vision_measurements,
                        climbing_focus_tags_,
                        CONSTANTS.CLIMBING_FOCUSED_STD_DEV,
                        CONSTANTS.CLIMBING_NOT_FOCUSED_STD_DEV);
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

        // If the robot is disabled and there is a client connection with valid vision data,
        // periodically update the gyro yaw to correct for drift. This is done AFTER all
        // measurements are applied to prevent feedback loops.
        // if (RobotState.isDisabled()
        //         && ProxyServerThread.getInstance().hasClientConnection()
        //         && !vision_measurements.isEmpty()
        //         && disabled_gyro_update_timer_.hasElapsed(1.0)) {
        //     SwerveSubsystem.getInstance()
        //             .setGyroYaw(field_pose_estimator_.getEstimatedPosition().getRotation());
        //     disabled_gyro_update_timer_.restart();
        // } else if (!RobotState.isDisabled()) {
        //     // Reset timer when robot is enabled
        //     disabled_gyro_update_timer_.restart();
        // }

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

    // =============================================================================
    // PUBLIC HELPER METHODS
    // =============================================================================

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
     * Resets the pose estimator to the starting pose for autonomous mode, considering alliance
     * color.
     */
    public void resetPoseEstimatorAuto() {
        // Move robot to starting pose
        Pose2d start_pose = AutoManager.getInstance().getSelectedAuto().getStartPose();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            start_pose = AllianceFlipUtil.apply(start_pose);
        }
        resetPoseEstimator(start_pose);
    }

    /**
     * @return The AprilTag field layout
     */
    public AprilTagFieldLayout getAprilTagLayout() {
        return CONSTANTS.APRIL_TAG_LAYOUT;
    }

    /**
     * @return The current chassis speeds in field-relative vectors
     */
    public ChassisSpeeds getCurrentChassisSpeedsFieldRelative() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
                SwerveSubsystem.getInstance().getCurrentChassisSpeeds(),
                getFieldPose().getRotation());
    }

    /**
     * Enables noise on swerve measurements for testing purposes. This will add random noise to all
     * swerve measurements before they are applied to the pose estimators, simulating real-world
     * sensor imperfections and allowing testing of the localization system's robustness to noisy
     * data.
     */
    public void enableSwerveMeasurementNoise() {
        DogLog.log(getSubsystemKey() + "SwerveNoise", true);
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

    // =============================================================================
    // PRIVATE HELPER METHODS
    // =============================================================================

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
                CONSTANTS.DEFAULT_VISION_STD_DEV,
                CONSTANTS.DEFAULT_VISION_STD_DEV);
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
            // -----------------------------------------------------------------
            // Step 1: basic sanity checks
            //   - ignore empty detections
            //   - ignore poses that are clearly off the field
            // -----------------------------------------------------------------
            int tagCount = vision_data.detectedIds.size();
            if (tagCount == 0) {
                continue;
            }

            if (vision_data.pose.getX() < 0
                    || vision_data.pose.getX() > FieldConstants.FIELD_LENGTH
                    || vision_data.pose.getY() < 0
                    || vision_data.pose.getY() > FieldConstants.FIELD_WIDTH) {
                // Vision solution outside field bounds -> discard
                continue;
            }

            // Compute distance and difference vector between odometry and vision poses.
            // These are used for single-tag acceptance heuristics and for logging.
            double distance_difference =
                    getFieldPose().getTranslation().getDistance(vision_data.pose.getTranslation());

            Translation2d difference_vector =
                    getFieldPose().getTranslation().minus(vision_data.pose.getTranslation());

            SmartDashboard.putNumber("distance difference", distance_difference);
            SmartDashboard.putNumber("vector difference x", difference_vector.getX());
            SmartDashboard.putNumber("vector difference y", difference_vector.getY());

            // Maximum distance used to clamp large vision-odometry differences when
            // creating an estimated pose for visualization. This prevents plotting
            // wildly distant single measurements as absolute jumps in the field
            // visualizer. Keep as a small local clamp (original default 1.0m).
            double maxdistance = 1.0;
            double ratio = (distance_difference > 0.0) ? (maxdistance / distance_difference) : 1.0;

            // -----------------------------------------------------------------
            // Step 2: rotation difference check
            //   If the rotation difference is very large and we're enabled, discard
            //   the measurement to avoid bad vision corrections during operation.
            // -----------------------------------------------------------------
            double rotation_difference =
                    Math.abs(
                            getFieldPose()
                                    .getRotation()
                                    .minus(vision_data.pose.getRotation())
                                    .getRadians());
            if (rotation_difference > CONSTANTS.MAX_ROTATION_DIFFERENCE
                    && DriverStation.isEnabled()) {
                continue;
            }

            // -----------------------------------------------------------------
            // Step 3: tag count enforcement (configurable)
            //   The previous quick patch enforced MIN_TAG_COUNT only when the robot
            //   was enabled and inside the alliance zone. That behavior is preserved
            //   but made explicit and configurable below.
            // -----------------------------------------------------------------
            boolean inAllianceZone = FieldRegions.ALLIANCE_ZONE.contains(getFieldPose());
            boolean enforceMinTags =
                    CONSTANTS.ENFORCE_MIN_TAGS_IN_ALLIANCE_ZONE_WHEN_ENABLED
                            && !DriverStation.isDisabled()
                            && inAllianceZone;

            if (enforceMinTags && tagCount < CONSTANTS.MIN_TAG_COUNT_FOR_VISION_UPDATE) {
                // Allow a single-tag measurement only if configured and quality checks
                // pass (e.g. close to odometry pose and optionally only while disabled).
                if (tagCount == 1 && CONSTANTS.ALLOW_SINGLE_TAG_UPDATES) {
                    if (CONSTANTS.SINGLE_TAG_ONLY_WHILE_DISABLED && !DriverStation.isDisabled()) {
                        // single-tag updates only allowed while disabled
                        continue;
                    }
                    if (distance_difference > CONSTANTS.SINGLE_TAG_MAX_ACCEPT_DISTANCE_METERS) {
                        // single-tag solution too far from odometry -> reject
                        continue;
                    }
                    // otherwise, accept single-tag (with higher covariance applied later)
                } else {
                    // Not enough tags and single-tag updates not allowed -> discard
                    continue;
                }
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

            if (distance_difference > maxdistance) {
                Translation2d new_vector =
                        new Translation2d(
                                getFieldPose().getX() + difference_vector.getX() * ratio,
                                getFieldPose().getY() + difference_vector.getY() * ratio);
                Pose2d new_pose = new Pose2d(new_vector, vision_data.pose.getRotation());
                estimated_vision_poses_.add(new_pose);
            } else estimated_vision_poses_.add(vision_data.pose);
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
    ;
}
