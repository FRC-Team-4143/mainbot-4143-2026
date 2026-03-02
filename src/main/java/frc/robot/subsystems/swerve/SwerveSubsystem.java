package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.marswars.auto.ChoreoEventTracker;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import com.marswars.swerve_lib.ChassisRequest;
import com.marswars.swerve_lib.ChassisRequest.XPositiveReference;
import com.marswars.swerve_lib.SwerveMech;
import com.marswars.swerve_lib.module.Module.DriveControlMode;
import com.marswars.swerve_lib.module.Module.SteerControlMode;
import com.marswars.util.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.OI;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants.OperatorPerspective;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class SwerveSubsystem extends MwSubsystem<SwerveStates, SwerveConstants> {

    private static SwerveSubsystem instance_ = null;

    // State Specific Members
    Trajectory<SwerveSample> desired_choreo_traj_;
    private final Timer choreo_timer_ = new Timer();
    private Optional<SwerveSample> choreo_sample_to_apply_;
    private final ChoreoEventTracker choreo_event_tracker_;
    private final PIDController choreo_x_controller_ =
            new PIDController(
                    CONSTANTS.CHOREO_TRANSLATION_CONTROLLER_KP,
                    CONSTANTS.CHOREO_TRANSLATION_CONTROLLER_KI,
                    CONSTANTS.CHOREO_TRANSLATION_CONTROLLER_KD);
    private final PIDController choreo_y_controller_ =
            new PIDController(
                    CONSTANTS.CHOREO_TRANSLATION_CONTROLLER_KP,
                    CONSTANTS.CHOREO_TRANSLATION_CONTROLLER_KI,
                    CONSTANTS.CHOREO_TRANSLATION_CONTROLLER_KD);
    private final PIDController choreo_theta_controller_ =
            new PIDController(
                    CONSTANTS.CHOREO_THETA_CONTROLLER_KP,
                    CONSTANTS.CHOREO_THETA_CONTROLLER_KI,
                    CONSTANTS.CHOREO_THETA_CONTROLLER_KD);

    private Pose2d desired_tractor_beam_pose_ = new Pose2d();
    private double max_lin_vel_for_tractor_beam_;
    private double max_ang_vel_for_tractor_beam_;
    private final PIDController tractor_beam_controller_ =
            new PIDController(
                    CONSTANTS.TRACTOR_BEAM_CONTROLLER_KP,
                    CONSTANTS.TRACTOR_BEAM_CONTROLLER_KI,
                    CONSTANTS.TRACTOR_BEAM_CONTROLLER_KD);
    private Rotation2d desired_rotation_lock_rot_ = new Rotation2d();
    private Translation2d desired_rotation_lock_cor_ = new Translation2d();
    private double desired_rotation_lock_feedforward_ = 0.0;
    private ChassisSpeeds desired_chassis_speeds_ = new ChassisSpeeds(0, 0, 0);
    private double tele_op_velocity_scalar_ = 1.0;

    // IO Members
    private SwerveMech swerve_mech_;
    private Rotation2d operator_forward_direction_ = OperatorPerspective.BLUE_ALLIANCE.heading;

    private ChassisRequest.FieldCentric field_centric_request_;
    private ChassisRequest.RobotCentric robot_centric_request_;
    private ChassisRequest.FieldCentricFacingAngle choreo_rotation_lock_request_;
    private ChassisRequest.FieldCentricFacingAngle field_centric_rotation_lock_request_;
    private ChassisRequest.RobotCentricFacingAngle robot_centric_rotation_lock_request_;
    private ChassisRequest.ApplyFieldSpeeds field_speeds_request_;
    private ChassisRequest.ApplyChassisSpeeds chassis_speeds_request_;

    // getInstance
    public static SwerveSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new SwerveSubsystem();
        }
        return instance_;
    }

    /**
     * Creates a new Swerve subsystem with the specified gyro and module IOs.
     *
     * @param io The swerve I/O container to use
     */
    // Constructor
    public SwerveSubsystem() {
        super(SwerveStates.IDLE, new SwerveConstants());

        swerve_mech_ = new SwerveMech(getSubsystemKey(), CONSTANTS.SWERVE_DRIVE_CONFIG);

        // Initialize event tracker with pose supplier
        choreo_event_tracker_ =
                new ChoreoEventTracker(
                        getSubsystemKey() + "Choreo/Events/",
                        () -> LocalizationSubsystem.getInstance().getFieldPose());
        choreo_theta_controller_.enableContinuousInput(-Math.PI / 2.0, Math.PI / 2.0);
        // Initialize drive mode requests
        field_centric_request_ =
                new ChassisRequest.FieldCentric()
                        .withDriveRequestType(DriveControlMode.OPEN_LOOP)
                        .withSteerRequestType(SteerControlMode.CLOSED_LOOP)
                        .withDeadband(CONSTANTS.MAX_TRANSLATION_RATE * 0.01)
                        .withRotationalDeadband(CONSTANTS.MAX_ANGULAR_RATE * 0.01)
                        .withXPositiveReference(XPositiveReference.OperatorPerspective);
        robot_centric_request_ =
                new ChassisRequest.RobotCentric()
                        .withDriveRequestType(DriveControlMode.OPEN_LOOP)
                        .withSteerRequestType(SteerControlMode.CLOSED_LOOP)
                        .withDeadband(CONSTANTS.MAX_TRANSLATION_RATE * 0.01)
                        .withRotationalDeadband(CONSTANTS.MAX_ANGULAR_RATE * 0.01);
        field_centric_rotation_lock_request_ =
                new ChassisRequest.FieldCentricFacingAngle()
                        .withDriveRequestType(DriveControlMode.OPEN_LOOP)
                        .withSteerRequestType(SteerControlMode.CLOSED_LOOP)
                        .withDeadband(CONSTANTS.MAX_TRANSLATION_RATE * 0.01)
                        .withRotationalDeadband(CONSTANTS.MAX_ANGULAR_RATE * 0.01)
                        .withHeadingController(CONSTANTS.HEADING_CONTROLLER)
                        .withXPositiveReference(XPositiveReference.OperatorPerspective);
        choreo_rotation_lock_request_ =
                new ChassisRequest.FieldCentricFacingAngle()
                        .withDriveRequestType(DriveControlMode.CLOSED_LOOP)
                        .withSteerRequestType(SteerControlMode.CLOSED_LOOP)
                        .withDeadband(CONSTANTS.MAX_TRANSLATION_RATE * 0.01)
                        .withRotationalDeadband(CONSTANTS.MAX_ANGULAR_RATE * 0.01)
                        .withHeadingController(CONSTANTS.HEADING_CONTROLLER)
                        .withXPositiveReference(XPositiveReference.TowardsRedAlliance);
        robot_centric_rotation_lock_request_ =
                new ChassisRequest.RobotCentricFacingAngle()
                        .withDriveRequestType(DriveControlMode.CLOSED_LOOP)
                        .withSteerRequestType(SteerControlMode.CLOSED_LOOP)
                        .withDeadband(CONSTANTS.MAX_TRANSLATION_RATE * 0.01)
                        .withRotationalDeadband(CONSTANTS.MAX_ANGULAR_RATE * 0.01)
                        .withHeadingController(CONSTANTS.HEADING_CONTROLLER);
        field_speeds_request_ =
                new ChassisRequest.ApplyFieldSpeeds()
                        .withDriveRequestType(DriveControlMode.CLOSED_LOOP)
                        .withSteerRequestType(SteerControlMode.CLOSED_LOOP);
        chassis_speeds_request_ =
                new ChassisRequest.ApplyChassisSpeeds()
                        .withDriveRequestType(DriveControlMode.CLOSED_LOOP)
                        .withSteerRequestType(SteerControlMode.CLOSED_LOOP);

        TunablePid.create(getSubsystemKey() + "TractorBeam/Gains", tractor_beam_controller_);
        TunablePid.create(
                getSubsystemKey() + "ChoreoPath/Translation/Gains",
                choreo_x_controller_,
                choreo_y_controller_);
        TunablePid.create(
                getSubsystemKey() + "ChoreoPath/Rotation/Gains", choreo_theta_controller_);
        TunablePid.create(
                getSubsystemKey() + "RotationLock/Gains",
                field_centric_rotation_lock_request_.HeadingController);
    }

    // reset
    @Override
    public void reset() {
        system_state_ = SwerveStates.IDLE;
    }

    // getIos
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(swerve_mech_);
    }

    // handleStateTransition
    /**
     * Gets the current system state of the Swerve subsystem.
     *
     * @return the current system state
     */
    @Override
    protected void handleStateTransition(SwerveStates wanted_state) {
        // Stop event tracker if leaving choreo states
        if ((system_state_ == SwerveStates.CHOREO_PATH
                        || system_state_ == SwerveStates.CHOREO_PATH_ROTATION_LOCK)
                && wanted_state != SwerveStates.CHOREO_PATH
                && wanted_state != SwerveStates.CHOREO_PATH_ROTATION_LOCK) {
            choreo_timer_.stop();
            choreo_event_tracker_.stop();
        }

        // switch to CRAWL or CRAWL_ROTATION_LOCK if POV is being used
        if (OI.getDriverJoystickPOV().isPresent()) {
            // Determine which crawl state to use based on the wanted state
            if (wanted_state == SwerveStates.FIELD_CENTRIC_ROTATION_LOCK) {
                system_state_ = SwerveStates.CRAWL_FIELD_CENTRIC_ROTATION_LOCK;
            } else if (wanted_state == SwerveStates.ROBOT_CENTRIC_ROTATION_LOCK
                    || wanted_state == SwerveStates.CHASSIS_SPEEDS_ROTATION_LOCK) {
                system_state_ = SwerveStates.CRAWL_ROBOT_CENTRIC_ROTATION_LOCK;
            } else if (wanted_state == SwerveStates.FIELD_CENTRIC) {
                system_state_ = SwerveStates.CRAWL_FIELD_CENTRIC;
            } else {
                // Default to robot-centric crawl for all other states
                system_state_ = SwerveStates.CRAWL_ROBOT_CENTRIC;
            }
            return;
        }

        system_state_ =
                switch (wanted_state) {
                    case ROBOT_CENTRIC -> SwerveStates.ROBOT_CENTRIC;
                    case FIELD_CENTRIC -> SwerveStates.FIELD_CENTRIC;
                    case CHOREO_PATH -> {
                        // If we are not already in a choreo path state, restart the timer
                        // The additional check is needed to prevent resetting the timer when
                        // switching between the two choreo states
                        if (system_state_ != SwerveStates.CHOREO_PATH
                                && system_state_ != SwerveStates.CHOREO_PATH_ROTATION_LOCK) {
                            choreo_x_controller_.reset();
                            choreo_y_controller_.reset();
                            choreo_theta_controller_.reset();
                            choreo_timer_.restart();
                            choreo_event_tracker_.start();
                            choreo_sample_to_apply_ =
                                    desired_choreo_traj_.sampleAt(
                                            choreo_timer_.get(), CONSTANTS.FLIP_TRAJECTORY_ON_RED);
                            yield SwerveStates.CHOREO_PATH;
                        } else {
                            choreo_sample_to_apply_ =
                                    desired_choreo_traj_.sampleAt(
                                            choreo_timer_.get(), CONSTANTS.FLIP_TRAJECTORY_ON_RED);
                            yield SwerveStates.CHOREO_PATH;
                        }
                    }
                    case TRACTOR_BEAM -> SwerveStates.TRACTOR_BEAM;
                    case CHASSIS_SPEEDS -> SwerveStates.CHASSIS_SPEEDS;
                    case CRAWL_ROBOT_CENTRIC -> SwerveStates.CRAWL_ROBOT_CENTRIC;
                    case CRAWL_FIELD_CENTRIC -> SwerveStates.CRAWL_FIELD_CENTRIC;
                    case ROBOT_CENTRIC_ROTATION_LOCK -> SwerveStates.ROBOT_CENTRIC_ROTATION_LOCK;
                    case FIELD_CENTRIC_ROTATION_LOCK -> SwerveStates.FIELD_CENTRIC_ROTATION_LOCK;
                    case CHOREO_PATH_ROTATION_LOCK -> {
                        // If we are not already in a choreo path state, restart the timer
                        // The additional check is needed to prevent resetting the timer when
                        // switching between the two choreo states
                        if (system_state_ != SwerveStates.CHOREO_PATH_ROTATION_LOCK
                                && system_state_ != SwerveStates.CHOREO_PATH) {
                            choreo_x_controller_.reset();
                            choreo_y_controller_.reset();
                            choreo_theta_controller_.reset();
                            choreo_timer_.restart();
                            choreo_event_tracker_.start();
                            choreo_sample_to_apply_ =
                                    desired_choreo_traj_.sampleAt(
                                            choreo_timer_.get(), CONSTANTS.FLIP_TRAJECTORY_ON_RED);
                            yield SwerveStates.CHOREO_PATH_ROTATION_LOCK;
                        } else {
                            choreo_sample_to_apply_ =
                                    desired_choreo_traj_.sampleAt(
                                            choreo_timer_.get(), CONSTANTS.FLIP_TRAJECTORY_ON_RED);
                            yield SwerveStates.CHOREO_PATH_ROTATION_LOCK;
                        }
                    }
                    case CRAWL_ROBOT_CENTRIC_ROTATION_LOCK ->
                            SwerveStates.CRAWL_ROBOT_CENTRIC_ROTATION_LOCK;
                    case CRAWL_FIELD_CENTRIC_ROTATION_LOCK ->
                            SwerveStates.CRAWL_FIELD_CENTRIC_ROTATION_LOCK;
                    case CHASSIS_SPEEDS_ROTATION_LOCK -> SwerveStates.CHASSIS_SPEEDS_ROTATION_LOCK;
                    case TUNING -> SwerveStates.TUNING;
                    default -> SwerveStates.IDLE;
                };
    }

    // updateLogic
    @Override
    public void updateLogic(double timestamp) {
        // Update the request to apply based on the system state
        switch (system_state_) {
            case ROBOT_CENTRIC:
                swerve_mech_.setChassisRequest(
                        robot_centric_request_.withTwist(calculateSpeedsBasedOnJoystickInputs()));
                break;
            case FIELD_CENTRIC:
                swerve_mech_.setChassisRequest(
                        field_centric_request_.withTwist(calculateSpeedsBasedOnJoystickInputs()));
                break;
            case CHOREO_PATH:
                choreoPathState();
                break;
            case TRACTOR_BEAM:
                tractorBeamState();
                break;
            case CHASSIS_SPEEDS:
                swerve_mech_.setChassisRequest(
                        chassis_speeds_request_.withSpeeds(desired_chassis_speeds_));
                DogLog.log(getSubsystemKey() + "DesiredChassisSpeeds", desired_chassis_speeds_);
                break;
            case CRAWL_ROBOT_CENTRIC:
                handleCrawlState(false);
                break;
            case CRAWL_FIELD_CENTRIC:
                handleFieldCentricCrawlState(false);
                break;
            case ROBOT_CENTRIC_ROTATION_LOCK:
                Twist2d robot_centric_twist = calculateSpeedsBasedOnJoystickInputs();
                swerve_mech_.setChassisRequest(
                        robot_centric_rotation_lock_request_
                                .withTargetHeading(desired_rotation_lock_rot_)
                                .withSpeeds(
                                        new ChassisSpeeds(
                                                robot_centric_twist.dx,
                                                robot_centric_twist.dy,
                                                0.0))
                                .withCenterOfRotation(desired_rotation_lock_cor_)
                                .withHeadingFeedforward(desired_rotation_lock_feedforward_));
                DogLog.log(
                        getSubsystemKey() + "RotationLock/Rotation",
                        desired_rotation_lock_rot_.getRadians(),
                        Radians);
                DogLog.log(getSubsystemKey() + "RotationLock/COR", desired_rotation_lock_cor_);
                break;
            case FIELD_CENTRIC_ROTATION_LOCK:
                swerve_mech_.setChassisRequest(
                        field_centric_rotation_lock_request_
                                .withTargetHeading(desired_rotation_lock_rot_)
                                .withTwist(calculateSpeedsBasedOnJoystickInputs())
                                .withCenterOfRotation(desired_rotation_lock_cor_)
                                .withHeadingFeedforward(desired_rotation_lock_feedforward_));
                DogLog.log(
                        getSubsystemKey() + "RotationLock/Rotation",
                        desired_rotation_lock_rot_.getRadians(),
                        Radians);
                DogLog.log(getSubsystemKey() + "RotationLock/COR", desired_rotation_lock_cor_);
                break;
            case CHOREO_PATH_ROTATION_LOCK:
                choreoPathRotationLockState();
                break;
            case CRAWL_ROBOT_CENTRIC_ROTATION_LOCK:
                handleCrawlState(true);
                break;
            case CRAWL_FIELD_CENTRIC_ROTATION_LOCK:
                handleFieldCentricCrawlState(true);
                break;
            case CHASSIS_SPEEDS_ROTATION_LOCK:
                swerve_mech_.setChassisRequest(
                        robot_centric_rotation_lock_request_
                                .withTargetHeading(desired_rotation_lock_rot_)
                                .withSpeeds(desired_chassis_speeds_)
                                .withCenterOfRotation(desired_rotation_lock_cor_)
                                .withHeadingFeedforward(desired_rotation_lock_feedforward_));
                DogLog.log(
                        getSubsystemKey() + "RotationLock/ChassisSpeed", desired_chassis_speeds_);
                DogLog.log(
                        getSubsystemKey() + "RotationLock/Rotation",
                        desired_rotation_lock_rot_.getRadians(),
                        Radians);
                DogLog.log(getSubsystemKey() + "RotationLock/COR", desired_rotation_lock_cor_);
                break;
            case TUNING:
                swerve_mech_.setChassisRequest(
                        chassis_speeds_request_.withSpeeds(desired_chassis_speeds_));
                DogLog.log(getSubsystemKey() + "DesiredChassisSpeeds", desired_chassis_speeds_);
                break;
            case IDLE:
            default:
                swerve_mech_.setChassisRequest(new ChassisRequest.Idle());
                break;
        }

        // Set state static request parameters
        swerve_mech_.setChassisRequestParameters(
                LocalizationSubsystem.getInstance().getFieldPose(), operator_forward_direction_);
    }

    // =============================================================================
    // PUBLIC HELPER METHODS
    // =============================================================================

    /**
     * Handles the TRACTOR_BEAM state by calculating the necessary chassis speeds to move towards
     * the desired tractor beam pose.
     */
    private void tractorBeamState() {
        Translation2d translation_to_desired_point =
                desired_tractor_beam_pose_
                        .getTranslation()
                        .minus(LocalizationSubsystem.getInstance().getFieldPose().getTranslation());
        double linear_distance = translation_to_desired_point.getNorm();
        double friction_constant = 0.0;
        if (linear_distance >= Units.inchesToMeters(0.5)) {
            friction_constant =
                    CONSTANTS.TRACTOR_BEAM_STATIC_FRICTION_CONSTANT
                            * CONSTANTS.MAX_TRANSLATION_RATE;
        }
        Rotation2d direction_of_travel = translation_to_desired_point.getAngle();
        double velocity_output =
                Math.min(
                        Math.abs(tractor_beam_controller_.calculate(linear_distance, 0))
                                + friction_constant,
                        max_lin_vel_for_tractor_beam_);
        double x_component = velocity_output * direction_of_travel.getCos();
        double y_component = velocity_output * direction_of_travel.getSin();

        DogLog.log(
                getSubsystemKey() + "TractorBeam/XVelocityComponent", x_component, MetersPerSecond);
        DogLog.log(
                getSubsystemKey() + "TractorBeam/YVelocityComponent", y_component, MetersPerSecond);
        DogLog.log(
                getSubsystemKey() + "TractorBeam/VelocityOutput", velocity_output, MetersPerSecond);
        DogLog.log(getSubsystemKey() + "TractorBeam/LinearDistance", linear_distance, Meters);
        DogLog.log(
                getSubsystemKey() + "TractorBeam/DirectionOfTravel",
                direction_of_travel.getRadians(),
                Radians);
        DogLog.log(getSubsystemKey() + "TractorBeam/DesiredPose", desired_tractor_beam_pose_);

        if (Double.isNaN(max_ang_vel_for_tractor_beam_)) {
            swerve_mech_.setChassisRequest(
                    field_centric_rotation_lock_request_
                            .withTwist(new Twist2d(x_component, y_component, 0.0))
                            .withTargetHeading(desired_tractor_beam_pose_.getRotation()));
        } else {
            swerve_mech_.setChassisRequest(
                    field_centric_rotation_lock_request_
                            .withTwist(new Twist2d(x_component, y_component, 0.0))
                            .withTargetHeading(desired_tractor_beam_pose_.getRotation())
                            .withMaxAbsRotationalRate(max_ang_vel_for_tractor_beam_));
        }
    }

    /**
     * Handles the CHOREO_PATH state by applying the appropriate chassis speeds based on the current
     * trajectory sample and PID controller outputs.
     */
    private void choreoPathState() {
        // Update event tracker with current time
        choreo_event_tracker_.update(choreo_timer_.get());

        if (choreo_sample_to_apply_.isPresent()) {
            SwerveSample sample = choreo_sample_to_apply_.get();
            DogLog.log(getSubsystemKey() + "Choreo/TimerValue", choreo_timer_.get(), Seconds);
            DogLog.log(getSubsystemKey() + "Choreo/TrajName", desired_choreo_traj_.name());
            DogLog.log(
                    getSubsystemKey() + "Choreo/TotalTime",
                    desired_choreo_traj_.getTotalTime(),
                    Seconds);
            DogLog.log(getSubsystemKey() + "Choreo/sample/DesiredPose", sample.getPose());
            DogLog.log(
                    getSubsystemKey() + "Choreo/sample/DesiredChassisSpeeds",
                    sample.getChassisSpeeds());
            Pose2d pose = LocalizationSubsystem.getInstance().getFieldPose();
            ChassisSpeeds target_speeds = sample.getChassisSpeeds();
            target_speeds.vxMetersPerSecond +=
                    choreo_x_controller_.calculate(pose.getX(), sample.x);
            target_speeds.vyMetersPerSecond +=
                    choreo_y_controller_.calculate(pose.getY(), sample.y);
            target_speeds.omegaRadiansPerSecond +=
                    choreo_theta_controller_.calculate(
                            pose.getRotation().getRadians(), sample.heading);

            swerve_mech_.setChassisRequest(field_speeds_request_.withSpeeds(target_speeds));
        } else {
            // If no sample is available, we will just stop the robot
            swerve_mech_.setChassisRequest(new ChassisRequest.Idle());
        }
    }

    /**
     * Handles the CHOREO_PATH_ROTATION_LOCK state by applying chassis speeds based on the current
     * trajectory sample, but overriding the rotation to a fixed desired rotation.
     */
    private void choreoPathRotationLockState() {
        // Update event tracker with current time
        choreo_event_tracker_.update(choreo_timer_.get());

        if (choreo_sample_to_apply_.isPresent()) {
            SwerveSample sample = choreo_sample_to_apply_.get();
            DogLog.log(getSubsystemKey() + "Choreo/TimerValue", choreo_timer_.get(), Seconds);
            DogLog.log(getSubsystemKey() + "Choreo/TrajName", desired_choreo_traj_.name());
            DogLog.log(
                    getSubsystemKey() + "Choreo/TotalTime",
                    desired_choreo_traj_.getTotalTime(),
                    Seconds);
            // Generate new pose with overridden rotation
            Pose2d overridden_pose =
                    new Pose2d(sample.getPose().getTranslation(), desired_rotation_lock_rot_);
            DogLog.log(getSubsystemKey() + "Choreo/sample/DesiredPose", overridden_pose);
            // Generate new chassis speeds with overridden rotation speed
            ChassisSpeeds overridden_speeds = sample.getChassisSpeeds();
            overridden_speeds.omegaRadiansPerSecond = 0.0;
            DogLog.log(getSubsystemKey() + "Choreo/sample/DesiredChassisSpeeds", overridden_speeds);
            Pose2d pose = LocalizationSubsystem.getInstance().getFieldPose();
            ChassisSpeeds target_speeds = overridden_speeds;
            target_speeds.vxMetersPerSecond +=
                    choreo_x_controller_.calculate(pose.getX(), sample.x);
            target_speeds.vyMetersPerSecond +=
                    choreo_y_controller_.calculate(pose.getY(), sample.y);

            swerve_mech_.setChassisRequest(
                    choreo_rotation_lock_request_
                            .withTwist(target_speeds.toTwist2d(1.0))
                            .withTargetHeading(desired_rotation_lock_rot_));
        } else {
            // If no sample is available, we will just stop the robot
            swerve_mech_.setChassisRequest(new ChassisRequest.Idle());
        }
    }

    /**
     * Handles crawl states (robot-centric POV movement at reduced speed).
     *
     * @param is_rotation_locked whether the rotation should be locked to a target heading
     */
    private void handleCrawlState(boolean is_rotation_locked) {
        Twist2d crawl_twist = calculateSpeedBasedonPOVInputs();
        if (is_rotation_locked) {
            swerve_mech_.setChassisRequest(
                    robot_centric_rotation_lock_request_
                            .withTargetHeading(desired_rotation_lock_rot_)
                            .withSpeeds(new ChassisSpeeds(crawl_twist.dx, crawl_twist.dy, 0.0))
                            .withCenterOfRotation(desired_rotation_lock_cor_)
                            .withHeadingFeedforward(desired_rotation_lock_feedforward_));
            DogLog.log(
                    getSubsystemKey() + "RotationLock/Rotation",
                    desired_rotation_lock_rot_.getRadians(),
                    Radians);
            DogLog.log(getSubsystemKey() + "RotationLock/COR", desired_rotation_lock_cor_);
        } else {
            swerve_mech_.setChassisRequest(robot_centric_request_.withTwist(crawl_twist));
        }
    }

    /**
     * Handles field-centric crawl states (POV movement at reduced speed relative to field).
     *
     * @param is_rotation_locked whether the rotation should be locked to a target heading
     */
    private void handleFieldCentricCrawlState(boolean is_rotation_locked) {
        Twist2d crawl_twist = calculateSpeedBasedonPOVInputs();
        if (is_rotation_locked) {
            swerve_mech_.setChassisRequest(
                    field_centric_rotation_lock_request_
                            .withTargetHeading(desired_rotation_lock_rot_)
                            .withTwist(crawl_twist)
                            .withCenterOfRotation(desired_rotation_lock_cor_)
                            .withHeadingFeedforward(desired_rotation_lock_feedforward_));
            DogLog.log(
                    getSubsystemKey() + "RotationLock/Rotation",
                    desired_rotation_lock_rot_.getRadians(),
                    Radians);
            DogLog.log(getSubsystemKey() + "RotationLock/COR", desired_rotation_lock_cor_);
        } else {
            swerve_mech_.setChassisRequest(field_centric_request_.withTwist(crawl_twist));
        }
    }

    // ------------------------------------------------
    // Chassis Control Methods
    // ------------------------------------------------

    /**
     * Updates the internal target for the robot to follow in CHOREO_PATH or
     * CHOREO_PATH_ROTATION_LOCK
     *
     * @param trajectory the trajectory for the robot to follow
     */
    public void setDesiredChoreoTrajectory(Trajectory<SwerveSample> trajectory) {
        desired_choreo_traj_ = trajectory;

        // Load events from the trajectory (passes trajectory so poses can be extracted)
        choreo_event_tracker_.setEvents(trajectory, CONSTANTS.FLIP_TRAJECTORY_ON_RED);

        // Reset the timer if we are already in a choreo path state to restart the new trajectory
        if (system_state_ == SwerveStates.CHOREO_PATH
                || system_state_ == SwerveStates.CHOREO_PATH_ROTATION_LOCK) {
            choreo_timer_.reset();
            choreo_event_tracker_.start();
            choreo_x_controller_.reset();
            choreo_y_controller_.reset();
            choreo_theta_controller_.reset();
        }

        // Log the trajectory poses for debugging
        DogLog.log(
                getSubsystemKey() + "Choreo/Trajectory",
                (CONSTANTS.FLIP_TRAJECTORY_ON_RED ? trajectory.flipped() : trajectory).getPoses());
    }

    /**
     * Command version of {@link #setDesiredChoreoTrajectory(Trajectory)}
     *
     * @param trajectory the trajectory for the robot to follow
     * @return A command that sets the desired choreo trajectory
     */
    public Command setDesiredChoreoTrajectoryCommand(Trajectory<SwerveSample> trajectory) {
        return Commands.runOnce(() -> setDesiredChoreoTrajectory(trajectory))
                .withName("Set Choreo Trajectory : " + trajectory.name());
    }

    /**
     * Updates the internal target for the robot to reach in TRACTOR_BEAM
     *
     * @param target_pose target pose for the robot to reach
     */
    public void setDesiredTractorBeamPose(Pose2d pose) {
        desired_tractor_beam_pose_ = pose;
        max_lin_vel_for_tractor_beam_ = CONSTANTS.MAX_TRANSLATION_RATE;
        max_ang_vel_for_tractor_beam_ = Double.NaN;
    }

    /**
     * Updates the internal target for the robot to reach in TRACTOR_BEAM
     *
     * @param pose target pose for the robot to reach
     * @param max_lin_vel maximum linear velocity for the robot to reach the target pose
     */
    public void setDesiredTractorBeamPoseWithMaxLinVel(Pose2d pose, double max_lin_vel) {
        max_lin_vel_for_tractor_beam_ = max_lin_vel;
        max_ang_vel_for_tractor_beam_ = Double.NaN;
        desired_tractor_beam_pose_ = pose;
    }

    /**
     * Updates the internal target for the robot to reach in TRACTOR_BEAM
     *
     * @param pose target pose for the robot to reach
     * @param max_ang_vel maximum angular velocity for the robot to reach the target pose
     */
    public void setDesiredTractorBeamPoseWithMaxAngVel(Pose2d pose, double max_ang_vel) {
        max_lin_vel_for_tractor_beam_ = CONSTANTS.MAX_TRANSLATION_RATE;
        max_ang_vel_for_tractor_beam_ = max_ang_vel;
        desired_tractor_beam_pose_ = pose;
    }

    /**
     * Updates the internal target for the robot to reach in TRACTOR_BEAM
     *
     * @param pose target pose for the robot to reach
     * @param max_lin_vel maximum linear velocity for the robot to reach the target pose
     * @param max_ang_vel maximum angular velocity for the robot to reach the target pose
     */
    public void setTractorBeamPoseWithConstraints(
            Pose2d pose, double max_lin_vel, double max_ang_vel) {
        max_lin_vel_for_tractor_beam_ = max_lin_vel;
        max_ang_vel_for_tractor_beam_ = max_ang_vel;
        desired_tractor_beam_pose_ = pose;
    }

    /**
     * Updates the internal target for the robot to follow in CHASSIS_SPEED_ROTATION_LOCK
     *
     * @param speeds desired chassis speeds
     * @param rotation desired rotation to lock to
     */
    public void setDesiredChassisSpeed(ChassisSpeeds speeds) {
        desired_chassis_speeds_ = speeds;
    }

    /**
     * Updates the internal target for the robot to face turing around a desired center point in
     * FIELD_CENTRIC_ROTATION_LOCK or CHOREO_PATH_ROTATION_LOCK
     *
     * @param rotation desired rotation to lock to
     */
    public void setDesiredRotationLock(Rotation2d rotation) {
        setDesiredRotationLockCOR(rotation, Translation2d.kZero);
    }

    /**
     * Updates the internal target for the robot to face turing around a desired center point in
     * FIELD_CENTRIC_ROTATION_LOCK or CHOREO_PATH_ROTATION_LOCK
     *
     * @param rotation desired rotation to lock to
     * @param center_point desired center point to rotate around
     */
    public void setDesiredRotationLockCOR(Rotation2d rotation, Translation2d center_point) {
        desired_rotation_lock_rot_ = rotation;
        desired_rotation_lock_cor_ = center_point;
        desired_rotation_lock_feedforward_ = 0.0;
    }

    /**
     * Updates the internal target for the robot to face with feedforward, turning around a desired
     * center point in FIELD_CENTRIC_ROTATION_LOCK or CHOREO_PATH_ROTATION_LOCK
     *
     * @param rotation desired rotation to lock to
     * @param center_point desired center point to rotate around
     * @param feedforward feedforward rotational velocity in rad/s
     */
    public void setDesiredRotationLockCORWithFF(
            Rotation2d rotation, Translation2d center_point, double feedforward) {
        desired_rotation_lock_rot_ = rotation;
        desired_rotation_lock_cor_ = center_point;
        desired_rotation_lock_feedforward_ = feedforward;
    }

    public Command chassisTuningCommand(ChassisSpeeds speeds) {
        return Commands.startEnd(
                        () -> {
                            setDesiredChassisSpeed(speeds);
                            setWantedState(SwerveStates.TUNING);
                        },
                        () -> {
                            setWantedState(SwerveStates.FIELD_CENTRIC);
                        })
                .withName(
                        "Chassis Tuning : X="
                                + speeds.vxMetersPerSecond
                                + " Y="
                                + speeds.vyMetersPerSecond
                                + " Omega="
                                + speeds.omegaRadiansPerSecond);
    }

    /**
     * Toggles between FIELD_CENTRIC and ROBOT_CENTRIC modes.
     *
     * @return A command that toggles the field centric mode
     */
    public Command toggleFieldCentric() {
        String new_mode =
                (system_state_ == SwerveStates.FIELD_CENTRIC) ? "ROBOT_CENTRIC" : "FIELD_CENTRIC";
        return Commands.runOnce(
                        () -> {
                            if (system_state_ == SwerveStates.FIELD_CENTRIC) {
                                setWantedState(SwerveStates.ROBOT_CENTRIC);
                            } else {
                                setWantedState(SwerveStates.FIELD_CENTRIC);
                            }
                        })
                .withName("Toggle Field Centric: " + new_mode);
    }

    // ------------------------------------------------
    // Operator Interface Methods
    // ------------------------------------------------

    /**
     * Calculates chassis speeds based on joystick inputs.
     *
     * @return the controller inputs as a Twist2d object, where the x and y components represent the
     *     translation speeds and the theta component represents the angular speed
     */
    private Twist2d calculateSpeedsBasedOnJoystickInputs() {
        if (DriverStation.getAlliance().isEmpty()) {
            return new Twist2d();
        }

        double x_magnitude =
                -MathUtil.applyDeadband(OI.getDriverJoystickLeftY(), CONSTANTS.CONTROLLER_DEADBAND);
        double y_magnitude =
                -MathUtil.applyDeadband(OI.getDriverJoystickLeftX(), CONSTANTS.CONTROLLER_DEADBAND);
        double angular_magnitude =
                -MathUtil.applyDeadband(
                        OI.getDriverJoystickRightX(), CONSTANTS.CONTROLLER_DEADBAND);

        Twist2d twist =
                new Twist2d(
                        x_magnitude * CONSTANTS.MAX_TRANSLATION_RATE * tele_op_velocity_scalar_,
                        y_magnitude * CONSTANTS.MAX_TRANSLATION_RATE * tele_op_velocity_scalar_,
                        angular_magnitude * CONSTANTS.MAX_ANGULAR_RATE);
        DogLog.log(getSubsystemKey() + "RequestedTwist", twist);
        return twist;
    }

    /**
     * Calculates chassis speeds based on POV inputs. The POV angle is used to determine the
     * direction
     *
     * @return the controller inputs as a Twist2d object, where the x and y components represent the
     *     translation speeds and the theta component represents the angular speed
     */
    private Twist2d calculateSpeedBasedonPOVInputs() {
        Optional<Rotation2d> pov = OI.getDriverJoystickPOV();
        if (pov.isEmpty()) {
            return new Twist2d();
        }

        // Calculate the x and y magnitudes based on the POV angle
        double x_magnitude = pov.get().getCos();
        double y_magnitude = -pov.get().getSin(); // Negate because WPILib Y+ is left
        double angular_magnitude =
                -MathUtil.applyDeadband(
                        OI.getDriverJoystickRightX(), CONSTANTS.CONTROLLER_DEADBAND);

        Twist2d twist =
                new Twist2d(
                        x_magnitude * CONSTANTS.MAX_CRAWL_RATE,
                        y_magnitude * CONSTANTS.MAX_CRAWL_RATE,
                        angular_magnitude * CONSTANTS.MAX_ANGULAR_RATE);
        DogLog.log(getSubsystemKey() + "RequestedTwist", twist);
        return twist;
    }

    /**
     * Sets the operator forward direction based on the operator's perspective.
     *
     * @param reference the operator's perspective reference
     */
    public void setOperatorForwardDirection(OperatorPerspective reference) {
        operator_forward_direction_ = reference.heading;
        CONSTANTS.FLIP_TRAJECTORY_ON_RED =
                (reference.heading == OperatorPerspective.RED_ALLIANCE.heading);
        DogLog.log(
                getSubsystemKey() + "OperatorForwardDirection",
                operator_forward_direction_.getDegrees(),
                Degrees);
    }

    /**
     * Sets the teleop velocity scalar to scale the robot's speed.
     *
     * @param scalar the scalar value to set, clamped between 0 and 1
     */
    public void setTeleOpVelocityScalar(double scalar) {
        tele_op_velocity_scalar_ = MathUtil.clamp(scalar, 0, 1);
    }

    // ------------------------------------------------
    // Status Check Methods
    // ------------------------------------------------

    /**
     * Checks if the robot is at the tractor beam setpoint.
     *
     * @return true if the robot is at the tractor beam setpoint, false otherwise
     */
    public boolean isAtTractorBeamSetpoint() {
        double distance =
                desired_tractor_beam_pose_
                        .getTranslation()
                        .minus(LocalizationSubsystem.getInstance().getFieldPose().getTranslation())
                        .getNorm();
        return MathUtil.isNear(0.0, distance, CONSTANTS.TRACTOR_BEAM_TRANSLATION_ERROR_MARGIN);
    }

    /**
     * Checks if the robot is at the desired rotation.
     *
     * @return true if the robot is at the desired rotation, false otherwise
     */
    public boolean isAtDesiredRotation() {
        return isAtDesiredRotation(Units.degreesToRadians(10.0));
    }

    /**
     * Checks if the robot is at the desired rotation within a specified tolerance.
     *
     * @param tolerance the tolerance in radians
     * @return true if the robot is at the desired rotation within the tolerance, false otherwise
     */
    public boolean isAtDesiredRotation(double tolerance) {
        return Math.abs(field_centric_rotation_lock_request_.HeadingController.getPositionError())
                < tolerance;
    }

    /**
     * Checks if the choreo trajectory time has elapsed.
     *
     * @return true if the choreo trajectory time has elapsed, false otherwise
     */
    public boolean hasChoreoTimeElapsed() {
        if (system_state_ != SwerveStates.CHOREO_PATH
                && system_state_ != SwerveStates.CHOREO_PATH_ROTATION_LOCK) {
            return false;
        }
        return choreo_timer_.get() >= desired_choreo_traj_.getTotalTime();
    }

    /**
     * Checks if the choreo trajectory time has elapsed for a specified total time.
     *
     * @param total_time the total time to check against
     * @return true if the choreo trajectory time has elapsed, false otherwise
     */
    public boolean hasChoreoTimeElapsed(double total_time) {
        if (system_state_ != SwerveStates.CHOREO_PATH
                && system_state_ != SwerveStates.CHOREO_PATH_ROTATION_LOCK) {
            return false;
        }
        return choreo_timer_.get() >= total_time;
    }

    /**
     * Checks if the robot is at the choreo setpoint.
     *
     * @return true if the robot is at the choreo setpoint, false otherwise
     */
    public boolean isAtChoreoSetpoint() {
        if (system_state_ != SwerveStates.CHOREO_PATH
                && system_state_ != SwerveStates.CHOREO_PATH_ROTATION_LOCK) {
            return false;
        }
        return MathUtil.isNear(
                        desired_choreo_traj_
                                .getFinalPose(CONSTANTS.FLIP_TRAJECTORY_ON_RED)
                                .get()
                                .getX(),
                        LocalizationSubsystem.getInstance().getFieldPose().getX(),
                        CONSTANTS.CHOREO_TRANSLATION_ERROR_MARGIN)
                && MathUtil.isNear(
                        desired_choreo_traj_
                                .getFinalPose(CONSTANTS.FLIP_TRAJECTORY_ON_RED)
                                .get()
                                .getY(),
                        LocalizationSubsystem.getInstance().getFieldPose().getY(),
                        CONSTANTS.CHOREO_TRANSLATION_ERROR_MARGIN);
    }

    /**
     * Checks if the robot is at the end of the choreo trajectory or at the tractor beam setpoint.
     *
     * @return true if the robot is at the end of the choreo trajectory or at the tractor beam
     *     setpoint, false otherwise
     */
    public boolean isAtEndOfChoreoTrajectoryOrTractorBeam() {
        if (desired_choreo_traj_ != null) {
            return (MathUtil.isNear(
                                    desired_choreo_traj_
                                            .getFinalPose(CONSTANTS.FLIP_TRAJECTORY_ON_RED)
                                            .get()
                                            .getX(),
                                    LocalizationSubsystem.getInstance().getFieldPose().getX(),
                                    CONSTANTS.CHOREO_TRANSLATION_ERROR_MARGIN))
                            && MathUtil.isNear(
                                    desired_choreo_traj_
                                            .getFinalPose(CONSTANTS.FLIP_TRAJECTORY_ON_RED)
                                            .get()
                                            .getY(),
                                    LocalizationSubsystem.getInstance().getFieldPose().getY(),
                                    CONSTANTS.CHOREO_TRANSLATION_ERROR_MARGIN)
                    || isAtTractorBeamSetpoint();
        } else {
            return isAtTractorBeamSetpoint();
        }
    }

    /**
     * Gets the distance from the choreo endpoint.
     *
     * @return the distance from the choreo endpoint in meters
     */
    public double getDistanceFromChoreoEndpoint() {
        double distance =
                Math.abs(
                        desired_choreo_traj_
                                .getFinalPose(CONSTANTS.FLIP_TRAJECTORY_ON_RED)
                                .get()
                                .minus(LocalizationSubsystem.getInstance().getFieldPose())
                                .getTranslation()
                                .getNorm());
        return distance;
    }

    /**
     * Gets the distance from the tractor beam setpoint.
     *
     * @return the distance from the tractor beam setpoint in meters
     */
    public double getDistanceFromTractorBeamSetpoint() {
        double diff =
                desired_tractor_beam_pose_
                        .getTranslation()
                        .minus(LocalizationSubsystem.getInstance().getFieldPose().getTranslation())
                        .getNorm();
        return diff;
    }

    // ------------------------------------------------
    // Choreo Event Methods
    // ------------------------------------------------

    /**
     * Gets a trigger for the specified Choreo event. The trigger will be true from when the event
     * timestamp is passed until trajectory ends. Use .onTrue() for one-time actions or .whileTrue()
     * for continuous actions.
     *
     * @param event_name The name of the event defined in Choreo
     * @return A Trigger that reads from the event HashMap
     */
    public Trigger getChoreoEventTimeTrigger(String event_name) {
        return choreo_event_tracker_.getTimeTrigger(event_name);
    }

    /**
     * Gets a trigger for the specified Choreo event that checks if the robot is at the event's
     * pose. This is a pose-only check (does not require time condition). The trigger will be true
     * when the robot is within tolerances of the event's pose.
     *
     * @param event_name The name of the event defined in Choreo
     * @param translation_tol The translation distance tolerance in meters
     * @param rotation_tol The rotation tolerance in radians
     * @return A Trigger that checks pose only
     */
    public Trigger getChoreoEventPoseTrigger(
            String event_name, double translation_tol, double rotation_tol) {
        return choreo_event_tracker_.getPoseTrigger(event_name, translation_tol, rotation_tol);
    }

    /**
     * Checks if a specific Choreo event has been passed during trajectory following.
     *
     * @param event_name The name of the event to check
     * @return true if the event has been passed, false otherwise
     */
    public boolean hasChoreoEventBeenPassed(String event_name) {
        return choreo_event_tracker_.hasEventBeenPassed(event_name);
    }

    // ------------------------------------------------
    // Chassis Property Methods
    // ------------------------------------------------

    /** Stores the current encoder readings as offsets */
    public Command setModuleOffsets() {
        return Commands.runOnce(() -> swerve_mech_.setModuleOffsets())
                .withName("Set Module Offsets");
    }

    /** Zeros the gyro yaw to the operator forward direction */
    public Command zeroGyroYaw() {
        return Commands.runOnce(() -> swerve_mech_.setGyroYaw(operator_forward_direction_))
                .withName(
                        "Zero Gyro Yaw: " + operator_forward_direction_.getDegrees() + " Degrees");
    }

    /**
     * Sets the gyro yaw to a specific heading. This is useful for setting the gyro to a known
     * heading during autonomous or if the gyro drifts significantly during a match.
     *
     * @param yaw the desired heading to set the gyro to
     */
    public void setGyroYaw(Rotation2d yaw) {
        swerve_mech_.setGyroYaw(yaw);
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    public SwerveModuleState[] getModuleStates() {
        return swerve_mech_.getModuleStates();
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    public SwerveModulePosition[] getModulePositions() {
        return swerve_mech_.getModulePositions();
    }

    /** Returns the measured chassis speeds of the robot. */
    public ChassisSpeeds getChassisSpeeds() {
        return swerve_mech_.getChassisSpeeds();
    }

    /** Returns the raw gyro rotation */
    public Rotation2d getGyroYaw() {
        return swerve_mech_.getGyroYaw();
    }

    /** Returns the raw gyro yaw rate */
    public double getGyroYawRate() {
        return swerve_mech_.getGyroYawRate();
    }

    /**
     * Returns the swerve drive kinematics instance.
     *
     * @return SwerveDriveKinematics object representing the swerve drive
     */
    public SwerveDriveKinematics getKinematics() {
        return swerve_mech_.getKinematics();
    }
}
