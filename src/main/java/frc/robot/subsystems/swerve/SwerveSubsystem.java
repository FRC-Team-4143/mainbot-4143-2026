package frc.robot.subsystems.swerve;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import com.marswars.swerve_lib.ChassisRequest;
import com.marswars.swerve_lib.ChassisRequest.XPositiveReference;
import com.marswars.swerve_lib.SwerveMech;
import com.marswars.swerve_lib.module.Module.DriveControlMode;
import com.marswars.swerve_lib.module.Module.SteerControlMode;
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
import frc.robot.OI;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants.OperatorPerspective;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class SwerveSubsystem extends MwSubsystem<SwerveStates, SwerveConstants> {

    private static SwerveSubsystem instance_ = null;

    public static SwerveSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new SwerveSubsystem();
        }
        return instance_;
    }

    // State Specific Members
    Trajectory<SwerveSample> desired_choreo_traj_;
    private final Timer choreo_timer_ = new Timer();
    private Optional<SwerveSample> choreo_sample_to_apply_;
    private final PIDController choreo_x_controller_ =
            new PIDController(
                    CONSTANTS.CHOREO_X_CONTROLLER_KP,
                    CONSTANTS.CHOREO_X_CONTROLLER_KI,
                    CONSTANTS.CHOREO_X_CONTROLLER_KD);
    private final PIDController choreo_y_controller_ =
            new PIDController(
                    CONSTANTS.CHOREO_Y_CONTROLLER_KP,
                    CONSTANTS.CHOREO_Y_CONTROLLER_KI,
                    CONSTANTS.CHOREO_Y_CONTROLLER_KD);
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
    private double tele_op_velocity_scalar_ = 1.0;

    // IO Members
    private SwerveMech swerve_mech_;

    private Rotation2d operator_forward_direction_ = OperatorPerspective.BLUE_ALLIANCE.heading;

    private ChassisRequest.FieldCentric field_centric_request_;
    private ChassisRequest.RobotCentric robot_centric_request_;
    private ChassisRequest.FieldCentricFacingAngle rotation_lock_request_;
    private ChassisRequest.ApplyFieldSpeeds field_speeds_request_;

    /**
     * Creates a new Swerve subsystem with the specified gyro and module IOs.
     *
     * @param io The swerve I/O container to use
     */
    public SwerveSubsystem() {
        super(SwerveStates.IDLE, new SwerveConstants());

        swerve_mech_ =
                new SwerveMech(
                        getSubsystemKey(),
                        CONSTANTS.SWERVE_DRIVE_CONFIG,
                        CONSTANTS.SIM_SWERVE_DRIVE_CONFIG);

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
        rotation_lock_request_ =
                new ChassisRequest.FieldCentricFacingAngle()
                        .withDriveRequestType(DriveControlMode.OPEN_LOOP)
                        .withSteerRequestType(SteerControlMode.CLOSED_LOOP)
                        .withDeadband(CONSTANTS.MAX_TRANSLATION_RATE * 0.01)
                        .withRotationalDeadband(CONSTANTS.MAX_ANGULAR_RATE * 0.01)
                        .withXPositiveReference(XPositiveReference.OperatorPerspective);
        field_speeds_request_ =
                new ChassisRequest.ApplyFieldSpeeds()
                        .withDriveRequestType(DriveControlMode.CLOSED_LOOP)
                        .withSteerRequestType(SteerControlMode.CLOSED_LOOP);
    }

    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(swerve_mech_);
    }

    @Override
    public void reset() {}

    @Override
    public void updateLogic(double timestamp) {
        // Update the request to apply based on the system state
        switch (system_state_) {
            case FIELD_CENTRIC:
                swerve_mech_.setChassisRequest(
                        field_centric_request_.withTwist(calculateSpeedsBasedOnJoystickInputs()));
                break;
            case ROBOT_CENTRIC:
                swerve_mech_.setChassisRequest(
                        robot_centric_request_.withTwist(calculateSpeedsBasedOnJoystickInputs()));
                break;
            case TRACTOR_BEAM:
                tractorBeamState();
                break;
            case ROTATION_LOCK:
                swerve_mech_.setChassisRequest(
                        rotation_lock_request_
                                .withTargetHeading(desired_rotation_lock_rot_)
                                .withTwist(calculateSpeedsBasedOnJoystickInputs()));
                break;
            case CHOREO_PATH:
                choreoPathState();
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

    // ------------------------------------------------
    // Subsystem State Update Methods
    // ------------------------------------------------

    /**
     * Gets the current system state of the Swerve subsystem.
     *
     * @return the current system state
     */
    @Override
    protected void handleStateTransition(SwerveStates wanted_state) {
        system_state_ =
                switch (wanted_state) {
                    case FIELD_CENTRIC -> SwerveStates.FIELD_CENTRIC;
                    case ROBOT_CENTRIC -> SwerveStates.ROBOT_CENTRIC;
                    case CHOREO_PATH -> {
                        if (system_state_ != SwerveStates.CHOREO_PATH) {
                            choreo_timer_.restart();
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
                    case ROTATION_LOCK -> SwerveStates.ROTATION_LOCK;
                    case TRACTOR_BEAM -> SwerveStates.TRACTOR_BEAM;
                    default -> SwerveStates.IDLE;
                };
    }

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

        DogLog.log(getSubsystemKey() + "TractorBeam/XVelocityComponent", x_component);
        DogLog.log(getSubsystemKey() + "TractorBeam/YVelocityComponent", y_component);
        DogLog.log(getSubsystemKey() + "TractorBeam/VelocityOutput", velocity_output);
        DogLog.log(getSubsystemKey() + "TractorBeam/LinearDistance", linear_distance);
        DogLog.log(getSubsystemKey() + "TractorBeam/DirectionOfTravel", direction_of_travel);
        DogLog.log(getSubsystemKey() + "TractorBeam/DesiredPoint", desired_tractor_beam_pose_);

        if (Double.isNaN(max_ang_vel_for_tractor_beam_)) {
            swerve_mech_.setChassisRequest(
                    rotation_lock_request_
                            .withTwist(new Twist2d(x_component, y_component, 0.0))
                            .withTargetHeading(desired_tractor_beam_pose_.getRotation()));
        } else {
            swerve_mech_.setChassisRequest(
                    rotation_lock_request_
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
        if (choreo_sample_to_apply_.isPresent()) {
            SwerveSample sample = choreo_sample_to_apply_.get();
            DogLog.log(getSubsystemKey() + "Choreo/TimerValue", choreo_timer_.get());
            DogLog.log(getSubsystemKey() + "Choreo/TrajName", desired_choreo_traj_.name());
            DogLog.log(getSubsystemKey() + "Choreo/TotalTime", desired_choreo_traj_.getTotalTime());
            DogLog.log(getSubsystemKey() + "Choreo/sample/DesiredPose", sample.getPose());
            DogLog.log(
                    getSubsystemKey() + "Choreo/sample/DesiredChassisSpeeds",
                    sample.getChassisSpeeds());
            DogLog.log(getSubsystemKey() + "Choreo/sample/ModuleForcesX", sample.moduleForcesX());
            DogLog.log(getSubsystemKey() + "Choreo/sample/ModuleForcesY", sample.moduleForcesY());
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

    // ------------------------------------------------
    // Chassis Control Methods
    // ------------------------------------------------

    /**
     * Updates the internal target for the robot to follow and begins CHOREO_PATH
     *
     * @param trajectory the trajectory for the robot to follow
     */
    public void setDesiredChoreoTrajectory(Trajectory<SwerveSample> trajectory) {
        desired_choreo_traj_ = trajectory;
    }

    /**
     * Updates the internal target for the robot to reach and begins TRACTOR_BEAM
     *
     * @param target_pose target pose for the robot to reach
     */
    public void setDesiredTractorBeamPose(Pose2d pose) {
        desired_tractor_beam_pose_ = pose;
        max_lin_vel_for_tractor_beam_ = Double.NaN;
        max_ang_vel_for_tractor_beam_ = Double.NaN;
    }

    /**
     * Updates the internal target for the robot to reach and begins TRACTOR_BEAM
     *
     * @param pose target pose for the robot to reach
     * @param max_lin_vel maximum linear velocity for the robot to reach the target pose
     */
    public void setDesiredTractorBeamPoseWithMaxLinVel(Pose2d pose, double max_lin_vel) {
        max_lin_vel_for_tractor_beam_ = max_lin_vel;
        max_lin_vel_for_tractor_beam_ = Double.NaN;
        desired_tractor_beam_pose_ = pose;
    }

    /**
     * Updates the internal target for the robot to reach and begins TRACTOR_BEAM
     *
     * @param pose target pose for the robot to reach
     * @param max_ang_vel maximum angular velocity for the robot to reach the target pose
     */
    public void setDesiredTractorBeamPoseWithMaxAngVel(Pose2d pose, double max_ang_vel) {
        max_lin_vel_for_tractor_beam_ = max_ang_vel;
        max_lin_vel_for_tractor_beam_ = Double.NaN;
        desired_tractor_beam_pose_ = pose;
    }

    /**
     * Updates the internal target for the robot to reach and begins TRACTOR_BEAM
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
     * Updates the internal target for the robot to face and begins ROTATION_LOCK
     *
     * @param rotation
     */
    public void setDesiredRotationLock(Rotation2d rotation) {
        desired_rotation_lock_rot_ = rotation;
    }

    public Command toggleFieldCentric() {
        return Commands.runOnce(
                () -> {
                    if (system_state_ == SwerveStates.FIELD_CENTRIC) {
                        setWantedState(SwerveStates.ROBOT_CENTRIC);
                    } else {
                        setWantedState(SwerveStates.FIELD_CENTRIC);
                    }
                });
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
     * Sets the operator forward direction based on the operator's perspective.
     *
     * @param reference the operator's perspective reference
     */
    public void setOperatorForwardDirection(OperatorPerspective reference) {
        operator_forward_direction_ = reference.heading;
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
        return rotation_lock_request_.HeadingController.getPositionError() < tolerance;
    }

    /**
     * Checks if the robot is at the choreo setpoint.
     *
     * @return true if the robot is at the choreo setpoint, false otherwise
     */
    public boolean isAtChoreoSetpoint() {
        if (system_state_ != SwerveStates.CHOREO_PATH) {
            return false;
        }
        return MathUtil.isNear(
                        desired_choreo_traj_.getFinalPose(true).get().getX(),
                        LocalizationSubsystem.getInstance().getFieldPose().getX(),
                        CONSTANTS.CHOREO_TRANSLATION_ERROR_MARGIN)
                && MathUtil.isNear(
                        desired_choreo_traj_.getFinalPose(true).get().getY(),
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
                                    desired_choreo_traj_.getFinalPose(true).get().getX(),
                                    LocalizationSubsystem.getInstance().getFieldPose().getX(),
                                    CONSTANTS.CHOREO_TRANSLATION_ERROR_MARGIN))
                            && MathUtil.isNear(
                                    desired_choreo_traj_.getFinalPose(true).get().getY(),
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
    // Chassis Property Methods
    // ------------------------------------------------

    /** Stores the current encoder readings as offsets */
    public Command setModuleOffsets() {
        return Commands.runOnce(() -> swerve_mech_.setModuleOffsets());
    }

    /** Zeros the gyro to the operator forward direction */
    public Command zeroGyro() {
        return Commands.runOnce(() -> swerve_mech_.setGyro(operator_forward_direction_));
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
    public Rotation2d getGyroRotation() {
        return swerve_mech_.getRawGyroRotation();
    }

    /**
     * Returns the swerve drive simulation instance.
     *
     * @return SwerveDriveSimulation object representing the swerve drive simulation
     */
    public SwerveDriveSimulation getSwerveSimulation() {
        return swerve_mech_.getSwerveSimulation();
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
