package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.marswars.geometry.AllianceFlipUtil;
import com.marswars.geometry.LaunchCalculator;
import com.marswars.mechanisms.FlywheelMech;
import com.marswars.mechanisms.RollerMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib2026.FieldTargets;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class ShooterSubsystem extends MwSubsystem<ShooterStates, ShooterConstants> {
    private static ShooterSubsystem instance_ = null;

    // Mechanisms
    private RollerMech indexer_;
    private FlywheelMech flywheel_;
    private RollerMech hood_;

    // Shooter parameters calculated from LaunchCalculator
    private double flywheel_omega_ = CONSTANTS.FLYWHEEL_MANUAL_HUB_VELOCITY;
    private double hood_angle_ = CONSTANTS.HOOD_MAX_ANGLE;
    private double heading_angle_ = 0.0;
    private double hood_feedforward_ = 0.0;
    private double heading_feedforward_ = 0.0;
    private Translation3d target_ = new Translation3d(0.0, 0.0, 0.0);
    private LaunchCalculator launch_calculator_ = CONSTANTS.HUB_LAUNCH_CALCULATOR;
    private LaunchCalculator.LaunchParameters launch_params_ = null;
    private final Debouncer shooter_ready_debouncer =
            new Debouncer(CONSTANTS.SHOOTER_READY_DEBOUNCE_TIME, DebounceType.kRising);

    private final LinearFilter flywheel_velocity_filter_ =
            LinearFilter.singlePoleIIR(CONSTANTS.FLYWHEEL_FILTER_TIME_CONSTANT, 0.02);

    // Adjustable shooting tolerances - initialized to strict defaults
    private double flywheel_vel_tol_ = FieldTargets.Shooter.FLYWHEEL_SPEED_TOLERANCE;
    private double hood_pos_tol_ = FieldTargets.Shooter.HOOD_POSITION_TOLERANCE;
    private double rot_pos_tol_ = FieldTargets.Shooter.ROTATION_ANGLE_TOLERANCE;
    private double hood_adj_ = 0.0;
    private double flywheel_adj_ = 0.0;
    private double filtered_flywheel_velocity_ = 0.0;

    // Hood feedforward gain - made tunable for easy adjustment
    private double hood_kv_ = CONSTANTS.HOOD_KV;

    private double manual_indexer_velocity_ = CONSTANTS.INDEXER_VELOCITY;

    // Ball detection: track recent flywheel velocity samples to detect the brief
    // velocity dip when a ball is fired. We expose a simple "likely empty"
    // predicate which returns true when we've not observed a ball for a
    // configurable timeout while in the SHOOT state.
    private double last_ball_seen_time_ = 0.0;
    private double last_filtered_velocity_sample_ = 0.0;
    private double no_ball_timeout_seconds_ = 0.75; // tunable
    private double ball_velocity_drop_threshold_ = 40.0; // tunable (rad/s)
    private ShooterStates prev_state_ = system_state_;

    // getInstance
    public static ShooterSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new ShooterSubsystem();
        }
        return instance_;
    }

    // Constructor
    public ShooterSubsystem() {
        super(ShooterStates.IDLE, new ShooterConstants());
        indexer_ =
                new RollerMech(
                        getSubsystemKey(),
                        "Indexer",
                        List.of(
                                CONSTANTS.INDEXER_LEADER_MOTOR_CONFIG,
                                CONSTANTS.INDEXER_FOLLOWER_MOTOR_CONFIG),
                        CONSTANTS.INDEXER_GEAR_RATIO);
        flywheel_ =
                new FlywheelMech(
                        getSubsystemKey(),
                        "Flywheel",
                        List.of(
                                CONSTANTS.SHOOTER_LEADER_MOTOR_CONFIG,
                                CONSTANTS.SHOOTER_FOLLOWER_MOTOR_1_CONFIG,
                                CONSTANTS.SHOOTER_FOLLOWER_MOTOR_2_CONFIG,
                                CONSTANTS.SHOOTER_FOLLOWER_MOTOR_3_CONFIG),
                        CONSTANTS.FLYWHEEL_GEAR_RATIO,
                        CONSTANTS.FLYWHEEL_INERTIA,
                        CONSTANTS.FLYWHEEL_WHEEL_RADIUS_METERS);
        hood_ =
                new RollerMech(
                        getSubsystemKey(),
                        "Hood",
                        List.of(CONSTANTS.HOOD_MOTOR_CONFIGS),
                        CONSTANTS.HOOD_GEAR_RATIO);
        hood_.setCurrentPosition(CONSTANTS.HOOD_HOME_POSITION);

        // Setup tunable for hood feedforward gain
        DogLog.tunable(
                getSubsystemKey() + "Hood/kV", CONSTANTS.HOOD_KV, (newKv) -> hood_kv_ = newKv);

        DogLog.tunable(
                getSubsystemKey() + "/Indexer/TargetVelocity",
                manual_indexer_velocity_,
                (v) -> manual_indexer_velocity_ = v);

        // Tunables for ball-detection (flywheel velocity drop detection)
        DogLog.tunable(
                getSubsystemKey() + "Shooter/NoBallTimeout",
                no_ball_timeout_seconds_,
                (v) -> no_ball_timeout_seconds_ = v);
        DogLog.tunable(
                getSubsystemKey() + "Shooter/BallVelocityDropThreshold",
                ball_velocity_drop_threshold_,
                (v) -> ball_velocity_drop_threshold_ = v);

        SmartDashboard.putData(
                "Home Hood Position",
                Commands.runOnce(() -> hood_.setCurrentPosition(CONSTANTS.HOOD_HOME_POSITION))
                        .ignoringDisable(true));
        SmartDashboard.putData(
                "Auto Home Hood",
                Commands.runOnce(() -> setWantedState(ShooterStates.HOOD_HOMING)));
    }

    // reset
    @Override
    public void reset() {
        system_state_ = ShooterStates.IDLE;
        prev_state_ = system_state_;
        last_ball_seen_time_ = 0.0;
        last_filtered_velocity_sample_ = 0.0;
    }

    // getIos
    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(indexer_, flywheel_, hood_);
    }

    // handleStateTransition
    @Override
    public void handleStateTransition(ShooterStates wanted) {
        if (hood_.getLeaderCurrent() > CONSTANTS.HOOD_HOMMING_CURRENT_THRESHOLD
                && system_state_ == ShooterStates.HOOD_HOMING) {
            hood_.setCurrentPosition(CONSTANTS.HOOD_HOME_POSITION);
            setWantedState(ShooterStates.IDLE);
            system_state_ = ShooterStates.IDLE;
        } else if (wanted == ShooterStates.SHOOT
                && system_state_ != ShooterStates.SHOOT_WAIT
                && system_state_ != ShooterStates.SHOOT) {
            system_state_ = ShooterStates.SHOOT_WAIT;
        } else if (system_state_ == ShooterStates.SHOOT_WAIT
                && isShooterReady()
                && wanted == ShooterStates.SHOOT) {
            system_state_ = ShooterStates.SHOOT;
        } else if (system_state_ == ShooterStates.SHOOT_WAIT
                && !isShooterReady()
                && wanted == ShooterStates.SHOOT) {
            // Nap time : Blocks deafult transition from occuring
        } else if (system_state_ == ShooterStates.SHOOT && !isShooterReady()) {
            system_state_ = ShooterStates.SHOOT_WAIT;
        } else {
            system_state_ = wanted;
        }
    }

    @Override
    public void updateLogic(double timestamp) {
        // Get the current robot pose and velocity
        Pose2d robot_pose = LocalizationSubsystem.getInstance().getFieldPose();

        // Use DESIRED/SETPOINT velocity (what the robot is commanded to do) rather than
        // measured velocity for better motion compensation predictability.
        // This matches 6328's approach and is more predictive than measured velocity.
        // Use field-relative version to ensure motion compensation works correctly on both
        // blue and red alliances (LaunchCalculator needs global coordinate system velocities)
        ChassisSpeeds robot_velocity = SwerveSubsystem.getInstance().getDesiredChassisSpeeds();

        // Calculate launch parameters using the LaunchCalculator
        launch_params_ =
                launch_calculator_.calculateLaunchParameters(
                        robot_pose, robot_velocity, target_.toTranslation2d());

        // Update shooter parameters based on the launch calculator
        // Skip the shooter parameters update if invalid or in manual modes to allow for
        // testing with manual setpoints
        if (launch_params_.is_valid
                && system_state_ != ShooterStates.TUNING
                && system_state_ != ShooterStates.MANUAL_HUB
                && system_state_ != ShooterStates.MANUAL_PASS) {
            flywheel_omega_ = launch_params_.flywheel_speed;

            // Store heading angle for robot rotation
            heading_angle_ = launch_params_.heading_angle.getRadians();

            // Clamp hood angle to mechanical limits
            hood_angle_ = launch_params_.hood_angle;

            // Calculate hood feedforward once for use in all states
            hood_feedforward_ = launch_params_.hood_velocity * hood_kv_;

            // Store heading feedforward velocity (rad/s) for use in all states
            heading_feedforward_ = launch_params_.heading_velocity;
        }

        // Apply flywheel velocity filtering
        filtered_flywheel_velocity_ =
                flywheel_velocity_filter_.calculate(flywheel_.getCurrentVelocity());

        // Detect ball firing by observing a brief velocity drop in the filtered flywheel
        // signal while actively shooting. Update the "last ball seen" timestamp when we
        // observe a downward spike larger than the configured threshold.
        double vel_drop = last_filtered_velocity_sample_ - filtered_flywheel_velocity_;
        if ((system_state_ == ShooterStates.SHOOT || system_state_ == ShooterStates.SHOOT_WAIT)
                && vel_drop > ball_velocity_drop_threshold_) {
            last_ball_seen_time_ = timestamp;
        }

        // If we just entered the SHOOT state, assume we have at least one ball to start
        // (avoid immediate early-stop). Reset the last-seen time to now on entry.
        if (prev_state_ != system_state_ && system_state_ == ShooterStates.SHOOT) {
            last_ball_seen_time_ = timestamp;
        }

        flywheel_omega_ += flywheel_adj_;
        hood_angle_ += hood_adj_;
        hood_angle_ =
                MathUtil.clamp(hood_angle_, CONSTANTS.HOOD_MIN_ANGLE, CONSTANTS.HOOD_MAX_ANGLE);

        // Execute state-specific behavior
        switch (system_state_) {
            case TRACKING:
                flywheel_.setTargetVelocity(flywheel_omega_);
                indexer_.setTargetDutyCycle(CONSTANTS.IDLE_INDEXER_DUTY_CYCLE);
                hood_.setTargetPositionWithFF(hood_angle_, hood_feedforward_);
                break;
            case SHOOT_WAIT:
                flywheel_.setTargetVelocity(flywheel_omega_);
                indexer_.setTargetDutyCycle(CONSTANTS.IDLE_INDEXER_DUTY_CYCLE);
                hood_.setTargetPositionWithFF(hood_angle_, hood_feedforward_);
                SwerveSubsystem.getInstance()
                        .setDesiredRotationLockCORWithFF(
                                Rotation2d.fromRadians(heading_angle_),
                                new Translation2d(
                                        CONSTANTS.SHOOTER_CENTER.getX(),
                                        CONSTANTS.SHOOTER_CENTER.getY()),
                                heading_feedforward_);
                break;
            case AIMING:
                flywheel_.setTargetVelocity(flywheel_omega_);
                indexer_.setTargetDutyCycle(CONSTANTS.IDLE_INDEXER_DUTY_CYCLE);
                hood_.setTargetPositionWithFF(hood_angle_, hood_feedforward_);
                SwerveSubsystem.getInstance()
                        .setDesiredRotationLockCORWithFF(
                                Rotation2d.fromRadians(heading_angle_),
                                new Translation2d(
                                        CONSTANTS.SHOOTER_CENTER.getX(),
                                        CONSTANTS.SHOOTER_CENTER.getY()),
                                heading_feedforward_);
                break;
            case DUMP:
                flywheel_.setTargetVelocity(flywheel_omega_);
                indexer_.setTargetDutyCycle(-CONSTANTS.INDEXER_DUTY_CYCLE);
                hood_.setTargetPosition(CONSTANTS.HOOD_MAX_ANGLE);
                break;
            case SHOOT:
                flywheel_.setTargetVelocity(flywheel_omega_);
                indexer_.setTargetDutyCycle(CONSTANTS.INDEXER_DUTY_CYCLE);
                hood_.setTargetPositionWithFF(hood_angle_, hood_feedforward_);
                SwerveSubsystem.getInstance()
                        .setDesiredRotationLockCORWithFF(
                                Rotation2d.fromRadians(heading_angle_),
                                new Translation2d(
                                        CONSTANTS.SHOOTER_CENTER.getX(),
                                        CONSTANTS.SHOOTER_CENTER.getY()),
                                heading_feedforward_);
                break;
            case MANUAL_HUB:
                // Manual hub shooting mode - uses fixed setpoints for hub shots
                flywheel_.setTargetVelocity(CONSTANTS.FLYWHEEL_MANUAL_HUB_VELOCITY + flywheel_adj_);
                indexer_.setTargetDutyCycle(CONSTANTS.INDEXER_DUTY_CYCLE);
                hood_.setTargetPosition(CONSTANTS.HOOD_MANUAL_HUB_ANGLE + hood_adj_);
                break;
            case MANUAL_PASS:
                // Manual pass mode - uses fixed setpoints for passing
                flywheel_.setTargetVelocity(
                        CONSTANTS.FLYWHEEL_MANUAL_PASS_VELOCITY + flywheel_adj_);
                indexer_.setTargetDutyCycle(CONSTANTS.INDEXER_DUTY_CYCLE);
                hood_.setTargetPosition(CONSTANTS.HOOD_MANUAL_PASS_ANGLE + hood_adj_);
                break;
            case TUNING:
                // code does NOTHING to allow for testing
                break;
            default:
            case IDLE:
                flywheel_.setTargetVelocityMotionProfile(CONSTANTS.SHOOTER_IDLE_SPEED);
                indexer_.setTargetDutyCycle(0);
                hood_.setTargetPosition(CONSTANTS.HOOD_IDLE_POSITION);
                break;
            case SPIN_DOWN:
                flywheel_.setTargetVelocityMotionProfile(0);
                indexer_.setTargetDutyCycle(0);
                hood_.setTargetPosition(CONSTANTS.HOOD_IDLE_POSITION);
                break;
            case HOOD_HOMING:
                hood_.setTargetDutyCycle(CONSTANTS.HOOD_HOMING_DUTY_CYCLE);
                break;
        }

        // LaunchCalculator Logging
        DogLog.log(getSubsystemKey() + "LaunchCalculator/Valid", launch_params_.is_valid);
        DogLog.log(
                getSubsystemKey() + "LaunchCalculator/Hood/Angle",
                launch_params_.hood_angle,
                Radians);
        DogLog.log(
                getSubsystemKey() + "LaunchCalculator/Hood/Velocity",
                launch_params_.hood_velocity,
                RadiansPerSecond);
        DogLog.log(
                getSubsystemKey() + "LaunchCalculator/Heading/Angle",
                launch_params_.heading_angle.getRadians(),
                Radians);
        DogLog.log(
                getSubsystemKey() + "LaunchCalculator/Heading/Velocity",
                launch_params_.heading_velocity,
                RadiansPerSecond);
        DogLog.log(
                getSubsystemKey() + "LaunchCalculator/FlywheelSpeed",
                launch_params_.flywheel_speed,
                RadiansPerSecond);
        DogLog.log(getSubsystemKey() + "LaunchCalculator/Target", target_);
        DogLog.log(
                getSubsystemKey() + "LaunchCalculator/Distance/Lookahead",
                launch_params_.distance,
                Meters);
        DogLog.log(
                getSubsystemKey() + "LaunchCalculator/Distance/Raw",
                launch_params_.distance_no_lookahead,
                Meters);
        DogLog.log(
                getSubsystemKey() + "LaunchCalculator/TimeOfFlight",
                launch_params_.time_of_flight,
                Seconds);

        DogLog.log(
                getSubsystemKey() + "Flywheel/FilteredVelocity",
                filtered_flywheel_velocity_,
                RadiansPerSecond);

        // Setpoint Logging
        DogLog.log(getSubsystemKey() + "Setpoint/FlywheelOmega", flywheel_omega_, RadiansPerSecond);
        DogLog.log(getSubsystemKey() + "Setpoint/HoodAngle", hood_angle_, Radians);
        DogLog.log(getSubsystemKey() + "Setpoint/HeadingAngle", heading_angle_, Radians);

        // Hood Feedforward Logging
        DogLog.log(
                getSubsystemKey() + "LaunchCalculator/Hood/Feedforward", hood_feedforward_, Volts);

        // Heading Feedforward Logging
        DogLog.log(
                getSubsystemKey() + "LaunchCalculator/Heading/Feedforward",
                heading_feedforward_,
                RadiansPerSecond);

        // System at Desired Setpoints
        DogLog.log(getSubsystemKey() + "ShooterIsReady/Hood", isHoodAtPosition());
        DogLog.log(getSubsystemKey() + "ShooterIsReady/Flywheel", isFlywheelAtSpeed());
        DogLog.log(getSubsystemKey() + "ShooterIsReady/Rotation", isRotationAtPosition());
        DogLog.log(getSubsystemKey() + "ShooterIsReady/Full", isShooterReady());

        // Active Tolerances
        DogLog.log(
                getSubsystemKey() + "Tolerance/FlywheelVelocity",
                flywheel_vel_tol_,
                RadiansPerSecond);
        DogLog.log(getSubsystemKey() + "Tolerance/HoodAngle", hood_pos_tol_, Radians);
        DogLog.log(getSubsystemKey() + "Tolerance/HeadingAngle", rot_pos_tol_, Radians);

        // Ball detection telemetry
        DogLog.log(getSubsystemKey() + "Shooter/LastBallSeenTime", last_ball_seen_time_, Seconds);
        DogLog.log(
                getSubsystemKey() + "Shooter/VelDrop",
                last_filtered_velocity_sample_ - filtered_flywheel_velocity_,
                RadiansPerSecond);
        DogLog.log(getSubsystemKey() + "Shooter/IsLikelyEmpty", isLikelyEmpty() ? 1.0 : 0.0);

        // Update prev state and last sample for next iteration
        prev_state_ = system_state_;
        last_filtered_velocity_sample_ = filtered_flywheel_velocity_;
    }

    // =============================================================================
    // PUBLIC HELPER METHODS
    // =============================================================================

    /**
     * Check if the shooter is ready to shoot
     *
     * @return true all the active mechanisms are at within tolerance of their target
     *     positions/velocities
     */
    public boolean isShooterReady() {
        // If there is no valid solution, the shooter cannot be ready
        if (launch_params_ == null || !launch_params_.is_valid) {
            return false;
        }
        return shooter_ready_debouncer.calculate(
                isFlywheelAtSpeed() && isHoodAtPosition() && isRotationAtPosition());
    }

    /**
     * Get the current launch angle from the solution
     *
     * @return the launch angle in radians, or the max hood angle if no valid solution
     */
    public double getLaunchAngle() {
        return hood_.getCurrentPosition();
    }

    /**
     * Get the current launch velocity from the solution
     *
     * @return the launch velocity in meters per second, or 0.0 if no valid solution
     */
    public double getLaunchVelocity() {
        // angular speed * radius * eff_factor
        return filtered_flywheel_velocity_ * CONSTANTS.FLYWHEEL_WHEEL_RADIUS_METERS * 0.5;
    }

    /**
     * Heuristic: return true if we are in the SHOOT state and we have not observed a ball (a
     * flywheel velocity dip) for longer than the configured timeout.
     *
     * @return true if shooting and likely empty
     */
    public boolean isLikelyEmpty() {
        if (system_state_ != ShooterStates.SHOOT && system_state_ != ShooterStates.SHOOT_WAIT) {
            return false;
        }
        double now = Timer.getFPGATimestamp();
        return (now - last_ball_seen_time_) > no_ball_timeout_seconds_;
    }

    /**
     * Applies an external load torque to the flywheel (for simulation purposes)
     *
     * @param load_torque the load torque to apply in N*m
     */
    public void applyLoadFromBall(double load_torque) {
        flywheel_.applyLoadTorque(load_torque);
    }

    /**
     * Sets the target for the shooter to calculate a solution for. The target is a 3D translation
     * where x and y are the horizontal coordinates of the target relative to the field. The z
     * coordinate is ignored as the LaunchCalculator uses 2D targets.
     *
     * @param target the target translation in meters, where x and y are the horizontal coordinates
     *     of the target relative to the field
     */
    public void setTarget(Translation3d target) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            target = AllianceFlipUtil.apply(target);
        }
        target_ = target;

        // Automatically switch LaunchCalculator based on target selection to use appropriate set of
        // empirically determined parameters for hub shots vs passes
        if (MathUtil.isNear(FieldTargets.Shooter.HUB.getZ(), target.getZ(), 1E-6)) {
            launch_calculator_ = CONSTANTS.HUB_LAUNCH_CALCULATOR;
        } else {
            launch_calculator_ = CONSTANTS.PASS_LAUNCH_CALCULATOR;
        }
    }

    /**
     * Sets the shooting tolerances for determining when the shooter is ready. This allows external
     * control of tolerance levels without the subsystem needing to know about game states or other
     * subsystem behavior. Tolerances are used to determine when the shooter mechanisms are close
     * enough to their target values to be considered "ready to shoot".
     *
     * @param flywheel_vel_tol the tolerance for the flywheel speed in rad/s
     * @param hood_pos_tol the tolerance for the hood position in radians
     * @param rot_pos_tol the tolerance for the robot rotation angle in radians
     */
    public void setShootingTolerances(
            double flywheel_vel_tol, double hood_pos_tol, double rot_pos_tol) {
        flywheel_vel_tol_ = flywheel_vel_tol;
        hood_pos_tol_ = hood_pos_tol;
        rot_pos_tol_ = rot_pos_tol;
    }

    /**
     * adjustes flywheel speed in radians per second
     *
     * @param adj amount speed is changed by
     */
    public void adjustFlywheel(double adj) {
        flywheel_adj_ += adj;
        DogLog.log(getSubsystemKey() + "Setpoint/Flywheel Adjust", flywheel_adj_);
    }

    /**
     * adjusts hood angle in radians
     *
     * @param adj amount of offset
     */
    public void adjustHood(double adj) {
        hood_adj_ += adj;
        DogLog.log(getSubsystemKey() + "Setpoint/Hood Adjust", hood_adj_);
    }

    // =============================================================================
    // PRIVATE HELPER METHODS
    // =============================================================================

    /**
     * Check if the flywheel is at the target speed within a certain tolerance
     *
     * @return true if the flywheel speed is within tolerance, false otherwise
     */
    private boolean isFlywheelAtSpeed() {
        boolean status =
                MathUtil.isNear(flywheel_omega_, filtered_flywheel_velocity_, flywheel_vel_tol_);
        return status;
    }

    /**
     * Check if the hood is at the target position within a certain tolerance
     *
     * @return true if the hood is within tolerance, false otherwise
     */
    private boolean isHoodAtPosition() {
        boolean status = MathUtil.isNear(hood_angle_, hood_.getCurrentPosition(), hood_pos_tol_);
        return status;
    }

    /**
     * Check if the robot rotation is at the target heading within a certain tolerance
     *
     * @return true if the robot rotation is within tolerance, false otherwise
     */
    private boolean isRotationAtPosition() {
        // Calculate the angular error with proper wraparound handling
        // angleModulus normalizes to [-pi, pi], ensuring 0 and 2π are treated as equivalent
        double angle_error =
                MathUtil.angleModulus(
                        heading_angle_
                                - LocalizationSubsystem.getInstance()
                                        .getFieldPose()
                                        .getRotation()
                                        .getRadians());
        boolean status = Math.abs(angle_error) <= rot_pos_tol_;
        return status;
    }
}
