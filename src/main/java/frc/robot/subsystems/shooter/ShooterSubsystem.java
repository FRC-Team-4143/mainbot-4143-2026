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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    // Adjustable shooting tolerances - initialized to strict defaults
    private double flywheel_vel_tol_ = FieldTargets.Shooter.FLYWHEEL_SPEED_TOLERANCE;
    private double hood_pos_tol_ = FieldTargets.Shooter.HOOD_POSITION_TOLERANCE;
    private double rot_pos_tol_ = FieldTargets.Shooter.ROTATION_ANGLE_TOLERANCE;

    // Hood feedforward gain - made tunable for easy adjustment
    private double hood_kv_ = CONSTANTS.HOOD_KV;

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

        SmartDashboard.putData(
                "Home Hood Position",
                Commands.runOnce(() -> hood_.setCurrentPosition(CONSTANTS.HOOD_HOME_POSITION))
                        .ignoringDisable(true));
    }

    // reset
    @Override
    public void reset() {
        system_state_ = ShooterStates.IDLE;
    }

    // getIos
    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(indexer_, flywheel_, hood_);
    }

    // handleStateTransition
    @Override
    public void handleStateTransition(ShooterStates wanted) {
        if (wanted == ShooterStates.SHOOT
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
        } else {
            system_state_ = wanted;
        }
    }

    @Override
    public void updateLogic(double timestamp) {
        // Get the current robot pose and velocity
        Pose2d robot_pose = LocalizationSubsystem.getInstance().getFieldPose();
        ChassisSpeeds robot_velocity = new ChassisSpeeds();
                //LocalizationSubsystem.getInstance().getChassisSpeedsFieldRelative();

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
            hood_angle_ =
                    MathUtil.clamp(
                            launch_params_.hood_angle,
                            CONSTANTS.HOOD_MIN_ANGLE,
                            CONSTANTS.HOOD_MAX_ANGLE);

            // Calculate hood feedforward once for use in all states
            hood_feedforward_ = launch_params_.hood_velocity * hood_kv_;

            // Store heading feedforward velocity (rad/s) for use in all states
            heading_feedforward_ = launch_params_.heading_velocity;
        }

        // Execute state-specific behavior
        switch (system_state_) {
            case TRACKING:
                flywheel_.setTargetVelocity(flywheel_omega_);
                indexer_.setTargetDutyCycle(0);
                hood_.setTargetPositionWithFF(hood_angle_, hood_feedforward_);
                break;
            case SHOOT_WAIT:
                flywheel_.setTargetVelocity(flywheel_omega_);
                indexer_.setTargetDutyCycle(0);
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
                indexer_.setTargetDutyCycle(0);
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
                flywheel_.setTargetVelocity(CONSTANTS.FLYWHEEL_MANUAL_HUB_VELOCITY);
                indexer_.setTargetDutyCycle(CONSTANTS.INDEXER_DUTY_CYCLE);
                hood_.setTargetPosition(CONSTANTS.HOOD_MANUAL_HUB_ANGLE);
                break;
            case MANUAL_PASS:
                // Manual pass mode - uses fixed setpoints for passing
                flywheel_.setTargetVelocity(CONSTANTS.FLYWHEEL_MANUAL_PASS_VELOCITY);
                indexer_.setTargetDutyCycle(CONSTANTS.INDEXER_DUTY_CYCLE);
                hood_.setTargetPosition(CONSTANTS.HOOD_MANUAL_PASS_ANGLE);
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
            return flywheel_.getCurrentVelocity() * CONSTANTS.FLYWHEEL_WHEEL_RADIUS_METERS * 0.5;
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
                MathUtil.isNear(flywheel_omega_, flywheel_.getCurrentVelocity(), flywheel_vel_tol_);
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
        boolean status =
                MathUtil.isNear(
                        heading_angle_,
                        LocalizationSubsystem.getInstance()
                                .getFieldPose()
                                .getRotation()
                                .getRadians(),
                        rot_pos_tol_);
        return status;
    }
}
