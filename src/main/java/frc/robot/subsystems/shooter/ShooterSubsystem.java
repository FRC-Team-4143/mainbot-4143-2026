package frc.robot.subsystems.shooter;

import com.marswars.geometry.AllianceFlipUtil;
import com.marswars.geometry.LaunchCalculator;
import com.marswars.mechanisms.FlywheelMech;
import com.marswars.mechanisms.RollerMech;
import com.marswars.mechanisms.TurretMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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
    private TurretMech turret_;

    // Shooter parameters calculated from LaunchCalculator
    private double flywheel_omega_ = CONSTANTS.FLYWHEEL_MANUAL_VELOCITY;
    private double turret_heading_ = 0.0;
    private double hood_angle_ = CONSTANTS.HOOD_MAX_ANGLE;
    private Translation2d target_ = new Translation2d(0.0, 0.0);
    private LaunchCalculator.LaunchParameters launch_params_ = null;

    // Adjustable shooting tolerances - initialized to strict defaults
    private double flywheel_speed_tolerance_ = FieldTargets.Shooter.FLYWHEEL_SPEED_TOLERANCE;
    private double hood_position_tolerance_ = FieldTargets.Shooter.HOOD_POSITION_TOLERANCE;
    private double turret_angle_tolerance_ = FieldTargets.Shooter.TURRET_ANGLE_TOLERANCE;
    private double rotation_angle_tolerance_ = FieldTargets.Shooter.ROTATION_ANGLE_TOLERANCE;

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

        // Current 4143 robot does not have a turret
        if (CONSTANTS.TURRET_ENABLED) {
            turret_ =
                    new TurretMech(
                            getSubsystemKey(),
                            List.of(CONSTANTS.TURRET_MOTOR_CONFIGS),
                            CONSTANTS.TURRET_GEAR_RATIO,
                            CONSTANTS.TURRET_MOI);
        }

        SmartDashboard.putData(
                "Home Hood",
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
        // Current 4143 robot does not have a turret
        if (CONSTANTS.TURRET_ENABLED) {
            return Arrays.asList(indexer_, flywheel_, hood_, turret_);
        } else {
            return Arrays.asList(indexer_, flywheel_, hood_);
        }
    }

    // handleStateTransition
    @Override
    public void handleStateTransition(ShooterStates wanted) {
        Pose2d robotPose = LocalizationSubsystem.getInstance().getFieldPose();
        if (wanted == ShooterStates.SHOOT
                && system_state_ != ShooterStates.AIMING
                && system_state_ != ShooterStates.SHOOT) {
            system_state_ = ShooterStates.AIMING;
        } else if (system_state_ == ShooterStates.AIMING && isShooterReady()) {
            system_state_ = ShooterStates.SHOOT;
        } else if (system_state_ == ShooterStates.AIMING && !isShooterReady()) {
            // Nap time : Blocks default transition from occurring
        } else {
            system_state_ = wanted;
        }

        // Update launch heading for turret (wrapping logic is not needed for drivetrain rotation)
        if (CONSTANTS.TURRET_ENABLED && (launch_params_ != null && launch_params_.is_valid)) {
            turret_heading_ =
                    launch_params_.heading_angle.getRadians()
                            - robotPose.getRotation().getRadians();
            handleTurretWrap();
        } else if (launch_params_ != null && launch_params_.is_valid) {
            turret_heading_ = launch_params_.heading_angle.getRadians();
        }
    }

    @Override
    public void updateLogic(double timestamp) {
        // Get the current robot pose and velocity
        Pose2d robot_pose = LocalizationSubsystem.getInstance().getFieldPose();
        ChassisSpeeds robot_velocity =
                LocalizationSubsystem.getInstance().getChassisSpeedsFieldRelative();

        // Calculate launch parameters using the LaunchCalculator
        launch_params_ =
                CONSTANTS.LAUNCH_CALCULATOR.calculateLaunchParameters(
                        robot_pose, robot_velocity, target_);

        // Update shooter parameters based on the launch calculator
        // Skip the shooter parameters update if invalid or in manual mode to allow for
        // testing with manual setpoints
        if (launch_params_.is_valid && system_state_ != ShooterStates.MANUAL) {
            flywheel_omega_ = launch_params_.flywheel_speed;

            // Calculate launch heading and handle turret wrapping
            if (CONSTANTS.TURRET_ENABLED) {
                turret_heading_ =
                        launch_params_.heading_angle.minus(robot_pose.getRotation()).getRadians();
                handleTurretWrap();
            } else {
                turret_heading_ = launch_params_.heading_angle.getRadians();
            }

            // Clamp hood angle to mechanical limits
            hood_angle_ =
                    MathUtil.clamp(
                            launch_params_.hood_angle,
                            CONSTANTS.HOOD_MIN_ANGLE,
                            CONSTANTS.HOOD_MAX_ANGLE);
        }

        // Execute state-specific behavior
        switch (system_state_) {
            case TRACKING:
                flywheel_.setTargetVelocity(flywheel_omega_);
                indexer_.setTargetDutyCycle(0);
                hood_.setTargetPosition(hood_angle_);
                if (CONSTANTS.TURRET_ENABLED) {
                    turret_.setTargetPosition(turret_heading_);
                }
                break;
            case AIMING:
                flywheel_.setTargetVelocity(flywheel_omega_);
                indexer_.setTargetDutyCycle(0);
                hood_.setTargetPosition(hood_angle_);
                if (CONSTANTS.TURRET_ENABLED) {
                    turret_.setTargetPosition(turret_heading_);
                } else {
                    SwerveSubsystem.getInstance()
                            .setDesiredRotationLockCOR(
                                    Rotation2d.fromRadians(turret_heading_),
                                    new Translation2d(
                                            CONSTANTS.SHOOTER_CENTER.getX(),
                                            CONSTANTS.SHOOTER_CENTER.getY()));
                }
                break;
            case DUMP:
                flywheel_.setTargetVelocity(flywheel_omega_);
                indexer_.setTargetDutyCycle(-CONSTANTS.INDEXER_DUTY_CYCLE);
                hood_.setTargetPosition(CONSTANTS.HOOD_MAX_ANGLE);
                if (CONSTANTS.TURRET_ENABLED) turret_.setTargetDutyCycle(0);
                break;
            case SHOOT:
                flywheel_.setTargetVelocity(flywheel_omega_);
                indexer_.setTargetDutyCycle(CONSTANTS.INDEXER_DUTY_CYCLE);
                hood_.setTargetPosition(hood_angle_);
                if (CONSTANTS.TURRET_ENABLED) {
                    turret_.setTargetPosition(turret_heading_);
                } else {
                    SwerveSubsystem.getInstance()
                            .setDesiredRotationLockCOR(
                                    Rotation2d.fromRadians(turret_heading_),
                                    new Translation2d(
                                            CONSTANTS.SHOOTER_CENTER.getX(),
                                            CONSTANTS.SHOOTER_CENTER.getY()));
                }
                break;
            case MANUAL:
                flywheel_.setTargetVelocity(CONSTANTS.FLYWHEEL_MANUAL_VELOCITY);
                indexer_.setTargetDutyCycle(CONSTANTS.INDEXER_DUTY_CYCLE);
                hood_.setCurrentPosition(CONSTANTS.HOOD_MANUAL_ANGLE);
                break;
            case TUNING:
                // code does NOTHING to allow for testing
                break;
            default:
            case IDLE:
                flywheel_.setTargetVelocity(CONSTANTS.SHOOTER_IDLE_SPEED);
                indexer_.setTargetDutyCycle(0);
                hood_.setTargetPosition(CONSTANTS.HOOD_IDLE_POSITION);
                if (CONSTANTS.TURRET_ENABLED) turret_.setTargetDutyCycle(0);
                break;
        }

        // LaunchCalculator Logging
        DogLog.log(getSubsystemKey() + "LaunchCalculator/Valid", launch_params_.is_valid);
        DogLog.log(
                getSubsystemKey() + "LaunchCalculator/HoodAngle",
                Units.radiansToDegrees(launch_params_.hood_angle));
        DogLog.log(
                getSubsystemKey() + "LaunchCalculator/HeadingAngle", launch_params_.heading_angle);
        DogLog.log(
                getSubsystemKey() + "LaunchCalculator/FlywheelSpeed",
                launch_params_.flywheel_speed);
        DogLog.log(getSubsystemKey() + "LaunchCalculator/Target", target_);
        DogLog.log(getSubsystemKey() + "LaunchCalculator/Distance", launch_params_.distance);
        DogLog.log(
                getSubsystemKey() + "LaunchCalculator/DistanceNoLookahead",
                launch_params_.distance_no_lookahead);
        DogLog.log(
                getSubsystemKey() + "LaunchCalculator/TimeOfFlight", launch_params_.time_of_flight);

        // Setpoint Logging
        DogLog.log(getSubsystemKey() + "Setpoint/FlywheelOmega", flywheel_omega_);
        DogLog.log(getSubsystemKey() + "Setpoint/HoodAngle", hood_angle_);
        DogLog.log(getSubsystemKey() + "Setpoint/HeadingAngle", turret_heading_);

        // System at Desired Setpoints
        DogLog.log(getSubsystemKey() + "ShooterIsReady/Hood", isHoodAtPosition());
        DogLog.log(getSubsystemKey() + "ShooterIsReady/Flywheel", isFlywheelAtSpeed());
        DogLog.log(getSubsystemKey() + "ShooterIsReady/Turret", isTurretAtPosition());
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
        return isFlywheelAtSpeed() && isHoodAtPosition() && isTurretAtPosition();
    }

    /**
     * Get the current launch angle from the solution
     *
     * @return the launch angle in radians, or the max hood angle if no valid solution
     */
    public double getLaunchAngle() {
        if (launch_params_ == null || !launch_params_.is_valid) {
            return CONSTANTS.HOOD_MAX_ANGLE;
        } else {
            return hood_angle_;
        }
    }

    /**
     * Get the current launch velocity from the solution
     *
     * @return the launch velocity in meters per second, or 0.0 if no valid solution
     */
    public double getLaunchVelocity() {
        if (launch_params_ == null || !launch_params_.is_valid) {
            return 0.0;
        } else {
            return launch_params_.flywheel_speed * CONSTANTS.FLYWHEEL_WHEEL_RADIUS_METERS;
        }
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
        Translation2d target2d = target.toTranslation2d();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            target2d = AllianceFlipUtil.apply(target2d);
        }
        target_ = target2d;
    }

    /**
     * Sets the target for the shooter to calculate a solution for. The target is a 2D translation
     * where x and y are the horizontal coordinates of the target relative to the field.
     *
     * @param target the target translation in meters, where x and y are the horizontal coordinates
     *     of the target relative to the field
     */
    public void setTarget(Translation2d target) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            target = AllianceFlipUtil.apply(target);
        }
        target_ = target;
    }

    /**
     * Sets whether to use the high arc or low arc solution. Note: LaunchCalculator uses map-based
     * shooting, so this method is kept for compatibility but has no effect.
     *
     * @param highArc true to use the high arc solution, false to use the low arc solution
     */
    public void setHighArc(boolean highArc) {
        // LaunchCalculator doesn't use high/low arc - kept for compatibility
    }

    /**
     * Sets the shooting tolerances for determining when the shooter is ready. This allows external
     * control of tolerance levels without the subsystem needing to know about game states or other
     * subsystem behavior. Tolerances are used to determine when the shooter mechanisms are close
     * enough to their target values to be considered "ready to shoot".
     *
     * @param flywheelSpeedTolerance the tolerance for the flywheel speed in rad/s
     * @param hoodPositionTolerance the tolerance for the hood position in radians
     * @param turretAngleTolerance the tolerance for the turret angle in radians (or rotation
     *     tolerance in degrees if no turret)
     */
    public void setShootingTolerances(
            double flywheelSpeedTolerance,
            double hoodPositionTolerance,
            double turretAngleTolerance) {
        this.flywheel_speed_tolerance_ = flywheelSpeedTolerance;
        this.hood_position_tolerance_ = hoodPositionTolerance;
        if (CONSTANTS.TURRET_ENABLED) {
            this.turret_angle_tolerance_ = turretAngleTolerance;
        } else {
            // When no turret, turretAngleTolerance is used as rotation tolerance in degrees
            this.rotation_angle_tolerance_ = turretAngleTolerance;
        }
    }

    // =============================================================================
    // PRIVATE HELPER METHODS
    // =============================================================================

    /**
     * Handles turret wrap-around logic. If the turret angle exceeds the maximum wrap angle, it will
     * wrap around to the other side. If currently shooting, this will transition back to AIMING
     * state to allow the turret to reposition.
     */
    private void handleTurretWrap() {
        if (turret_heading_ > CONSTANTS.MAX_TURRET_WRAP) {
            turret_heading_ -= 2 * Math.PI;
            if (system_state_ == ShooterStates.SHOOT) {
                setWantedState(ShooterStates.AIMING);
            }
        } else if (turret_heading_ < -CONSTANTS.MAX_TURRET_WRAP) {
            turret_heading_ += 2 * Math.PI;
            if (system_state_ == ShooterStates.SHOOT) {
                setWantedState(ShooterStates.AIMING);
            }
        }
    }

    /**
     * Check if the flywheel is at the target speed within a certain tolerance
     *
     * @return true if the flywheel speed is within tolerance, false otherwise
     */
    private boolean isFlywheelAtSpeed() {
        boolean status =
                MathUtil.isNear(
                        flywheel_omega_, flywheel_.getCurrentVelocity(), flywheel_speed_tolerance_);
        return status;
    }

    /**
     * Check if the hood is at the target position within a certain tolerance
     *
     * @return true if the hood is within tolerance, false otherwise
     */
    private boolean isHoodAtPosition() {
        boolean status =
                MathUtil.isNear(hood_angle_, hood_.getCurrentPosition(), hood_position_tolerance_);
        return status;
    }

    /**
     * Check if the turret is at the target position within a certain tolerance
     *
     * @return true if the turret is within tolerance, false otherwise
     */
    private boolean isTurretAtPosition() {
        boolean status;
        if (CONSTANTS.TURRET_ENABLED) {
            status =
                    MathUtil.isNear(
                            turret_heading_, turret_.getCurrentPosition(), turret_angle_tolerance_);
        } else {
            status =
                    MathUtil.isNear(
                            turret_heading_,
                            LocalizationSubsystem.getInstance()
                                    .getFieldPose()
                                    .getRotation()
                                    .getRadians(),
                            rotation_angle_tolerance_);
        }
        return status;
    }
}
