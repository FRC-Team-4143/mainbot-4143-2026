package frc.robot.subsystems.shooter;

import com.marswars.geometry.LaunchTrajectory.TrajectorySol;
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
import edu.wpi.first.math.util.Units;
import frc.robot.lib2026.FieldTargets;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.Arrays;
import java.util.List;

public class ShooterSubsystem extends MwSubsystem<ShooterStates, ShooterConstants> {
    private static ShooterSubsystem instance_ = null;

    public static ShooterSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new ShooterSubsystem();
        }
        return instance_;
    }

    private RollerMech indexer_;
    private FlywheelMech flywheel_;
    private RollerMech hood_;
    private TurretMech turret_;

    private double flywheel_eff_factor_ = CONSTANTS.FLYWHEEL_EFF_FACTOR;
    private double flywheel_omega_ = 0;
    double launch_heading_ = 0;
    double launch_exit_angle_ = CONSTANTS.HOOD_MAX_ANGLE;
    Translation3d target_ = new Translation3d(0.0, 0.0, 0.0);
    TrajectorySol solution_;

    // Adjustable shooting tolerances - initialized to strict defaults
    private double flywheel_speed_tolerance_ = FieldTargets.Shooter.FLYWHEEL_SPEED_TOLERANCE;
    private double hood_position_tolerance_ = FieldTargets.Shooter.HOOD_POSITION_TOLERANCE;
    private double turret_angle_tolerance_ = FieldTargets.Shooter.TURRET_ANGLE_TOLERANCE;
    private double rotation_angle_tolerance_ = FieldTargets.Shooter.ROTATION_ANGLE_TOLERANCE;

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

        DogLog.tunable(
                getSubsystemKey() + "Flywheel/EffFactor",
                CONSTANTS.FLYWHEEL_EFF_FACTOR,
                (val) -> flywheel_eff_factor_ = val);
    }

    @Override
    public void handleStateTransition(ShooterStates wanted) {
        Pose2d robotPose = LocalizationSubsystem.getInstance().getFieldPose();
        if (wanted == ShooterStates.SHOOT && !(system_state_ == ShooterStates.AIMING)) {
            system_state_ = ShooterStates.AIMING;
        }
        if (system_state_ == ShooterStates.AIMING && shooterIsReady()) {
            system_state_ = ShooterStates.SHOOT;
        } else {
            system_state_ = wanted;
        }

        // Current 4143 robot does not have a turret (wrapping logic is not need for the drivetrain)
        if (CONSTANTS.TURRET_ENABLED && (solution_ != null && solution_.valid)) {
            launch_heading_ = solution_.heading_angle - robotPose.getRotation().getRadians();
            if (launch_heading_ > CONSTANTS.MAX_TURRET_WRAP) {
                launch_heading_ -= 2 * Math.PI;
                if (system_state_ == ShooterStates.SHOOT) {
                    system_state_ = ShooterStates.AIMING;
                }
            } else if (launch_heading_ < -CONSTANTS.MAX_TURRET_WRAP) {
                launch_heading_ += 2 * Math.PI;
                if (system_state_ == ShooterStates.SHOOT) {
                    system_state_ = ShooterStates.AIMING;
                }
            }
        } else if (solution_ != null) {
            launch_heading_ = solution_.heading_angle;
        }
    }

    @Override
    public void updateLogic(double timestamp) {
        Pose2d robotPose = LocalizationSubsystem.getInstance().getFieldPose();
        solution_ = CONSTANTS.SOLVER.getSolution(robotPose.transformBy(CONSTANTS.SHOOTER_CENTER));
        if (solution_.valid) {
            flywheel_omega_ =
                    solution_.velocity
                            / CONSTANTS.FLYWHEEL_WHEEL_RADIUS_METERS
                            * flywheel_eff_factor_;
        }
        if (CONSTANTS.TURRET_ENABLED) {
            launch_heading_ = solution_.heading_angle - robotPose.getRotation().getRadians();
            if (launch_heading_ > CONSTANTS.MAX_TURRET_WRAP) {
                launch_heading_ -= 2 * Math.PI;
                if (system_state_ == ShooterStates.SHOOT) {
                    setWantedState(ShooterStates.AIMING);
                }
            } else if (launch_heading_ < -CONSTANTS.MAX_TURRET_WRAP) {
                launch_heading_ += 2 * Math.PI;
                if (system_state_ == ShooterStates.SHOOT) {
                    setWantedState(ShooterStates.AIMING);
                }
            }
        } else {
            launch_heading_ = solution_.heading_angle;
        }
        launch_exit_angle_ =
                MathUtil.clamp(
                        solution_.exit_angle, CONSTANTS.HOOD_MIN_ANGLE, CONSTANTS.HOOD_MAX_ANGLE);

        switch (system_state_) {
            case TRACKING:
                flywheel_.setTargetVelocity(flywheel_omega_);
                indexer_.setTargetDutyCycle(0);
                hood_.setTargetPosition(launch_exit_angle_);
                if (CONSTANTS.TURRET_ENABLED) {
                    turret_.setTargetPosition(launch_heading_);
                }
                break;
            case AIMING:
                flywheel_.setTargetVelocity(flywheel_omega_);
                indexer_.setTargetDutyCycle(0);
                hood_.setTargetPosition(launch_exit_angle_);
                if (CONSTANTS.TURRET_ENABLED) {
                    turret_.setTargetPosition(launch_heading_);
                } else {
                    SwerveSubsystem.getInstance()
                            .setDesiredRotationLockCOR(
                                    Rotation2d.fromRadians(launch_heading_),
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
                hood_.setTargetPosition(launch_exit_angle_);
                if (CONSTANTS.TURRET_ENABLED) {
                    turret_.setTargetPosition(launch_heading_);
                } else {
                    SwerveSubsystem.getInstance()
                            .setDesiredRotationLockCOR(
                                    Rotation2d.fromRadians(launch_heading_),
                                    new Translation2d(
                                            CONSTANTS.SHOOTER_CENTER.getX(),
                                            CONSTANTS.SHOOTER_CENTER.getY()));
                }
                break;
            default:
            case IDLE:
                flywheel_.setTargetVelocity(flywheel_omega_);
                indexer_.setTargetDutyCycle(0);
                hood_.setTargetPosition(CONSTANTS.HOOD_MAX_ANGLE);
                if (CONSTANTS.TURRET_ENABLED) turret_.setTargetPosition(0);
                break;
            case PROFILE:
                // code does NOTHING to allow for testing
                break;
        }
        // TrajectorySolver Logging
        DogLog.log(getSubsystemKey() + "TrajectorySolver/Valid", solution_.valid);
        DogLog.log(
                getSubsystemKey() + "TrajectorySolver/LaunchAngle",
                Units.radiansToDegrees(solution_.exit_angle));
        DogLog.log(
                getSubsystemKey() + "TrajectorySolver/LaunchHeading",
                Rotation2d.fromRadians(solution_.heading_angle));
        DogLog.log(getSubsystemKey() + "TrajectorySolver/LaunchVelocity", solution_.velocity);
        DogLog.log(getSubsystemKey() + "TrajectorySolver/Target", target_);

        // Setpoint Logging
        DogLog.log(getSubsystemKey() + "Setpoint/FlywheelOmega", flywheel_omega_);
        DogLog.log(getSubsystemKey() + "Setpoint/HoodAngle", launch_exit_angle_);
        DogLog.log(getSubsystemKey() + "Setpoint/HeadingAngle", launch_heading_);
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        // Current 4143 robot does not have a turret
        if (CONSTANTS.TURRET_ENABLED) {
            return Arrays.asList(indexer_, flywheel_, hood_, turret_);
        } else {
            return Arrays.asList(indexer_, flywheel_, hood_);
        }
    }

    @Override
    public void reset() {
        system_state_ = ShooterStates.IDLE;
    }

    /**
     * Check if the shooter is ready to shoot
     *
     * @return true all the active mechanisms are at within tolerance of their target
     *     positions/velocities
     */
    public boolean shooterIsReady() {
        // If there is no valid solution, the shooter cannot be ready
        if (solution_ == null || !solution_.valid) {
            return false;
        }
        return isFlywheelAtSpeed() && isHoodAtPosition() && isTurretAtPosition();
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
        DogLog.log(getSubsystemKey() + "ShooterIsReady/Flywheel", status);
        return status;
    }

    /**
     * Check if the hood is at the target position within a certain tolerance
     *
     * @return true if the hood is within tolerance, false otherwise
     */
    private boolean isHoodAtPosition() {
        boolean status =
                MathUtil.isNear(
                        launch_exit_angle_, hood_.getCurrentPosition(), hood_position_tolerance_);
        DogLog.log(getSubsystemKey() + "ShooterIsReady/Hood", status);
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
                            launch_heading_, turret_.getCurrentPosition(), turret_angle_tolerance_);
        } else {
            status =
                    SwerveSubsystem.getInstance()
                            .isAtDesiredRotation(Units.degreesToRadians(rotation_angle_tolerance_));
        }
        DogLog.log(getSubsystemKey() + "ShooterIsReady/Turret", status);
        return status;
    }

    /**
     * Get the current launch angle from the solution
     *
     * @return the launch angle in radians, or the max hood angle if no valid solution
     */
    public double getLaunchAngle() {
        if (solution_ == null || !solution_.valid) {
            return CONSTANTS.HOOD_MAX_ANGLE;
        } else {
            return launch_exit_angle_;
        }
    }

    /**
     * Get the current launch velocity from the solution
     *
     * @return the launch velocity in meters per second, or 0.0 if no valid solution
     */
    public double getLaunchVelocity() {
        if (solution_ == null || !solution_.valid) {
            return 0.0;
        } else {
            return solution_.velocity;
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
     * where x and y are the horizontal coordinates of the target relative to the field, and z is
     * the height of the target relative to the floor. This method will update the TrajectorySolver
     * with the new target.
     *
     * @param target the target translation in meters, where x and y are the horizontal coordinates
     *     of the target relative to the field, and z is the height of the target relative to the
     *     floor
     */
    public void setTarget(Translation3d target) {
        target_ = target;
        CONSTANTS.SOLVER.setTarget(target_);
    }

    /**
     * Sets whether to use the high arc or low arc solution from the TrajectorySolver. This will
     * update the TrajectorySolver with the new arc preference.
     *
     * @param highArc true to use the high arc solution, false to use the low arc solution
     */
    public void setHighArc(boolean highArc) {
        CONSTANTS.SOLVER.setHighArc(highArc);
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
}
