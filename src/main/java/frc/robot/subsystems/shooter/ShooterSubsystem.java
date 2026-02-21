package frc.robot.subsystems.shooter;

import com.marswars.geometry.AllianceFlipUtil;
import com.marswars.geometry.LaunchTrajectory;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib2026.FieldTargets;
import frc.robot.lib2026.FieldConstants.Hub;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

public class ShooterSubsystem extends MwSubsystem<ShooterStates, ShooterConstants> {
    private static ShooterSubsystem instance_ = null;

    public static ShooterSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new ShooterSubsystem();
        }
        return instance_;
    }

    // Mechanisms
    private RollerMech indexer_;
    private FlywheelMech flywheel_;
    private RollerMech hood_;
    private TurretMech turret_;

    // Shooter parameters calculated from TrajectorySolver
    private final LaunchTrajectory SOLVER =
            new LaunchTrajectory(
                    getSubsystemKey() + "TrajectorySolver/",
                    new Translation3d(),
                    CONSTANTS.LAUNCH_HEIGHT,
                    true);
    private double flywheel_omega_ = 0.0;
    private double turret_heading_ = 0.0;
    private double hood_angle_ = CONSTANTS.HOOD_MAX_ANGLE;
    private double manual_indexer_percent_ = 0.0;
    private Translation3d target_ = new Translation3d(0.0, 0.0, 0.0);
    private TrajectorySol solution_;

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

        populateInitialVelocityMap();
        SmartDashboard.putData(
                "Home Hood",
                Commands.runOnce(() -> hood_.setCurrentPosition(CONSTANTS.HOOD_HOME_POSITION))
                        .ignoringDisable(true));
    }

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
            // Nap time : Blocks deafult transition from occuring
        } else {
            system_state_ = wanted;
        }

        // Update launch heading for turret (wrapping logic is not needed for drivetrain rotation)
        if (CONSTANTS.TURRET_ENABLED && (solution_ != null && solution_.valid)) {
            turret_heading_ = solution_.heading_angle - robotPose.getRotation().getRadians();
            handleTurretWrap();
        } else if (solution_ != null && solution_.valid) {
            turret_heading_ = solution_.heading_angle;
        }
    }

    @Override
    public void updateLogic(double timestamp) {
        // Get the current robot pose
        Pose2d robot_pose = LocalizationSubsystem.getInstance().getFieldPose();

        // Calculate the trajectory solution for the current target and robot pose
        solution_ = SOLVER.getSolution(robot_pose.transformBy(CONSTANTS.SHOOTER_CENTER));

        // Update shooter parameters based on the solution
        // Skip the shooter parameters update if solution invalid on in manual mode to allow for
        // testing with manual setpoints
        if (solution_.valid && system_state_ != ShooterStates.MANUAL) {
            flywheel_omega_ = solution_.velocity / CONSTANTS.FLYWHEEL_WHEEL_RADIUS_METERS;

            // Calculate launch heading and handle turret wrapping
            if (CONSTANTS.TURRET_ENABLED) {
                turret_heading_ = solution_.heading_angle - robot_pose.getRotation().getRadians();
                handleTurretWrap();
            } else {
                turret_heading_ = solution_.heading_angle + CONSTANTS.SHOOTER_ROTATION.getRadians();
            }

            // Clamp hood angle to mechanical limits
            hood_angle_ =
                    MathUtil.clamp(
                            solution_.exit_angle,
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
                ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.HUB);
                flywheel_.setTargetVelocity(flywheel_omega_);
                indexer_.setTargetDutyCycle(manual_indexer_percent_);
                hood_.setTargetPosition(hood_angle_);
                if (CONSTANTS.TURRET_ENABLED) turret_.setTargetPosition(0);
                break;
            case TUNING:
                // code does NOTHING to allow for testing
                break;
            default:
            case IDLE:
                flywheel_.setTargetVelocity(CONSTANTS.SHOOTER_IDLE_SPEED);
                indexer_.setTargetDutyCycle(0);
                hood_.setTargetDutyCycle(CONSTANTS.HOOD_IDLE_POSITION);
                if (CONSTANTS.TURRET_ENABLED) turret_.setTargetDutyCycle(0);
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
        DogLog.log(
                getSubsystemKey() + "TrajectorySolver/Distance",
                target_.toTranslation2d().getDistance(robot_pose.getTranslation()));

        // Setpoint Logging
        DogLog.log(getSubsystemKey() + "Setpoint/FlywheelOmega", flywheel_omega_);
        DogLog.log(getSubsystemKey() + "Setpoint/HoodAngle", hood_angle_);
        DogLog.log(getSubsystemKey() + "Setpoint/HeadingAngle", turret_heading_);

        // System at Desired Setpoints
        DogLog.log(getSubsystemKey() + "ShooterIsReady/Hood", isHoodAtPosition());
        DogLog.log(getSubsystemKey() + "ShooterIsReady/Flywheel", isFlywheelAtSpeed());
        DogLog.log(getSubsystemKey() + "ShooterIsReady/Turret", isTurretAtPosition());
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
     * Check if the shooter is ready to shoot
     *
     * @return true all the active mechanisms are at within tolerance of their target
     *     positions/velocities
     */
    public boolean isShooterReady() {
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

    /**
     * Get the current launch angle from the solution
     *
     * @return the launch angle in radians, or the max hood angle if no valid solution
     */
    public double getLaunchAngle() {
        if (solution_ == null || !solution_.valid) {
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
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            target = AllianceFlipUtil.apply(target);
        }
        target_ = target;
        SOLVER.setTarget(target_);
    }

    /**
     * Sets whether to use the high arc or low arc solution from the TrajectorySolver. This will
     * update the TrajectorySolver with the new arc preference.
     *
     * @param highArc true to use the high arc solution, false to use the low arc solution
     */
    public void setHighArc(boolean highArc) {
        SOLVER.setHighArc(highArc);
    }

    /**
     * Populates the initial velocity map for the TrajectorySolver with empirical data points. This
     * method can be modified to add or adjust velocity points as needed for tuning the shooter
     * performance.
     */
    private void populateInitialVelocityMap() {
        // Solver Map Population
        SOLVER.addVelocityPoint(0.0, 5.283);
        SOLVER.addVelocityPoint(0.5, 5.382);
        SOLVER.addVelocityPoint(1.0, 6.635);
        SOLVER.addVelocityPoint(1.5, 5.977);
        SOLVER.addVelocityPoint(2.0, 6.367);
        SOLVER.addVelocityPoint(2.5, 6.764);
        SOLVER.addVelocityPoint(3.0, 7.160);
        SOLVER.addVelocityPoint(3.5, 7.544);
        SOLVER.addVelocityPoint(4.0, 7.928);
        SOLVER.addVelocityPoint(4.5, 8.288);
        SOLVER.addVelocityPoint(5.0, 8.648);
        SOLVER.addVelocityPoint(5.5, 8.996);
        SOLVER.addVelocityPoint(6.0, 10.332);
        SOLVER.addVelocityPoint(6.5, 10.656);
        SOLVER.addVelocityPoint(7.0, 10.980);
        SOLVER.addVelocityPoint(7.5, 11.292);
        SOLVER.addVelocityPoint(8.0, 11.592);
        SOLVER.addVelocityPoint(8.5, 11.892);
        SOLVER.addVelocityPoint(9.0, 12.180);
        SOLVER.addVelocityPoint(9.5, 12.456);
        SOLVER.addVelocityPoint(10.0, 12.744);
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
