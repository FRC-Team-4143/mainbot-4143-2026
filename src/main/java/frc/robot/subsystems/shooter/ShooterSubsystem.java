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
import edu.wpi.first.math.util.Units;
import frc.robot.OI;
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
    TrajectorySol solution;
    double newHeadingAngle;

    public ShooterSubsystem() {
        super(ShooterStates.IDLE, new ShooterConstants());
        indexer_ =
                new RollerMech(
                        getSubsystemKey(),
                        "Indexer",
                        List.of(CONSTANTS.INDEX_MOTOR_CONFIG),
                        CONSTANTS.INDEXER_GEAR_RATIO);
        flywheel_ =
                new FlywheelMech(
                        getSubsystemKey(),
                        List.of(
                                CONSTANTS.SHOOTER_LEADER_MOTOR_CONFIG,
                                CONSTANTS.SHOOTER_FOLLOWER_MOTOR_CONFIG),
                        CONSTANTS.FLYWHEEL_GEAR_RATIO,
                        CONSTANTS.FLYWHEEL_INERTIA,
                        CONSTANTS.FLYWHEEL_WHEEL_RADIUS_METERS);
        hood_ =
                new RollerMech(
                        getSubsystemKey(),
                        "Hood",
                        List.of(CONSTANTS.HOOD_MOTOR_CONFIGS),
                        CONSTANTS.HOOD_GEAR_RATIO);

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
            setWantedState(ShooterStates.AIMING);
        }
        if (system_state_ == ShooterStates.AIMING && shooterIsReady()) {
            setWantedState(ShooterStates.SHOOT);
        } else {
            system_state_ = wanted;
        }

        // Current 4143 robot does not have a turret (wrapping logic is not need for the drivetrain)
        if (CONSTANTS.TURRET_ENABLED && solution.valid) {
            newHeadingAngle = solution.heading_angle - robotPose.getRotation().getRadians();
            if (newHeadingAngle > CONSTANTS.MAX_TURRET_WRAP) {
                newHeadingAngle -= 2 * Math.PI;
                if (system_state_ == ShooterStates.SHOOT) {
                    setWantedState(ShooterStates.AIMING);
                }
            } else if (newHeadingAngle < -CONSTANTS.MAX_TURRET_WRAP) {
                newHeadingAngle += 2 * Math.PI;
                if (system_state_ == ShooterStates.SHOOT) {
                    setWantedState(ShooterStates.AIMING);
                }
            }
        }
    }

    @Override
    public void updateLogic(double timestamp) {
        Pose2d robotPose = LocalizationSubsystem.getInstance().getFieldPose();
        solution = CONSTANTS.SOLVER.getSolution(robotPose);
        if (solution.valid) {
            flywheel_omega_ =
                    solution.velocity
                            / CONSTANTS.FLYWHEEL_WHEEL_RADIUS_METERS
                            * flywheel_eff_factor_;
        }

        switch (system_state_) {
            case TRACKING:
            case AIMING:
                flywheel_.setTargetVelocity(flywheel_omega_);
                indexer_.setTargetDutyCycle(0);
                hood_.setTargetPosition(solution.exit_angle);
                if (CONSTANTS.TURRET_ENABLED) {
                    turret_.setTargetPosition(solution.heading_angle);
                } else {
                    SwerveSubsystem.getInstance()
                            .setDesiredRotationLockCOR(
                                    Rotation2d.fromRadians(solution.heading_angle),
                                    CONSTANTS.SHOOTER_CENTER); // placeholder Translation
                }
                ;
                break;
            case DUMP:
                flywheel_.setTargetVelocity(flywheel_omega_);
                indexer_.setTargetDutyCycle(-CONSTANTS.INDEXER_DUTY_CYCLE);
                hood_.setTargetPosition(0);
                if (CONSTANTS.TURRET_ENABLED) turret_.setTargetDutyCycle(0);
                break;
            case SHOOT:
                flywheel_.setTargetVelocity(flywheel_omega_);
                indexer_.setTargetDutyCycle(CONSTANTS.INDEXER_DUTY_CYCLE);
                hood_.setTargetPosition(solution.exit_angle);
                if (CONSTANTS.TURRET_ENABLED) turret_.setTargetPosition(solution.heading_angle);
                break;
            default:
            case IDLE:
                flywheel_.setTargetVelocity(flywheel_omega_);
                indexer_.setTargetDutyCycle(0);
                hood_.setTargetPosition(0);
                if (CONSTANTS.TURRET_ENABLED) turret_.setTargetPosition(0);
                break;
            case MANUAL:
                indexer_.setTargetDutyCycle(.8);
                hood_.setTargetPosition(
                        hood_.getCurrentPosition() + OI.getOperatorJoystickRightY());
                flywheel_.setTargetVelocity(
                        flywheel_.getCurrentVelocity() + OI.getOperatorJoystickLeftY());
                if (CONSTANTS.TURRET_ENABLED)
                    turret_.setTargetPosition(
                            turret_.getCurrentPosition() + OI.getOperatorJoystickRightX());
                break;
            case PROFILE:
                // code does NOTHING to allow for testing
                break;
        }
        // Log Data
        DogLog.log(getSubsystemKey() + "TrajectorySolver/Valid", solution.valid);
        DogLog.log(
                getSubsystemKey() + "TrajectorySolver/LaunchAngle",
                Units.radiansToDegrees(solution.exit_angle));
        DogLog.log(
                getSubsystemKey() + "TrajectorySolver/LaunchHeading",
                Rotation2d.fromRadians(solution.heading_angle));
        DogLog.log(getSubsystemKey() + "TrajectorySolver/LaunchVelocity", solution.velocity);
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
        return ((CONSTANTS.TURRET_ENABLED)
                        ? MathUtil.isNear(
                                newHeadingAngle,
                                turret_.getCurrentPosition(),
                                CONSTANTS.TURRET_ANGLE_TOLERANCE)
                        : true)
                && MathUtil.isNear(
                        flywheel_omega_,
                        flywheel_.getCurrentVelocity(),
                        CONSTANTS.FLYWHEEL_SPEED_TOLERANCE)
                && MathUtil.isNear(
                        solution.exit_angle,
                        hood_.getCurrentPosition(),
                        CONSTANTS.HOOD_ANGLE_TOLERANCE);
    }
}
