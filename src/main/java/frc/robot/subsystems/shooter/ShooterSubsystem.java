package frc.robot.subsystems.shooter;

import com.marswars.geometry.LaunchTrajectory;
import com.marswars.geometry.LaunchTrajectory.TrajectorySol;
import com.marswars.mechanisms.ArmMech;
import com.marswars.mechanisms.FlywheelMech;
import com.marswars.mechanisms.RollerMech;
import com.marswars.mechanisms.TurretMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;

import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
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
    private RollerMech top_spin_;
    private ArmMech hood_;
    private TurretMech turret_;

    private double flywheel_eff_factor_ = CONSTANTS.FLYWHEEL_EFF_FACTOR;
    private double top_spin_eff_factor_ = CONSTANTS.TOP_SPIN_EFF_FACTOR;
    private double flywheel_omega_ = 0;
    private double top_spin_omega_ = 0;

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
                new ArmMech(
                        getSubsystemKey(),
                        "Hood",
                        List.of(CONSTANTS.HOOD_MOTOR_CONFIGS),
                        CONSTANTS.HOOD_GEAR_RATIO,
                        CONSTANTS.HOOD_LENGTH,
                        CONSTANTS.HOOD_MASS_KG,
                        CONSTANTS.HOOD_MIN_ANGLE,
                        CONSTANTS.HOOD_MAX_ANGLE);
        top_spin_ =
                new RollerMech(
                        getSubsystemKey(),
                        "TopSpin",
                        List.of(CONSTANTS.TOP_SPIN_CONFIG),
                        CONSTANTS.TOP_SPIN_GEAR_RATIO);
        turret_ =
                new TurretMech(
                        getSubsystemKey(),
                        List.of(CONSTANTS.TURRET_MOTOR_CONFIGS),
                        CONSTANTS.TURRET_GEAR_RATIO,
                        CONSTANTS.TURRET_MOI);

        DogLog.tunable(getSubsystemKey()+"Flywheel/EffFactor", CONSTANTS.FLYWHEEL_EFF_FACTOR, (val) -> flywheel_eff_factor_ = val);
        DogLog.tunable(getSubsystemKey()+"TopSpin/EffFactor", CONSTANTS.TOP_SPIN_EFF_FACTOR, (val) -> top_spin_eff_factor_ = val);
    }

    // @Override
    // public void handleStateTransition(ShooterStates wanted) {
    // }

    @Override
    public void updateLogic(double timestamp) {
        TrajectorySol solution = CONSTANTS.SOLVER.getSolution(LocalizationSubsystem.getInstance().getFieldPose());
        if(solution.valid){
            flywheel_omega_ = solution.velocity / CONSTANTS.FLYWHEEL_WHEEL_RADIUS_METERS * flywheel_eff_factor_;
            top_spin_omega_ = solution.velocity / CONSTANTS.TOP_SPIN_RADIUS_METERS * top_spin_eff_factor_;
        }
        
        switch (system_state_) {
            case UNWIND:
                break;
            case AIMING:
                flywheel_.setTargetVelocity(flywheel_omega_); // calculate flywheel speed from exit speed
                indexer_.setTargetDutyCycle(0);
                turret_.setTargetPosition(solution.heading_angle);
                hood_.setTargetPosition(solution.exit_angle);
                top_spin_.setTargetVelocity(top_spin_omega_); // calculate speed from exit speed
                break;
            case DUMP:
                flywheel_.setTargetVelocity(flywheel_omega_); // calculate flywheel speed from exit speed
                indexer_.setTargetDutyCycle(-CONSTANTS.INDEXER_DUTY_CYCLE);
                turret_.setTargetDutyCycle(0);
                hood_.setTargetPosition(0);
                top_spin_.setTargetVelocity(top_spin_omega_); // calculate speed from exit speed
                break;
            case SHOOT:
                flywheel_.setTargetVelocity(flywheel_omega_); // calculate flywheel speed from exit speed
                indexer_.setTargetDutyCycle(CONSTANTS.INDEXER_DUTY_CYCLE);
                turret_.setTargetPosition(solution.heading_angle);
                hood_.setTargetPosition(solution.exit_angle);
                top_spin_.setTargetVelocity(top_spin_omega_); // calculate speed from exit speed
                break;
            default:
            case IDLE:
                flywheel_.setTargetVelocity(flywheel_omega_); // calculate flywheel speed from exit speed
                indexer_.setTargetDutyCycle(0);
                turret_.setTargetPosition(0);
                hood_.setTargetPosition(0);
                top_spin_.setTargetVelocity(top_spin_omega_); // calculate speed from exit speed
                break;
            case PROFILE:
                // code does NOTHING to allow for testing
                break;
        }
        // Log Data
        DogLog.log(
                getSubsystemKey() + "TrajectorySolver/Valid",
                solution.valid);
        DogLog.log(
                getSubsystemKey() + "TrajectorySolver/LaunchAngle",
                Units.radiansToDegrees(solution.exit_angle));
        DogLog.log(
                getSubsystemKey() + "TrajectorySolver/LaunchHeading",
                Units.radiansToDegrees(solution.heading_angle));
        DogLog.log(
                getSubsystemKey() + "TrajectorySolver/LaunchVelocity",
                solution.velocity);
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(indexer_, flywheel_, hood_, top_spin_, turret_);
    }

    @Override
    public void reset() {
        system_state_ = ShooterStates.IDLE;
    }
}
