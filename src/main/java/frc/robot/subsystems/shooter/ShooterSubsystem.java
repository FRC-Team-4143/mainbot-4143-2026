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

    private LaunchTrajectory solver;
    private double flywheelEffFactor = 1;
    private double topSpinEffFactor = 1;
    private double shooterRotationalVelocity;
    private double topSpinRotationalVelocity;

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
                        "Flywheel",
                        List.of(
                                CONSTANTS.SHOOTER_LEADER_MOTOR_CONFIG,
                                CONSTANTS.SHOOTER_FOLLOWER_MOTOR_CONFIG),
                        CONSTANTS.SHOOTER_GEAR_RATIO,
                        CONSTANTS.SHOOTER_WHEEL_INERTIA,
                        CONSTANTS.SHOOTER_WHEEL_RADIUS_METERS);
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

        solver = new LaunchTrajectory(CONSTANTS.HUB_TRANSLATION, CONSTANTS.LAUNCH_HIGHT, true);
        solver.addVelocityPoint(0.0, 6.283);
        solver.addVelocityPoint(0.5, 6.382);
        solver.addVelocityPoint(1.0, 6.635);
        solver.addVelocityPoint(1.5, 6.977);
        solver.addVelocityPoint(2.0, 7.367);
        solver.addVelocityPoint(2.5, 7.764);
        solver.addVelocityPoint(3.0, 8.160);
        solver.addVelocityPoint(3.5, 8.544);
        solver.addVelocityPoint(4.0, 8.928);
        solver.addVelocityPoint(4.5, 9.288);
        solver.addVelocityPoint(5.0, 9.648);
        solver.addVelocityPoint(5.5, 9.996);
        solver.addVelocityPoint(6.0, 10.332);
        solver.addVelocityPoint(6.5, 10.656);
        solver.addVelocityPoint(7.0, 10.980);
        solver.addVelocityPoint(7.5, 11.292);
        solver.addVelocityPoint(8.0, 11.592);
        solver.addVelocityPoint(8.5, 11.892);
        solver.addVelocityPoint(9.0, 12.180);
        solver.addVelocityPoint(9.5, 12.456);
        solver.addVelocityPoint(10.0, 12.744);


        DogLog.tunable(getSubsystemKey()+" flywheel eff factor", 1.0, (val) -> flywheelEffFactor = val);
        DogLog.tunable(getSubsystemKey()+" topspin eff factor", 1.0, (val) -> topSpinEffFactor = val);
    }

    // @Override
    // public void handleStateTransition(ShooterStates wanted) {
    // }

    @Override
    public void updateLogic(double timestamp) {
        TrajectorySol solution =
                solver.getSolution(LocalizationSubsystem.getInstance().getFieldPose());
        if(solution.valid){
            shooterRotationalVelocity = solution.velocity / CONSTANTS.SHOOTER_WHEEL_RADIUS_METERS * flywheelEffFactor; //Tune the multiplication later
            topSpinRotationalVelocity = solution.velocity / CONSTANTS.TOP_SPIN_RADIUS_METERS * topSpinEffFactor; //Tune the multiplication later
        }
        
        switch (system_state_) {
            case UNWIND:
                break;
            case AIMING:
                flywheel_.setTargetVelocity(shooterRotationalVelocity); // calculate flywheel speed from exit speed
                indexer_.setTargetDutyCycle(0);
                turret_.setTargetPosition(solution.heading_angle);
                hood_.setTargetPosition(solution.exit_angle);
                top_spin_.setTargetVelocity(topSpinRotationalVelocity); // calculate speed from exit speed
                break;
            case DUMP:
                flywheel_.setTargetVelocity(shooterRotationalVelocity); // calculate flywheel speed from exit speed
                indexer_.setTargetDutyCycle(-CONSTANTS.INDEXER_DUTY_CYCLE);
                turret_.setTargetDutyCycle(0);
                hood_.setTargetPosition(0);
                top_spin_.setTargetVelocity(topSpinRotationalVelocity); // calculate speed from exit speed
                break;
            case SHOOT:
                flywheel_.setTargetVelocity(shooterRotationalVelocity); // calculate flywheel speed from exit speed
                indexer_.setTargetDutyCycle(CONSTANTS.INDEXER_DUTY_CYCLE);
                turret_.setTargetPosition(solution.heading_angle);
                hood_.setTargetPosition(solution.exit_angle);
                top_spin_.setTargetVelocity(topSpinRotationalVelocity); // calculate speed from exit speed
                break;
            case IDLE:
                flywheel_.setTargetVelocity(shooterRotationalVelocity); // calculate flywheel speed from exit speed
                indexer_.setTargetDutyCycle(0);
                turret_.setTargetPosition(0);
                hood_.setTargetPosition(0);
                top_spin_.setTargetVelocity(topSpinRotationalVelocity); // calculate speed from exit speed
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
                Units.radiansToDegrees(solution.velocity));
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
