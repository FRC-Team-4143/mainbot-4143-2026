package frc.robot.subsystems.shooter;

import com.marswars.mechanisms.ArmMech;
import com.marswars.mechanisms.FlywheelMech;
import com.marswars.mechanisms.RollerMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
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
    private ArmMech hood_;

    public ShooterSubsystem() {
        super(ShooterStates.IDLE, new ShooterConstants());
        indexer_ =
                new RollerMech(
                        getSubsystemKey(),
                        List.of(CONSTANTS.INDEX_MOTOR_CONFIG),
                        CONSTANTS.INDEXER_GEAR_RATIO);
        flywheel_ =
                new FlywheelMech(
                        getSubsystemKey(),
                        List.of(CONSTANTS.SHOOTER_MOTOR_CONFIGS),
                        CONSTANTS.SHOOTER_GEAR_RATIO,
                        CONSTANTS.SHOOTER_WHEEL_INERTIA,
                        CONSTANTS.SHOOTER_WHEEL_RADIUS_METERS);
        hood_ =
                new ArmMech(
                        getSubsystemKey(),
                        List.of(CONSTANTS.HOOD_MOTOR_CONFIG),
                        CONSTANTS.HOOD_GEAR_RATIO,
                        CONSTANTS.HOOD_LENGTH,
                        CONSTANTS.HOOD_MASS_KG,
                        CONSTANTS.HOOD_MIN_ANGLE,
                        CONSTANTS.HOOD_MAX_ANGLE);
    }

    @Override
    public void handleStateTransition(ShooterStates wanted) {
        
    }

    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case UNWIND:

                break;
            case AIMING:

                break;
            case DUMP:

                break;
            case SHOOT:

                break;
            case IDLE:

                break;
            case PROFILE:

                break;
        }
        // Log Data
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(indexer_, flywheel_, hood_);
    }

    @Override
    public void reset() {
        system_state_ = ShooterStates.IDLE;
    }
}
