package frc.robot.subsystems.shooter;

import java.util.List;
import java.util.Arrays;
import dev.doglog.DogLog;
import frc.mw_lib.mechanisms.ArmMech;
import frc.mw_lib.mechanisms.FlywheelMech;
import frc.mw_lib.mechanisms.RollerMech;
import frc.mw_lib.subsystem.MwSubsystem;
import frc.mw_lib.subsystem.SubsystemIoBase;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;

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

    public ShooterSubsystem() {
        super(ShooterStates.IDLE, new ShooterConstants());
        indexer_ = new RollerMech(getSubsystemKey(), CONSTANTS.INDEX_MOTOR_CONFIG);
        flywheel_ = new FlywheelMech(getSubsystemKey(), List.of(CONSTANTS.SHOOTER_MOTOR_CONFIGS), CONSTANTS.SHOOTER_GEAR_RATIO, CONSTANTS.SHOOTER_WHEEL_INERTIA, CONSTANTS.SHOOTER_WHEEL_RADIUS_METERS);
        hood_ = new ArmMech(getSubsystemKey(), 0, 0, 0, 0, 0, 0)
        indexer_.setLoggingPrefix(getSubsystemKey());
        flywheel_.setLoggingPrefix(getSubsystemKey());

    }

    // @Override
    // public void handleStateTransition(ShooterStates wanted) {
    // }

    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case UNWIND:

            case AIMING:

            case DUMP:

            case SHOOT:

            case IDLE:

            case PROFILE:

        }
        // Log Data
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(indexer_, flywheel_);
    }

    @Override
    public void reset() {
        system_state_ = ShooterStates.IDLE;
    }
}