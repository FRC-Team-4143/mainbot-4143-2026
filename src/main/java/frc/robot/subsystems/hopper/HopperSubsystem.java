package frc.robot.subsystems.hopper;

import com.marswars.mechanisms.RollerMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import frc.robot.subsystems.hopper.HopperConstants.HopperStates;
import java.util.Arrays;
import java.util.List;

public class HopperSubsystem extends MwSubsystem<HopperStates, HopperConstants> {
    private static HopperSubsystem instance_ = null;

    public static HopperSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new HopperSubsystem();
        }
        return instance_;
    }

    private RollerMech feeder_;
    private RollerMech indexer_;

    public HopperSubsystem() {
        super(HopperStates.IDLE, new HopperConstants());
        feeder_ =
                new RollerMech(
                        getSubsystemKey(),
                        "Feeder",
                        List.of(CONSTANTS.FEEDER_MOTOR_CONFIG),
                        CONSTANTS.FEEDER_GEAR_RATIO);

        indexer_ =
                new RollerMech(
                        getSubsystemKey(),
                        "Hopper",
                        List.of(CONSTANTS.INDEXER_MOTOR_CONFIG),
                        CONSTANTS.INDEXER_GEAR_RATIO);
    }

    /*
     * @override
     * public void handleStateTransistion(HopperStates wanted){
     * }
     */
    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case IDLE:
                feeder_.setTargetDutyCycle(0.0);
                indexer_.setTargetDutyCycle(0.0);
                break;
            case STIRRING:
                break;
            case SHOOTING:
                feeder_.setTargetDutyCycle(CONSTANTS.FEEDER_DUTY_CYCLE_SHOOT);
                indexer_.setTargetDutyCycle(CONSTANTS.INDEXER_DUTY_CYCLE_SHOOT);
                break;
            case PROFILE:
                break;
        }
        // Log Data
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(feeder_, indexer_);
    }

    @Override
    public void reset() {
        system_state_ = HopperStates.IDLE;
    }
}
