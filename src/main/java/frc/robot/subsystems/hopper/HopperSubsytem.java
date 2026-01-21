package frc.robot.subsystems.hopper;

import com.marswars.mechanisms.RollerMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import frc.robot.subsystems.hopper.HopperContstants.HopperStates;
import java.util.Arrays;
import java.util.List;

public class HopperSubsytem extends MwSubsystem<HopperStates, HopperContstants> {
    private static HopperSubsytem instance_ = null;

    public static HopperSubsytem getInstance() {
        if (instance_ == null) {
            instance_ = new HopperSubsytem();
        }
        return instance_;
    }

    private RollerMech Feeder;
    private RollerMech Hopper;

    public HopperSubsytem() {
        super(HopperStates.IDLE, new HopperContstants());
        Feeder = new RollerMech(
                getSubsystemKey(),
                "Feeder",
                List.of(CONSTANTS.FEED_MOTOR_CONFIG),
                CONSTANTS.FEED_GEAR_RATIO);

        Hopper = new RollerMech(
                getSubsystemKey(),
                "Hopper",
                List.of(CONSTANTS.HOPPER_MOTOR_CONFIG),
                CONSTANTS.HOPPER_GEAR_RATIO);
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
                Feeder.setTargetDutyCycle(0.0);
                Hopper.setTargetDutyCycle(0.0);
                break;
            case STIRRING:
                break;
            case SHOOTING:
                Feeder.setTargetDutyCycle(.5);
                Hopper.setTargetDutyCycle(.5);
                break;
            case PROFILE:
                break;
        }
        // Log Data
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(Feeder, Hopper);
    }

    @Override
    public void reset() {
        system_state_ = HopperStates.IDLE;
    }
}
