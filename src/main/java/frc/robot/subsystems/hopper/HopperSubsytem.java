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

    private RollerMech feed_motor;
    private RollerMech hopper_motor;

    public HopperSubsytem() {
        super(HopperStates.IDLE, new HopperContstants());
        feed_motor =
                new RollerMech(
                        getSubsystemKey(),
                        List.of(CONSTANTS.FEED_MOTOR_CONFIG),
                        CONSTANTS.FEED_GEAR_RATIO);

        hopper_motor =
                new RollerMech(
                        getSubsystemKey(),
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
                break;
            case STIRRING:
                break;
            case SHOOTING:
                break;
        }
        // Log Data
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(feed_motor, hopper_motor);
    }

    @Override
    public void reset() {
        system_state_ = HopperStates.IDLE;
    }
}
