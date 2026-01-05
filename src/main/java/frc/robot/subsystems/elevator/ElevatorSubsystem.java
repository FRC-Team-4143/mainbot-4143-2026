package frc.robot.subsystems.elevator;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.configs.SlotConfigs;

import dev.doglog.DogLog;
import frc.mw_lib.mechanisms.ElevatorMech;
import frc.mw_lib.subsystem.MwSubsystem;
import frc.mw_lib.subsystem.SubsystemIoBase;
import frc.mw_lib.util.TunablePid;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;

public class ElevatorSubsystem extends MwSubsystem<ElevatorStates, ElevatorConstants> {
    private static ElevatorSubsystem instance_ = null;

    public static ElevatorSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new ElevatorSubsystem();
        }
        return instance_;
    }

    private ElevatorMech elevator_mech_;
    private double target_elevator_position_ = 0.0;

    public ElevatorSubsystem() {
        super(ElevatorStates.IDLE, new ElevatorConstants());

        elevator_mech_ = new ElevatorMech(Arrays.asList(CONSTANTS.leader_motor_config), CONSTANTS.elevator_gear_ratio,
                CONSTANTS.elevator_drum_radius, CONSTANTS.elevator_carrige_mass, CONSTANTS.elevator_max_extension,
                CONSTANTS.elevator_rigging_ratio);
        elevator_mech_.setLoggingPrefix(getSubsystemKey());

        TunablePid.create(getSubsystemKey() + "Elevator", elevator_mech_::setPositionSlot, SlotConfigs.from(CONSTANTS.leader_motor_config.config.Slot0));
        DogLog.tunable(getSubsystemKey() + "Elevator/Setpoint", target_elevator_position_, v -> target_elevator_position_ = v);
    }

    // @Override
    // public void handleStateTransition(ElevatorStates wanted) {

    // }

    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case IDLE:
                elevator_mech_.setTargetDutyCycle(0);
                break;
            case MOVE_TO_POSITION:
                elevator_mech_.setTargetPosition(target_elevator_position_);
                break;
            case HOLD:
                elevator_mech_.setTargetVelocity(0);
                break;
            case TUNING:
                elevator_mech_.setTargetPosition(target_elevator_position_);
                break;
        }
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(elevator_mech_);
    }

    public void requestMove(double position) {
        elevator_mech_.setTargetPosition(position);
        system_state_ = ElevatorStates.MOVE_TO_POSITION;
    }

    @Override
    public void reset() {
        elevator_mech_.setCurrentPosition(0);
        system_state_ = ElevatorStates.IDLE;
    }

}
