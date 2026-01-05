package frc.robot.subsystems.arm;

import java.lang.constant.Constable;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.configs.SlotConfigs;

import dev.doglog.DogLog;
import frc.mw_lib.mechanisms.ArmMech;
import frc.mw_lib.subsystem.MwSubsystem;
import frc.mw_lib.subsystem.SubsystemIoBase;
import frc.mw_lib.util.TunablePid;
import frc.robot.subsystems.arm.ArmConstants.ArmStates;

public class ArmSubsystem extends MwSubsystem<ArmStates, ArmConstants> {
    private static ArmSubsystem instance_ = null;

    public static ArmSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new ArmSubsystem();
        }
        return instance_;
    }

    private ArmMech  arm_mech_;
    private double target_arm_position_ = 0.0;

    public ArmSubsystem() {
        super(ArmStates.IDLE, new ArmConstants());

        arm_mech_ = new ArmMech(Arrays.asList(CONSTANTS.leader_motor_config), CONSTANTS.arm_gear_ratio, CONSTANTS.arm_length, CONSTANTS.arm_mass, CONSTANTS.arm_min_angle, CONSTANTS.arm_max_angle);
        arm_mech_.setLoggingPrefix(getSubsystemKey());

        TunablePid.create(getSubsystemKey() + "Arm", arm_mech_::setPositionSlot, SlotConfigs.from(CONSTANTS.leader_motor_config.config.Slot0));
        DogLog.tunable(getSubsystemKey() + "Arm/Setpoint", target_arm_position_, v -> target_arm_position_ = v);
    }

    // @Override
    // public void handleStateTransition(ElevatorStates wanted) {

    // }

    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case IDLE:
                arm_mech_.setTargetDutyCycle(0);
                break;
            case MOVE_TO_POSITION:
                arm_mech_.setTargetPosition(target_arm_position_);
                break;
            case HOLD:
                arm_mech_.setTargetVelocity(0);
                break;
            case TUNING:
                arm_mech_.setTargetPosition(target_arm_position_);
                break;
        }
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(arm_mech_);
    }

    public void requestMove(double position) {
        arm_mech_.setTargetPosition(position);
        system_state_ = ArmStates.MOVE_TO_POSITION;
    }

    @Override
    public void reset() {
        arm_mech_.setCurrentPosition(0);
        system_state_ = ArmStates.IDLE;
    }

}
