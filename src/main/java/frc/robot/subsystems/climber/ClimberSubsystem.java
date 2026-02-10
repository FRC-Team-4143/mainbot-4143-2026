package frc.robot.subsystems.climber;

import com.marswars.mechanisms.ArmMech;
import com.marswars.mechanisms.RollerMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.climber.ClimberConstants.ClimberStates;
import java.util.Arrays;
import java.util.List;

public class ClimberSubsystem extends MwSubsystem<ClimberStates, ClimberConstants> {
    private static ClimberSubsystem instance_ = null;

    public static ClimberSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new ClimberSubsystem();
        }
        return instance_;
    }

    private RollerMech Extender_;
    private ArmMech Arm_;

    // climer constructor
    public ClimberSubsystem() {
        super(ClimberStates.STOWED, new ClimberConstants());
        Extender_ =
                new RollerMech(
                        getSubsystemKey(),
                        "Extender",
                        List.of(CONSTANTS.EXTENDER_MOTOR_CONFIG),
                        CONSTANTS.EXTENDER_GEAR_RATIO);
        Arm_ =
                new ArmMech(
                        getSubsystemKey(),
                        "Arm",
                        List.of(CONSTANTS.ARM_MOTOR_CONFIG),
                        CONSTANTS.ARM_GEAR_RATIO,
                        CONSTANTS.ARM_LENGTH,
                        CONSTANTS.ARM_MASS,
                        CONSTANTS.ARM_MAX_ANGLE,
                        CONSTANTS.ARM_MIN_ANGLE);
    }

    // state machine
    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case STOWED:
                Extender_.setTargetPosition(CONSTANTS.EXTENDER_STOWED_ANGLE);
                Arm_.setTargetPosition(CONSTANTS.ARM_L0_POSITION);
                break;
            case DEPLOY:
                Extender_.setTargetPosition(CONSTANTS.EXTENDER_DEPLOYED_ANGLE);
                break;
            case L1_CLIMB:
                Arm_.setTargetPosition(CONSTANTS.ARM_L1_CLIMB);
                Extender_.setTargetPosition(CONSTANTS.EXTENDER_DEPLOYED_ANGLE);
                break;
            case L1_DOWN:
                Arm_.setTargetPosition(CONSTANTS.ARM_L0_POSITION);
                Extender_.setTargetPosition(CONSTANTS.EXTENDER_DEPLOYED_ANGLE);
                break;
            case L3_CLIMB:
                Arm_.setTargetPosition(CONSTANTS.ARM_L3_CLIMB);
                Extender_.setTargetPosition(CONSTANTS.EXTENDER_DEPLOYED_ANGLE);
                break;
        }
    }

    // states transitions, tell what each state can transition too based on conditions
    @Override
    public void handleStateTransition(ClimberStates wanted_state) {
        if (system_state_ == ClimberStates.STOWED && wanted_state == ClimberStates.DEPLOY) {
            system_state_ = ClimberStates.DEPLOY;
        } else {
        } // no command
        if (system_state_ == ClimberStates.DEPLOY
                && isDeployed()
                && wanted_state == ClimberStates.L1_CLIMB) {
            system_state_ = ClimberStates.L1_CLIMB;
        } else if (system_state_ == ClimberStates.DEPLOY
                && isDeployed()
                && wanted_state == ClimberStates.L3_CLIMB) {
            system_state_ = ClimberStates.L3_CLIMB;
        } else if (system_state_ == ClimberStates.DEPLOY && wanted_state == ClimberStates.STOWED) {
            system_state_ = ClimberStates.STOWED;
        } else {
        } // no commands
        if (system_state_ == ClimberStates.L1_CLIMB && wanted_state == ClimberStates.L1_DOWN) {
            system_state_ = ClimberStates.L1_DOWN;
        } else {
        } // no commands
        if (system_state_ == ClimberStates.L1_DOWN && wanted_state == ClimberStates.DEPLOY) {
            system_state_ = ClimberStates.DEPLOY;
        } else {
        } // no commands
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(Extender_, Arm_);
    }

    @Override
    public void reset() {
        system_state_ = ClimberStates.STOWED;
    }

    private boolean isDeployed() {
        return (MathUtil.isNear(
                CONSTANTS.EXTENDER_DEPLOYED_ANGLE,
                Extender_.getCurrentPosition(),
                CONSTANTS.EXTENDER_TOLERANCE_ANGLE));
    }
}
