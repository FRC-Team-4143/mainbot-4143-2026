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

    private RollerMech deploy_joint_;
    private ArmMech flip_joint_;

    // getInstance
    public static ClimberSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new ClimberSubsystem();
        }
        return instance_;
    }

    // Constructor
    public ClimberSubsystem() {
        super(ClimberStates.STOWED, new ClimberConstants());
        deploy_joint_ =
                new RollerMech(
                        getSubsystemKey(),
                        "Extender",
                        List.of(CONSTANTS.EXTENDER_MOTOR_CONFIG),
                        CONSTANTS.EXTENDER_GEAR_RATIO);
        flip_joint_ =
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

    // reset
    @Override
    public void reset() {
        system_state_ = ClimberStates.STOWED;
    }

    // getIos
    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(deploy_joint_, flip_joint_);
    }

    // handleStateTransition
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

    // updateLogic
    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case STOWED:
                deploy_joint_.setTargetPosition(CONSTANTS.EXTENDER_STOWED_ANGLE);
                flip_joint_.setTargetPosition(CONSTANTS.ARM_L0_POSITION);
                break;
            case DEPLOY:
                deploy_joint_.setTargetPosition(CONSTANTS.EXTENDER_DEPLOYED_ANGLE);
                break;
            case L1_CLIMB:
                flip_joint_.setTargetPosition(CONSTANTS.ARM_L1_CLIMB);
                deploy_joint_.setTargetPosition(CONSTANTS.EXTENDER_DEPLOYED_ANGLE);
                break;
            case L1_DOWN:
                flip_joint_.setTargetPosition(CONSTANTS.ARM_L0_POSITION);
                deploy_joint_.setTargetPosition(CONSTANTS.EXTENDER_DEPLOYED_ANGLE);
                break;
            case L3_CLIMB:
                flip_joint_.setTargetPosition(CONSTANTS.ARM_L3_CLIMB);
                deploy_joint_.setTargetPosition(CONSTANTS.EXTENDER_DEPLOYED_ANGLE);
                break;
        }
    }

    // =============================================================================
    // PRIVATE HELPER METHODS
    // =============================================================================

    private boolean isDeployed() {
        return (MathUtil.isNear(
                CONSTANTS.EXTENDER_DEPLOYED_ANGLE,
                deploy_joint_.getCurrentPosition(),
                CONSTANTS.EXTENDER_TOLERANCE_ANGLE));
    }
}
