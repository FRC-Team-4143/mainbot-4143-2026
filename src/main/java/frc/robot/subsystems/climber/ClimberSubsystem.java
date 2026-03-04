package frc.robot.subsystems.climber;

import com.marswars.mechanisms.ArmMech;
import com.marswars.mechanisms.RollerMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.climber.ClimberConstants.ClimberStates;
import java.util.Arrays;
import java.util.List;

public class ClimberSubsystem extends MwSubsystem<ClimberStates, ClimberConstants> {
    private static ClimberSubsystem instance_ = null;

    private double stowed_angle_ = CONSTANTS.DEPLOY_STOWED_ANGLE;
    private double flip_angle_ = CONSTANTS.FLIP_L0_POSITION;

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
                        List.of(CONSTANTS.DEPLOY_MOTOR_CONFIG),
                        CONSTANTS.DEPLOY_GEAR_RATIO);
        flip_joint_ =
                new ArmMech(
                        getSubsystemKey(),
                        "Arm",
                        List.of(CONSTANTS.FLIP_MOTOR_CONFIG),
                        CONSTANTS.FLIP_GEAR_RATIO,
                        CONSTANTS.FLIP_LENGTH,
                        CONSTANTS.FLIP_MASS,
                        CONSTANTS.FLIP_MAX_ANGLE,
                        CONSTANTS.FLIP_MIN_ANGLE);
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
        if (wanted_state == ClimberStates.TUNNING) {
            system_state_ = ClimberStates.TUNNING;
        }
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
                deploy_joint_.setTargetPosition(stowed_angle_);
                flip_joint_.setTargetPosition(flip_angle_);
                break;
            case DEPLOY:
                deploy_joint_.setTargetPosition(CONSTANTS.DEPLOY_DEPLOYED_ANGLE);
                break;
            case L1_CLIMB:
                flip_joint_.setTargetPosition(CONSTANTS.FLIP_L1_CLIMB);
                deploy_joint_.setTargetPosition(CONSTANTS.DEPLOY_DEPLOYED_ANGLE);
                break;
            case L1_DOWN:
                flip_joint_.setTargetPosition(CONSTANTS.FLIP_L0_POSITION);
                deploy_joint_.setTargetPosition(CONSTANTS.DEPLOY_DEPLOYED_ANGLE);
                break;
            case L3_CLIMB:
                flip_joint_.setTargetPosition(CONSTANTS.FLIP_L3_CLIMB);
                deploy_joint_.setTargetPosition(CONSTANTS.DEPLOY_DEPLOYED_ANGLE);
                break;
            case TUNNING:
                // allow manual control of both joints through the dashboard for testing and
                // calibration
                break;
            case MANUAL:
                // State to allow manual control of clibmer without the motors moving on their own
                break;
        }
        DogLog.log(getSubsystemKey() + "Flip Angle", flip_angle_);
        DogLog.log(getSubsystemKey() + "Stowed Angle", stowed_angle_);
    }

    // =============================================================================
    // PUBLIC HELPER METHODS
    // =============================================================================

    /**
     * Returns the current angle of the deploy joint in radians. (Used for testing and visualization
     * purposes)
     *
     * @return the current angle of the deploy joint in radians
     */
    public double getDeployAngle() {
        return deploy_joint_.getCurrentPosition();
    }

    /**
     * Returns the current angle of the flip joint in radians. (Used for testing and visualization
     * purposes)
     *
     * @return the current angle of the flip joint in radians
     */
    public double getFlipAngle() {
        return flip_joint_.getCurrentPosition();
    }

    // =============================================================================
    // PRIVATE HELPER METHODS
    // =============================================================================

    private boolean isDeployed() {
        return (MathUtil.isNear(
                CONSTANTS.DEPLOY_DEPLOYED_ANGLE,
                deploy_joint_.getCurrentPosition(),
                CONSTANTS.DEPLOY_TOLERANCE_ANGLE));
    }

    // methods for manualy tuning climber out side of tuning states
    public void rotateFlipClockwise() {
        stowed_angle_ = stowed_angle_ + Units.degreesToRadians(5);
    }

    public void rotateFlipCounterClockwise() {
        stowed_angle_ = stowed_angle_ - Units.degreesToRadians(5);
    }

    public void rotateDeployClockwise() {
        flip_angle_ = flip_angle_ + Units.degreesToRadians(5);
    }

    public void rotateDeployCounterClockwise() {
        flip_angle_ = flip_angle_ - Units.degreesToRadians(5);
    }
}
