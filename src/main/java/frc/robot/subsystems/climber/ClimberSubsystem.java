package frc.robot.subsystems.climber;

import com.marswars.mechanisms.ArmMech;
import com.marswars.mechanisms.RollerMech;
import com.marswars.sensors.tof.PwfTof;
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
    private PwfTof left_sensor_;
    private PwfTof right_sensor_;

    private double left_sensor_range_;
    private double right_sensor_range_;

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
        left_sensor_ =
                new PwfTof(
                    getSubsystemKey(),
                    CONSTANTS.LEFT_SENSOR_ID,
                    CONSTANTS.SENSOR_MODE,
                    0.002); // correct?
        right_sensor_ =
                new PwfTof(
                    getSubsystemKey(),
                    CONSTANTS.RIGHT_SENSOR_ID,
                    CONSTANTS.SENSOR_MODE,
                    0.002); // correct?
    }

    /* sets the sensors ranges (how close the nearest object is)
     */
    public void setSensors() { // incomplete
        left_sensor_range_ = left_sensor_.getRange();
        right_sensor_range_ = right_sensor_.getRange();
    }

    /* return which climber sensor is out of place, 
     * returns null otherwise
    */
    public PwfTof checkSensors() { // incomplete
        setSensors();
        if(CONSTANTS.SENSOR_RANGE_TOLERANCE < left_sensor_range_) {
            return left_sensor_; // when returned, have the robot then move right a bit
        }
        else if(CONSTANTS.SENSOR_RANGE_TOLERANCE < right_sensor_range_) {
            return right_sensor_; // when returned, have the robot then move left a bit
        }
        return null; // when returned, simply have the robot move forward
        // would a new climber state work? called APPROCHING?
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
