package frc.robot.subsystems.Climber;

import java.util.Arrays;
import java.util.List;

import com.marswars.mechanisms.RollerMech;
import com.marswars.mechanisms.ArmMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Climber.ClimberConstants.ClimberStates;

public class ClimberSubsystem extends MwSubsystem<ClimberStates, ClimberConstants> {
    private static ClimberSubsystem instance_ = null;

    // temporary varibale for state machine transstions
    private static boolean teleop_climb_state_enabled_ = false;
    private static boolean teleop_climb_state_canceled_ = false;
    private static boolean auto_climb_state_enabled_ = false;
    private static boolean all_other_attachments_retracted_ = false;
    private static boolean extended_ = false;
    private static boolean finished_deploying_ = false;
    private static boolean robot_allined_and_ready_ = false;
    private static boolean engaged_ = false;
    private static boolean finished_teleop_climb_up_ = false;
    private static boolean finished_teleop_climb_down_ = false;
    private static boolean finshed_auto_clim_up_ = false;
    private static boolean down_climb_state_enabled_ = false;
    private static boolean finished_auto_climb_down_ = false;

    public static ClimberSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new ClimberSubsystem();
        }
        return instance_;
    }

    private RollerMech Extender;
    private ArmMech Arm;

    // climer constructor
    public ClimberSubsystem() {
        super(ClimberStates.STORED, new ClimberConstants());
        Extender = new RollerMech(
                getSubsystemKey(),
                "Extender",
                List.of(CONSTANTS.EXTENDER_MOTOR_CONFIG),
                CONSTANTS.EXTENDER_GEAR_RATIO);
        Arm = new ArmMech(
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
            case STORED:
                break;
            case EXTENDING:
                break;
            case DEPLOYED:
                break;
            case TELEOP_ENGAGE:
                break;
            case TELEOP_CLIMB_UP:
                break;
            case IDLE_FINALE:
                break;
            case TELEOP_CLIMB_DOWN:
                break;
            case AUTO_ENGAGE:
                break;
            case AUTO_CLIMB_UP:
                break;
            case IDLE_AUTO:
                break;
            case AUTO_CLIMB_DOWN:
                break;
            case DISENGAGE:
                break;
            case STORING:
                break;
        }
    }

    public void handleStateTransistion(ClimberStates wanted) {
        if (system_state_ == ClimberStates.STORED && all_other_attachments_retracted_ && (teleop_climb_state_enabled_ || auto_climb_state_enabled_)) {
            system_state_ = ClimberStates.EXTENDING;
        } else {
        } //no other actions
        if (system_state_ == ClimberStates.EXTENDING && extended_) {
            system_state_ = ClimberStates.DEPLOYED;
        } else if (system_state_ == ClimberStates.EXTENDING && teleop_climb_state_canceled_) {
            system_state_ = ClimberStates.STORING;
        } else {
        } // no other actions

    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(Extender, Arm);
    }

    @Override
    public void reset() {
        system_state_ = ClimberStates.STORED;
    }
}
