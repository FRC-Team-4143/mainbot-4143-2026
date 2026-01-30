package frc.robot.subsystems.Climber;

import com.marswars.subsystem.MwConstants;

public class ClimberConstants extends MwConstants {
    
    public enum ClimberStates {
        IDLE_FINALE,
        IDLE_AUTO,
        STORED,
        STORING,
        EXTENDING,
        DEPLOYED,
        ENGAGE,
        AUTO_CLIMB_UP,
        AUTO_CLIMB_DOWN,
        TELEOP_CLIMB,
        DISENGAGE,
    }
}
