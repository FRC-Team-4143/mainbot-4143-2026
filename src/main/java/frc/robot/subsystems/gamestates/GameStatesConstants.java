package frc.robot.subsystems.gamestates;

import com.marswars.subsystem.MwConstants;

import edu.wpi.first.math.geometry.Translation3d;

public class GameStatesConstants extends MwConstants {
    public enum GameStates {
        HOLD,
        SCORE,
        PASS,
        AUTO_CLIMB,
        TELEOP_CLIMB,
        DOWN_CLIMB
    }
    //POSITIONS OF SHOOTING TARGETS
    public final Translation3d LEFT_PASS_TRANSLATION = new Translation3d(0,0,0); // where to pass to on the left side - placeholder value
    public final Translation3d RIGHT_PASS_TRANSLATION = new Translation3d(0,0,0); // where to pass to on the left side - placeholder value
    public final Translation3d HUB_TRANSLATION = new Translation3d(4.611624, 4.021328, 1.397); // where the hub is
}
