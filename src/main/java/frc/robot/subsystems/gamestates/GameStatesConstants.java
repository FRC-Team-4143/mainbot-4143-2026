package frc.robot.subsystems.gamestates;

import com.marswars.subsystem.MwConstants;

public class GameStatesConstants extends MwConstants {

    // =============================================================================
    // ENUMS AND STATE DEFINITIONS
    // =============================================================================

    public enum GameStates {
        HOLD,
        SCORE,
        PASS,
        //AUTO_CLIMB,
        TELEOP_CLIMB,
        DOWN_CLIMB,
        AUTO
    }
}
