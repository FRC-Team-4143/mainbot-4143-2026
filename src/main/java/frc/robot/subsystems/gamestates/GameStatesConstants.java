package frc.robot.subsystems.gamestates;

import com.marswars.subsystem.MwConstants;

public class GameStatesConstants extends MwConstants {

    // =============================================================================
    // ENUMS AND STATE DEFINITIONS
    // =============================================================================

    public enum GameStates {
        /** Holding game pieces, not actively scoring */
        HOLD,
        /** When trigger is pulled this is the first state the robot goes into */
        SHOOT,
        /** Actively scoring game pieces */
        SCORE,
        /** Passing game pieces to another robot */
        PASS,
        /** Climbing during teleop period */
        TELEOP_CLIMB,
        /** Moving down from climb position */
        DOWN_CLIMB,
        /** Autonomous mode state - This prevents GSM from interfering */
        AUTO
    }
}
