package frc.robot.subsystems.gamestates;

import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import frc.robot.subsystems.gamestates.GameStatesConstants.GameStates;
import java.util.ArrayList;
import java.util.List;

public class GameStatesSubsystem extends MwSubsystem<GameStates, GameStatesConstants> {

    // Variables
    Boolean in_alliance_zone_ = false;
    Boolean in_neutral_zone_ = false;
    Boolean goal_active_ = false;
    Boolean operator_presses_climb_button_ = false;
    Boolean full_load_ = false;
    Boolean in_hold_zone_ = false;
    boolean pass_overide_ = false;
    Boolean auto_climb_ready_ = false;
    Boolean teleop_start_ = false;
    Boolean down_climb_finished_ = false;

    public GameStatesSubsystem() {
        super(GameStates.HOLD, new GameStatesConstants());
    }

    // state machine transtions (incomplete)
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case HOLD:
            //shooter innactive 
            //hopper holding and accepting balls unless full
            //pickup active to allow ball intake unless full
            //climber inactive
                break;
            case SCORE:
            //shooter active
            //hopper holding and accepting balls unless full
            //pickup active to allow ball intake unless full
            //climber inactive
                break;
            case PASS:
            //shooter active
            //hopper holding and accepting balls unless full
            //pickup active to allow ball intake unless full
            //climber inactive
                break;
            case AUTO_CLIMB:
            //shooter inactive
            //hopper holding balls only
            //pickup innactve
            //climber actively climbing
                break;
            case TELEOP_CLIMB:
            //shooter inactive
            //hopper holding balls only
            //pickup innactve
            //climber actively climbing up
                break;
            case DOWN_CLIMB:
            //shooter inactive
            //hopper holding balls only
            //pickup innactve
            //climber actively climbing down
                break;
        }
    }

    public void handleStateTransition(GameStates wanted) {
        // transtions out of TELEOP_CLIMB, no transtions
        if (system_state_ == GameStates.TELEOP_CLIMB) {
            system_state_ = GameStates.TELEOP_CLIMB;
            return;
        }
        // HOLD transitions
        if (system_state_ == GameStates.HOLD && in_alliance_zone_ && goal_active_) {
            system_state_ = GameStates.SCORE;
        } else if (system_state_ == GameStates.HOLD && in_neutral_zone_) {
            system_state_ = GameStates.PASS;
        } else if (system_state_ == GameStates.HOLD && pass_overide_) {
            system_state_ = GameStates.PASS;
        } else if (system_state_ == GameStates.HOLD && auto_climb_ready_) {
            system_state_ = GameStates.AUTO_CLIMB;
        } else if (system_state_ == GameStates.HOLD && operator_presses_climb_button_) {
            system_state_ = GameStates.TELEOP_CLIMB;
        } else {
        } // empty to not interfere with rest of state machine
        // SCORE transistions
        if (system_state_ == GameStates.SCORE && in_neutral_zone_) {
            system_state_ = GameStates.HOLD;
        } else if (system_state_ == GameStates.SCORE && !goal_active_) {
            system_state_ = GameStates.HOLD;
        } else if (system_state_ == GameStates.SCORE && auto_climb_ready_) {
            system_state_ = GameStates.AUTO_CLIMB;
        } else if (system_state_ == GameStates.SCORE && operator_presses_climb_button_) {
            system_state_ = GameStates.TELEOP_CLIMB;
        } else {
        } // empty to not interfere with rest of state machine
        // PASS transistions
        if (system_state_ == GameStates.PASS && in_hold_zone_) {
            system_state_ = GameStates.HOLD;
        } else if (system_state_ == GameStates.PASS && !pass_overide_) {
            system_state_ = GameStates.HOLD;
        } else {
        } // empty to not interfere with rest of state machine
        // AUTO_CLIMB transitions
        if (system_state_ == GameStates.AUTO_CLIMB && teleop_start_) {
            system_state_ = GameStates.DOWN_CLIMB;
        } else {
        } // empty to not interfere with rest of state machine
        // DOWN_CLIMB transistions
        if (system_state_ == GameStates.DOWN_CLIMB && down_climb_finished_) {
            system_state_ = GameStates.HOLD;
        } else {
        } // empty to not interfere with rest of state machine
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return new ArrayList<SubsystemIoBase>();
    }

    @Override
    public void reset() {
        system_state_ = GameStates.HOLD;
    }
}
