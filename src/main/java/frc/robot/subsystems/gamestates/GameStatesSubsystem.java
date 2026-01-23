package frc.robot.subsystems.gamestates;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;

import frc.robot.subsystems.gamestates.GameStatesConstants.GameStates;

public class GameStatesSubsystem extends MwSubsystem<GameStates, GameStatesConstants> {

    //Variables
    Boolean in_alliance_zone_ = false;
    Boolean in_neutral_zone = true;



    public GameStatesSubsystem() {
        super(GameStates.HOLD, new GameStatesConstants());
    }
    
        //state machine transtions (incomplete)
     public void updateLogic(double timestamp) {
        switch (system_state_) { 
            case HOLD:
            break;
            case GOAL:
            break;
            case PASS:
            break;
            case AUTO_CLIMB:
            break;
            case TELOP_CLIMB:
            break;
            case DOWN_CLIMB:
            break;
        }
    }

    public void handleStateTransition(GameStates wanted) {
        // transtions out of TELO_CLIMB, no transtions
        if (system_state_ == GameStates.TELOP_CLIMB) {
            system_state_ = GameStates.TELOP_CLIMB;
            return;
        } 
        //hold transitions
        if (system_state_ == GameStates.HOLD) {
            system_state_ = GameStates.GOAL;
        }
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
