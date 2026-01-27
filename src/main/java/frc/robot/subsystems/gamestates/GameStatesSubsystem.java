package frc.robot.subsystems.gamestates;

import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.lib2026.FieldRegions;
import frc.robot.subsystems.gamestates.GameStatesConstants.GameStates;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import java.util.ArrayList;
import java.util.List;

public class GameStatesSubsystem extends MwSubsystem<GameStates, GameStatesConstants> {

    // Variables, temporary
    Boolean goal_active_ = false;
    Boolean operator_presses_climb_button_ = false;
    Boolean full_load_ = false;
    boolean pass_overide_ = false;
    Boolean auto_climb_ready_ = false;

    public GameStatesSubsystem() {
        super(GameStates.HOLD, new GameStatesConstants());
    }

    // state machine transtions (incomplete)
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case HOLD:
                // shooter innactive
                // hopper holding and accepting balls unless full
                // pickup active to allow ball intake unless full
                // climber inactive
                break;
            case SCORE:
                // shooter active
                // hopper holding and accepting balls unless full
                // pickup active to allow ball intake unless full
                // climber inactive
                break;
            case PASS:
                // shooter active
                // hopper holding and accepting balls unless full
                // pickup active to allow ball intake unless full
                // climber inactive
                break;
            case AUTO_CLIMB:
                // shooter inactive
                // hopper holding balls only
                // pickup innactve
                // climber actively climbing
                break;
            case TELEOP_CLIMB:
                // shooter inactive
                // hopper holding balls only
                // pickup innactve
                // climber actively climbing up
                break;
            case DOWN_CLIMB:
                // shooter inactive
                // hopper holding balls only
                // pickup innactve
                // climber actively climbing down
                break;
        }
    }

    public void handleStateTransition(GameStates wanted) {
        Pose2d robotpose = LocalizationSubsystem.getInstance().getFieldPose();
        // transtions out of TELEOP_CLIMB, no transtions
        if (system_state_ == GameStates.TELEOP_CLIMB) {
            system_state_ = GameStates.TELEOP_CLIMB;
            return;
        }
        // HOLD transitions
        if (system_state_ == GameStates.HOLD && inAllianceZone(robotpose) && goal_active_) {
            system_state_ = GameStates.SCORE;
        } else if (system_state_ == GameStates.HOLD && (passZone(robotpose) || pass_overide_)) {
            system_state_ = GameStates.PASS;
        } else if (system_state_ == GameStates.HOLD && auto_climb_ready_) {
            system_state_ = GameStates.AUTO_CLIMB;
        } else if (system_state_ == GameStates.HOLD && operator_presses_climb_button_) {
            system_state_ = GameStates.TELEOP_CLIMB;
        } else {
        } // empty to not interfere with rest of state machine
        // SCORE transistions
        if (system_state_ == GameStates.SCORE && (passZone(robotpose) || !goal_active_)) {
            system_state_ = GameStates.HOLD;
        } else if (system_state_ == GameStates.SCORE && auto_climb_ready_) {
            system_state_ = GameStates.AUTO_CLIMB;
        } else if (system_state_ == GameStates.SCORE && operator_presses_climb_button_) {
            system_state_ = GameStates.TELEOP_CLIMB;
        } else {
        } // empty to not interfere with rest of state machine
        // PASS transistions
        if (system_state_ == GameStates.PASS && (inHoldZone(robotpose) || !pass_overide_)) {
            system_state_ = GameStates.HOLD;
        } else {
        } // empty to not interfere with rest of state machine
        // AUTO_CLIMB transitions
        if (system_state_ == GameStates.AUTO_CLIMB && isTeleop()) {
            system_state_ = GameStates.DOWN_CLIMB;
        } else {
        } // empty to not interfere with rest of state machine
        // DOWN_CLIMB transistions
        if (system_state_ == GameStates.DOWN_CLIMB && downClimbFinished()) {
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

    // methods to determin wich state is in wich region
    private boolean inAllianceZone(Pose2d pose) {
        return FieldRegions.ALLIANCE_ZONE.contains(pose);
    }

    private boolean passZone(Pose2d pose) {
        return FieldRegions.NEUTRAL_ZONE.contains(pose)
                || FieldRegions.OPP_ALLIANCE_ZONE.contains(pose);
    }

    private boolean inHoldZone(Pose2d pose) {
        boolean in_zone = false;
        for (int i = 0; i < FieldRegions.HOLD_REGIONS.size(); i++) {
            in_zone = (in_zone || FieldRegions.HOLD_REGIONS.get(i).contains(pose));
        }
        return in_zone;
    }

    // condition much be in any hold zone

    private boolean isTeleop() {
        return RobotState.isTeleop();
    }

    private boolean downClimbFinished() {
        return false;
    }
}
