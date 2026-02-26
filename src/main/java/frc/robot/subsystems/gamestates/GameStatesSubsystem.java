package frc.robot.subsystems.gamestates;

import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.lib2026.FieldRegions;
import frc.robot.lib2026.FieldTargets;
import frc.robot.subsystems.gamestates.GameStatesConstants.GameStates;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.ArrayList;
import java.util.List;

public class GameStatesSubsystem extends MwSubsystem<GameStates, GameStatesConstants> {
     private static GameStatesSubsystem instance_ = null;

    public static GameStatesSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new GameStatesSubsystem();
        }
        return instance_;
    }

    // Variables, temporary
    // prefer primitives to avoid accidental nulls from boxed Boolean
    private boolean goal_active_ = false;
    private boolean is_climb_time_ = false;
    private boolean auto_climb_ready_ = true;

    public GameStatesSubsystem() {
        super(GameStates.HOLD, new GameStatesConstants());
    }

    // state machine transtions (incomplete)
    public void updateLogic(double timestamp) {

        // After high arc editing capabilities are added to MWLib, adjust the method in shooter
        // subsystem then add here.

        if (FieldRegions.LEFT_PASS_REGION.contains(
                LocalizationSubsystem.getInstance().getFieldPose())) {
            ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.LEFT_PASS);
        } else if (FieldRegions.RIGHT_PASS_REGION.contains(
                LocalizationSubsystem.getInstance().getFieldPose())) {
            ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.RIGHT_PASS);
        } else if (FieldRegions.ALLIANCE_ZONE.contains(
                LocalizationSubsystem.getInstance().getFieldPose())) {
            ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.HUB);
        } else if (FieldRegions.HOLD_REGIONS.contains(
                LocalizationSubsystem.getInstance().getFieldPose())) {
            ShooterSubsystem.getInstance().setWantedState(ShooterStates.AIMING);
        }
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
        } else if (system_state_ == GameStates.HOLD && (isPassZone(robotpose) )) {
            system_state_ = GameStates.PASS;
        } else if (system_state_ == GameStates.HOLD && auto_climb_ready_) {
            system_state_ = GameStates.AUTO_CLIMB;
        } else if (system_state_ == GameStates.HOLD && is_climb_time_) {
            system_state_ = GameStates.TELEOP_CLIMB;
        } else {
        } // empty to not interfere with rest of state machine
        // SCORE transistions
        if (system_state_ == GameStates.SCORE && (isPassZone(robotpose) || !goal_active_)) {
            system_state_ = GameStates.HOLD;
        } else if (system_state_ == GameStates.SCORE && auto_climb_ready_) {
            system_state_ = GameStates.AUTO_CLIMB;
        } else if (system_state_ == GameStates.SCORE && is_climb_time_) {
            system_state_ = GameStates.TELEOP_CLIMB;
        } else {
        } // empty to not interfere with rest of state machine
        // PASS transistions
        if (system_state_ == GameStates.PASS && (inHoldZone(robotpose))) {
            system_state_ = GameStates.HOLD;
        } else {
        } // empty to not interfere with rest of state machine
        // AUTO_CLIMB transitions
        if (system_state_ == GameStates.AUTO_CLIMB && RobotState.isTeleop()) {
            system_state_ = GameStates.DOWN_CLIMB;
        } else {
        } // empty to not interfere with rest of state machine
        // DOWN_CLIMB transistions
        if (system_state_ == GameStates.DOWN_CLIMB && isDownClimbFinished()) {
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

    private boolean isPassZone(Pose2d pose) {
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

    private boolean isDownClimbFinished() {
        return true;
    }

    //modify temp variables
    public boolean getGoalActive() {
        return goal_active_;
    }

    public void setGoalActive(boolean trueORfalse) {
        goal_active_ = trueORfalse;
    }

    public boolean getAutoClimbReady() {
        return auto_climb_ready_;
    }

    public void setAutoClimbReady(boolean trueORfalse) {
        auto_climb_ready_ = trueORfalse;
    }

    public void setIsClimbTime(boolean value){
        is_climb_time_ = value; 
    }
}
