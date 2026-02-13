package frc.robot.subsystems.gamestates;

import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.lib2026.FieldRegions;
import frc.robot.subsystems.climber.ClimberConstants.ClimberStates;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.gamestates.GameStatesConstants.GameStates;
import frc.robot.subsystems.hopper.HopperConstants.HopperStates;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
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
                ShooterSubsystem.getInstance().setWantedState(ShooterStates.IDLE);
                IntakeSubsystem.getInstance().setWantedState(IntakeStates.ROLLING);
                HopperSubsystem.getInstance().setWantedState(HopperStates.SHOOTING);
                ClimberSubsystem.getInstance().setWantedState(ClimberStates.STOWED);
                break;
            case SCORE:
                ShooterSubsystem.getInstance().setWantedState(ShooterStates.AIMING);
                IntakeSubsystem.getInstance().setWantedState(IntakeStates.ROLLING);
                HopperSubsystem.getInstance().setWantedState(HopperStates.SHOOTING);
                ClimberSubsystem.getInstance().setWantedState(ClimberStates.STOWED);
                break;
            case PASS:
                ShooterSubsystem.getInstance().setWantedState(ShooterStates.AIMING);
                IntakeSubsystem.getInstance().setWantedState(IntakeStates.ROLLING);
                HopperSubsystem.getInstance().setWantedState(HopperStates.SHOOTING);
                ClimberSubsystem.getInstance().setWantedState(ClimberStates.STOWED);
                break;
            case AUTO_CLIMB:
                ShooterSubsystem.getInstance().setWantedState(ShooterStates.IDLE);
                HopperSubsystem.getInstance().setWantedState(HopperStates.IDLE);
                IntakeSubsystem.getInstance().setWantedState(IntakeStates.CLOSED);
                ClimberSubsystem.getInstance().setWantedState(ClimberStates.L1_CLIMB);
                break;
            case TELEOP_CLIMB:
                ShooterSubsystem.getInstance().setWantedState(ShooterStates.IDLE);
                HopperSubsystem.getInstance().setWantedState(HopperStates.IDLE);
                IntakeSubsystem.getInstance().setWantedState(IntakeStates.CLOSED);
                ClimberSubsystem.getInstance().setWantedState(ClimberStates.L3_CLIMB);
                break;
            case DOWN_CLIMB:
                ShooterSubsystem.getInstance().setWantedState(ShooterStates.IDLE);
                HopperSubsystem.getInstance().setWantedState(HopperStates.IDLE);
                IntakeSubsystem.getInstance().setWantedState(IntakeStates.CLOSED);
                ClimberSubsystem.getInstance().setWantedState(ClimberStates.L1_DOWN);
                break;
        }
    }

    @Override
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
            System.out.println("hold to score");
        } else if (system_state_ == GameStates.HOLD
                && isPassZone(robotpose)
                && !pass_overide_
                && !inHoldZone(robotpose)) {
            system_state_ = GameStates.PASS;
            System.out.println("hold to pass");
        } else if (system_state_ == GameStates.HOLD && auto_climb_ready_) {
            system_state_ = GameStates.AUTO_CLIMB;
        } else if (system_state_ == GameStates.HOLD && operator_presses_climb_button_) {
            system_state_ = GameStates.TELEOP_CLIMB;
        } else {
        } // empty to not interfere with rest of state machine
        // SCORE transistions
        if (system_state_ == GameStates.SCORE && (isPassZone(robotpose) || !goal_active_)) {
            system_state_ = GameStates.HOLD;
            System.out.println("score to hold");
        } else if (system_state_ == GameStates.SCORE && auto_climb_ready_) {
            system_state_ = GameStates.AUTO_CLIMB;
        } else if (system_state_ == GameStates.SCORE && operator_presses_climb_button_) {
            system_state_ = GameStates.TELEOP_CLIMB;
        } else {
        } // empty to not interfere with rest of state machine
        // PASS transistions
        if (system_state_ == GameStates.PASS
                && (inHoldZone(robotpose) || inAllianceZone(robotpose) || pass_overide_)) {
            system_state_ = GameStates.HOLD;
            System.out.println("pass to hold");
            // if (system_state_ == GameStates.PASS && inHoldZone(robotpose)) {
            //    system_state_ = GameStates.HOLD;
            //    System.out.println("pass to hold, inHoldZone");
            // } else if (system_state_ == GameStates.PASS && inAllianceZone(robotpose)) {
            //    system_state_ = GameStates.HOLD;
            //    System.out.println("pass to hold, inAllianceZone");
            // } else if (system_state_ == GameStates.PASS && pass_overide_) {
            //    system_state_ = GameStates.HOLD;
            //    System.out.println("pass to hold, pass_overide_");
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
        return false;
    }

    // for testing only

    // set goal to active
    public void GoalActive() {
        goal_active_ = true;
        return;
    }

    // sets goal to inactive
    public void GoalInactive() {
        goal_active_ = false;
        return;
    }
}
