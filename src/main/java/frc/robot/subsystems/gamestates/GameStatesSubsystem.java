package frc.robot.subsystems.gamestates;

import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.lib2026.FieldRegions;
import frc.robot.lib2026.FieldTargets;
import frc.robot.subsystems.gamestates.GameStatesConstants.GameStates;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.ArrayList;
import java.util.List;

public class GameStatesSubsystem extends MwSubsystem<GameStates, GameStatesConstants> {
    private static GameStatesSubsystem instance_ = null;

    // Variables, temporary
    boolean goal_active_ = false;
    boolean operator_presses_climb_button_ = false;
    boolean full_load_ = false;
    boolean pass_overide_ = false;
    boolean auto_climb_ready_ = false;

    // getInstance
    public static GameStatesSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new GameStatesSubsystem();
        }
        return instance_;
    }

    // Constructor
    private GameStatesSubsystem() {
        super(GameStates.HOLD, new GameStatesConstants());
    }

    // reset
    @Override
    public void reset() {
        system_state_ = GameStates.HOLD;
    }

    // getIos
    @Override
    public List<SubsystemIoBase> getIos() {
        return new ArrayList<SubsystemIoBase>();
    }

    // handleStateTransition
    @Override
    public void handleStateTransition(GameStates wanted) {

        if (RobotState.isAutonomous()) {
            system_state_ = GameStates.AUTO;
            return;
        }

        // Transition out of AUTO when teleop starts
        if (system_state_ == GameStates.AUTO) {
            system_state_ = GameStates.HOLD;
        }

        Pose2d robotpose = LocalizationSubsystem.getInstance().getFieldPose();
        // transtions out of TELEOP_CLIMB, no transtions
        if (system_state_ == GameStates.TELEOP_CLIMB) {
            system_state_ = GameStates.TELEOP_CLIMB;
            return;
        }
        // HOLD transitions
        if (system_state_ == GameStates.HOLD && isInAllianceZone(robotpose) && goal_active_) {
            system_state_ = GameStates.SCORE;
        } else if (system_state_ == GameStates.HOLD
                && isPassZone(robotpose)
                && !pass_overide_
                && !isInHoldZone(robotpose)) {
            system_state_ = GameStates.PASS;
            // Set strict tolerances for scoring
            ShooterSubsystem.getInstance()
                    .setShootingTolerances(
                            FieldTargets.Shooter.FLYWHEEL_SPEED_TOLERANCE,
                            FieldTargets.Shooter.HOOD_POSITION_TOLERANCE,
                            FieldTargets.Shooter.ROTATION_ANGLE_TOLERANCE);
        } else if (system_state_ == GameStates.HOLD && (isPassZone(robotpose) || pass_overide_)) {
            system_state_ = GameStates.PASS;
        } else if (system_state_ == GameStates.HOLD && operator_presses_climb_button_) {
            system_state_ = GameStates.TELEOP_CLIMB;
        } else {
        } // empty to not interfere with rest of state machine
        // SCORE transistions
        if (system_state_ == GameStates.SCORE && (isPassZone(robotpose) || !goal_active_)) {
            system_state_ = GameStates.HOLD;
        } else if (system_state_ == GameStates.SCORE && operator_presses_climb_button_) {
            system_state_ = GameStates.TELEOP_CLIMB;
        } else {
        } // empty to not interfere with rest of state machine
        // PASS transistions
        if (system_state_ == GameStates.PASS
                && (isInHoldZone(robotpose) || isInAllianceZone(robotpose) || pass_overide_)) {
            system_state_ = GameStates.HOLD;
            // Return to strict tolerances when leaving PASS state
            ShooterSubsystem.getInstance()
                    .setShootingTolerances(
                            FieldTargets.Shooter.FLYWHEEL_SPEED_TOLERANCE,
                            FieldTargets.Shooter.HOOD_POSITION_TOLERANCE,
                            FieldTargets.Shooter.ROTATION_ANGLE_TOLERANCE);
        } else {
        } // empty to not interfere with rest of state machine
        // DOWN_CLIMB transistions
        if (system_state_ == GameStates.DOWN_CLIMB && isDownClimbFinished()) {
            system_state_ = GameStates.HOLD;
        } else {
        } // empty to not interfere with rest of state machine
    }

    // updateLogic
    @Override
    public void updateLogic(double timestamp) {

        // GSM does nothing in auto mode
        if (RobotState.isAutonomous()) {
            return;
        }

        Pose2d pose = LocalizationSubsystem.getInstance().getFieldPose();

        if (FieldRegions.LEFT_PASS_REGION.contains(pose)) {
            ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.LEFT_PASS);
        } else if (FieldRegions.RIGHT_PASS_REGION.contains(pose)) {
            ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.RIGHT_PASS);
        } else if (FieldRegions.ALLIANCE_ZONE.contains(pose)) {
            // If operator is holding the dead-man pass override, allow selecting a pass
            // target even while inside the alliance zone. Choose left/right pass by Y
            // position (field is split roughly at y=4.021). Otherwise default to HUB.
            if (pass_overide_) {
                double passSplitY = 4.021; // matches FieldRegions pass region split
                if (pose.getY() > passSplitY) {
                    ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.LEFT_PASS);
                } else {
                    ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.RIGHT_PASS);
                }
            } else {
                ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.HUB);
            }
        }
        //  else if (FieldRegions.HOLD_REGIONS.contains(
        //         LocalizationSubsystem.getInstance().getFieldPose())) {
        //     ShooterSubsystem.getInstance().setWantedState(ShooterStates.SHOOT_WAIT);
        // }
        // switch (system_state_) {
        //     case HOLD:
        //         ShooterSubsystem.getInstance().setWantedState(ShooterStates.IDLE);
        //         IntakeSubsystem.getInstance().setWantedState(IntakeStates.INTAKE);
        //         HopperSubsystem.getInstance().setWantedState(HopperStates.SHOOTING);
        //         ClimberSubsystem.getInstance().setWantedState(ClimberStates.STOWED);
        //         break;
        //     case SCORE:
        //         ShooterSubsystem.getInstance().setWantedState(ShooterStates.SHOOT_WAIT);
        //         IntakeSubsystem.getInstance().setWantedState(IntakeStates.INTAKE);
        //         HopperSubsystem.getInstance().setWantedState(HopperStates.SHOOTING);
        //         ClimberSubsystem.getInstance().setWantedState(ClimberStates.STOWED);
        //         break;
        //     case PASS:
        //         ShooterSubsystem.getInstance().setWantedState(ShooterStates.SHOOT_WAIT);
        //         IntakeSubsystem.getInstance().setWantedState(IntakeStates.INTAKE);
        //         HopperSubsystem.getInstance().setWantedState(HopperStates.SHOOTING);
        //         ClimberSubsystem.getInstance().setWantedState(ClimberStates.STOWED);
        //         break;
        //     case TELEOP_CLIMB:
        //         ShooterSubsystem.getInstance().setWantedState(ShooterStates.IDLE);
        //         HopperSubsystem.getInstance().setWantedState(HopperStates.IDLE);
        //         IntakeSubsystem.getInstance().setWantedState(IntakeStates.STORE);
        //         ClimberSubsystem.getInstance().setWantedState(ClimberStates.L3_CLIMB);
        //         break;
        //     case DOWN_CLIMB:
        //         ShooterSubsystem.getInstance().setWantedState(ShooterStates.IDLE);
        //         HopperSubsystem.getInstance().setWantedState(HopperStates.IDLE);
        //         IntakeSubsystem.getInstance().setWantedState(IntakeStates.STORE);
        //         ClimberSubsystem.getInstance().setWantedState(ClimberStates.L1_DOWN);
        //         break;
        //     case AUTO:
        //         // GSM does nothing in auto mode
        //         break;
        // }
    }

    // =============================================================================
    // PRIVATE HELPER METHODS
    // =============================================================================

    private boolean isInAllianceZone(Pose2d pose) {
        return FieldRegions.ALLIANCE_ZONE.contains(pose);
    }

    private boolean isPassZone(Pose2d pose) {
        return FieldRegions.NEUTRAL_ZONE.contains(pose)
                || FieldRegions.OPP_ALLIANCE_ZONE.contains(pose);
    }

    private boolean isInHoldZone(Pose2d pose) {
        boolean in_zone = false;
        for (int i = 0; i < FieldRegions.HOLD_REGIONS.size(); i++) {
            in_zone = (in_zone || FieldRegions.HOLD_REGIONS.get(i).contains(pose));
        }
        return in_zone;
    }

    private boolean isDownClimbFinished() {
        return false;
    }

    // Public API for pass override (dead-man switch)
    public void setPassOverride(boolean active) {
        pass_overide_ = active;
    }

    public boolean getPassOverride() {
        return pass_overide_;
    }
}
