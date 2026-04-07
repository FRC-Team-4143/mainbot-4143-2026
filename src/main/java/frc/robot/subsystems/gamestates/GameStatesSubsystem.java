package frc.robot.subsystems.gamestates;

import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.lib2026.FieldConstants;
import frc.robot.lib2026.FieldRegions;
import frc.robot.lib2026.FieldTargets;
import frc.robot.subsystems.gamestates.GameStatesConstants.GameStates;
import frc.robot.subsystems.hopper.HopperConstants.HopperStates;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.localization.LocalizationConstants.LocalizationStates;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;
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
        if (wanted == GameStates.HOLD) {
            system_state_ = GameStates.HOLD;
        } else if (wanted == GameStates.SHOOT) {
            system_state_ = GameStates.SHOOT;
        } else {
            // empty to not interfere with rest of state machine
        }
        // SHOOT transition
        if (system_state_ == GameStates.SHOOT && isInAllianceZone(robotpose)) {
            system_state_ = GameStates.SCORE;
        } else if (system_state_ == GameStates.SHOOT && isPassZone(robotpose)) {
            system_state_ = GameStates.PASS;
        } else {
            // empty to not interfere with rest of state machine
        }
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
                if (pose.getY() > FieldConstants.FIELD_CENTER.getY()) {
                    ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.RIGHT_PASS);
                } else {
                    ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.LEFT_PASS);
                }
            } else {
                ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.HUB);
            }
        }

        // States
        switch (system_state_) {
            case HOLD:
                ShooterSubsystem.getInstance().setWantedState(ShooterStates.TRACKING);
                HopperSubsystem.getInstance().setWantedState(HopperStates.IDLE);
                SwerveSubsystem.getInstance().setTeleOpVelocityScalar(1.0);
                SwerveSubsystem.getInstance().setWantedState(SwerveStates.FIELD_CENTRIC);
                LocalizationSubsystem.getInstance().setWantedState(LocalizationStates.FULL);
                break;
            case SCORE:
                ShooterSubsystem.getInstance().setWantedState(ShooterStates.SHOOT);
                HopperSubsystem.getInstance().setWantedState(HopperStates.SHOOTING);
                SwerveSubsystem.getInstance().setTeleOpVelocityScalar(0.25);
                LocalizationSubsystem.getInstance()
                        .setWantedState(LocalizationStates.SHOOTING_FOCUS);
                if (SwerveSubsystem.getInstance().isChassisStationary()
                        && ShooterSubsystem.getInstance().isShooterReady()) {
                    SwerveSubsystem.getInstance().setWantedState(SwerveStates.BRAKE);
                } else {
                    SwerveSubsystem.getInstance()
                            .setWantedState(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
                }
                break;
            case PASS:
                ShooterSubsystem.getInstance().setWantedState(ShooterStates.SHOOT);
                HopperSubsystem.getInstance().setWantedState(HopperStates.SHOOTING);
                SwerveSubsystem.getInstance().setTeleOpVelocityScalar(0.25);
                LocalizationSubsystem.getInstance()
                        .setWantedState(LocalizationStates.SHOOTING_FOCUS);
                if (SwerveSubsystem.getInstance().isChassisStationary()
                        && ShooterSubsystem.getInstance().isShooterReady()) {
                    SwerveSubsystem.getInstance().setWantedState(SwerveStates.BRAKE);
                } else {
                    SwerveSubsystem.getInstance()
                            .setWantedState(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
                }
                break;
        }
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
