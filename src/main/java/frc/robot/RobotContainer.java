// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.marswars.logging.MwLog;
import com.marswars.subsystem.MwSuperStateMachine;
import com.marswars.swerve_lib.SwerveStates;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.lib2026.FieldConstants;
import frc.robot.lib2026.FieldRegions;
import frc.robot.lib2026.FieldTargets;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.localization.LocalizationConstants.LocalizationStates;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.simulation.SimulationSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * The robot's super state machine. Buttons (OI) and autos ask for a {@link RobotStates} with {@code
 * setWantedState(...)}; this class turns that into subsystem states. Every loop it runs before the
 * subsystems, so what it requests takes effect in the same loop.
 *
 * <ul>
 *   <li>{@link #handleStateTransition}: when the robot state changes, undoes what the old state
 *       asked for (exit) and asks for what the new state needs (enter). Requesting only on a change
 *       leaves subsystems free to be set by dashboard buttons (homing) and autos.
 *   <li>{@link #updateLogic}: works out the scoring mode from the robot's pose, picks the shooter
 *       target, and runs the parts of a state that are checked every loop (SHOOT: feed only when
 *       the shooter is ready).
 * </ul>
 */
public class RobotContainer extends MwSuperStateMachine<RobotStates> {
    private static RobotContainer instance;

    public static synchronized RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    private final SwerveSubsystem swerve_ = SwerveSubsystem.getInstance();
    private final LocalizationSubsystem localization_ = LocalizationSubsystem.getInstance();
    private final ShooterSubsystem shooter_ = ShooterSubsystem.getInstance();
    private final IntakeSubsystem intake_ = IntakeSubsystem.getInstance();

    private ScoringModes scoring_mode_ = ScoringModes.HOLD;
    private boolean goal_active_ = true; // must be true :)
    private boolean pass_override_ = false;
    private boolean is_able_to_rack_ = true;

    public RobotContainer() {
        super(RobotStates.IDLE, BuildConstants.class, RobotId.current().disabled_subsystems);
        // !!!!!! ALL SUBSYSTEMS MUST BE REGISTERED HERE TO RUN !!!!!!!
        registerSubsystem(swerve_);
        registerSubsystem(localization_);
        registerSubsystem(shooter_);
        registerSubsystem(intake_);

        // Only enable the simulation subsystem if we are in simulation (not replay)
        if (RobotBase.isSimulation() && !MwLog.isReplay()) {
            registerSubsystem(SimulationSubsystem.getInstance());
        }

        // !!!!! LEAVE THESE LINES AS THE LAST LINE IN THE CONSTRUCTOR !!!!!!
        reset();
    }

    @Override
    protected void handleStateTransition(RobotStates wanted) {
        if (wanted == system_state_) {
            return;
        }
        RobotStates previous = system_state_;
        exitState(previous);
        system_state_ = wanted;
        enterState(previous, wanted);
    }

    /** Undo what {@code state} asked for, as it is left. */
    private void exitState(RobotStates state) {
        switch (state) {
            case AIM -> {
                shooter_.setWantedState(ShooterStates.TRACKING);
                swerve_.setWantedState(SwerveStates.FIELD_CENTRIC);
                localization_.setWantedState(LocalizationStates.FULL);
            }
            case SHOOT -> {
                shooter_.setWantedState(ShooterStates.IDLE);
                swerve_.setTeleOpVelocityScalar(1.0);
                swerve_.setWantedState(SwerveStates.FIELD_CENTRIC);
                localization_.setWantedState(LocalizationStates.FULL);
                intake_.setWantedState(IntakeStates.DEPLOYED);
            }
            case MANUAL_HUB, MANUAL_PASS -> {
                shooter_.setWantedState(ShooterStates.IDLE);
                intake_.setWantedState(IntakeStates.DEPLOYED);
            }
            case INTAKE, OUTTAKE -> intake_.setWantedState(IntakeStates.DEPLOYED);
            case BRAKE -> swerve_.setWantedState(SwerveStates.FIELD_CENTRIC);
            case TUNING -> {
                shooter_.setWantedState(ShooterStates.IDLE);
                intake_.setWantedState(IntakeStates.DEPLOYED);
            }
            case IDLE, DRIVE, AUTO -> {}
        }
    }

    /** Ask for what {@code state} needs, as it is entered from {@code previous}. */
    private void enterState(RobotStates previous, RobotStates state) {
        switch (state) {
            case IDLE -> shooter_.setWantedState(ShooterStates.IDLE);
            case DRIVE -> {
                // Teleop starting (from disabled or auto): drive field centric, shooter idle.
                // Coming back from an action leaves the drive mode alone (robot centric toggle).
                if (previous == RobotStates.IDLE || previous == RobotStates.AUTO) {
                    swerve_.setWantedState(SwerveStates.FIELD_CENTRIC);
                    shooter_.setWantedState(ShooterStates.IDLE);
                }
            }
            case AIM -> {
                shooter_.setWantedState(ShooterStates.AIMING);
                swerve_.setWantedState(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
                localization_.setWantedState(LocalizationStates.SHOOTING_FOCUS);
            }
            case SHOOT -> {
                shooter_.setWantedState(ShooterStates.SHOOT);
                swerve_.setTeleOpVelocityScalar(0.25);
                localization_.setWantedState(LocalizationStates.SHOOTING_FOCUS);
            }
            case MANUAL_HUB -> {
                shooter_.setWantedState(ShooterStates.MANUAL_HUB);
                intake_.setWantedState(IntakeStates.SQUEEZE);
            }
            case MANUAL_PASS -> {
                shooter_.setWantedState(ShooterStates.MANUAL_PASS);
                intake_.setWantedState(IntakeStates.SQUEEZE);
            }
            case INTAKE -> intake_.setWantedState(IntakeStates.INTAKE);
            case OUTTAKE -> intake_.setWantedState(IntakeStates.OUTTAKE);
            case BRAKE -> swerve_.setWantedState(SwerveStates.BRAKE);
            case TUNING -> {
                swerve_.setWantedState(SwerveStates.FIELD_CENTRIC);
                shooter_.setWantedState(ShooterStates.TUNING);
                intake_.setWantedState(IntakeStates.TUNING);
            }
            case AUTO -> {}
        }
    }

    @Override
    protected void updateLogic(double timestamp) {
        updateScoringMode();
        MwLog.log(LOG_KEY + "ScoringMode", scoring_mode_);
        if (scoring_mode_ != ScoringModes.AUTO) {
            selectShooterTarget();
        }

        if (system_state_ == RobotStates.SHOOT) {
            updateShoot();
        }
    }

    /** SHOOT: feed only when the shooter is ready, and brake once stopped and ready. */
    private void updateShoot() {
        boolean should_squeeze = shooter_.isShooterReady() && is_able_to_rack_;
        boolean stationary = swerve_.isChassisStationary();
        boolean scoring = scoring_mode_ == ScoringModes.SCORE || scoring_mode_ == ScoringModes.AUTO;

        if (scoring && should_squeeze) {
            intake_.setWantedState(IntakeStates.SQUEEZE);
        } else if (scoring_mode_ == ScoringModes.PASS && stationary && should_squeeze) {
            intake_.setWantedState(IntakeStates.SQUEEZE);
        } else if (scoring_mode_ == ScoringModes.PASS && !stationary) {
            intake_.setWantedState(IntakeStates.INTAKE);
        } else {
            // hold - do nothing
        }

        if (stationary && shooter_.isShooterReady()) {
            swerve_.setWantedState(SwerveStates.BRAKE);
        } else {
            swerve_.setWantedState(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
        }
    }

    /** HOLD / SCORE / PASS from where the robot is on the field. */
    private void updateScoringMode() {
        if (RobotState.isAutonomous()) {
            scoring_mode_ = ScoringModes.AUTO;
            return;
        }
        // Transition out of AUTO when teleop starts
        if (scoring_mode_ == ScoringModes.AUTO) {
            scoring_mode_ = ScoringModes.HOLD;
        }

        Pose2d pose = localization_.getFieldPose();
        switch (scoring_mode_) {
            case HOLD -> {
                if (isInAllianceZone(pose) && goal_active_) {
                    scoring_mode_ = ScoringModes.SCORE;
                } else if (isPassZone(pose) && !pass_override_ && !isInHoldZone(pose)) {
                    scoring_mode_ = ScoringModes.PASS;
                    setStrictTolerances();
                } else if (isPassZone(pose) || pass_override_) {
                    scoring_mode_ = ScoringModes.PASS;
                }
            }
            case SCORE -> {
                if (isPassZone(pose) || !goal_active_) {
                    scoring_mode_ = ScoringModes.HOLD;
                }
            }
            case PASS -> {
                if (isInHoldZone(pose) || isInAllianceZone(pose) || pass_override_) {
                    scoring_mode_ = ScoringModes.HOLD;
                    // Return to strict tolerances when leaving PASS
                    setStrictTolerances();
                }
            }
            case AUTO -> {}
        }
    }

    /** Point the shooter at the hub, or at a pass target, from the robot's pose. */
    private void selectShooterTarget() {
        Pose2d pose = localization_.getFieldPose();

        if (FieldRegions.LEFT_PASS_REGION.contains(pose)) {
            shooter_.setTarget(FieldTargets.Shooter.LEFT_PASS);
        } else if (FieldRegions.RIGHT_PASS_REGION.contains(pose)) {
            shooter_.setTarget(FieldTargets.Shooter.RIGHT_PASS);
        } else if (FieldRegions.ALLIANCE_ZONE.contains(pose)) {
            // If operator is holding the dead-man pass override, allow selecting a pass
            // target even while inside the alliance zone. Choose left/right pass by Y
            // position (field is split roughly at y=4.021). Otherwise default to HUB.
            if (pass_override_) {
                if (pose.getY() > FieldConstants.FIELD_CENTER.getY()) {
                    shooter_.setTarget(FieldTargets.Shooter.RIGHT_PASS);
                } else {
                    shooter_.setTarget(FieldTargets.Shooter.LEFT_PASS);
                }
            } else {
                shooter_.setTarget(FieldTargets.Shooter.HUB);
            }
        }
    }

    private void setStrictTolerances() {
        shooter_.setShootingTolerances(
                FieldTargets.Shooter.FLYWHEEL_SPEED_TOLERANCE,
                FieldTargets.Shooter.HOOD_POSITION_TOLERANCE,
                FieldTargets.Shooter.ROTATION_ANGLE_TOLERANCE);
    }

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

    // =============================================================================
    // OPERATOR SETTINGS (not robot states)
    // =============================================================================

    /** Dead-man pass override: while active, pick a pass target even in the alliance zone. */
    public void setPassOverride(boolean active) {
        pass_override_ = active;
    }

    public boolean getPassOverride() {
        return pass_override_;
    }

    public ScoringModes getScoringMode() {
        return scoring_mode_;
    }

    /** Allow or block feeding fuel into the shooter while shooting. */
    public void toggleIsAbleToRack() {
        is_able_to_rack_ = !is_able_to_rack_;
        MwLog.log("Subsystem/Intake/IsAbleToRack", is_able_to_rack_);
    }

    /** Stow the intake, or deploy it if it is stowed. */
    public void toggleStoreIntake() {
        if (intake_.getSystemState() == IntakeStates.STORE) {
            intake_.setWantedState(IntakeStates.DEPLOYED);
        } else {
            intake_.setWantedState(IntakeStates.STORE);
        }
    }
}
