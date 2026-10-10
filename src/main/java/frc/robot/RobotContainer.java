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
 * setWantedState(...)}. It works like a subsystem's state machine, and runs every loop before the
 * subsystems:
 *
 * <ul>
 *   <li>{@link #handleStateTransition}: the default, straight to the wanted state.
 *   <li>{@link #updateLogic}: a switch on the robot state that asks each subsystem for the state it
 *       needs, every loop. It also works out the scoring mode from the robot's pose and picks the
 *       shooter target.
 * </ul>
 *
 * <p>Maintenance (homing, tuning) lives outside it: in Test mode the robot is in MAINTENANCE and
 * asks for nothing, so the dashboard buttons set subsystem states directly.
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
    private boolean field_centric_ = true;
    private boolean intake_stored_ = false;

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
    protected void updateLogic(double timestamp) {
        updateScoringMode();
        MwLog.log(LOG_KEY + "ScoringMode", scoring_mode_);
        if (scoring_mode_ != ScoringModes.AUTO) {
            selectShooterTarget();
        }

        switch (system_state_) {
            case IDLE, DRIVE -> {
                swerve_.setWantedState(driveMode());
                swerve_.setTeleOpVelocityScalar(1.0);
                shooter_.setWantedState(ShooterStates.IDLE);
                intake_.setWantedState(intakeRest());
                localization_.setWantedState(LocalizationStates.FULL);
            }
            case TRACK -> {
                swerve_.setWantedState(driveMode());
                swerve_.setTeleOpVelocityScalar(1.0);
                shooter_.setWantedState(ShooterStates.TRACKING);
                intake_.setWantedState(intakeRest());
                localization_.setWantedState(LocalizationStates.FULL);
            }
            case AIM -> {
                swerve_.setWantedState(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
                swerve_.setTeleOpVelocityScalar(1.0);
                shooter_.setWantedState(ShooterStates.AIMING);
                intake_.setWantedState(intakeRest());
                localization_.setWantedState(LocalizationStates.SHOOTING_FOCUS);
            }
            case SHOOT -> {
                swerve_.setWantedState(
                        swerve_.isChassisStationary() && shooter_.isShooterReady()
                                ? SwerveStates.BRAKE
                                : SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
                swerve_.setTeleOpVelocityScalar(0.25);
                shooter_.setWantedState(ShooterStates.SHOOT);
                feedWhenReady();
                localization_.setWantedState(LocalizationStates.SHOOTING_FOCUS);
            }
            case MANUAL_HUB -> {
                swerve_.setWantedState(driveMode());
                swerve_.setTeleOpVelocityScalar(1.0);
                shooter_.setWantedState(ShooterStates.MANUAL_HUB);
                intake_.setWantedState(IntakeStates.SQUEEZE);
                localization_.setWantedState(LocalizationStates.FULL);
            }
            case MANUAL_PASS -> {
                swerve_.setWantedState(driveMode());
                swerve_.setTeleOpVelocityScalar(1.0);
                shooter_.setWantedState(ShooterStates.MANUAL_PASS);
                intake_.setWantedState(IntakeStates.SQUEEZE);
                localization_.setWantedState(LocalizationStates.FULL);
            }
            case INTAKE -> {
                swerve_.setWantedState(driveMode());
                swerve_.setTeleOpVelocityScalar(1.0);
                shooter_.setWantedState(ShooterStates.IDLE);
                intake_.setWantedState(IntakeStates.INTAKE);
                localization_.setWantedState(LocalizationStates.FULL);
            }
            case OUTTAKE -> {
                swerve_.setWantedState(driveMode());
                swerve_.setTeleOpVelocityScalar(1.0);
                shooter_.setWantedState(ShooterStates.IDLE);
                intake_.setWantedState(IntakeStates.OUTTAKE);
                localization_.setWantedState(LocalizationStates.FULL);
            }
            case BRAKE -> {
                swerve_.setWantedState(SwerveStates.BRAKE);
                shooter_.setWantedState(ShooterStates.IDLE);
                intake_.setWantedState(intakeRest());
                localization_.setWantedState(LocalizationStates.FULL);
            }
            case AUTO, MAINTENANCE -> {
                // Hands off: the auto (FollowTrajectory, SetIntakeState, ...) or the maintenance
                // dashboard buttons (homing, tuning) set subsystem states themselves.
            }
        }
    }

    /** SHOOT: feed fuel into the shooter only when it is ready (and the scoring mode allows). */
    private void feedWhenReady() {
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
            // hold - leave the intake where it is
        }
    }

    /** Field centric or robot centric driving, toggled by the driver. */
    private SwerveStates driveMode() {
        return field_centric_ ? SwerveStates.FIELD_CENTRIC : SwerveStates.ROBOT_CENTRIC;
    }

    /** Where the intake sits when it isn't doing anything: stowed or deployed. */
    private IntakeStates intakeRest() {
        return intake_stored_ ? IntakeStates.STORE : IntakeStates.DEPLOYED;
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
        intake_stored_ = !intake_stored_;
    }

    /** Switch between field centric and robot centric driving. */
    public void toggleFieldCentric() {
        field_centric_ = !field_centric_;
        MwLog.log(LOG_KEY + "FieldCentric", field_centric_);
    }
}
