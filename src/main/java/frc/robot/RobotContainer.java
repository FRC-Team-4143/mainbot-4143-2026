// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.marswars.logging.MwLog;
import com.marswars.subsystem.MwSuperStateMachine;
import com.marswars.swerve_lib.SwerveStates;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
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
 *   <li>{@link #handleStateTransition}: every decision. It works out the scoring mode, the shooter
 *       target, the driving mode and where the intake rests, and picks the robot state (SHOOT is
 *       split into step states, chosen from whether the shooter is ready and the robot is moving).
 *   <li>{@link #updateLogic}: no decisions. A switch on the robot state that asks each subsystem
 *       for the state it needs, every loop.
 * </ul>
 *
 * <p>Maintenance (homing, tuning) lives outside it: in Test mode the robot is in MAINTENANCE and
 * asks for nothing, so the dashboard buttons set subsystem states directly.
 */
public class RobotContainer
        extends MwSuperStateMachine<RobotContainer.RobotStates, RobotConstants> {
    /**
     * The robot's super state machine states, owned by {@link RobotContainer}. Buttons and autos
     * ask for one of these with {@code RobotContainer.getInstance().setWantedState(...)}; every
     * loop the container asks each subsystem for the state the robot's state needs.
     */
    public enum RobotStates {
        /** Disabled: shooter idle */
        IDLE,
        /** Teleop driving with no button held, shooter idle */
        DRIVE,
        /** Driving with the flywheel kept at the target speed, so the next shot is quick */
        TRACK,
        /** Autonomous: the auto sets subsystem states itself, so the robot asks for nothing */
        AUTO,
        /** Aim at the target (shooter aiming, swerve rotation lock) without shooting */
        AIM,
        /**
         * Shoot (what buttons and autos ask for). The robot is never in SHOOT itself:
         * handleStateTransition picks one of the SHOOT_ step states below every loop. It keeps
         * shooting until something asks for another state.
         */
        SHOOT,
        /** Shooting step: spinning up and aiming, not feeding yet */
        SHOOT_WAIT,
        /**
         * Shooting step: ready, feeding fuel (intake squeeze) while driving with the heading locked
         */
        SHOOT_FEED,
        /** Shooting step: ready and stopped, feeding fuel with the wheels crossed */
        SHOOT_FEED_BRAKED,
        /** Shooting step: passing while moving, keep intaking until the robot stops */
        SHOOT_PASS_INTAKE,
        /** Shoot at the hub with a fixed flywheel speed (vision not working) */
        MANUAL_HUB,
        /** Pass with a fixed flywheel speed and hood angle (vision not working) */
        MANUAL_PASS,
        /** Intake fuel */
        INTAKE,
        /** Eject fuel out of the intake */
        OUTTAKE,
        /** Cross the wheels so the robot can't be pushed */
        BRAKE,
        /**
         * Test mode: maintenance (homing, tuning, spin down) from the dashboard. Like AUTO, the
         * robot asks for nothing, so the dashboard buttons set subsystem states directly.
         */
        MAINTENANCE
    }

    /**
     * Where the robot is on the field decides what shooting means. {@link RobotContainer} works
     * this out every loop from the robot's pose.
     */
    public enum ScoringModes {
        /** Holding game pieces, not actively scoring */
        HOLD,
        /** In our alliance zone with the goal active: shoot at the hub */
        SCORE,
        /** In the neutral or opponent zone (or pass override): pass to our side */
        PASS,
        /** Autonomous: the auto picks the target */
        AUTO
    }

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

    // Operator settings
    private boolean goal_active_ = true; // must be true :)
    private boolean pass_override_ = false;
    private boolean is_able_to_rack_ = true;
    private boolean field_centric_ = true;
    private boolean intake_stored_ = false;

    // Decided in handleStateTransition, used in updateLogic
    private ScoringModes scoring_mode_ = ScoringModes.HOLD;
    private Translation3d shooter_target_ = FieldTargets.Shooter.HUB;
    private SwerveStates drive_mode_ = SwerveStates.FIELD_CENTRIC;
    private IntakeStates intake_rest_ = IntakeStates.DEPLOYED;

    public RobotContainer() {
        super(RobotStates.IDLE, new RobotConstants(), BuildConstants.class);
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
        // Where the robot is decides the scoring mode and what the shooter aims at
        Pose2d pose = localization_.getFieldPose();
        updateScoringMode(pose);
        updateShooterTarget(pose);

        // Operator settings
        if (field_centric_) {
            drive_mode_ = SwerveStates.FIELD_CENTRIC;
        } else {
            drive_mode_ = SwerveStates.ROBOT_CENTRIC;
        }
        if (intake_stored_) {
            intake_rest_ = IntakeStates.STORE;
        } else {
            intake_rest_ = IntakeStates.DEPLOYED;
        }

        // The robot state
        if (wanted == RobotStates.SHOOT) {
            system_state_ = shootStep();
        } else {
            system_state_ = wanted;
        }
    }

    /**
     * Which shooting step the robot is in. Feed only when the shooter is ready (and the operator
     * allows racking); when passing, only feed while stopped and keep intaking while moving; cross
     * the wheels once stopped and feeding.
     */
    private RobotStates shootStep() {
        boolean ready = shooter_.isShooterReady() && is_able_to_rack_;
        boolean stationary = swerve_.isChassisStationary();
        boolean scoring = scoring_mode_ == ScoringModes.SCORE || scoring_mode_ == ScoringModes.AUTO;
        boolean passing = scoring_mode_ == ScoringModes.PASS;

        if (ready && stationary && (scoring || passing)) {
            return RobotStates.SHOOT_FEED_BRAKED;
        } else if (ready && scoring) {
            return RobotStates.SHOOT_FEED;
        } else if (passing && !stationary) {
            return RobotStates.SHOOT_PASS_INTAKE;
        } else {
            return RobotStates.SHOOT_WAIT;
        }
    }

    @Override
    protected void updateLogic(double timestamp) {
        MwLog.log(LOG_KEY + "ScoringMode", scoring_mode_);

        switch (system_state_) {
            case IDLE, DRIVE -> {
                swerve_.setWantedState(drive_mode_);
                swerve_.setTeleOpVelocityScalar(CONSTANTS.DRIVE_SPEED_SCALE);
                shooter_.setWantedState(ShooterStates.IDLE);
                shooter_.setTarget(shooter_target_);
                intake_.setWantedState(intake_rest_);
                localization_.setWantedState(LocalizationStates.FULL);
            }
            case TRACK -> {
                swerve_.setWantedState(drive_mode_);
                swerve_.setTeleOpVelocityScalar(CONSTANTS.DRIVE_SPEED_SCALE);
                shooter_.setWantedState(ShooterStates.TRACKING);
                shooter_.setTarget(shooter_target_);
                intake_.setWantedState(intake_rest_);
                localization_.setWantedState(LocalizationStates.FULL);
            }
            case AIM -> {
                swerve_.setWantedState(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
                swerve_.setTeleOpVelocityScalar(CONSTANTS.DRIVE_SPEED_SCALE);
                shooter_.setWantedState(ShooterStates.AIMING);
                shooter_.setTarget(shooter_target_);
                intake_.setWantedState(intake_rest_);
                localization_.setWantedState(LocalizationStates.SHOOTING_FOCUS);
            }
            case SHOOT_WAIT -> {
                swerve_.setWantedState(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
                swerve_.setTeleOpVelocityScalar(CONSTANTS.SHOOTING_SPEED_SCALE);
                shooter_.setWantedState(ShooterStates.SHOOT);
                shooter_.setTarget(shooter_target_);
                // the intake stays where it is until the shooter is ready
                localization_.setWantedState(LocalizationStates.SHOOTING_FOCUS);
            }
            case SHOOT_FEED -> {
                swerve_.setWantedState(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
                swerve_.setTeleOpVelocityScalar(CONSTANTS.SHOOTING_SPEED_SCALE);
                shooter_.setWantedState(ShooterStates.SHOOT);
                shooter_.setTarget(shooter_target_);
                intake_.setWantedState(IntakeStates.SQUEEZE);
                localization_.setWantedState(LocalizationStates.SHOOTING_FOCUS);
            }
            case SHOOT_FEED_BRAKED -> {
                swerve_.setWantedState(SwerveStates.BRAKE);
                swerve_.setTeleOpVelocityScalar(CONSTANTS.SHOOTING_SPEED_SCALE);
                shooter_.setWantedState(ShooterStates.SHOOT);
                shooter_.setTarget(shooter_target_);
                intake_.setWantedState(IntakeStates.SQUEEZE);
                localization_.setWantedState(LocalizationStates.SHOOTING_FOCUS);
            }
            case SHOOT_PASS_INTAKE -> {
                swerve_.setWantedState(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
                swerve_.setTeleOpVelocityScalar(CONSTANTS.SHOOTING_SPEED_SCALE);
                shooter_.setWantedState(ShooterStates.SHOOT);
                shooter_.setTarget(shooter_target_);
                intake_.setWantedState(IntakeStates.INTAKE);
                localization_.setWantedState(LocalizationStates.SHOOTING_FOCUS);
            }
            case MANUAL_HUB -> {
                swerve_.setWantedState(drive_mode_);
                swerve_.setTeleOpVelocityScalar(CONSTANTS.DRIVE_SPEED_SCALE);
                shooter_.setWantedState(ShooterStates.MANUAL_HUB);
                intake_.setWantedState(IntakeStates.SQUEEZE);
                localization_.setWantedState(LocalizationStates.FULL);
            }
            case MANUAL_PASS -> {
                swerve_.setWantedState(drive_mode_);
                swerve_.setTeleOpVelocityScalar(CONSTANTS.DRIVE_SPEED_SCALE);
                shooter_.setWantedState(ShooterStates.MANUAL_PASS);
                intake_.setWantedState(IntakeStates.SQUEEZE);
                localization_.setWantedState(LocalizationStates.FULL);
            }
            case INTAKE -> {
                swerve_.setWantedState(drive_mode_);
                swerve_.setTeleOpVelocityScalar(CONSTANTS.DRIVE_SPEED_SCALE);
                shooter_.setWantedState(ShooterStates.IDLE);
                intake_.setWantedState(IntakeStates.INTAKE);
                localization_.setWantedState(LocalizationStates.FULL);
            }
            case OUTTAKE -> {
                swerve_.setWantedState(drive_mode_);
                swerve_.setTeleOpVelocityScalar(CONSTANTS.DRIVE_SPEED_SCALE);
                shooter_.setWantedState(ShooterStates.IDLE);
                intake_.setWantedState(IntakeStates.OUTTAKE);
                localization_.setWantedState(LocalizationStates.FULL);
            }
            case BRAKE -> {
                swerve_.setWantedState(SwerveStates.BRAKE);
                shooter_.setWantedState(ShooterStates.IDLE);
                intake_.setWantedState(intake_rest_);
                localization_.setWantedState(LocalizationStates.FULL);
            }
            case AUTO, MAINTENANCE -> {
                // Hands off: the auto (FollowTrajectory, SetIntakeState, ...) or the maintenance
                // dashboard buttons (homing, tuning) set subsystem states themselves.
            }
            case SHOOT -> {
                // Never the robot's state: handleStateTransition picks a SHOOT_ step instead.
            }
        }
    }

    /** HOLD / SCORE / PASS from where the robot is on the field. */
    private void updateScoringMode(Pose2d pose) {
        if (RobotState.isAutonomous()) {
            scoring_mode_ = ScoringModes.AUTO;
            return;
        }
        // Transition out of AUTO when teleop starts
        if (scoring_mode_ == ScoringModes.AUTO) {
            scoring_mode_ = ScoringModes.HOLD;
        }

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

    /** The hub, or a pass target, from the robot's pose. Autos pick their own target. */
    private void updateShooterTarget(Pose2d pose) {
        if (scoring_mode_ == ScoringModes.AUTO) {
            return;
        }

        if (FieldRegions.LEFT_PASS_REGION.contains(pose)) {
            shooter_target_ = FieldTargets.Shooter.LEFT_PASS;
        } else if (FieldRegions.RIGHT_PASS_REGION.contains(pose)) {
            shooter_target_ = FieldTargets.Shooter.RIGHT_PASS;
        } else if (FieldRegions.ALLIANCE_ZONE.contains(pose)) {
            // If operator is holding the dead-man pass override, allow selecting a pass
            // target even while inside the alliance zone. Choose left/right pass by Y
            // position (field is split roughly at y=4.021). Otherwise default to HUB.
            if (pass_override_) {
                if (pose.getY() > FieldConstants.FIELD_CENTER.getY()) {
                    shooter_target_ = FieldTargets.Shooter.RIGHT_PASS;
                } else {
                    shooter_target_ = FieldTargets.Shooter.LEFT_PASS;
                }
            } else {
                shooter_target_ = FieldTargets.Shooter.HUB;
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

    /** Autos: aim the shooter at {@code target} (in autonomous the pose doesn't pick it). */
    public void setAutoShooterTarget(Translation3d target) {
        shooter_target_ = target;
        shooter_.setTarget(target);
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
