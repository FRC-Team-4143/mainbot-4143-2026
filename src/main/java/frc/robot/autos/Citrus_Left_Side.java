package frc.robot.autos;

import com.marswars.auto.Auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib2026.FieldTargets;
import frc.robot.subsystems.climber.ClimberConstants.ClimberStates;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.hopper.HopperConstants.HopperStates;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Citrus_Left_Side extends Auto {

    public Citrus_Left_Side() {
        // =============================================================================
        // TRAJECTORY LOADING
        // These should be loaded in the order they will be used to ensure correct start poses
        // =============================================================================
        loadTrajectory(ChoreoTraj.CitrusLeftSide.name());
        loadTrajectory(ChoreoTraj.CitrusLeftSideSecondPassNew.name());

        // =============================================================================
        // EVENT TRIGGER BINDING
        // =============================================================================
        SwerveSubsystem.getInstance()
                .getChoreoEventTimeTrigger("Intake Out")
                .onTrue(
                        Commands.runOnce(
                                () ->
                                        IntakeSubsystem.getInstance()
                                                .setWantedState(IntakeStates.INTAKE)));
        SwerveSubsystem.getInstance()
                .getChoreoEventTimeTrigger("Intake In")
                .onTrue(
                        Commands.runOnce(
                                () ->
                                        IntakeSubsystem.getInstance()
                                                .setWantedState(IntakeStates.STORE)));
        SwerveSubsystem.getInstance()
                .getChoreoEventTimeTrigger("Shooting")
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    ShooterSubsystem.getInstance()
                                            .setWantedState(ShooterStates.SHOOT);
                                    HopperSubsystem.getInstance()
                                            .setWantedState(HopperStates.SHOOTING);
                                    SwerveSubsystem.getInstance()
                                            .setWantedState(SwerveStates.CHOREO_PATH_ROTATION_LOCK);
                                }));
        SwerveSubsystem.getInstance()
                .getChoreoEventTimeTrigger("Shooting and Intake Out")
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    ShooterSubsystem.getInstance()
                                            .setWantedState(ShooterStates.SHOOT);
                                    HopperSubsystem.getInstance()
                                            .setWantedState(HopperStates.SHOOTING);
                                    SwerveSubsystem.getInstance()
                                            .setWantedState(SwerveStates.CHOREO_PATH_ROTATION_LOCK);
                                    IntakeSubsystem.getInstance()
                                            .setWantedState(IntakeStates.INTAKE);
                                }));
        SwerveSubsystem.getInstance()
                .getChoreoEventTimeTrigger("Stop Shooting")
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    ShooterSubsystem.getInstance()
                                            .setWantedState(ShooterStates.TRACKING);
                                    HopperSubsystem.getInstance().setWantedState(HopperStates.IDLE);
                                    SwerveSubsystem.getInstance()
                                            .setWantedState(SwerveStates.CHOREO_PATH);
                                }));
        SwerveSubsystem.getInstance()
                .getChoreoEventTimeTrigger("Deploy Climber")
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    ClimberSubsystem.getInstance()
                                            .setWantedState(ClimberStates.DEPLOY);
                                }));

        // =============================================================================
        // AUTO COMMAND SEQUENCE
        // =============================================================================
        addCommands(
                // Drive over the bump at a set speed
                Commands.startEnd(
                        () -> {SwerveSubsystem.getInstance().setDesiredChassisSpeed(new ChassisSpeeds(5,0,0));
                        SwerveSubsystem.getInstance().setWantedState(SwerveStates.CHASSIS_SPEEDS);},
                        () -> {ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.HUB);
                                LocalizationSubsystem.getInstance().resetPoseEstimatorAuto();
                                ShooterSubsystem.getInstance()
                                        .setWantedState(ShooterStates.TRACKING);
                        }).withTimeout(1.7 ),
                // Set the initial trajectory
                SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.CitrusLeftSide.name())),
                // Start Choreo following
                Commands.startEnd(
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.CHOREO_PATH),
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(
                                                        SwerveStates.FIELD_CENTRIC_ROTATION_LOCK))
                        .until(SwerveSubsystem.getInstance()::isAtChoreoSetpoint),
                // Start shooting here
                Commands.runOnce(
                        () -> {
                            ShooterSubsystem.getInstance().setWantedState(ShooterStates.SHOOT);
                            HopperSubsystem.getInstance().setWantedState(HopperStates.SHOOTING);
                        }),
                // Shoot for 3 seconds
                new WaitCommand(3),
                // Pull the intake in while we shoot to help index more balls
                Commands.runOnce(
                        () -> IntakeSubsystem.getInstance().setWantedState(IntakeStates.STORE)),
                // Continue to shoot for 3 more seconds
                new WaitCommand(3),
                // Stop shooting
                Commands.runOnce(
                        () -> {
                            ShooterSubsystem.getInstance().setWantedState(ShooterStates.TRACKING);
                            HopperSubsystem.getInstance().setWantedState(HopperStates.IDLE);
                        }),
                // Set the second trajectory for the second pass
                SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.CitrusLeftSideSecondPassNew.name())),
                // Start Choreo following
                Commands.startEnd(
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.CHOREO_PATH),
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK))
                        .until(() -> SwerveSubsystem.getInstance().isAtChoreoSetpoint() && SwerveSubsystem.getInstance()
                                                        .hasChoreoTimeElapsed(1)),
                // Start shooting here
                Commands.runOnce(
                () -> {
                    ShooterSubsystem.getInstance().setWantedState(ShooterStates.SHOOT);
                    HopperSubsystem.getInstance().setWantedState(HopperStates.SHOOTING);
                }),
                // Shoot for 3 seconds
                new WaitCommand(3),
                // Pull the intake in while we shoot to help index more balls
                Commands.runOnce(
                        () -> IntakeSubsystem.getInstance().setWantedState(IntakeStates.STORE))
                );
    }
}
