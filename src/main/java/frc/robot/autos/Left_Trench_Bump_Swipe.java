package frc.robot.autos;

import com.marswars.auto.Auto;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib2026.FieldTargets;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Left_Trench_Bump_Swipe extends Auto {

    public Left_Trench_Bump_Swipe() {
        // =============================================================================
        // TRAJECTORY LOADING
        // These should be loaded in the order they will be used to ensure correct start
        // poses
        // =============================================================================
        loadTrajectory(ChoreoTraj.LTrenchStartBumpReturn.name());
        loadTrajectory(ChoreoTraj.LTrenchSwipeBumpReturn.name());

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
                .getChoreoEventTimeTrigger("Shoot")
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    ShooterSubsystem.getInstance()
                                            .setWantedState(ShooterStates.SHOOT);
                                    SwerveSubsystem.getInstance()
                                            .setWantedState(SwerveStates.CHOREO_PATH_ROTATION_LOCK);
                                }));

        // =============================================================================
        // AUTO COMMAND SEQUENCE
        // =============================================================================
        addCommands(
                // Drive over the bump at a set speed
                Commands.runOnce(
                        () -> {
                            ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.HUB);
                            ShooterSubsystem.getInstance().setWantedState(ShooterStates.IDLE);
                        }),
                // Set the initial trajectory
                SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.LTrenchStartBumpReturn.name())),
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
                            IntakeSubsystem.getInstance().setWantedState(IntakeStates.SQUEEZE);
                        }),
                // Shoot for 3 seconds
                new WaitCommand(2.5),
                // Continue to shoot for 3 more seconds
                new WaitCommand(1),
                // Stop shooting
                Commands.runOnce(
                        () -> {
                            ShooterSubsystem.getInstance().setWantedState(ShooterStates.IDLE);
                            IntakeSubsystem.getInstance().setWantedState(IntakeStates.INTAKE);
                        }),
                // Set the second trajectory for the second pass
                SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.LTrenchSwipeBumpReturn.name())),
                // Start Choreo following
                Commands.startEnd(
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.CHOREO_PATH),
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(
                                                        SwerveStates.FIELD_CENTRIC_ROTATION_LOCK))
                        .until(
                                () ->
                                        SwerveSubsystem.getInstance().isAtChoreoSetpoint()
                                                && SwerveSubsystem.getInstance()
                                                        .hasChoreoTimeElapsed(1)),
                // Start shooting here
                Commands.runOnce(
                        () -> {
                            ShooterSubsystem.getInstance().setWantedState(ShooterStates.SHOOT);
                            IntakeSubsystem.getInstance().setWantedState(IntakeStates.SQUEEZE);
                        }),
                // Shoot for 3 seconds
                new WaitCommand(2.5),
                // Continue to shoot for 3 more seconds
                new WaitCommand(1),
                // Stop shooting
                Commands.runOnce(
                        () -> {
                            ShooterSubsystem.getInstance().setWantedState(ShooterStates.IDLE);
                            IntakeSubsystem.getInstance().setWantedState(IntakeStates.INTAKE);
                        }),
                // Set the second trajectory for the second pass
                SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.LTrenchSwipeBumpReturn.name())),
                // Start Choreo following
                Commands.startEnd(
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.CHOREO_PATH),
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(
                                                        SwerveStates.FIELD_CENTRIC_ROTATION_LOCK))
                        .until(
                                () ->
                                        SwerveSubsystem.getInstance().isAtChoreoSetpoint()
                                                && SwerveSubsystem.getInstance()
                                                        .hasChoreoTimeElapsed(1)));
    }
}
