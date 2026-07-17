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

public class CitrusSynergyFarBump extends Auto {

    public CitrusSynergyFarBump() {
        // =============================================================================
        // TRAJECTORY LOADING
        // These should be loaded in the order they will be used to ensure correct start poses
        // =============================================================================
        loadTrajectory(ChoreoTraj.SynergyP1.name());
        loadTrajectory(ChoreoTraj.SynergyFarBumpP2.name());
        loadTrajectory(ChoreoTraj.SynergyFarBumpP3.name());
        // loadTrajectory(ChoreoTraj.SynergyP4.name());

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

        // =============================================================================
        // AUTO COMMAND SEQUENCE
        // =============================================================================
        addCommands(
                Commands.runOnce(
                        () -> {
                            ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.HUB);
                            ShooterSubsystem.getInstance().setWantedState(ShooterStates.IDLE);
                        }),

                // Set the initial trajectory
                SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.SynergyP1.name())),
                // Start Choreo following
                Commands.startEnd(
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.CHOREO_PATH),
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.IDLE))
                        .until(SwerveSubsystem.getInstance()::isAtChoreoSetpoint),
                // Wait for 4 seconds for other robots to clear the middle
                new DynamicWaitCommand(getName() + "/MiddleWaitTime", 4),
                // Set the second trajectory for the second part
                SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.SynergyFarBumpP2.name())),
                // Start Choreo following
                Commands.startEnd(
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.CHOREO_PATH),
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.IDLE))
                        .until(SwerveSubsystem.getInstance()::isAtChoreoSetpoint),
                // Wait for 1 second to allow the other bots to get out of the way in the alliance
                // zone
                new DynamicWaitCommand(getName() + "/BeforeAllianceZoneWaitTime", 1),
                // Set the third trajectory for the third part
                SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.SynergyFarBumpP3.name())),
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
                // Shoot for 2 seconds
                new WaitCommand(2)
                // ,
                // // Set the fourth trajectory for the fourth part
                // SwerveSubsystem.getInstance()
                //         .setDesiredChoreoTrajectoryCommand(
                //                 getTrajectory(ChoreoTraj.SynergyP4.name())),
                // // Start Choreo following
                // Commands.startEnd(
                //                 () ->
                //                         SwerveSubsystem.getInstance()
                //                                 .setWantedState(
                //                                         SwerveStates.CHOREO_PATH_ROTATION_LOCK),
                //                 () ->
                //                         SwerveSubsystem.getInstance()
                //                                 .setWantedState(
                //
                // SwerveStates.FIELD_CENTRIC_ROTATION_LOCK))
                //         .until(SwerveSubsystem.getInstance()::isAtChoreoSetpoint)
                );
    }
}
