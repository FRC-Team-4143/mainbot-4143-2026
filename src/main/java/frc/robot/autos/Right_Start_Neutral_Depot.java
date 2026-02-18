package frc.robot.autos;

import com.marswars.auto.Auto;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Right_Start_Neutral_Depot extends Auto {

    public Right_Start_Neutral_Depot() {
        // Register trajectories first
        // These should be loaded in the order they will be used to ensure correct start
        // poses
        loadTrajectory(ChoreoTraj.RightStartNeutralDepot1.name());
        loadTrajectory(ChoreoTraj.RightStartNeutralDepot2.name());
        loadTrajectory(ChoreoTraj.DepotClimb.name());
        // Add commands here to execute during the auto
        SwerveSubsystem.getInstance()
                .getChoreoEventTimeTrigger("Input On")
                .onTrue(
                        Commands.runOnce(
                                () ->
                                        IntakeSubsystem.getInstance()
                                                .setWantedState(IntakeStates.INTAKE)));

        SwerveSubsystem.getInstance()
                .getChoreoEventTimeTrigger("Input Off")
                .onTrue(
                        Commands.runOnce(
                                () ->
                                        IntakeSubsystem.getInstance()
                                                .setWantedState(IntakeStates.STORE)));
        SwerveSubsystem.getInstance()
                .getChoreoEventTimeTrigger("Shoot On")
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    ShooterSubsystem.getInstance()
                                            .setWantedState(ShooterStates.SHOOT);
                                    SwerveSubsystem.getInstance()
                                            .setWantedState(SwerveStates.CHOREO_PATH_ROTATION_LOCK);
                                }));
        SwerveSubsystem.getInstance()
                .getChoreoEventTimeTrigger("Shoot Off")
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    ShooterSubsystem.getInstance()
                                            .setWantedState(ShooterStates.TRACKING);
                                    SwerveSubsystem.getInstance()
                                            .setWantedState(SwerveStates.CHOREO_PATH);
                                }));

        // (Implementation of commands would go here)
        addCommands(
                SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.RightStartNeutralDepot1.name())),
                Commands.startEnd(
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.CHOREO_PATH),
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.FIELD_CENTRIC))
                        .until(SwerveSubsystem.getInstance()::isAtChoreoSetpoint),
                SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.RightStartNeutralDepot2.name())),
                Commands.startEnd(
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.CHOREO_PATH),
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.FIELD_CENTRIC))
                        .until(SwerveSubsystem.getInstance()::isAtChoreoSetpoint),
                new WaitCommand(3),
                // Move to the climb position
                SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.DepotClimb.name())),
                Commands.startEnd(
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.CHOREO_PATH),
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.FIELD_CENTRIC))
                        .until(SwerveSubsystem.getInstance()::isAtChoreoSetpoint));
    }
}
