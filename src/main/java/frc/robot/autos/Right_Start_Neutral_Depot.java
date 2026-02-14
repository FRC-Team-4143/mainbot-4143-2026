package frc.robot.autos;

import com.marswars.auto.Auto;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
