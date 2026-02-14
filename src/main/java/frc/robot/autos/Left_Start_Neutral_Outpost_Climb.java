package frc.robot.autos;

import com.marswars.auto.Auto;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Left_Start_Neutral_Outpost_Climb extends Auto {

    public Left_Start_Neutral_Outpost_Climb() {
        // Register trajectories first
        // These should be loaded in the order they will be used to ensure correct start
        // poses
        loadTrajectory(ChoreoTraj.LeftStartNeutralOutpost1.name());
        loadTrajectory(ChoreoTraj.LeftStartNeutralOutpost2.name());
        loadTrajectory(ChoreoTraj.OutpostClimb.name());

        // Add commands here to execute during the auto
        addCommands(
                // Set the initial trajectory
                SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.LeftStartNeutralOutpost1.name())),
                // Start Choreo following
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
                                getTrajectory(ChoreoTraj.LeftStartNeutralOutpost2.name())),
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
                                getTrajectory(ChoreoTraj.OutpostClimb.name())),
                Commands.startEnd(
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(
                                                        SwerveStates.CHOREO_PATH_ROTATION_LOCK),
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.FIELD_CENTRIC))
                        .until(SwerveSubsystem.getInstance()::isAtChoreoSetpoint));
    }
}
