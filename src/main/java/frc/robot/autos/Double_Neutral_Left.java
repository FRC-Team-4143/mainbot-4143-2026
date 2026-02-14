package frc.robot.autos;

import com.marswars.auto.Auto;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Double_Neutral_Left extends Auto {

    public Double_Neutral_Left() {
        // Register trajectories first
        // These should be loaded in the order they will be used to ensure correct start
        // poses
        loadTrajectory(ChoreoTraj.DoubleNeutralLeft.name());

        // Add commands here to execute during the auto
        addCommands(
                // Set the initial trajectory
                SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.DoubleNeutralLeft.name())),
                // Start Choreo following
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
