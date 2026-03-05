package frc.robot.autos;

import com.marswars.auto.Auto;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class TestAuto extends Auto {

    public TestAuto() {
        // Register trajectories first
        // These should be loaded in the order they will be used to ensure correct start poses\
        loadTrajectory(ChoreoTraj.TestPath.name());

        // Add commands here to execute during the auto
        addCommands(
                SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.TestPath.name())),
                // Start Choreo following
                Commands.startEnd(
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.CHOREO_PATH),
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.FIELD_CENTRIC))
                        .until(() -> SwerveSubsystem.getInstance().isAtChoreoSetpoint()));
    }
}
