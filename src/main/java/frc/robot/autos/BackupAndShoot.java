package frc.robot.autos;

import com.marswars.auto.Auto;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib2026.FieldTargets;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class BackupAndShoot extends Auto {

    public BackupAndShoot() {
        // Register trajectories first
        // These should be loaded in the order they will be used to ensure correct start poses\
        loadTrajectory(ChoreoTraj.DriveBackwards.name());
        // Add commands here to execute during the auto
        addCommands(
                 SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.DriveBackwards.name())),
                // Start Choreo following
                Commands.startEnd(
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.CHOREO_PATH),
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK))
                        .until(SwerveSubsystem.getInstance()::isAtChoreoSetpoint),
                Commands.runOnce(
                        () -> ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.HUB)),
                Commands.runOnce(
                        () -> ShooterSubsystem.getInstance().setWantedState(ShooterStates.SHOOT)));
    }
}
