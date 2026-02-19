package frc.robot.autos;

import com.marswars.auto.Auto;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.climber.ClimberConstants.ClimberStates;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.lib2026.FieldTargets.Shooter;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Double_Neutral_Right extends Auto {

    public Double_Neutral_Right() {
        // Register trajectories first
        // These should be loaded in the order they will be used to ensure correct start
        // poses
        loadTrajectory(ChoreoTraj.DoubleNeutralRight.name());

        SwerveSubsystem.getInstance()
                .getChoreoEventTimeTrigger("Input On")
                .onTrue(
                        Commands.runOnce(
                                () ->
                                        IntakeSubsystem.getInstance()
                                                .setWantedState(IntakeStates.INTAKE)));
        SwerveSubsystem.getInstance()                .getChoreoEventTimeTrigger("Input Off")
                .onTrue(
                        Commands.runOnce(
                                () ->                                        IntakeSubsystem.getInstance()
                                                .setWantedState(IntakeStates.STORE)));
        SwerveSubsystem.getInstance()
                .getChoreoEventTimeTrigger("Shoot On")
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    // ShooterSubsystem.getInstance()
                                    //         .setWantedState(ShooterStates.SHOOT);
                                    SwerveSubsystem.getInstance()
                                            .setWantedState(SwerveStates.CHOREO_PATH_ROTATION_LOCK);
                                    ShooterSubsystem.getInstance()
                                            .setWantedState(ShooterStates.SHOOT);
                                }));
        SwerveSubsystem.getInstance()               .getChoreoEventTimeTrigger("Shoot Off")
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    // ShooterSubsystem.getInstance()
                                    //         .setWantedState(ShooterStates.TRACKING);
                                    SwerveSubsystem.getInstance()
                                            .setWantedState(SwerveStates.CHOREO_PATH);
                                        ShooterSubsystem.getInstance()
                                            .setWantedState(ShooterStates.TRACKING);
                                }));
                                SwerveSubsystem.getInstance()
                .getChoreoEventTimeTrigger("Climb")
                .onTrue(
                        Commands.runOnce(
                                () -> { 
                                    // ShooterSubsystem.getInstance()
                                    //         .setWantedState(ShooterStates.SHOOT);
                                    ClimberSubsystem.getInstance()
                                            .setWantedState(ClimberStates.L1_CLIMB);
                                }));
        // Add commands here to execute during the auto
        addCommands(
                // Set the initial trajectory
                SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.DoubleNeutralRight.name())),
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
