package frc.robot.autos;

import com.marswars.auto.Auto;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib2026.FieldTargets;
import frc.robot.subsystems.hopper.HopperConstants.HopperStates;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Neutral_Cycles_Left extends Auto {

    public Neutral_Cycles_Left() {
        // Register trajectories first
        // These should be loaded in the order they will be used to ensure correct start poses\
        loadTrajectory(ChoreoTraj.CycleNeutralFirstLeft.name());
        loadTrajectory(ChoreoTraj.CycleNeutralSecondLeft.name());
        loadTrajectory(ChoreoTraj.OutpostClimbNeutralLeft.name());

        // Add commands here to execute during the auto
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
        addCommands(
                // Set the initial trajectory
                Commands.runOnce(
                        () -> ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.HUB)),
                Commands.runOnce(
                        () ->
                                ShooterSubsystem.getInstance()
                                        .setWantedState(ShooterStates.TRACKING)),
                SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.CycleNeutralFirstLeft.name())),
                // Start Choreo following
                Commands.startEnd(
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.CHOREO_PATH),
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.FIELD_CENTRIC))
                        .until(
                                () ->
                                        SwerveSubsystem.getInstance().isAtChoreoSetpoint()
                                                && SwerveSubsystem.getInstance()
                                                        .hasChoreoTimeElapsed(1)),
                // Shoot here if needed
                Commands.runOnce(
                        () -> {
                            ShooterSubsystem.getInstance().setWantedState(ShooterStates.SHOOT);
                            SwerveSubsystem.getInstance()
                                    .setWantedState(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
                        }),
                new WaitCommand(3),
                Commands.runOnce(
                        () ->
                                ShooterSubsystem.getInstance()
                                        .setWantedState(ShooterStates.TRACKING)),
                // SwerveSubsystem.getInstance()
                //         .setDesiredChoreoTrajectoryCommand(
                //                 getTrajectory(ChoreoTraj.CycleNeutralSecond.name())),
                // // Start Choreo following
                // Commands.startEnd(
                //                 () ->
                //                         SwerveSubsystem.getInstance()
                //                                 .setWantedState(SwerveStates.CHOREO_PATH),
                //                 () ->
                //                         SwerveSubsystem.getInstance()
                //                                 .setWantedState(SwerveStates.FIELD_CENTRIC))
                //         .until(() -> SwerveSubsystem.getInstance().isAtChoreoSetpoint() &&
                // SwerveSubsystem.getInstance().hasChoreoTimeElapsed(1)),
                // // Shoot here if needed
                // Commands.runOnce(
                //         () ->
                // ShooterSubsystem.getInstance().setWantedState(ShooterStates.SHOOT)),
                // new WaitCommand(3),
                // Commands.runOnce(
                //         () ->
                // ShooterSubsystem.getInstance().setWantedState(ShooterStates.TRACKING)),
                SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.CycleNeutralSecondLeft.name())),
                // Start Choreo following
                Commands.startEnd(
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.CHOREO_PATH),
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.FIELD_CENTRIC))
                        .until(
                                () ->
                                        SwerveSubsystem.getInstance().isAtChoreoSetpoint()
                                                && SwerveSubsystem.getInstance()
                                                        .hasChoreoTimeElapsed(1)),
                // Shoot here if needed

                // Move to the climb position
                Commands.runOnce(
                        () -> ShooterSubsystem.getInstance().setWantedState(ShooterStates.SHOOT)),
                SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.OutpostClimbNeutralLeft.name())),
                Commands.startEnd(
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(
                                                        SwerveStates.CHOREO_PATH_ROTATION_LOCK),
                                () ->
                                        SwerveSubsystem.getInstance()
                                                .setWantedState(SwerveStates.FIELD_CENTRIC))
                        .until(
                                () ->
                                        SwerveSubsystem.getInstance().isAtChoreoSetpoint()
                                                && SwerveSubsystem.getInstance()
                                                        .hasChoreoTimeElapsed(1)),
                Commands.runOnce(
                        () ->
                                ShooterSubsystem.getInstance()
                                        .setWantedState(ShooterStates.TRACKING)));
    }
}
