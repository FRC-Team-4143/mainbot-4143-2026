package frc.robot.autos;

import com.marswars.auto.Auto;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib2026.FieldTargets;
import frc.robot.subsystems.hopper.FloorConstants.FloorStates;
import frc.robot.subsystems.hopper.FloorSubsystem;
import frc.robot.subsystems.hopper.RoofConstants.RoofStates;
import frc.robot.subsystems.hopper.RoofSubsystem;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class Citrus_Right_Side_Trench extends Auto {

    public Citrus_Right_Side_Trench() {
        // =============================================================================
        // TRAJECTORY LOADING
        // These should be loaded in the order they will be used to ensure correct start poses
        // =============================================================================
        loadTrajectory(ChoreoTraj.CitrusRightSideTrench.name());
        loadTrajectory(ChoreoTraj.CitrusRightSideSecondPassTrench.name());
        loadTrajectory(ChoreoTraj.CitrusRightSideSecondPassBumpTrench.name());

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
                // Drive over the bump at a set speed
                Commands.runOnce(
                        () -> {
                            ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.HUB);
                            ShooterSubsystem.getInstance().setWantedState(ShooterStates.TRACKING);
                            RoofSubsystem.getInstance().setWantedState(RoofStates.DOWN);
                        }),

                // Set the initial trajectory
                SwerveSubsystem.getInstance()
                        .setDesiredChoreoTrajectoryCommand(
                                getTrajectory(ChoreoTraj.CitrusRightSideTrench.name())),
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
                            FloorSubsystem.getInstance().setWantedState(FloorStates.SHOOTING);
                        }),
                // Shoot for 3 seconds
                new WaitCommand(3),
                // Pull the intake in while we shoot to help index more balls
                Commands.runOnce(
                        () -> IntakeSubsystem.getInstance().setWantedState(IntakeStates.INTAKE)),
                Commands.runOnce(
                        () -> {
                            RoofSubsystem.getInstance().setWantedState(RoofStates.DOWN);
                        }),
                // Continue to shoot for 3 more seconds
                new WaitCommand(3),
                // Stop shooting
                Commands.runOnce(
                        () -> {
                            ShooterSubsystem.getInstance().setWantedState(ShooterStates.TRACKING);
                            FloorSubsystem.getInstance().setWantedState(FloorStates.IDLE);
                            IntakeSubsystem.getInstance().setWantedState(IntakeStates.STORE);
                        }),
                // Set the second trajectory for the second pass
                new ConditionalCommand(
                        SwerveSubsystem.getInstance()
                                .setDesiredChoreoTrajectoryCommand(
                                        getTrajectory(ChoreoTraj.CitrusRightSideSecondPassTrench.name())),
                        SwerveSubsystem.getInstance()
                                .setDesiredChoreoTrajectoryCommand(
                                        getTrajectory(
                                                ChoreoTraj.CitrusRightSideSecondPassBumpTrench.name())),
                        RoofSubsystem.getInstance()::isDown),
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
                            FloorSubsystem.getInstance().setWantedState(FloorStates.SHOOTING);
                        }),
                // Shoot for 3 seconds
                new WaitCommand(3),
                // Pull the intake in while we shoot to help index more balls
                Commands.runOnce(
                        () -> IntakeSubsystem.getInstance().setWantedState(IntakeStates.STORE)),
                Commands.runOnce(
                        () -> {
                            RoofSubsystem.getInstance().setWantedState(RoofStates.DOWN);
                        }));
    }
}
