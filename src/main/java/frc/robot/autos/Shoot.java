package frc.robot.autos;

import com.marswars.auto.Auto;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib2026.FieldTargets;
import frc.robot.subsystems.hopper.HopperConstants.HopperStates;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class Shoot extends Auto {

    public Shoot() {
        // Register trajectories first
        // These should be loaded in the order they will be used to ensure correct start poses\

        // Add commands here to execute during the auto
        addCommands(
                Commands.runOnce(
                        () -> ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.HUB)),
                Commands.runOnce(
                        () -> ShooterSubsystem.getInstance().setWantedState(ShooterStates.SHOOT)),
                Commands.runOnce(
                        () -> HopperSubsystem.getInstance().setWantedState(HopperStates.SHOOTING)));
    }
}
