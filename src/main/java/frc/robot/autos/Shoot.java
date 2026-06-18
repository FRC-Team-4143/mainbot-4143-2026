package frc.robot.autos;

import com.marswars.auto.Auto;
import org.wpilib.command2.Commands;
import frc.robot.lib2026.FieldTargets;
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
                        () -> ShooterSubsystem.getInstance().setWantedState(ShooterStates.SHOOT)));
    }
}
