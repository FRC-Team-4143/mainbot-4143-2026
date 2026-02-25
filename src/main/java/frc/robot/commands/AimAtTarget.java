// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.localization.LocalizationConstants.LocalizationStates;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class AimAtTarget extends Command {

    /**
     * This command aims the robot at the target with no intent to shoot, used for lining up shots
     * or for teleop control while aiming
     */
    public AimAtTarget() {}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        ShooterSubsystem.getInstance().setWantedState(ShooterStates.AIMING);
        SwerveSubsystem.getInstance().setWantedState(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
        LocalizationSubsystem.getInstance().setWantedState(LocalizationStates.SHOOTING_FOCUS);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        ShooterSubsystem.getInstance().setWantedState(ShooterStates.TRACKING);
        SwerveSubsystem.getInstance().setWantedState(SwerveStates.FIELD_CENTRIC);
        LocalizationSubsystem.getInstance().setWantedState(LocalizationStates.FULL);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
