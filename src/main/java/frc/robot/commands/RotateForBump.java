// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib2026.FieldTargets;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RotateForBump extends Command {

    /**
     * This command snaps robot to nearest bump crossing angle while held, returns to normal field
     * centric when released
     */
    public RotateForBump() {}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SwerveSubsystem.getInstance().setDesiredRotationLock(FieldTargets.getNearestSnapAngle());
        SwerveSubsystem.getInstance().setWantedState(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        SwerveSubsystem.getInstance().setWantedState(SwerveStates.FIELD_CENTRIC);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
