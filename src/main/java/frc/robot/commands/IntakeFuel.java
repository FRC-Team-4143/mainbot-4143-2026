// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.HopperConstants.HopperStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.hopper.HopperSubsystem;

public class IntakeFuel extends Command {

    /** This command intakes fuel and agitates the hopper, used for teleop control while intaking */
    public IntakeFuel() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        IntakeSubsystem.getInstance().setWantedState(IntakeStates.INTAKE);
        HopperSubsystem.getInstance().setWantedState(HopperStates.INTAKE);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        IntakeSubsystem.getInstance().setWantedState(IntakeStates.IDLE);
        HopperSubsystem.getInstance().setWantedState(HopperStates.IDLE);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
