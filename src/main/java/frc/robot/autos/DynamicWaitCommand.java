// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DynamicWaitCommand extends Command {
  /** Creates a new DynamicWaitCommand. */
  protected Timer timer = new Timer();
  private double waitTime;
  public DynamicWaitCommand(String name, double defaultTime) {
    this.setName(name);
    this.waitTime = defaultTime;
    // Use addRequirements() here to declare subsystem dependencies.
    SmartDashboard.putNumber(this.getName(), defaultTime);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    waitTime = SmartDashboard.getNumber(getName(), waitTime);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(waitTime);
  }
}
