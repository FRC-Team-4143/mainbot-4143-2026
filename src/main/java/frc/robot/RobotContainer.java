// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.mw_lib.subsystem.SubsystemManager;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class RobotContainer extends SubsystemManager {
  private static RobotContainer instance;

  public static synchronized RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }
    return instance;
  }

  public RobotContainer() {
    // !!!!!! ALL SUBSYSTEMS MUST BE REGISTERED HERE TO RUN !!!!!!!
    // registerSubsystem(Superstructure.getInstance());
    registerSubsystem(SwerveSubsystem.getInstance());
    // registerSubsystem(IntakeSubsystem.getInstance());
    //registerSubsystem(ShooterSubsystem.getInstance());

    // !!!!! LEAVE THESE LINES AS THE LAST LINE IN THE CONSTRUCTOR !!!!!!
    reset();
  }
}
