// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.mw_lib.subsystem.SubsystemManager;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.swerve.Swerve;

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
    // registerSubsystem(Swerve.getInstance());
    registerSubsystem(ElevatorSubsystem.getInstance());

    // !!!!! LEAVE THESE LINES AS THE LAST LINE IN THE CONSTRUCTOR !!!!!!
    reset();
  }
}
