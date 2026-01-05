// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import java.util.function.BooleanSupplier;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public final class Constants {

  public static final Mode CURRENT_MODE = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
  public static final BooleanSupplier IS_REAL = () -> CURRENT_MODE == Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM
  }

  public static final double CONTROLLER_DEADBAND = 0.05;
  public static SwerveDriveSimulation SWERVE_SIMULATOR;
}
