// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.marswars.subsystem.SubsystemManager;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.simulation.SimulationSubsystem;
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
        super(BuildConstants.class);
        // !!!!!! ALL SUBSYSTEMS MUST BE REGISTERED HERE TO RUN !!!!!!!
        registerSubsystem(SwerveSubsystem.getInstance());
        registerSubsystem(LocalizationSubsystem.getInstance());
        // registerSubsystem(ShooterSubsystem.getInstance());
        registerSubsystem(HopperSubsystem.getInstance());
        // registerSubsystem(IntakeSubsystem.getInstance());
        // registerSubsystem(ClimberSubsystem.getInstance());
        // registerSubsystem(GameStatesSubsystem.getInstance());

        // Only enable the simulation subsystem if we are in simulation
        if (RobotBase.isSimulation()) {
            registerSubsystem(SimulationSubsystem.getInstance());
        }

        // !!!!! LEAVE THESE LINES AS THE LAST LINE IN THE CONSTRUCTOR !!!!!!
        reset();
    }
}
