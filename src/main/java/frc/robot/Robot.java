// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.hopper.HopperConstants.HopperStates;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.Optional;

public class Robot extends TimedRobot {

    private Alliance alliance_ = Alliance.Blue; // Current alliance, used to set driver perspective
    private RobotContainer robot_container_;

    public Robot() {
        // Load the subsystems
        robot_container_ = RobotContainer.getInstance();

        // Configure External Interfaces
        OI.configureBindings();
    }

    @Override
    public void robotInit() {}

    @Override
    public void robotPeriodic() {
        // Call the scheduler so that commands work for buttons
        CommandScheduler.getInstance().run();

        // run the main robot loop for each subsystem
        robot_container_.doControlLoop();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {

            if (alliance.get() != alliance_) {
                alliance_ = alliance.get();
                SwerveSubsystem.getInstance()
                        .setOperatorForwardDirection(
                                alliance_ == Alliance.Blue
                                        ? SwerveConstants.OperatorPerspective.BLUE_ALLIANCE
                                        : SwerveConstants.OperatorPerspective.RED_ALLIANCE);
            }
        }
    }

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        SwerveSubsystem.getInstance().setWantedState(SwerveStates.FIELD_CENTRIC);
        ShooterSubsystem.getInstance().setWantedState(ShooterStates.MANUAL);
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        ShooterSubsystem.getInstance().setWantedState(ShooterStates.PROFILE);
        HopperSubsystem.getInstance().setWantedState(HopperStates.PROFILE);
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
