// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.marswars.proxy_server.ProxyServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.Optional;
import org.ironmaple.simulation.SimulatedArena;

public class Robot extends TimedRobot {

    private Alliance alliance_ = Alliance.Blue; // Current alliance, used to set driver perspective
    private RobotContainer robot_container_;

    public Robot() {
        // Load the subsystems
        robot_container_ = RobotContainer.getInstance();

        // Configure External Interfaces
        OI.configureBindings();
        ProxyServer.configureServer();
    }

    @Override
    public void robotInit() {}

    @Override
    public void robotPeriodic() {
        // updates data from chassis proxy server
        ProxyServer.updateData();

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
        ProxyServer.syncMatchData();
        CommandScheduler.getInstance().cancelAll();

        // In simulation, keep simple sim control. Otherwise use robot centric.
        if (RobotBase.isSimulation()) {
            SwerveSubsystem.getInstance().setWantedState(SwerveStates.SIMPLE_SIM_CONTROL);
        } else {
            SwerveSubsystem.getInstance().setWantedState(SwerveStates.FIELD_CENTRIC);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationInit() {
        // Configure the simulated robot state
        SimulatedRobotState.configure();
    }

    @Override
    public void simulationPeriodic() {
        // Update the physics simulation - this is CRITICAL for proper simulation data
        SimulatedArena.getInstance().simulationPeriodic();
    }
}
