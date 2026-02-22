// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.marswars.auto.Auto;
import com.marswars.auto.AutoManager;
import com.marswars.geometry.AllianceFlipUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autos.Center_Depot_Outpost_Climb;
import frc.robot.autos.Left_Start_Neutral_Outpost_Climb;
import frc.robot.autos.Neutral_Cycles;
import frc.robot.autos.Neutral_Cycles_Left;
import frc.robot.autos.Right_Start_Neutral_Depot_Climb;
import frc.robot.lib2026.FieldConstants;
import frc.robot.lib2026.FieldRegions;
import frc.robot.lib2026.FieldTargets;
import frc.robot.lib2026.HubMonitor;
import frc.robot.subsystems.hopper.HopperConstants.HopperStates;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.Optional;

public class Robot extends TimedRobot {

    private Alliance alliance_ = null; // Current alliance, used to set driver perspective
    private RobotContainer robot_container_;

    public Robot() {
        // Load the subsystems
        robot_container_ = RobotContainer.getInstance();
        AllianceFlipUtil.configureFieldGeometry(
                FieldConstants.FIELD_SYMMETRY_TYPE, FieldConstants.FIELD_CENTER);

        // Configure External Interfaces
        OI.configureBindings();

        // Initialize AutoManager and register auto routines
        AutoManager.getInstance()
                .registerAutos(
                        // Add your auto routines here as you create them
                        new Left_Start_Neutral_Outpost_Climb(),
                        new Right_Start_Neutral_Depot_Climb(),
                        new Neutral_Cycles(),
                        new Neutral_Cycles_Left(),
                        new Center_Depot_Outpost_Climb());

        // Set the default target for the shooter to be the hub
        ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.HUB);
    }

    @Override
    public void robotInit() {}

    @Override
    public void robotPeriodic() {
        // Call the scheduler so that commands work for buttons
        CommandScheduler.getInstance().run();
        // run the main robot loop for each subsystem
        robot_container_.doControlLoop();

        // Update the hub active status
        HubMonitor.isHubActive(DriverStation.getMatchTime());
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        // Allow chaning alliance perspective while disabled
        if (hasAllianceChanged()) {
            SwerveSubsystem.getInstance()
                    .setOperatorForwardDirection(
                            alliance_ == Alliance.Blue
                                    ? SwerveConstants.OperatorPerspective.BLUE_ALLIANCE
                                    : SwerveConstants.OperatorPerspective.RED_ALLIANCE);
            FieldRegions.flipRegions();
            LocalizationSubsystem.getInstance().setTagFocus(alliance_);
            HubMonitor.seedActiveAlliance(HubMonitor.ActiveAlliance.INVALID);
        }
    }

    @Override
    public void autonomousInit() {
        Auto selected_auto = AutoManager.getInstance().getSelectedAuto();
        CommandScheduler.getInstance().schedule(selected_auto);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        SwerveSubsystem.getInstance().setWantedState(SwerveStates.FIELD_CENTRIC);
    }

    @Override
    public void teleopPeriodic() {
        // Attempt to update the first active alliance until it return valid
        if (!HubMonitor.isFirstActiveAllianceValid()) HubMonitor.seedActiveAlliance();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        ShooterSubsystem.getInstance().setWantedState(ShooterStates.TUNING);
        HopperSubsystem.getInstance().setWantedState(HopperStates.TUNING);
        IntakeSubsystem.getInstance().setWantedState(IntakeStates.TUNING);
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    /**
     * Check if the alliance has changed since the last check
     *
     * @return true if the alliance has changed, false otherwise
     */
    public boolean hasAllianceChanged() {
        Optional<Alliance> current_alliance = DriverStation.getAlliance();
        if (alliance_ == null && current_alliance.isPresent()) {
            alliance_ = current_alliance.get();
            return true;
        } else if (current_alliance.isPresent() && alliance_ != current_alliance.get()) {
            alliance_ = current_alliance.get();
            return true;
        }
        return false;
    }
}
