// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.marswars.auto.Auto;
import com.marswars.auto.AutoManager;
import com.marswars.geometry.AllianceFlipUtil;
import com.marswars.logging.Elastic;
import dev.doglog.DogLog;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.MatchState;
import org.wpilib.framework.TimedRobot;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.CommandScheduler;
import frc.robot.autos.CitrusSynergy;
import frc.robot.autos.CitrusSynergyFarBump;
import frc.robot.autos.Left_Trench_Bump_Swipe;
import frc.robot.autos.Left_Trench_Trench_Swipe;
import frc.robot.autos.Right_Trench_Bump_Swipe;
import frc.robot.autos.Right_Trench_Trench_Swipe;
import frc.robot.autos.Shoot;
import frc.robot.autos.Slop_Auto;
import frc.robot.lib2026.FieldConstants;
import frc.robot.lib2026.FieldRegions;
import frc.robot.lib2026.FieldTargets;
import frc.robot.lib2026.HubMonitor;
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

    private Alliance alliance_ = Alliance.BLUE; // Current alliance, used to set driver perspective
    private RobotContainer robot_container_;

    public Robot() {
        AllianceFlipUtil.configureFieldGeometry(
                FieldConstants.FIELD_SYMMETRY_TYPE, FieldConstants.FIELD_CENTER);

        // Load the subsystems
        robot_container_ = RobotContainer.getInstance();

        // Configure External Interfaces
        OI.configureBindings();

        // Initialize AutoManager and register auto routines
        AutoManager.getInstance()
                .registerAutos(
                        // Add your auto routines here as you create them
                        new Left_Trench_Bump_Swipe(),
                        new Left_Trench_Trench_Swipe(),
                        new Right_Trench_Bump_Swipe(),
                        new Right_Trench_Trench_Swipe(),
                        new Shoot(),
                        new CitrusSynergy(),
                        new CitrusSynergyFarBump(),
                        new Slop_Auto()
                        // new TestAuto()
                        );

        // Set the default target for the shooter to be the hub
        ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.HUB);

        // Initialization formerly in robotInit()
        Elastic.selectTab("Autonomous");
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    @Override
    public void robotPeriodic() {
        // Call the scheduler so that commands work for buttons
        CommandScheduler.getInstance().run();
        // run the main robot loop for each subsystem
        robot_container_.doControlLoop();

        // Update the hub active status
        HubMonitor.isHubActive(MatchState.getMatchTime());
        DogLog.log("Match Time", MatchState.getMatchTime());

        // Visualize the 3D Robot
        Mechanism3dViz.publish();
    }

    @Override
    public void disabledInit() {
        ShooterSubsystem.getInstance().setWantedState(ShooterStates.IDLE);
        HubMonitor.seedActiveAlliance(HubMonitor.ActiveAlliance.INVALID);
    }

    @Override
    public void disabledPeriodic() {
        AutoManager.getInstance().periodic();

        // Allow chaning alliance perspective while disabled
        if (hasAllianceChanged()) {
            SwerveSubsystem.getInstance()
                    .setOperatorForwardDirection(
                            alliance_ == Alliance.BLUE
                                    ? SwerveConstants.OperatorPerspective.BLUE_ALLIANCE
                                    : SwerveConstants.OperatorPerspective.RED_ALLIANCE);
            FieldRegions.flipRegions();
            LocalizationSubsystem.getInstance().setTagFocus(alliance_);
            HubMonitor.seedActiveAlliance(HubMonitor.ActiveAlliance.INVALID);
        }
    }

    @Override
    public void autonomousInit() {
        Elastic.selectTab("Autonomous");
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
        Elastic.selectTab("Teleoperated");
        ShooterSubsystem.getInstance().setWantedState(ShooterStates.IDLE);
    }

    @Override
    public void teleopPeriodic() {
        // Attempt to update the first active alliance until it return valid
        if (!HubMonitor.isFirstActiveAllianceValid()) HubMonitor.seedActiveAlliance();
    }

    @Override
    public void utilityInit() {
        CommandScheduler.getInstance().cancelAll();
        SwerveSubsystem.getInstance().setWantedState(SwerveStates.FIELD_CENTRIC);
        ShooterSubsystem.getInstance().setWantedState(ShooterStates.TUNING);
        IntakeSubsystem.getInstance().setWantedState(IntakeStates.TUNING);
    }

    @Override
    public void utilityPeriodic() {}

    @Override
    public void utilityExit() {}

    /**
     * Check if the alliance has changed since the last check
     *
     * @return true if the alliance has changed, false otherwise
     */
    public boolean hasAllianceChanged() {
        Optional<Alliance> current_alliance = MatchState.getAlliance();
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
