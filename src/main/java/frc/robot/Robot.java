// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.marswars.auto.Auto;
import com.marswars.auto.AutoManager;
import com.marswars.auto.BehaviorTreeAuto;
import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.debug.BtLiveServer;
import com.marswars.bt.monitor.BehaviorTreeFileLogger;
import com.marswars.bt.monitor.BehaviorTreeMonitor;
import com.marswars.dashboard.Elastic;
import com.marswars.geometry.AllianceFlipUtil;
import com.marswars.logging.MwLog;
import com.marswars.swerve_lib.OperatorPerspective;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer.RobotStates;
import frc.robot.autos.bt.AutoNodes;
import frc.robot.lib2026.FieldConstants;
import frc.robot.lib2026.FieldRegions;
import frc.robot.lib2026.FieldTargets;
import frc.robot.lib2026.HubMonitor;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.Optional;
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {
    /** btlive WebSocket port for the BT editor's live view. */
    private static final int BT_LIVE_PORT = BtLiveServer.DEFAULT_PORT;

    private Alliance alliance_ = Alliance.Blue; // Current alliance, used to set driver perspective
    private RobotContainer robot_container_;

    public Robot() {
        AllianceFlipUtil.configureFieldGeometry(
                FieldConstants.FIELD_SYMMETRY_TYPE, FieldConstants.FIELD_CENTER);

        // Load the subsystems
        robot_container_ = RobotContainer.getInstance();

        // Configure External Interfaces
        OI.configureBindings();

        // Autos are behavior trees: every deploy/autos/*.xml is a chooser option (see the README
        // there). Robot-specific nodes are registered in AutoNodes.
        BehaviorTreeFactory bt_factory = AutoNodes.createFactory();
        BehaviorTreeMonitor.publishNodeModels(bt_factory);
        AutoManager.getInstance()
                .registerAutos(
                        BehaviorTreeAuto.loadAll(
                                bt_factory,
                                Filesystem.getDeployDirectory().toPath().resolve("autos")));
        startBtLiveServer();
        // One <auto>_<time>.btlog per auto run, replayable in Groot2 / the BT editor
        // (logs/bt in sim, /U/logs/bt on the robot)
        BehaviorTreeFileLogger.enableDefault();

        // Set the default target for the shooter to be the hub
        ShooterSubsystem.getInstance().setTarget(FieldTargets.Shooter.HUB);
    }

    @Override
    public void robotInit() {
        if (MwLog.isReplay()) setUseTiming(false);
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
        HubMonitor.isHubActive(DriverStation.getMatchTime());
        MwLog.log("Match Time", DriverStation.getMatchTime());

        // Visualize the 3D Robot
        Mechanism3dViz.publish();
    }

    @Override
    public void disabledInit() {
        robot_container_.setWantedState(RobotStates.IDLE);
        HubMonitor.seedActiveAlliance(HubMonitor.ActiveAlliance.INVALID);
    }

    @Override
    public void disabledPeriodic() {
        AutoManager.getInstance().periodic();

        // Allow chaning alliance perspective while disabled
        if (hasAllianceChanged()) {
            SwerveSubsystem.getInstance()
                    .setOperatorForwardDirection(
                            alliance_ == Alliance.Blue
                                    ? OperatorPerspective.BLUE_ALLIANCE
                                    : OperatorPerspective.RED_ALLIANCE);
            FieldRegions.flipRegions();
            LocalizationSubsystem.getInstance().setTagFocus(alliance_);
            HubMonitor.seedActiveAlliance(HubMonitor.ActiveAlliance.INVALID);
        }
    }

    @Override
    public void autonomousInit() {
        Elastic.selectTab("Autonomous");
        robot_container_.setWantedState(RobotStates.AUTO);
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
        // Starts field-centric driving with the shooter idle (see RobotContainer)
        robot_container_.setWantedState(RobotStates.DRIVE);
        Elastic.selectTab("Teleoperated");
    }

    @Override
    public void teleopPeriodic() {
        // Attempt to update the first active alliance until it return valid
        if (!HubMonitor.isFirstActiveAllianceValid()) HubMonitor.seedActiveAlliance();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        // Maintenance lives outside the robot state machine: it asks for nothing in MAINTENANCE,
        // and the subsystems start in TUNING so the dashboard buttons (homing, tuning) drive them.
        robot_container_.setWantedState(RobotStates.MAINTENANCE);
        ShooterSubsystem.getInstance().setWantedState(ShooterStates.TUNING);
        IntakeSubsystem.getInstance().setWantedState(IntakeStates.TUNING);
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    /** Streams running behavior trees to the BT editor (btlive protocol). Never fatal. */
    private static void startBtLiveServer() {
        if (MwLog.isReplay()) {
            return;
        }
        try {
            BtLiveServer.start(
                    BtLiveServer.Options.defaults()
                            .withPort(BT_LIVE_PORT)
                            .withRobotName(RobotBase.isSimulation() ? "sim" : "4143"));
        } catch (RuntimeException e) {
            DataLogManager.log("BT live server not started: " + e.getMessage());
        }
    }

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
