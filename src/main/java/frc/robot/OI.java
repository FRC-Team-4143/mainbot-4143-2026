// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotConstants.RobotStates;
import frc.robot.lib2026.HubMonitor;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.Optional;

public abstract class OI {

    // Sets up both controllers
    private static final CommandXboxController driver_controller_ = new CommandXboxController(0);
    private static final CommandXboxController operator_controller_ = new CommandXboxController(1);

    private static final SendableChooser<HubMonitor.ActiveAlliance> hub_first_alliance_chooser_ =
            new SendableChooser<>();

    public static void configureBindings() {
        DriverStation.silenceJoystickConnectionWarning(true);

        // =============================================================================
        // SMARTDASHBOARD COMMANDS
        // =============================================================================
        SmartDashboard.putData(
                "Zero Gyro Yaw", SwerveSubsystem.getInstance().zeroGyroYaw().ignoringDisable(true));
        SmartDashboard.putData(
                "Set Start Pose",
                Commands.runOnce(LocalizationSubsystem.getInstance()::resetPoseEstimatorAuto)
                        .ignoringDisable(true));
        SmartDashboard.putData(
                "Zero Wheel Offsets",
                SwerveSubsystem.getInstance().setModuleOffsets().ignoringDisable(true));
        // In teleop the flywheel only keeps spinning in TRACK (after aiming); DRIVE stops it.
        SmartDashboard.putData(
                "Spin Down Flywheel",
                Commands.runOnce(
                                () ->
                                        RobotContainer.getInstance()
                                                .setWantedState(RobotStates.DRIVE))
                        .ignoringDisable(true));

        // Set up the hub first alliance chooser on the dashboard
        hub_first_alliance_chooser_.setDefaultOption("Auto", HubMonitor.ActiveAlliance.INVALID);
        hub_first_alliance_chooser_.addOption("Blue", HubMonitor.ActiveAlliance.BLUE_ACTIVE);
        hub_first_alliance_chooser_.addOption("Red", HubMonitor.ActiveAlliance.RED_ACTIVE);
        hub_first_alliance_chooser_.onChange(HubMonitor::seedActiveAlliance);
        SmartDashboard.putData("Hub - First Alliance", hub_first_alliance_chooser_);

        // =============================================================================
        // DRIVER CONTROLLER BINDINGS
        // =============================================================================
        RobotContainer robot = RobotContainer.getInstance();
        driver_controller_
                .rightStick()
                .onTrue(Commands.runOnce(robot::toggleFieldCentric).ignoringDisable(true));
        // Hold a button: the robot is in that state; let go: back to DRIVE (or TRACK after aiming,
        // so the flywheel stays at speed for the shot)
        driver_controller_
                .rightTrigger()
                .whileTrue(
                        Commands.startEnd(
                                        () -> robot.setWantedState(RobotStates.SHOOT),
                                        () -> {
                                            if (robot.getWantedState() == RobotStates.SHOOT) {
                                                robot.setWantedState(RobotStates.DRIVE);
                                            }
                                        })
                                .ignoringDisable(true));
        driver_controller_
                .leftTrigger()
                .whileTrue(
                        Commands.startEnd(
                                        () -> robot.setWantedState(RobotStates.AIM),
                                        () -> {
                                            if (robot.getWantedState() == RobotStates.AIM) {
                                                robot.setWantedState(RobotStates.TRACK);
                                            }
                                        })
                                .ignoringDisable(true));
        driver_controller_
                .rightBumper()
                .whileTrue(
                        Commands.startEnd(
                                        () -> robot.setWantedState(RobotStates.INTAKE),
                                        () -> {
                                            if (robot.getWantedState() == RobotStates.INTAKE) {
                                                robot.setWantedState(RobotStates.DRIVE);
                                            }
                                        })
                                .ignoringDisable(true));
        driver_controller_
                .y()
                .whileTrue(
                        Commands.startEnd(
                                        () -> robot.setWantedState(RobotStates.MANUAL_HUB),
                                        () -> {
                                            if (robot.getWantedState() == RobotStates.MANUAL_HUB) {
                                                robot.setWantedState(RobotStates.DRIVE);
                                            }
                                        })
                                .ignoringDisable(true));
        driver_controller_
                .b()
                .whileTrue(
                        Commands.startEnd(
                                        () -> robot.setWantedState(RobotStates.MANUAL_PASS),
                                        () -> {
                                            if (robot.getWantedState() == RobotStates.MANUAL_PASS) {
                                                robot.setWantedState(RobotStates.DRIVE);
                                            }
                                        })
                                .ignoringDisable(true));
        driver_controller_
                .a()
                .whileTrue(
                        Commands.startEnd(
                                        () -> robot.setWantedState(RobotStates.OUTTAKE),
                                        () -> {
                                            if (robot.getWantedState() == RobotStates.OUTTAKE) {
                                                robot.setWantedState(RobotStates.DRIVE);
                                            }
                                        })
                                .ignoringDisable(true));
        driver_controller_
                .x()
                .whileTrue(
                        Commands.startEnd(
                                        () -> robot.setWantedState(RobotStates.BRAKE),
                                        () -> {
                                            if (robot.getWantedState() == RobotStates.BRAKE) {
                                                robot.setWantedState(RobotStates.DRIVE);
                                            }
                                        })
                                .ignoringDisable(true));
        driver_controller_
                .leftBumper()
                .onFalse(Commands.runOnce(robot::toggleStoreIntake).ignoringDisable(true));

        // =============================================================================
        // OPERATOR CONTROLLER BINDINGS
        // =============================================================================
        // Dead-man force-pass override: hold to pick a PASS target while inside the alliance
        // zone (left bumper acts as the dead-man).
        operator_controller_
                .leftBumper()
                .whileTrue(
                        Commands.startEnd(
                                        () -> robot.setPassOverride(true),
                                        () -> robot.setPassOverride(false))
                                .ignoringDisable(true));
        operator_controller_
                .povUp()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    ShooterSubsystem.getInstance().adjustFlywheel(2);
                                }));
        operator_controller_
                .povDown()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    ShooterSubsystem.getInstance().adjustFlywheel(-2);
                                }));
        operator_controller_
                .povRight()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    ShooterSubsystem.getInstance()
                                            .adjustHood(Units.degreesToRadians(-1));
                                }));
        operator_controller_
                .povLeft()
                .onTrue(
                        Commands.runOnce(
                                () -> {
                                    ShooterSubsystem.getInstance()
                                            .adjustHood(Units.degreesToRadians(1));
                                }));
        operator_controller_
                .x()
                .onTrue(Commands.runOnce(robot::toggleIsAbleToRack).ignoringDisable(true));
    }

    /**
     * @return driver controller left joystick x axis
     */
    public static double getDriverJoystickLeftX() {
        return driver_controller_.getLeftX();
    }

    /**
     * @return driver controller left joystick y axis
     */
    public static double getDriverJoystickLeftY() {
        return driver_controller_.getLeftY();
    }

    public static boolean getDriverRightTrigger() {
        return driver_controller_.getRightTriggerAxis() > 0.1;
    }

    /**
     * @return driver controller right joystick x axis
     */
    public static double getDriverJoystickRightX() {
        return driver_controller_.getRightX();
    }

    /**
     * @return operator controller right joystick x axis
     */
    public static double getOperatorJoystickRightX() {
        return operator_controller_.getRightX();
    }

    /**
     * @return operator controller right joystick y axis
     */
    public static double getOperatorJoystickRightY() {
        return operator_controller_.getRightY();
    }

    /**
     * @return operator controller left joystick y axis
     */
    public static double getOperatorJoystickLeftY() {
        return operator_controller_.getLeftY();
    }

    /**
     * @return driver controller joystick pov angle in degrees, empty if nothing is pressed
     */
    public static Optional<Rotation2d> getDriverJoystickPOV() {
        int pov = driver_controller_.getHID().getPOV();
        return (pov != -1) ? Optional.of(Rotation2d.fromDegrees(pov)) : Optional.empty();
    }

    /**
     * @return operator controller joystick pov angle in degrees, empty if nothing is pressed
     */
    public static Optional<Rotation2d> getOperatorJoystickPOV() {
        int pov = operator_controller_.getHID().getPOV();
        return (pov != -1) ? Optional.of(Rotation2d.fromDegrees(pov)) : Optional.empty();
    }
}
