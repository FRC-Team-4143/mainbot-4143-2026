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
import frc.robot.lib2026.HubMonitor;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.hopper.HopperConstants.HopperStates;
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
        SmartDashboard.putData(
                "Spin Down Flywheel",
                Commands.runOnce(
                        () ->
                                ShooterSubsystem.getInstance()
                                        .setWantedState(ShooterStates.SPIN_DOWN)));

        // Set up the hub first alliance chooser on the dashboard
        hub_first_alliance_chooser_.setDefaultOption("Auto", HubMonitor.ActiveAlliance.INVALID);
        hub_first_alliance_chooser_.addOption("Blue", HubMonitor.ActiveAlliance.BLUE_ACTIVE);
        hub_first_alliance_chooser_.addOption("Red", HubMonitor.ActiveAlliance.RED_ACTIVE);
        hub_first_alliance_chooser_.onChange(HubMonitor::seedActiveAlliance);
        SmartDashboard.putData("Hub - First Alliance", hub_first_alliance_chooser_);

        // =============================================================================
        // DRIVER CONTROLLER BINDINGS
        // =============================================================================
        driver_controller_
                .rightStick()
                .onTrue(SwerveSubsystem.getInstance().toggleFieldCentric().ignoringDisable(true));
        driver_controller_.rightTrigger().whileTrue(ControlCommands.shootFuelCommand());
        driver_controller_.leftTrigger().whileTrue(ControlCommands.aimAtTargetCommand());
        driver_controller_.leftStick().whileTrue(ControlCommands.rotateForBumpCommand());
        driver_controller_.rightBumper().whileTrue(ControlCommands.intakeFuelCommand());
    // A button runs outtake while held
    driver_controller_.a().whileTrue(ControlCommands.outtakeFuelCommand());
        driver_controller_.leftBumper().onFalse(ControlCommands.toggleStoreIntakeCommand());
    // X button controls explicit hopper forward/reverse states
    driver_controller_.x()
        .onTrue(
            Commands.runOnce(
                () ->
                    HopperSubsystem.getInstance()
                        .setWantedState(HopperStates.HOPPER_REVERSE))
                .ignoringDisable(true));
    driver_controller_.x()
        .onFalse(
            Commands.runOnce(
                () ->
                    HopperSubsystem.getInstance()
                        .setWantedState(HopperStates.HOPPER_FORWARD))
                .ignoringDisable(true));
        driver_controller_.start().onTrue(ControlCommands.advanceClimbingStage());
        driver_controller_.back().onTrue(ControlCommands.reverseClimbingStage());
        driver_controller_.y().whileTrue(ControlCommands.manualShootFuelCommand());
        driver_controller_.b().whileTrue(ControlCommands.manualPassFuelCommand());
        // =============================================================================
        // OPERATOR CONTROLLER BINDINGS
        // =============================================================================

        // operator_controller_.a().onTrue(ClimberSubsystem.getInstance().toggleDeployCommand());
        // operator_controller_.b().onTrue(Commands.runOnce(() ->
        // ClimberSubsystem.getInstance().setWantedState(ClimberStates.L1)));
        // operator_controller_.x().whileTrue(ClimberSubsystem.getInstance().bumpUpCommand());
        // operator_controller_.y().onTrue(Commands.runOnce(() ->
        // ClimberSubsystem.getInstance().setWantedState(ClimberStates.GROUND)));

        // driver_controller_.x().onTrue(Commands.runOnce(
        // () -> ClimberSubsystem.getInstance().setWantedState(ClimberStates.L1)));

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
        operator_controller_.rightBumper().onTrue(ClimberSubsystem.getInstance().bumpUpCommand());
        operator_controller_.leftBumper().onTrue(ClimberSubsystem.getInstance().bumpDownCommand());
        // =============================================================================
        // TESTING BINDINGS (THESE SHOULD BE REMOVED BEFORE COMPETITION)

        // =============================================================================
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
