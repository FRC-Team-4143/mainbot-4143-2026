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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib2026.HubMonitor;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
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
        RobotContainer robot = RobotContainer.getInstance();
        driver_controller_
                .rightStick()
                .onTrue(SwerveSubsystem.getInstance().toggleFieldCentric().ignoringDisable(true));
        // Hold a button: the robot is in that state; let go: back to DRIVE
        whileHeld(driver_controller_.rightTrigger(), RobotStates.SHOOT);
        whileHeld(driver_controller_.leftTrigger(), RobotStates.AIM);
        whileHeld(driver_controller_.rightBumper(), RobotStates.INTAKE);
        whileHeld(driver_controller_.y(), RobotStates.MANUAL_HUB);
        whileHeld(driver_controller_.b(), RobotStates.MANUAL_PASS);
        whileHeld(driver_controller_.a(), RobotStates.OUTTAKE);
        whileHeld(driver_controller_.x(), RobotStates.BRAKE);
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
     * While {@code button} is held the robot wants {@code state}; on release it goes back to DRIVE,
     * unless another button asked for a different state since.
     */
    private static void whileHeld(Trigger button, RobotStates state) {
        RobotContainer robot = RobotContainer.getInstance();
        button.whileTrue(
                Commands.startEnd(
                                () -> robot.setWantedState(state),
                                () -> {
                                    if (robot.getWantedState() == state) {
                                        robot.setWantedState(RobotStates.DRIVE);
                                    }
                                })
                        .ignoringDisable(true));
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
