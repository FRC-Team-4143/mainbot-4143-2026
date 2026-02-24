// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.hopper.HopperConstants.HopperStates;
import frc.robot.lib2026.FieldTargets;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.localization.LocalizationConstants.LocalizationStates;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.Optional;

public abstract class OI {

    // Sets up both controllers
    private static CommandXboxController driver_controller_ = new CommandXboxController(0);
    private static CommandXboxController operator_controller_ = new CommandXboxController(1);

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
                "Set Shooter Zero",
                Commands.runOnce(
                        () ->
                                ShooterSubsystem.getInstance()
                                        .setWantedState(ShooterStates.SPIN_DOWN)));
        // =============================================================================
        // DRIVER CONTROLLER BINDINGS
        // =============================================================================
        driver_controller_
                .rightStick()
                .onTrue(SwerveSubsystem.getInstance().toggleFieldCentric().ignoringDisable(true));

        // =============================================================================
        // OPERATOR CONTROLLER BINDINGS
        // =============================================================================

        // =============================================================================
        // TESTING BINDINGS (THESE SHOULD BE REMOVED BEFORE COMPETITION)
        // =============================================================================

        // Shoots the robot while held, returns to tracking when released
        driver_controller_
                .rightTrigger()
                .whileTrue(
                        Commands.startEnd(
                                () -> {
                                    ShooterSubsystem.getInstance()
                                            .setWantedState(ShooterStates.SHOOT);
                                    HopperSubsystem.getInstance()
                                            .setWantedState(HopperStates.SHOOTING);
                                    SwerveSubsystem.getInstance()
                                            .setWantedState(
                                                    SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
                                    LocalizationSubsystem.getInstance()
                                            .setWantedState(LocalizationStates.SHOOTING_FOCUS);
                                },
                                () -> {
                                    ShooterSubsystem.getInstance()
                                            .setWantedState(ShooterStates.TRACKING);
                                    HopperSubsystem.getInstance().setWantedState(HopperStates.IDLE);
                                    SwerveSubsystem.getInstance()
                                            .setWantedState(SwerveStates.FIELD_CENTRIC);
                                    LocalizationSubsystem.getInstance()
                                            .setWantedState(LocalizationStates.FULL);
                                }));

        // Aims the robot at the hub while held, returns to normal tracking when released
        driver_controller_
                .leftTrigger()
                .whileTrue(
                        Commands.startEnd(
                                () -> {
                                    ShooterSubsystem.getInstance()
                                            .setWantedState(ShooterStates.AIMING);
                                    SwerveSubsystem.getInstance()
                                            .setWantedState(
                                                    SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
                                    LocalizationSubsystem.getInstance()
                                            .setWantedState(LocalizationStates.SHOOTING_FOCUS);
                                },
                                () -> {
                                    ShooterSubsystem.getInstance()
                                            .setWantedState(ShooterStates.TRACKING);
                                    SwerveSubsystem.getInstance()
                                            .setWantedState(SwerveStates.FIELD_CENTRIC);
                                    LocalizationSubsystem.getInstance()
                                            .setWantedState(LocalizationStates.FULL);
                                }));

        // Snaps robot to nearest bump crossing angle while held, returns to normal field centric when released 
        driver_controller_
                .leftStick()
                .whileTrue(
                        Commands.startEnd(
                                () -> {
                                    SwerveSubsystem.getInstance()
                                            .setDesiredRotationLock(
                                                    FieldTargets.getNearestSnapAngle());
                                    SwerveSubsystem.getInstance()
                                            .setWantedState(
                                                    SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
                                },
                                () -> {
                                    SwerveSubsystem.getInstance()
                                            .setWantedState(SwerveStates.FIELD_CENTRIC);
                                }));

        // Intakes the robot while held, returns to stored when released
        driver_controller_
                .rightBumper()
                .whileTrue(
                        Commands.startEnd(
                                () -> {
                                    IntakeSubsystem.getInstance()
                                            .setWantedState(IntakeStates.INTAKE);
                                },
                                () -> {
                                    IntakeSubsystem.getInstance()
                                            .setWantedState(IntakeStates.STORE);
                                }));
        
        // Used for testing chassis velocity control, should be removed before competition
        driver_controller_
                .b()
                .whileTrue(
                        Commands.startEnd(
                                () -> {
                                    SwerveSubsystem.getInstance()
                                            .setChassisSpeedRotationLock(
                                                    new ChassisSpeeds(0, -1, 0),
                                                    Rotation2d.k180deg);
                                    SwerveSubsystem.getInstance()
                                            .setWantedState(
                                                    SwerveStates.CHASSIS_SPEED_ROTATION_LOCK);
                                },
                                () -> {
                                    SwerveSubsystem.getInstance()
                                            .setWantedState(SwerveStates.FIELD_CENTRIC);
                                }));

        // Used for testing chassis velocity control, should be removed before competition
        driver_controller_
                .x()
                .whileTrue(
                        Commands.startEnd(
                                () -> {
                                    SwerveSubsystem.getInstance()
                                            .setChassisSpeedRotationLock(
                                                    new ChassisSpeeds(0, 1, 0), Rotation2d.k180deg);
                                    SwerveSubsystem.getInstance()
                                            .setWantedState(
                                                    SwerveStates.CHASSIS_SPEED_ROTATION_LOCK);
                                },
                                () -> {
                                    SwerveSubsystem.getInstance()
                                            .setWantedState(SwerveStates.FIELD_CENTRIC);
                                }));
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
