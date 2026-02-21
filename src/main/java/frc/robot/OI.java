// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.gamestates.GameStatesSubsystem;
import frc.robot.subsystems.gamestates.GameStatesConstants;
import frc.robot.subsystems.gamestates.GameStatesConstants.GameStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
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

        driver_controller_.rightStick().onTrue(SwerveSubsystem.getInstance().toggleFieldCentric());

        driver_controller_
                .a()
                .whileTrue(
                        Commands.startEnd(
                                () -> {
                                    ShooterSubsystem.getInstance()
                                            .setWantedState(ShooterStates.SHOOT);
                                    SwerveSubsystem.getInstance()
                                            .setWantedState(
                                                    SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
                                    GameStatesSubsystem.getInstance()
                                            .setWantedState(
                                                    GameStates.SCORE);
                                },
                                () -> {
                                    ShooterSubsystem.getInstance()
                                            .setWantedState(ShooterStates.TRACKING);
                                    SwerveSubsystem.getInstance()
                                            .setWantedState(SwerveStates.FIELD_CENTRIC);
                                    GameStatesSubsystem.getInstance()
                                            .setWantedState(
                                                    GameStates.HOLD);
                                }));
        driver_controller_
                .b()
                .whileTrue(
                        Commands.startEnd(
                                () ->
                                        IntakeSubsystem.getInstance()
                                                .setWantedState(IntakeStates.ROLLING),
                                () ->
                                        IntakeSubsystem.getInstance()
                                                .setWantedState(IntakeStates.CLOSED)));
    }

    /**
     * @return driver controller left joystick x axis scaled quadratically
     */
    public static double getDriverJoystickLeftX() {
        return driver_controller_.getLeftX();
    }

    /**
     * @return driver controller left joystick y axis scaled quadratically
     */
    public static double getDriverJoystickLeftY() {
        return driver_controller_.getLeftY();
    }

    /**
     * @return driver controller right joystick x axis scaled quadratically
     */
    public static double getDriverJoystickRightX() {
        return driver_controller_.getRightX();
    }

    public static double getOperatorJoystickRightX() {
        return operator_controller_.getRightX();
    }

    public static double getOperatorJoystickRightY() {
        return operator_controller_.getRightY();
    }

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
}
