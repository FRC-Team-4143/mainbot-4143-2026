package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib2026.FieldTargets;
import frc.robot.subsystems.climber.ClimberConstants.ClimberStates;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.hopper.HopperConstants.HopperStates;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.localization.LocalizationConstants.LocalizationStates;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class ControlCommands {

    /**
     * This command aims the robot at the target with no intent to shoot, used for lining up shots
     * or for teleop control while aiming.
     *
     * <p>On Initialize:
     *
     * <ul>
     *   <li>Shooter: AIMING
     *   <li>Swerve: FIELD_CENTRIC_ROTATION_LOCK
     *   <li>Localization: SHOOTING_FOCUS
     * </ul>
     *
     * <p>On End:
     *
     * <ul>
     *   <li>Shooter: TRACKING
     *   <li>Swerve: FIELD_CENTRIC
     *   <li>Localization: FULL
     * </ul>
     */
    static Command aimAtTargetCommand() {
        return Commands.startEnd(
                        () -> {
                            ShooterSubsystem.getInstance().setWantedState(ShooterStates.AIMING);
                            SwerveSubsystem.getInstance()
                                    .setWantedState(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
                            LocalizationSubsystem.getInstance()
                                    .setWantedState(LocalizationStates.SHOOTING_FOCUS);
                        },
                        () -> {
                            ShooterSubsystem.getInstance().setWantedState(ShooterStates.TRACKING);
                            SwerveSubsystem.getInstance()
                                    .setWantedState(SwerveStates.FIELD_CENTRIC);
                            LocalizationSubsystem.getInstance()
                                    .setWantedState(LocalizationStates.FULL);
                        })
                .withName("Aim At Target")
                .ignoringDisable(true);
    }

    static Command advanceClimbingStage() {
        return Commands.runOnce(
                () -> {
                    if(ClimberSubsystem.getInstance().getSystemState() == ClimberStates.STOWED && IntakeSubsystem.getInstance().getSystemState() != IntakeStates.STORE){
                        IntakeSubsystem.getInstance().setWantedState(IntakeStates.STORE);
                    }
                    else if (ClimberSubsystem.getInstance().getSystemState() == ClimberStates.STOWED && IntakeSubsystem.getInstance().getSystemState() == IntakeStates.STORE) {
                        ClimberSubsystem.getInstance().setWantedState(ClimberStates.DEPLOY);
                    } else if (ClimberSubsystem.getInstance().getSystemState()
                            == ClimberStates.DEPLOY) {
                        ClimberSubsystem.getInstance().setWantedState(ClimberStates.L2);
                    }
                });
    }

    static Command reverseClimbingStage() {
        return Commands.runOnce(
                () -> {
                    if (ClimberSubsystem.getInstance().getSystemState() == ClimberStates.CLIMB_HOLD) {
                        ClimberSubsystem.getInstance().setWantedState(ClimberStates.GROUND);
                    } else if (ClimberSubsystem.getInstance().getSystemState()
                            == ClimberStates.DEPLOY) {
                        ClimberSubsystem.getInstance().setWantedState(ClimberStates.STOWED);
                    }
                });
    }

    /**
     * This command snaps robot to nearest bump crossing angle while held, returns to normal field
     * centric when released.
     *
     * <p>On Initialize:
     *
     * <ul>
     *   <li>Swerve: Sets desired rotation lock to nearest snap angle
     *   <li>Swerve: FIELD_CENTRIC_ROTATION_LOCK
     * </ul>
     *
     * <p>On End:
     *
     * <ul>
     *   <li>Swerve: FIELD_CENTRIC
     * </ul>
     */
    static Command rotateForBumpCommand() {
        return Commands.startEnd(
                        () -> {
                            SwerveSubsystem.getInstance()
                                    .setDesiredRotationLock(FieldTargets.getNearestSnapAngle());
                            SwerveSubsystem.getInstance()
                                    .setWantedState(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
                        },
                        () -> {
                            SwerveSubsystem.getInstance()
                                    .setWantedState(SwerveStates.FIELD_CENTRIC);
                        })
                .withName("Rotate For Bump")
                .ignoringDisable(true);
    }

    /**
     * This command shoots fuel at the target, used for teleop control while shooting.
     *
     * <p>On Initialize:
     *
     * <ul>
     *   <li>Shooter: SHOOT
     *   <li>Hopper: SHOOTING
     *   <li>Swerve: Sets velocity scalar to 0.25
     *   <li>Swerve: FIELD_CENTRIC_ROTATION_LOCK
     *   <li>Localization: SHOOTING_FOCUS
     * </ul>
     *
     * <p>On End:
     *
     * <ul>
     *   <li>Shooter: TRACKING
     *   <li>Hopper: IDLE
     *   <li>Swerve: Sets velocity scalar to 1.0
     *   <li>Swerve: FIELD_CENTRIC
     *   <li>Localization: FULL
     * </ul>
     */
    static Command shootFuelCommand() {
        return Commands.startEnd(
                        () -> {
                            ShooterSubsystem.getInstance().setWantedState(ShooterStates.SHOOT);
                            HopperSubsystem.getInstance().setWantedState(HopperStates.SHOOTING);
                            SwerveSubsystem.getInstance().setTeleOpVelocityScalar(0.25);
                            SwerveSubsystem.getInstance()
                                    .setWantedState(SwerveStates.FIELD_CENTRIC_ROTATION_LOCK);
                            LocalizationSubsystem.getInstance()
                                    .setWantedState(LocalizationStates.SHOOTING_FOCUS);
                        },
                        () -> {
                            ShooterSubsystem.getInstance().setWantedState(ShooterStates.TRACKING);
                            HopperSubsystem.getInstance().setWantedState(HopperStates.IDLE);
                            SwerveSubsystem.getInstance().setTeleOpVelocityScalar(1.0);
                            SwerveSubsystem.getInstance()
                                    .setWantedState(SwerveStates.FIELD_CENTRIC);
                            LocalizationSubsystem.getInstance()
                                    .setWantedState(LocalizationStates.FULL);
                        })
                .withName("Shoot Fuel")
                .ignoringDisable(true);
    }

    /**
     * This command shoots fuel at the target with a fixed flywheel speed, used for teleop control
     * while shooting when the vision tracking is not working.
     *
     * <p>On Initialize:
     *
     * <ul>
     *   <li>Shooter: MANUAL_HUB
     *   <li>Hopper: SHOOTING
     * </ul>
     *
     * <p>On End:
     *
     * <ul>
     *   <li>Shooter: TRACKING
     *   <li>Hopper: IDLE
     * </ul>
     */
    static Command manualShootFuelCommand() {
        return Commands.startEnd(
                        () -> {
                            ShooterSubsystem.getInstance().setWantedState(ShooterStates.MANUAL_HUB);
                            HopperSubsystem.getInstance().setWantedState(HopperStates.SHOOTING);
                        },
                        () -> {
                            ShooterSubsystem.getInstance().setWantedState(ShooterStates.TRACKING);
                            HopperSubsystem.getInstance().setWantedState(HopperStates.IDLE);
                        })
                .withName("Shoot Fuel Manual")
                .ignoringDisable(true);
    }

    /**
     * This command shoots fuel at the target with a fixed flywheel speed and hood angle for
     * passing, used for teleop control while passing when the vision tracking is not working.
     *
     * <p>On Initialize:
     *
     * <ul>
     *   <li>Shooter: MANUAL_PASS
     *   <li>Hopper: SHOOTING
     * </ul>
     *
     * <p>On End:
     *
     * <ul>
     *   <li>Shooter: TRACKING
     *   <li>Hopper: IDLE
     * </ul>
     */
    static Command manualPassFuelCommand() {
        return Commands.startEnd(
                        () -> {
                            ShooterSubsystem.getInstance()
                                    .setWantedState(ShooterStates.MANUAL_PASS);
                            HopperSubsystem.getInstance().setWantedState(HopperStates.SHOOTING);
                        },
                        () -> {
                            ShooterSubsystem.getInstance().setWantedState(ShooterStates.TRACKING);
                            HopperSubsystem.getInstance().setWantedState(HopperStates.IDLE);
                        })
                .withName("Pass Fuel Manual")
                .ignoringDisable(true);
    }

    /**
     * This command intakes fuel, used for teleop control while intaking.
     *
     * <p>On Initialize:
     *
     * <ul>
     *   <li>Intake: INTAKE
     * </ul>
     *
     * <p>On End:
     *
     * <ul>
     *   <li>Intake: IDLE
     * </ul>
     */
    static Command intakeFuelCommand() {
        return Commands.startEnd(
                        () -> {
                            IntakeSubsystem.getInstance().setWantedState(IntakeStates.INTAKE);
                        },
                        () -> {
                            IntakeSubsystem.getInstance().setWantedState(IntakeStates.IDLE);
                        })
                .withName("Intake Fuel")
                .ignoringDisable(true);
    }

    /**
     * This command stores the intake mechanism.
     *
     * <p>On Execute:
     *
     * <ul>
     *   <li>Intake: STORE
     * </ul>
     */
    static Command storeIntakeCommand() {
        return Commands.runOnce(
                        () -> {
                            IntakeSubsystem.getInstance().setWantedState(IntakeStates.STORE);
                        })
                .withName("Store Intake")
                .ignoringDisable(true);
    }
}
