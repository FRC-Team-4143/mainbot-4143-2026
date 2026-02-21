package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Command to align the robot with the tower's vertical support using Time-of-Flight sensors.
 * This command uses a "line following" approach where the ToF sensors on the climber
 * detect the vertical tower support and provide feedback to strafe the robot into alignment.
 * 
 * The robot will continue moving forward (or at a specified speed) while making lateral
 * corrections to stay centered on the tower support.
 */
public class AlignToTowerCommand extends Command {
    private final ClimberSubsystem climber_;
    private final SwerveSubsystem swerve_;
    private final LocalizationSubsystem localization_;
    private final double forwardSpeed_;
    private final double maxStrafeSpeed_;
    private final boolean stopWhenAligned_;
    private final boolean maintainHeading_;

    /**
     * Creates a new AlignToTowerCommand.
     * 
     * @param forwardSpeed Forward speed in meters per second (positive = forward)
     * @param maxStrafeSpeed Maximum lateral correction speed in meters per second
     * @param stopWhenAligned If true, command ends when aligned; if false, continues indefinitely
     * @param maintainHeading If true, maintains current heading while aligning
     */
    public AlignToTowerCommand(double forwardSpeed, double maxStrafeSpeed, 
                               boolean stopWhenAligned, boolean maintainHeading) {
        climber_ = ClimberSubsystem.getInstance();
        swerve_ = SwerveSubsystem.getInstance();
        localization_ = LocalizationSubsystem.getInstance();
        
        this.forwardSpeed_ = forwardSpeed;
        this.maxStrafeSpeed_ = maxStrafeSpeed;
        this.stopWhenAligned_ = stopWhenAligned;
        this.maintainHeading_ = maintainHeading;
        
        // Note: MwSubsystem doesn't extend WPILib Subsystem, so we manage state manually
        // The swerve subsystem state will be changed in initialize() and end()
    }

    /**
     * Convenience constructor that creates a command that stops when aligned and maintains heading.
     * Uses default forward speed of 0.3 m/s and max strafe speed of 0.5 m/s.
     */
    public AlignToTowerCommand() {
        this(0.3, 0.5, true, true);
    }

    @Override
    public void initialize() {
        // Switch to chassis speed rotation lock mode for precise control
        swerve_.setWantedState(SwerveStates.CHASSIS_SPEED_ROTATION_LOCK);
    }

    @Override
    public void execute() {
        // Get lateral correction from climber subsystem based on ToF sensor readings
        double lateralCorrection = climber_.getLateralCorrectionValue(maxStrafeSpeed_);
        
        // Get current robot rotation for field-relative movement
        Rotation2d currentRotation = localization_.getFieldPose().getRotation();
        
        // Create chassis speeds for field-relative movement
        // vx = forward/backward, vy = strafe left/right, omega = rotation
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            forwardSpeed_,      // X: Forward (positive = forward)
            lateralCorrection,  // Y: Strafe (positive = left in field coords)
            0.0,                // Omega: No rotation
            currentRotation
        );
        
        // Determine heading to maintain
        Rotation2d targetHeading = maintainHeading_ 
            ? currentRotation
            : new Rotation2d();
        
        // Apply the correction with rotation lock
        swerve_.setChassisSpeedRotationLock(speeds, targetHeading);
    }

    @Override
    public void end(boolean interrupted) {
        // Return to field-centric driving when command ends
        swerve_.setWantedState(SwerveStates.FIELD_CENTRIC);
    }

    @Override
    public boolean isFinished() {
        if (!stopWhenAligned_) {
            return false; // Run indefinitely until interrupted
        }
        
        // End when aligned or if sensors lose the tower
        return climber_.isAlignedWithTower() || !climber_.areSensorsDetectingTower();
    }
}
