package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.ClimberConstants.ClimberStates;
import frc.robot.subsystems.climber.ClimberSubsystem;

/**
 * Command to align the robot with the tower's vertical support using Time-of-Flight sensors.
 * This command transitions the climber to the APPROACHING state, which uses discrete sensor
 * feedback to perform line-following of the tower's vertical support.
 * 
 * The climber subsystem state machine will:
 * - Drive forward while both sensors see the tower (aligned)
 * - Strafe left if only the left sensor sees the tower (too far right)
 * - Strafe right if only the right sensor sees the tower (too far left)
 * - Stop if both sensors lose the tower
 */
public class AlignToTowerCommand extends Command {
    private final ClimberSubsystem climber_;
    private final boolean stopWhenAligned_;

    /**
     * Creates a new AlignToTowerCommand.
     * 
     * @param stopWhenAligned If true, command ends when aligned; if false, continues until interrupted
     */
    public AlignToTowerCommand(boolean stopWhenAligned) {
        climber_ = ClimberSubsystem.getInstance();
        this.stopWhenAligned_ = stopWhenAligned;
        
        // Note: We don't add requirements because MwSubsystem doesn't extend WPILib Subsystem
        // The state machine handles coordination with SwerveSubsystem
    }

    /**
     * Convenience constructor that continues approaching until interrupted.
     * Use this for continuous tower approach during climb sequences.
     */
    public AlignToTowerCommand() {
        this(false);
    }

    @Override
    public void initialize() {
        // Transition climber to APPROACHING state
        // The state machine will handle all sensor logic and swerve control
        climber_.setWantedState(ClimberStates.APPROACHING);
    }

    @Override
    public void execute() {
        // State machine handles everything in updateLogic()
        // Nothing needed here - just monitor for completion
    }

    @Override
    public void end(boolean interrupted) {
        // Return climber to DEPLOY state when done
        climber_.setWantedState(ClimberStates.DEPLOY);
    }

    @Override
    public boolean isFinished() {
        if (!stopWhenAligned_) {
            return false; // Run indefinitely until interrupted
        }
        
        // Check if we've lost the tower (both sensors can't see it)
        if (!climber_.canSeeTower()) {
            return true; // Lost the line - stop
        }
        
        // Optionally end when aligned (both sensors see tower)
        return climber_.isAlignedWithTower();
    }
}
