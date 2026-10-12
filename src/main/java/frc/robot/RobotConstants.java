package frc.robot;

import com.marswars.subsystem.MwConstants;
import com.marswars.util.RobotIdentity;

/**
 * The robot's own constants ({@link RobotContainer}'s {@code CONSTANTS}), the way each subsystem
 * has a {@code *Constants} class.
 */
public class RobotConstants extends MwConstants {

    // =============================================================================
    // ROBOT IDENTITY
    // =============================================================================
    // Static, not per-instance: subsystems read it while RobotContainer (which owns CONSTANTS) is
    // still being constructed, e.g. SwerveConstants.create() picks its variant from it.

    /** The robots this code can run on; per-robot constants variants switch on {@link #ROBOT}. */
    public enum Robot {
        ALPHA_BOT,
        BETA_BOT,
        SIM_BOT
    }

    /** The robot the code is running on, from the burned "RobotName" (SimBot in simulation). */
    public static final Robot ROBOT = RobotIdentity.getInstance().getRobot(Robot.class);

    // =============================================================================
    // DRIVING
    // =============================================================================

    /** Driver speed scale while driving normally (1 = full speed). */
    public final double DRIVE_SPEED_SCALE = 1.0;

    /** Driver speed scale while shooting, so the robot can creep without spoiling the shot. */
    public final double SHOOTING_SPEED_SCALE = 0.25;
}
