package frc.robot;

import com.marswars.util.RobotIdentity;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;

/**
 * The robots this code can run on. The active robot is resolved once from MW-Lib's {@link
 * RobotIdentity} (the "RobotName" preference burned onto the RoboRIO, or SimBot/ROBOT_NAME env var
 * in simulation). Subsystems with per-robot constants switch on {@link #current()} to pick their
 * constants variant.
 */
public enum RobotId {
    ALPHA_BOT("AlphaBot"),
    BETA_BOT("BetaBot"),
    SIM_BOT("SimBot");

    /** The burned RobotName string this ID matches. */
    public final String robot_name;

    /** Subsystems (by MwSubsystem name) that should not run on this robot. */
    public final List<String> disabled_subsystems = List.of();

    private RobotId(String robot_name) {
        this.robot_name = robot_name;
    }

    private static RobotId current_ = null;

    /** The robot the code is currently running on. Unknown names fall back to ALPHA_BOT. */
    public static synchronized RobotId current() {
        if (current_ == null) {
            String name = RobotIdentity.getInstance().getRobotName();
            for (RobotId id : values()) {
                if (id.robot_name.equals(name)) {
                    current_ = id;
                    break;
                }
            }
            if (current_ == null) {
                DriverStation.reportError(
                        "Unknown robot name '" + name + "', defaulting to AlphaBot", false);
                current_ = ALPHA_BOT;
            }
            DataLogManager.log("Robot ID: " + current_.name());
        }
        return current_;
    }
}
