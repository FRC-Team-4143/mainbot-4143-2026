package frc.robot.lib2026;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Small helper that publishes the hub active state and detects transitions (on->off, off->on).
 * Displays current state and the last transition time on SmartDashboard.
 */
public class HubDisplay {
    private static boolean prevState = false;
    private static boolean initialized = false;

    /**
     * Initialize SmartDashboard fields. Call once during robotInit().
     */
    public static void init() {
        SmartDashboard.putBoolean("Hub/Active", false);
        SmartDashboard.putString("Hub/LastTransition", "None");
        SmartDashboard.putNumber("Hub/LastTransitionTime", -1.0);
        initialized = true;
    }

    /**
     * Call each robotPeriodic with the current matchTime. Will publish current state and
     * log + publish any transitions (off->on or on->off).
     */
    public static void update(double matchTime) {
        if (!initialized) init();

        boolean active = HubMonitor.isHubActive(matchTime);
        SmartDashboard.putBoolean("Hub/Active", active);

        // On first run, just record state
        if (!initialized) {
            prevState = active;
            initialized = true;
            return;
        }

        if (active != prevState) {
            String transition = active ? "OFF->ON" : "ON->OFF";
            String message = String.format("%s at %.1fs", transition, matchTime);
            SmartDashboard.putString("Hub/LastTransition", message);
            SmartDashboard.putNumber("Hub/LastTransitionTime", matchTime);
            DogLog.log("HubDisplay/Transition", message);
        }

        prevState = active;
    }
}
