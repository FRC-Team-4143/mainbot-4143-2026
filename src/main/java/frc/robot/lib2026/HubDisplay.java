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

    /** Initialize SmartDashboard fields. Call once during robotInit(). */
    public static void init() {
        SmartDashboard.putBoolean("Hub/Active", false);
        SmartDashboard.putString("Hub/LastTransition", "None");
        SmartDashboard.putNumber("Hub/LastTransitionTime", -1.0);
        initialized = true;
    }

    /**
     * Call each robotPeriodic with the current matchTime. Will publish current state and log +
     * publish any transitions (off->on or on->off).
     */
    public static void update(double matchTime) {
        if (!initialized) init();

        boolean active = HubMonitor.isHubActive(matchTime);
        SmartDashboard.putBoolean("Hub/Active", active);

        // Periodic Elastic-friendly active payload (JSON-like string)
        try {
            String activeStructured =
                    String.format(
                            "{\"event\":\"hub_active\",\"active\":%b,\"match_time\":%.1f,\"source\":\"HubDisplay\"}",
                            active, matchTime);
            DogLog.log("Elastic/Hub/Active", activeStructured);
            // also emit simple keyed fields for easier indexing
            DogLog.log("Elastic/Hub/Active/active", active);
            DogLog.log("Elastic/Hub/Active/match_time", matchTime);
        } catch (Exception e) {
            // don't let logging break robot loop
        }

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
            // Structured payload for Elastic: { event: hub_transition, transition: string,
            // match_time: double, active: bool }
            // Fallback to a JSON-like string since DogLog.log doesn't accept Map in this build
            String structured =
                    String.format(
                            "{\"event\":\"hub_transition\",\"transition\":\"%s\",\"match_time\":%.1f,\"active\":%b,\"source\":\"HubDisplay\"}",
                            transition, matchTime, active);
            DogLog.log("Elastic/Hub/Transition", structured);
            // also emit simple keyed fields for easier indexing
            DogLog.log("Elastic/Hub/Transition/transition", transition);
            DogLog.log("Elastic/Hub/Transition/match_time", matchTime);
            DogLog.log("Elastic/Hub/Transition/active", active);
        }

        prevState = active;
    }
}
