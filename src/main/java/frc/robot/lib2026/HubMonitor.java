package frc.robot.lib2026;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import java.util.Optional;

public class HubMonitor {

    private static String game_data_;
    private static ActiveAlliance first_active_alliance_;

    private static final String ACTIVE_COLOR = new Color(0, 255, 0).toHexString();
    private static final String INACTIVE_COLOR = new Color(0, 0, 0).toHexString();
    private static final String RED_ACTIVE_COLOR = new Color(255, 0, 0).toHexString();
    private static final String BLUE_ACTIVE_COLOR = new Color(0, 0, 255).toHexString();

    public enum ActiveAlliance {
        RED_ACTIVE,
        BLUE_ACTIVE,
        INVALID,
        BOTH_ACTIVE;
    }

    private static final int AUTO_LENGTH = 20;
    private static final int TELEOP_LENGTH = (60 * 2) + 20;
    private static final int SHIFT_LENGTH = 25;
    private static final int TRANSITION_LENGTH = 10;
    private static final int END_GAME_LENGTH = 30;

    private static final int AUTO =
            AUTO_LENGTH - AUTO_LENGTH; // Timer reads 20 - 0 (Entire AUTO period is active)
    private static final int TRANSITION = TELEOP_LENGTH - TRANSITION_LENGTH; // Timer ends at 2:10
    private static final int SHIFT_1 = TRANSITION - SHIFT_LENGTH; // Timer ends at 1:45
    private static final int SHIFT_2 = SHIFT_1 - SHIFT_LENGTH; // Timer ends at 1:20
    private static final int SHIFT_3 = SHIFT_2 - SHIFT_LENGTH; // Timer ends at 0:55
    private static final int SHIFT_4 = SHIFT_3 - SHIFT_LENGTH; // Timer ends at 0:30
    private static final int END_GAME = SHIFT_4 - END_GAME_LENGTH; // Timer ends at 0:00

    // AUTO happens to fall in the end game time period (20 - 0)

    /**
     * Updates the first_active_alliance_ variable with the first alliance that has an active hub.
     */
    public static void seedActiveAlliance() {
        seedActiveAlliance(firstActiveAlliance());
    }

    /**
     * Manual update for the first_active_alliance_ tracker
     *
     * @param alliance
     */
    public static void seedActiveAlliance(ActiveAlliance alliance) {
        first_active_alliance_ = alliance;
        DogLog.log("HubMonitor/GameData", first_active_alliance_);
    }

    /**
     * Gets if the first active alliance has been sucessuflly recieved from game data.
     *
     * @return true if valid alliance was received.
     */
    public static boolean isFirstActiveAllianceValid() {
        return !(first_active_alliance_ == ActiveAlliance.INVALID);
    }

    /**
     * Determines the first alliance that has an active hub based on the game data provided by the
     * DriverStation.
     *
     * @return ActiveAlliance containing the alliance provided by Game Data
     */
    private static ActiveAlliance firstActiveAlliance() {

        game_data_ = DriverStation.getGameSpecificMessage();
        if (game_data_.length() > 0) {
            switch (game_data_.charAt(0)) {
                case 'B':
                    return ActiveAlliance.RED_ACTIVE;
                case 'R':
                    return ActiveAlliance.BLUE_ACTIVE;
                default:
                    return ActiveAlliance.INVALID;
            }
        } else {
            return ActiveAlliance.INVALID;
        }
    }

    /**
     * @param match_time match time to check hub status
     * @return true/false representing your alliances hub active status
     */
    public static boolean isHubActive(double match_time) {
        ActiveAlliance active = getActiveAlliance(match_time);
        boolean status = false;

        // Stop check if invalid
        if (active == ActiveAlliance.INVALID) {
            status = false;
        }
        // Stop check if both active
        if (active == ActiveAlliance.BOTH_ACTIVE) {
            status = true;
        }
        // Determine active status based on current allaince
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && status != true) {
            status =
                    (alliance.get() == Alliance.Blue)
                            ? active == ActiveAlliance.BLUE_ACTIVE
                            : active == ActiveAlliance.RED_ACTIVE;
        }

        DogLog.log(
                "HubMonitor/Active", status ? HubMonitor.ACTIVE_COLOR : HubMonitor.INACTIVE_COLOR);
        DogLog.log("HubMonitor/Alliance", getActiveColor(active));

        // Log countdown timer for current stage
        logStageCountdown(match_time);

        return status;
    }

    /**
     * Determine ActiveAlliance for a given time in the match
     *
     * @param match_time
     * @return ActiveAlliance at provided match time
     */
    private static ActiveAlliance getActiveAlliance(double match_time) {
        if (match_time > TRANSITION) return ActiveAlliance.BOTH_ACTIVE;
        if (match_time > SHIFT_1)
            return (first_active_alliance_ == ActiveAlliance.RED_ACTIVE)
                    ? ActiveAlliance.RED_ACTIVE
                    : ActiveAlliance.BLUE_ACTIVE;
        if (match_time > SHIFT_2)
            return (first_active_alliance_ == ActiveAlliance.RED_ACTIVE)
                    ? ActiveAlliance.BLUE_ACTIVE
                    : ActiveAlliance.RED_ACTIVE;
        if (match_time > SHIFT_3)
            return (first_active_alliance_ == ActiveAlliance.RED_ACTIVE)
                    ? ActiveAlliance.RED_ACTIVE
                    : ActiveAlliance.BLUE_ACTIVE;
        if (match_time > SHIFT_4)
            return (first_active_alliance_ == ActiveAlliance.RED_ACTIVE)
                    ? ActiveAlliance.BLUE_ACTIVE
                    : ActiveAlliance.RED_ACTIVE;
        if (match_time > END_GAME || match_time > AUTO) return ActiveAlliance.BOTH_ACTIVE;
        return ActiveAlliance.INVALID;
    }

    /**
     * Gets the color associated with the active alliance
     *
     * @param active current active allaince
     * @return hex string array containing the active alliance
     */
    private static String[] getActiveColor(ActiveAlliance active) {
        return switch (active) {
            case RED_ACTIVE:
                yield new String[] {RED_ACTIVE_COLOR};
            case BLUE_ACTIVE:
                yield new String[] {BLUE_ACTIVE_COLOR};
            case BOTH_ACTIVE:
                yield new String[] {RED_ACTIVE_COLOR, BLUE_ACTIVE_COLOR};
            case INVALID:
                yield new String[] {INACTIVE_COLOR};
            default:
                yield new String[] {INACTIVE_COLOR};
        };
    }

    /**
     * Logs the countdown timer for the current stage based on match time
     *
     * @param match_time current match time
     */
    private static void logStageCountdown(double match_time) {
        String stageName;

        if (match_time > TRANSITION) {
            stageName = "TRANSITION";
        } else if (match_time > SHIFT_1) {
            stageName = "SHIFT 1";
        } else if (match_time > SHIFT_2) {
            stageName = "SHIFT 2";
        } else if (match_time > SHIFT_3) {
            stageName = "SHIFT 3";
        } else if (match_time > SHIFT_4) {
            stageName = "SHIFT 4";
        } else if (match_time > END_GAME) {
            stageName = "END GAME";
        } else {
            stageName = "INVALID";
        }

        DogLog.log("HubMonitor/CurrentShift", stageName);
        // Log time until this alliance becomes inactive. Previously this value
        // represented only time since the stage threshold; that caused confusion
        // for back-to-back shifts where the alliance would immediately change.
        // We now compute time until inactive (seconds remaining until the hub is
        // no longer active for the driver's alliance) and log it to the same
        // topic so dashboards update without further changes.
        double timeUntilInactive = computeTimeUntilInactiveForAlliance(match_time);
        DogLog.log("HubMonitor/ShiftTimeRemaining", timeUntilInactive);
    }

    /**
     * Returns true if the specified alliance would be considered active at the provided match_time.
     */
    private static boolean isAllianceActiveAtTime(Alliance alliance, double match_time) {
        ActiveAlliance active = getActiveAlliance(match_time);
        if (active == ActiveAlliance.BOTH_ACTIVE) return true;
        if (alliance == Alliance.Blue) return active == ActiveAlliance.BLUE_ACTIVE;
        return active == ActiveAlliance.RED_ACTIVE;
    }

    /**
     * Compute seconds until the hub becomes inactive for the driver's alliance. If the hub is not
     * currently active for the alliance, returns 0.
     *
     * @param match_time current match time
     * @return seconds until inactive (>= 0)
     */
    private static double computeTimeUntilInactiveForAlliance(double match_time) {
        Optional<Alliance> allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isEmpty()) return 0;
        Alliance alliance = allianceOpt.get();

        // If not currently active for this alliance, return 0
        if (!isAllianceActiveAtTime(alliance, match_time)) return 0;

        // Candidate breakpoints (times remaining) where active-alliance can change.
        double[] breakpoints =
                new double[] {TRANSITION, SHIFT_1, SHIFT_2, SHIFT_3, SHIFT_4, END_GAME, AUTO, 0.0};

        // Find the earliest future time (i.e., smaller match_time) when the alliance
        // is no longer active. Since match_time counts down, we look for the largest
        // breakpoint less than current match_time where the alliance becomes inactive
        for (double bp : breakpoints) {
            if (bp >= match_time) continue; // only consider future times (smaller remaining time)

            // Check just below the breakpoint to determine the active mapping after the transition
            double check_time = bp - 1e-6;
            if (!isAllianceActiveAtTime(alliance, check_time)) {
                // Hub becomes inactive at this breakpoint (or immediately after)
                return match_time - bp;
            }
        }

        // If none of the breakpoints cause inactivity, then hub remains active until time 0
        return match_time;
    }

    /**
     * Public accessor for time until the hub becomes inactive for the driver's alliance. Returns 0
     * if the hub is not currently active for the alliance.
     *
     * @param match_time current match time
     * @return seconds until inactive (>= 0)
     */
    public static double getTimeUntilInactiveForAlliance(double match_time) {
        return computeTimeUntilInactiveForAlliance(match_time);
    }
}
