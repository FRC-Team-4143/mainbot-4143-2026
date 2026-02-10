package frc.robot.lib2026;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class HubMonitor {

    String game_data_;
    ActiveAlliance first_active_alliance_;

    public enum ActiveAlliance {
        RED_ACTIVE,
        BLUE_ACTIVE,
        INVALID,
        BOTH_ACTIVE;
    }

    private final int AUTO_LENGTH = 20;
    private final int TELEOP_LENGTH = (60 * 2) + 20;
    private final int SHIFT_LENGTH = 25;
    private final int TRANSITION_LENGTH = 10;
    private final int END_GAME_LENGTH = 30;

    private final int AUTO = AUTO_LENGTH - AUTO_LENGTH; // Timer reads 20 - 0 (Entire AUTO period is active)
    private final int TRANSITION = TELEOP_LENGTH - TRANSITION_LENGTH; // Timer ends at 2:10
    private final int SHIFT_1 = TRANSITION - SHIFT_LENGTH; // Timer ends at 1:45
    private final int SHIFT_2 = SHIFT_1 - SHIFT_LENGTH; // Timer ends at 1:20
    private final int SHIFT_3 = SHIFT_2 - SHIFT_LENGTH; // Timer ends at 0:55
    private final int SHIFT_4 = SHIFT_3 - SHIFT_LENGTH; // Timer ends at 0:30
    private final int END_GAME = SHIFT_4 - END_GAME_LENGTH; // Timer ends at 0:00
    // AUTO happens to fall in the end game time period (20 - 0)

   /** Updates the first_active_alliance_ variable with the first alliance that has an active hub. */
    public void seedActiveAlliance() {
        first_active_alliance_ = firstActiveAlliance();
    }

    /**
     * Manual update for the first_active_alliance_ tracker
     * @param alliance 
     */
    public void seedActiveAlliance(ActiveAlliance alliance) {
        first_active_alliance_ = alliance;
    }

   /**
    * Determines the first alliance that has an active hub based on the game data provided by the DriverStation.
    * @return ActiveAlliance containing the alliance provided by Game Data
    */
    private ActiveAlliance firstActiveAlliance() {

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
     * 
     * @param match_time match time to check hub status
     * @return true/false representing your alliances hub active status
     */
    public boolean getActive(double match_time) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Blue) {
                return (getActiveAlliance(match_time) == ActiveAlliance.BLUE_ACTIVE)
                        || (getActiveAlliance(match_time) == ActiveAlliance.BOTH_ACTIVE);
            } else {
                return (getActiveAlliance(match_time) == ActiveAlliance.RED_ACTIVE)
                        || (getActiveAlliance(match_time) == ActiveAlliance.BOTH_ACTIVE);
            }
        } else {
            return false;
        }
    }

    /**
     * Determine ActiveAlliance for a given time in the match
     * @param match_time 
     * @return ActiveAlliance at provided match time
     */
    private ActiveAlliance getActiveAlliance(double match_time) {
        if(match_time > TRANSITION) return ActiveAlliance.BOTH_ACTIVE;
        if(match_time > SHIFT_1) return (first_active_alliance_ == ActiveAlliance.RED_ACTIVE) ? ActiveAlliance.RED_ACTIVE : ActiveAlliance.BLUE_ACTIVE;
        if(match_time > SHIFT_2) return (first_active_alliance_ == ActiveAlliance.RED_ACTIVE) ? ActiveAlliance.BLUE_ACTIVE : ActiveAlliance.RED_ACTIVE;
        if(match_time > SHIFT_3) return (first_active_alliance_ == ActiveAlliance.RED_ACTIVE) ? ActiveAlliance.RED_ACTIVE : ActiveAlliance.BLUE_ACTIVE;
        if(match_time > SHIFT_4) return (first_active_alliance_ == ActiveAlliance.RED_ACTIVE) ? ActiveAlliance.BLUE_ACTIVE : ActiveAlliance.RED_ACTIVE;
        if(match_time > END_GAME || match_time > AUTO) return ActiveAlliance.BOTH_ACTIVE;
        return ActiveAlliance.INVALID;
    }
}
