package frc.robot.lib2026;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class HubMonitor {

    String gameData;
    ActiveAlliance first_active_alliance_;

    public enum ActiveAlliance {
        RED_ACTIVE,
        BLUE_ACTIVE,
        INVALID,
        BOTH_ACTIVE;
    }

   /** Updates the first_active_alliance_ variable with the first alliance that has an active hub. */
    public void updateActiveAlliance() {
        first_active_alliance_ = firstActiveAlliance();
    }
    public void manualUpdateActiveAlliance(ActiveAlliance alliance) {
        first_active_alliance_ = alliance;
    }

   /**
    * Determines the first alliance that has an active hub based on the game data provided by the DriverStation.
    * @return
    */
    public ActiveAlliance firstActiveAlliance() {

        gameData = DriverStation.getGameSpecificMessage();

        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
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
     * @param matchTime
     * @return
     */
    public boolean getActive(double matchTime) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Blue) {
                return (getActiveAlliance(matchTime) == ActiveAlliance.BLUE_ACTIVE)
                        || (getActiveAlliance(matchTime) == ActiveAlliance.BOTH_ACTIVE);
            } else {
                return (getActiveAlliance(matchTime) == ActiveAlliance.RED_ACTIVE)
                        || (getActiveAlliance(matchTime) == ActiveAlliance.BOTH_ACTIVE);
            }
        } else {
            return false;
        }
    }

    /*
     * MatchTime: in seconds
     * FAA: First Active Alliance
     * SAA: Second Active Alliance
     * Auto: 160-140 or 20-10 in match time, Both
     * Transition Shift: 140-130, Both
     * Shift 1: 130-105, FAA
     * Shift 2: 105-80, SAA
     * Shift 3: 80-55, FAA
     * Shift 4: 55-30, SAA
     * Endgame: 30-0, Both
     */

    // returns what team has an active hub
    // parameter: matchTime in seconds remaining
    public ActiveAlliance getActiveAlliance(double matchTime) {
        if (matchTime > 140) {
            return ActiveAlliance.BOTH_ACTIVE;
        } else if (matchTime > 130) {
            return ActiveAlliance.BOTH_ACTIVE;
        } else if (matchTime > 105) {
            return first_active_alliance_;
        } else if (matchTime > 80) {
            if (first_active_alliance_ == ActiveAlliance.RED_ACTIVE) {
                return ActiveAlliance.BLUE_ACTIVE;
            }
            return ActiveAlliance.RED_ACTIVE;
        } else if (matchTime > 55) {
            return first_active_alliance_;
        } else if (matchTime > 30) {
            if (first_active_alliance_ == ActiveAlliance.RED_ACTIVE) {
                return ActiveAlliance.BLUE_ACTIVE;
            }
            return ActiveAlliance.RED_ACTIVE;
        } else if (matchTime > 0) {
            return ActiveAlliance.BOTH_ACTIVE;
        } else {
            return ActiveAlliance.INVALID;
        }
    }
}
