package frc.robot.lib2026;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class HubMonitor {

    String gameData;
    ActiveAlliance SAA; // Second Active Alliance

    public enum ActiveAlliance {
        RED_ACTIVE,
        BLUE_ACTIVE,
        INVALID,
        BOTH_ACTIVE;
    }

    // returns the first alliance that has an active hub
    public ActiveAlliance firstActiveAlliance() { //some changes may need to be made to SAA

        gameData = DriverStation.getGameSpecificMessage();

        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B':
                    SAA = ActiveAlliance.BLUE_ACTIVE;
                    return ActiveAlliance.RED_ACTIVE;
                case 'R':
                    SAA = ActiveAlliance.RED_ACTIVE;
                    return ActiveAlliance.BLUE_ACTIVE;
                default:
                    return ActiveAlliance.INVALID;
            }
        } else {
            return ActiveAlliance.INVALID;
        }
    }

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
     * Auto: 160-140, Both
     * Transition Shift: 140-130, Both
     * Shift 1: 130-105, FAA
     * Shift 2: 105-80, SAA
     * Shift 3: 80-55, FAA
     * Shift 4: 55-30, SAA
     * Endgame: 30-0, Both
     */
    public ActiveAlliance getActiveAlliance(double matchTime) {
        if (matchTime > 140) {
            return ActiveAlliance.BOTH_ACTIVE;
        } else if (matchTime > 130) {
            return ActiveAlliance.BOTH_ACTIVE;
        } else if (matchTime > 105) {
            return firstActiveAlliance();
        } else if (matchTime > 80) {
            return SAA;
        } else if (matchTime > 55) {
            return firstActiveAlliance();
        } else if (matchTime > 30) {
            return SAA;
        } else if (matchTime > 0) {
            return ActiveAlliance.BOTH_ACTIVE;
        } else {
            return ActiveAlliance.INVALID;
        }
    }
}
