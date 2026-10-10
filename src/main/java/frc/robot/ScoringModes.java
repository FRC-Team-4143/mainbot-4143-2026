package frc.robot;

/**
 * Where the robot is on the field decides what shooting means. {@link RobotContainer} works this
 * out every loop from the robot's pose.
 */
public enum ScoringModes {
    /** Holding game pieces, not actively scoring */
    HOLD,
    /** In our alliance zone with the goal active: shoot at the hub */
    SCORE,
    /** In the neutral or opponent zone (or pass override): pass to our side */
    PASS,
    /** Autonomous: the auto picks the target */
    AUTO
}
