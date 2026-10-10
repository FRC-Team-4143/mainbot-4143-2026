package frc.robot;

/**
 * The robot's super state machine states, owned by {@link RobotContainer}. Buttons and autos ask
 * for one of these with {@code RobotContainer.getInstance().setWantedState(...)}; every loop the
 * container asks each subsystem for the state the robot's state needs.
 */
public enum RobotStates {
    /** Disabled: shooter idle */
    IDLE,
    /** Teleop driving with no button held, shooter idle */
    DRIVE,
    /** Driving with the flywheel kept at the target speed, so the next shot is quick */
    TRACK,
    /** Autonomous: the auto sets subsystem states itself, so the robot asks for nothing */
    AUTO,
    /** Aim at the target (shooter aiming, swerve rotation lock) without shooting */
    AIM,
    /** Shoot: spin up, then feed (intake squeeze) once the shooter is ready */
    SHOOT,
    /** Shoot at the hub with a fixed flywheel speed (vision not working) */
    MANUAL_HUB,
    /** Pass with a fixed flywheel speed and hood angle (vision not working) */
    MANUAL_PASS,
    /** Intake fuel */
    INTAKE,
    /** Eject fuel out of the intake */
    OUTTAKE,
    /** Cross the wheels so the robot can't be pushed */
    BRAKE,
    /**
     * Test mode: maintenance (homing, tuning, spin down) from the dashboard. Like AUTO, the robot
     * asks for nothing, so the dashboard buttons set subsystem states directly.
     */
    MAINTENANCE
}
