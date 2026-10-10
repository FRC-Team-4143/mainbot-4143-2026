package frc.robot;

/**
 * The robot's super state machine states, owned by {@link RobotContainer}. Buttons and autos ask
 * for one of these with {@code RobotContainer.getInstance().setWantedState(...)}; the container
 * then requests the matching subsystem states.
 */
public enum RobotStates {
    /** Disabled: shooter idle */
    IDLE,
    /** Teleop driving with no button held. From IDLE or AUTO it starts field-centric driving */
    DRIVE,
    /** Autonomous: autos drive and set states; nothing extra is requested */
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
    /** Test mode: shooter and intake tuning from the dashboard */
    TUNING
}
