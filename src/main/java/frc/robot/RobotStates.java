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
    /**
     * Shoot (what buttons and autos ask for). The robot is never in SHOOT itself:
     * handleStateTransition picks one of the SHOOT_ step states below every loop. It keeps shooting
     * until something asks for another state.
     */
    SHOOT,
    /** Shooting step: spinning up and aiming, not feeding yet */
    SHOOT_WAIT,
    /** Shooting step: ready, feeding fuel (intake squeeze) while driving with the heading locked */
    SHOOT_FEED,
    /** Shooting step: ready and stopped, feeding fuel with the wheels crossed */
    SHOOT_FEED_BRAKED,
    /** Shooting step: passing while moving, keep intaking until the robot stops */
    SHOOT_PASS_INTAKE,
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
