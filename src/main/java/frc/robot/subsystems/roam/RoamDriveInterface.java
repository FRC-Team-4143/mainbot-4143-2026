package frc.robot.subsystems.roam;

import org.wpilib.driverstation.RobotState;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.IntegerSubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.system.Timer;
import frc.robot.OI;
import frc.robot.subsystems.swerve.SwerveConstants.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;

/**
 * Reads swerve-drive commands published by the Sparky voice/vision coprocessor
 * (chatbot_roam.py, drive_robot tool) over NetworkTables and applies them to the
 * existing SwerveSubsystem via its CHASSIS_SPEEDS state.
 *
 * Not a full MwSubsystem: it has no state machine of its own and is intentionally
 * simple, since its only job is "forward a bounded, watchdog-checked velocity
 * command into the swerve subsystem, or get out of the way." periodic() is called
 * directly from Robot.robotPeriodic(), same as Mechanism3dViz.publish() and
 * HubMonitor.isHubActive(...).
 *
 * Safety model, in order:
 *  1. cmd_seq must have advanced within CMD_STALE_SEC, or commands are zeroed.
 *     The coprocessor side republishes this every ~100ms while a move is in
 *     progress, so a crash or network drop there self-corrects within
 *     CMD_STALE_SEC regardless of what cmd_vx/vy/omega last said.
 *  2. The driver's stick always wins: any input past JOYSTICK_OVERRIDE_DEADBAND
 *     immediately hands control back to normal teleop driving.
 *  3. Only active while RobotState.isEnabled() and not RobotState.isAutonomous() —
 *     i.e. teleop/test, never autonomous, never disabled.
 *  4. Velocities are clamped server-side to MAX_LINEAR_MPS / MAX_ANGULAR_RPS
 *     regardless of what's published, since the NT values come from a process
 *     off the robot and shouldn't be trusted blindly.
 *
 * None of this replaces a human at the Driver Station with a hand on disable —
 * that remains the actual emergency stop for this feature.
 */
public class RoamDriveInterface {

    private static RoamDriveInterface instance_ = null;

    public static RoamDriveInterface getInstance() {
        if (instance_ == null) {
            instance_ = new RoamDriveInterface();
        }
        return instance_;
    }

    private static final String TABLE_PATH = "/Sparky/Drive/";
    private static final double MAX_LINEAR_MPS = 1.0; // hard cap, well below competition max speed
    private static final double MAX_ANGULAR_RPS = Math.PI / 2; // ~90 deg/s
    private static final double CMD_STALE_SEC = 0.25; // zero drive if cmd_seq hasn't advanced
    private static final double JOYSTICK_OVERRIDE_DEADBAND = 0.15;

    private final DoubleSubscriber vx_sub_;
    private final DoubleSubscriber vy_sub_;
    private final DoubleSubscriber omega_sub_;
    private final IntegerSubscriber seq_sub_;

    private long last_seq_seen_ = Long.MIN_VALUE;
    private double last_seq_change_time_ = 0.0;
    private boolean roam_driving_ = false;

    private RoamDriveInterface() {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        vx_sub_ = nt.getDoubleTopic(TABLE_PATH + "cmd_vx").subscribe(0.0);
        vy_sub_ = nt.getDoubleTopic(TABLE_PATH + "cmd_vy").subscribe(0.0);
        omega_sub_ = nt.getDoubleTopic(TABLE_PATH + "cmd_omega").subscribe(0.0);
        seq_sub_ = nt.getIntegerTopic(TABLE_PATH + "cmd_seq").subscribe(0L);
    }

    /** Call once per robotPeriodic, regardless of robot mode. */
    public void periodic() {
        double now = Timer.getTimestamp();
        long seq = seq_sub_.get();
        if (seq != last_seq_seen_) {
            last_seq_seen_ = seq;
            last_seq_change_time_ = now;
        }
        boolean cmd_fresh = (now - last_seq_change_time_) < CMD_STALE_SEC;
        boolean driver_override = isDriverOverriding();
        boolean should_drive =
                cmd_fresh
                        && !driver_override
                        && RobotState.isEnabled()
                        && !RobotState.isAutonomous();

        if (should_drive) {
            double vx = clamp(vx_sub_.get(), -MAX_LINEAR_MPS, MAX_LINEAR_MPS);
            double vy = clamp(vy_sub_.get(), -MAX_LINEAR_MPS, MAX_LINEAR_MPS);
            double omega = clamp(omega_sub_.get(), -MAX_ANGULAR_RPS, MAX_ANGULAR_RPS);
            SwerveSubsystem.getInstance().setDesiredChassisSpeed(new ChassisVelocities(vx, vy, omega));
            SwerveSubsystem.getInstance().setWantedState(SwerveStates.CHASSIS_SPEEDS);
            roam_driving_ = true;
        } else if (roam_driving_) {
            // Relinquish control the moment roam stops commanding, so the driver's
            // stick (or simply re-enabling) immediately resumes normal driving.
            SwerveSubsystem.getInstance().setWantedState(SwerveStates.FIELD_CENTRIC);
            roam_driving_ = false;
        }
    }

    private boolean isDriverOverriding() {
        return Math.abs(OI.getDriverJoystickLeftX()) > JOYSTICK_OVERRIDE_DEADBAND
                || Math.abs(OI.getDriverJoystickLeftY()) > JOYSTICK_OVERRIDE_DEADBAND
                || Math.abs(OI.getDriverJoystickRightX()) > JOYSTICK_OVERRIDE_DEADBAND;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
