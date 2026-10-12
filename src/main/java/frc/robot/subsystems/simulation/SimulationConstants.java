package frc.robot.subsystems.simulation;

import com.marswars.subsystem.MwConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.ShooterConstants;

public class SimulationConstants extends MwConstants {

    // =============================================================================
    // ENUMS AND STATE DEFINITIONS
    // =============================================================================

    public enum SimulationStates {
        ACTIVE
    }

    // =============================================================================
    // POSE ESTIMATION SIMULATION
    // =============================================================================
    public final boolean SIM_VISION_ENABLED = false;
    public final double GYRO_NOISE_STD_DEV = Math.toRadians(0.5); // radians (0.5 degrees)
    public final double MODULE_POSITION_NOISE_STD_DEV = 0.01; // meters (1cm per reading)
    public final Transform3d BACK_CAMERA_TRANSFORM =
            new Transform3d(
                    -0.330,
                    0.288,
                    0.540,
                    new Rotation3d(
                            0.0, Units.degreesToRadians(-10.0), Units.degreesToRadians(180.0)));
    public final Transform3d RIGHT_CAMERA_TRANSFORM =
            new Transform3d(
                    -0.022,
                    -0.360,
                    0.293,
                    new Rotation3d(
                            0.0, Units.degreesToRadians(-10.0), Units.degreesToRadians(-90.0)));
    public final Transform3d LEFT_CAMERA_TRANSFORM =
            new Transform3d(
                    -0.022,
                    0.360,
                    0.293,
                    new Rotation3d(
                            0.0, Units.degreesToRadians(-10.0), Units.degreesToRadians(90.0)));

    // =============================================================================
    // SHOOTER SIMULATION
    // =============================================================================
    public final Translation3d SHOOTER_LAUNCH_OFFSET =
            new Translation3d(
                    Units.inchesToMeters(ShooterConstants.SHOOTER_MOUNT_X_INCHES),
                    Units.inchesToMeters(ShooterConstants.SHOOTER_MOUNT_Y_INCHES),
                    Units.inchesToMeters(ShooterConstants.SHOOTER_MOUNT_Z_INCHES));
    public final Rotation2d SHOOTER_LAUNCH_ROTATION =
            new Rotation2d(Units.degreesToRadians(ShooterConstants.SHOOTER_MOUNT_YAW_DEGREES));
    public final double SHOOTER_WIDTH = Units.inchesToMeters(ShooterConstants.SHOOTER_WIDTH_INCHES);
    public final double FUEL_RADIUS = 0.075; // meters (from FuelSim)
    public final double SECONDS_PER_SHOT = 1.0 / 15.0; // balls per second

    // Flywheel load calculation:
    // Based on momentum transfer: τ_avg = (m_ball × v_launch × r_flywheel) / Δt_contact
    // Fuel mass ≈ 0.203 kg, launch velocity ≈ 12 m/s, flywheel radius ≈ 0.0762 m
    // Contact time ≈ 0.02 s → τ_avg ≈ 9.3 N⋅m per shot
    public final double FUEL_MASS_KG = 0.448 * 0.45392; // kg (from FuelSim)
    public final double CONTACT_TIME_SEC = 0.02; // seconds, time ball is in contact with flywheel
    public final double FLYWHEEL_RADIUS_M =
            Units.inchesToMeters(3); // meters (from ShooterConstants)

    // =============================================================================
    // INTAKE SIMULATION
    // =============================================================================
    public final boolean SIM_FUEL_ENABLED = true;
    public final double BASE_LENGTH = Units.inchesToMeters(27.5); // frame length
    public final double BASE_WIDTH = Units.inchesToMeters(27.5); // frame width
    public final double BUMPER_HEIGHT = Units.inchesToMeters(3.0);
    public final double INTAKE_MAX_EXTENSION = Units.inchesToMeters(12.0);
    public final int HOPPER_CAPACITY = 50; // number of game pieces

    // =============================================================================
    // CONSTRUCTOR
    // =============================================================================

    public SimulationConstants() {}
}
