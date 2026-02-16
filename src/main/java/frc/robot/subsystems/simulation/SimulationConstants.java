package frc.robot.subsystems.simulation;

import com.marswars.subsystem.MwConstants;
import com.marswars.util.ConstantsLoader;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class SimulationConstants extends MwConstants {

    private final ConstantsLoader LOADER = ConstantsLoader.getInstance();

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
                    -0.031,
                    -0.335,
                    0.202,
                    new Rotation3d(
                            0.0, Units.degreesToRadians(-35.0), Units.degreesToRadians(-90.0)));
    public final Transform3d LEFT_CAMERA_TRANSFORM =
            new Transform3d(
                    -0.239,
                    0.343,
                    0.540,
                    new Rotation3d(
                            0.0, Units.degreesToRadians(-10.0), Units.degreesToRadians(90.0)));

    // =============================================================================
    // SHOOTER SIMULATION
    // =============================================================================
    public final Translation3d SHOOTER_LAUNCH_OFFSET =
            new Translation3d(
                    Units.inchesToMeters(LOADER.getDoubleValue("shooter", "translation", "x")),
                    Units.inchesToMeters(LOADER.getDoubleValue("shooter", "translation", "y")),
                    Units.inchesToMeters(LOADER.getDoubleValue("shooter", "translation", "z")));
    public final Rotation2d SHOOTER_LAUNCH_ROTATION =
            new Rotation2d(
                    Units.degreesToRadians(LOADER.getDoubleValue("shooter", "rotation", "z")));
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
    public final boolean SIM_FUEL_ENABLED = false;
    public final double BASE_LENGTH =
            Units.inchesToMeters(LOADER.getDoubleValue("swerve", "com", "base_length"));
    public final double BASE_WIDTH =
            Units.inchesToMeters(LOADER.getDoubleValue("swerve", "com", "base_width"));
    public final double BUMPER_HEIGHT =
            Units.inchesToMeters(LOADER.getDoubleValue("swerve", "com", "bumper_height"));
    public final double INTAKE_MAX_EXTENSION =
            Units.inchesToMeters(LOADER.getDoubleValue("intake", "max_extension"));
    public final int HOPPER_CAPACITY = 50; // number of game pieces

    // =============================================================================
    // CONSTRUCTOR
    // =============================================================================

    public SimulationConstants() {}
}
