package frc.robot.lib2026;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.localization.LocalizationSubsystem;

public class FieldTargets {
    // =============================================================================
    // ROTATION SNAP ANGLES (This is used for BUMP crossing)
    // =============================================================================

    // Target angles for snapping rotations (diagonal angles: 45°, 135°, 225°, 315°)
    public static final Rotation2d[] SNAP_ANGLES = {
        Rotation2d.fromDegrees(45.0),
        Rotation2d.fromDegrees(135.0),
        Rotation2d.fromDegrees(225.0),
        Rotation2d.fromDegrees(315.0)
    };

    /**
     * Gets the nearest snap angle from a given rotation. Snaps to the nearest angle defined in
     * SNAP_ANGLES constant.
     *
     * @return The nearest snap angle
     */
    public static Rotation2d getNearestSnapAngle() {
        Rotation2d current_rotation =
                LocalizationSubsystem.getInstance().getFieldPose().getRotation();
        // Find nearest angle using Rotation2d methods to handle wrap-around properly
        Rotation2d nearest_angle = SNAP_ANGLES[0];
        double min_difference = Math.abs(current_rotation.minus(nearest_angle).getRadians());

        for (Rotation2d target_angle : SNAP_ANGLES) {
            double difference = Math.abs(current_rotation.minus(target_angle).getRadians());

            if (difference < min_difference) {
                min_difference = difference;
                nearest_angle = target_angle;
            }
        }

        return nearest_angle;
    }

    // =============================================================================
    // SHOOTING AND PASSING TARGETS
    // =============================================================================
    public class Shooter {
        public static final Translation3d LEFT_PASS =
                new Translation3d(1, 6.042, 0); // where to pass to on the left side
        public static final Translation3d RIGHT_PASS =
                new Translation3d(1, 2, 0); // where to pass to on the left side
        public static final Translation3d HUB =
                new Translation3d(4.611624, 4.021328, 1.397); // where the hub is

        // =============================================================================
        // SHOOTING TOLERANCES
        // =============================================================================

        // Strict tolerances for scoring (hub shots)
        public static final double FLYWHEEL_SPEED_TOLERANCE =
                Units.rotationsPerMinuteToRadiansPerSecond(100);
        public static final double HOOD_POSITION_TOLERANCE = Units.degreesToRadians(1.0);
        public static final double ROTATION_ANGLE_TOLERANCE = Units.degreesToRadians(3.0);

        // Lenient tolerances for passing
        public static final double FLYWHEEL_PASS_SPEED_TOLERANCE =
                Units.rotationsPerMinuteToRadiansPerSecond(800);
        public static final double HOOD_PASS_POSITION_TOLERANCE = Units.degreesToRadians(3.0);
        public static final double ROTATION_PASS_ANGLE_TOLERANCE = Units.degreesToRadians(6.0);
    }
}
