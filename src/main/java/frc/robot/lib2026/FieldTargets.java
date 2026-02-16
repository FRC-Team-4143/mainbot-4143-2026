package frc.robot.lib2026;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class FieldTargets {
    // POSITIONS OF SHOOTING TARGETS

    public class Shooter {
        public static final Translation3d LEFT_PASS =
                new Translation3d(1, 7.042, 0); // where to pass to on the left side
        public static final Translation3d RIGHT_PASS =
                new Translation3d(1, 1, 0); // where to pass to on the left side
        public static final Translation3d HUB =
                new Translation3d(4.611624, 4.021328, 1.397); // where the hub is

        // =============================================================================
        // SHOOTING TOLERANCES
        // =============================================================================

        // Strict tolerances for scoring (hub shots)
        public static final double FLYWHEEL_SPEED_TOLERANCE =
                Units.rotationsPerMinuteToRadiansPerSecond(50);
        public static final double HOOD_POSITION_TOLERANCE = Units.degreesToRadians(1.0);
        public static final double TURRET_ANGLE_TOLERANCE = Units.degreesToRadians(1.0);
        public static final double ROTATION_ANGLE_TOLERANCE = Units.degreesToRadians(3.0);

        // Lenient tolerances for passing
        public static final double FLYWHEEL_PASS_SPEED_TOLERANCE =
                Units.rotationsPerMinuteToRadiansPerSecond(500);
        public static final double HOOD_PASS_POSITION_TOLERANCE = Units.degreesToRadians(3.0);
        public static final double TURRET_PASS_ANGLE_TOLERANCE = Units.degreesToRadians(1.0);
        public static final double ROTATION_PASS_ANGLE_TOLERANCE = Units.degreesToRadians(6.0);
    }
}
