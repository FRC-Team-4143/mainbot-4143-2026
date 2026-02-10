package frc.robot.subsystems.localization;

import com.marswars.subsystem.MwConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import java.util.Set;

public class LocalizationConstants extends MwConstants {

    // =============================================================================
    // ENUMS AND STATE DEFINITIONS
    // =============================================================================

    public enum LocalizationStates {
        SHOOTING_FOCUS,
        CLIMBING_FOCUS,
        FULL
    }

    // =============================================================================
    // APRIL TAG CONFIGURATION
    // =============================================================================

    public final AprilTagFieldLayout APRIL_TAG_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    public final Set<Integer> SHOOTING_FOCUS_TAG_IDS_RED = Set.of(2, 3, 4, 5, 8, 9, 10, 11);
    public final Set<Integer> CLIMBING_FOCUS_TAG_IDS_RED = Set.of(15, 16);
    public final Set<Integer> SHOOTING_FOCUS_TAG_IDS_BLUE = Set.of(18, 19, 20, 21, 24, 25, 26, 27);
    public final Set<Integer> CLIMBING_FOCUS_TAG_IDS_BLUE = Set.of(31, 32);

    // =============================================================================
    // ODOMETRY FILTER CONFIGURATION
    // =============================================================================
    public final Matrix<N3, N1> DEFAULT_ODOM_COVARIANCE = VecBuilder.fill(0.1, 0.1, 0.1);

    // =============================================================================
    // VISION FILTER CONFIGURATION
    // =============================================================================
    // Vision covariance matrices for different focus modes
    // Standard deviations: [x (meters), y (meters), theta (radians)]
    public final Matrix<N3, N1> DEFAULT_VISION_COVARIANCE = VecBuilder.fill(0.9, 0.9, 0.9);
    public final Matrix<N3, N1> SHOOTING_FOCUSED_COVARIANCE = VecBuilder.fill(0.5, 0.5, 0.1);
    public final Matrix<N3, N1> SHOOTING_NOT_FOCUSED_COVARIANCE = VecBuilder.fill(1.0, 1.0, 0.5);
    public final Matrix<N3, N1> CLIMBING_FOCUSED_COVARIANCE = VecBuilder.fill(0.5, 0.5, 0.1);
    public final Matrix<N3, N1> CLIMBING_NOT_FOCUSED_COVARIANCE = VecBuilder.fill(1.0, 1.0, 0.5);

    // Maximum allowed rotation difference between vision measurement and current pose (radians)
    // Measurements with larger rotation differences will be discarded
    public final double MAX_ROTATION_DIFFERENCE = Units.degreesToRadians(360.0);

    // Maximum allowed yaw rate (rotation speed) for accepting vision measurements (radians per
    // second)
    public final double YAW_RATE_DISCARD = Units.degreesToRadians(720.0);

    // =============================================================================
    // CONSTRUCTOR
    // =============================================================================

    public LocalizationConstants() {
        // Some constants require dynamic initialization like through the JSON loader
    }
}
