package frc.robot.subsystems.localization;

import com.marswars.subsystem.MwConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
    // VISION COVARIANCE MATRICES
    // =============================================================================

    // Vision covariance matrices for different focus modes
    // Standard deviations: [x (meters), y (meters), theta (radians)]
    public final Matrix<N3, N1> SHOOTING_FOCUSED_COVARIANCE = VecBuilder.fill(0.5, 0.5, 0.1);
    public final Matrix<N3, N1> SHOOTING_NOT_FOCUSED_COVARIANCE = VecBuilder.fill(1.0, 1.0, 0.5);
    public final Matrix<N3, N1> CLIMBING_FOCUSED_COVARIANCE = VecBuilder.fill(0.5, 0.5, 0.1);
    public final Matrix<N3, N1> CLIMBING_NOT_FOCUSED_COVARIANCE = VecBuilder.fill(1.0, 1.0, 0.5);

    // =============================================================================
    // CONSTRUCTOR
    // =============================================================================

    public LocalizationConstants() {
        // Some constants require dynamic initialization like through the JSON loader
    }
}
