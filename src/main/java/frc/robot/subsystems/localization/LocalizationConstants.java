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
    public final Matrix<N3, N1> DEFAULT_VISION_STD_DEV = VecBuilder.fill(0.9, 0.9, 0.9);
    public final Matrix<N3, N1> SHOOTING_FOCUSED_STD_DEV = VecBuilder.fill(0.9, 0.9, 0.9);
    public final Matrix<N3, N1> SHOOTING_NOT_FOCUSED_STD_DEV = VecBuilder.fill(0.9, 0.9, 0.9);
    public final Matrix<N3, N1> CLIMBING_FOCUSED_STD_DEV = VecBuilder.fill(0.9, 0.9, 0.9);
    public final Matrix<N3, N1> CLIMBING_NOT_FOCUSED_STD_DEV = VecBuilder.fill(0.9, 0.9, 0.9);

    // Maximum allowed rotation difference between vision measurement and current pose (radians)
    // Measurements with larger rotation differences will be discarded
    public final double MAX_ROTATION_DIFFERENCE = Units.degreesToRadians(360.0);

    // Maximum allowed yaw rate (rotation speed) for accepting vision measurements (radians per
    // second)
    public final double YAW_RATE_DISCARD = Units.degreesToRadians(720.0);

    // Minimum number of visible tags required to perform a vision update
    public final int MIN_TAG_COUNT_FOR_VISION_UPDATE = 2;

    // -------------------------------------------------------------------------
    // Single-tag detection configuration
    // -------------------------------------------------------------------------
    // The codebase was recently updated to allow single-tag vision updates in some
    // circumstances. The following flags and thresholds make that behavior
    // configurable so it can be tuned or disabled without changing logic.

    // Allow using a single detected AprilTag as a vision update when other
    // quality checks (distance, field zone, disabled-state) pass.
    public boolean ALLOW_SINGLE_TAG_UPDATES = false;

    // If true, single-tag updates are only allowed while the robot is disabled
    // (useful for seeding state before matches without influencing enabled
    // operation). If false, single-tag updates may be allowed while enabled
    // subject to other checks.
    public boolean SINGLE_TAG_ONLY_WHILE_DISABLED = true;

    // Maximum allowed distance (meters) between the odometry pose and the
    // vision-derived pose for accepting a single-tag measurement. This avoids
    // large jumps caused by ambiguous single-tag solutions.
    public final double SINGLE_TAG_MAX_ACCEPT_DISTANCE_METERS = 1.0;

    // If true, enforce the MIN_TAG_COUNT_FOR_VISION_UPDATE check only when the robot
    // is enabled and inside the alliance-specific zone. This replicates the prior
    // behavior but makes it configurable for future tuning. Set to true to require
    // multiple tags for most enabled operation inside the alliance zone.
    public boolean ENFORCE_MIN_TAGS_IN_ALLIANCE_ZONE_WHEN_ENABLED = true;

    // =============================================================================
    // CONSTRUCTOR
    // =============================================================================

    public LocalizationConstants() {
        // Some constants require dynamic initialization like through the JSON loader
    }
}
