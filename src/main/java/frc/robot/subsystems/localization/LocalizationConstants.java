package frc.robot.subsystems.localization;

import com.marswars.subsystem.MwConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LocalizationConstants extends MwConstants {

    // Subsystem States
    public enum LocalizationStates {
        ODOM_ONLY,
        VISION_SIM,
        FULL
    }

    public final Pose2d START_POSE = new Pose2d(3.0, 3.0, Rotation2d.kZero);
    public final AprilTagFieldLayout APRIL_TAG_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public LocalizationConstants() {
        // Some constants require dynamic initialization like through the JSON loader
    }
}
