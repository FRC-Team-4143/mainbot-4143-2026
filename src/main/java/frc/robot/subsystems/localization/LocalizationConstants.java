package frc.robot.subsystems.localization;

import com.marswars.subsystem.MwConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class LocalizationConstants extends MwConstants {

    // Subsystem States
    public enum LocalizationStates {
        ODOM_ONLY,
        VISION_SIM,
        FULL
    }

    public final AprilTagFieldLayout APRIL_TAG_LAYOUT =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public LocalizationConstants() {
        // Some constants require dynamic initialization like through the JSON loader
    }
}
