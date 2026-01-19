package frc.robot.subsystems.localization;

import com.marswars.subsystem.MwConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LocalizationConstants extends MwConstants {

    // Subsystem States
    public enum LocalizationStates {
        ACTIVE,
        SIMPLE_SIM_CONTROL // Directly use simulation pose instead of odometry
    }

    public final Pose2d START_POSE = new Pose2d(3.0, 3.0, Rotation2d.kZero);

    public LocalizationConstants() {
        // Some constants require dynamic initialization like through the JSON loader
    }
}
