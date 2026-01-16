package frc.mw_lib.swerve_lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class SwerveMeasurments {
    public static class ModuleMeasurement {
        public double timestamp;
        public SwerveModulePosition module_positions;
    }

    public static class GyroMeasurement {
        public double timestamp;
        public Rotation2d gyro_yaw;
    }

    public static class SwerveMeasurement {
        public double timestamp;
        public SwerveModulePosition[] module_positions;
        public Rotation2d gyro_yaw;
    }
}
