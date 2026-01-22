package frc.robot.lib2026;

import com.marswars.geometry.PolygonRegion;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldRegions {
    public static PolygonRegion SHOOTING_Region =
            new PolygonRegion(
                    new Translation2d[] {
                        new Translation2d(0, 0),
                        new Translation2d(0, 8.042),
                        new Translation2d(3.963, 8.042),
                        new Translation2d(3.963, 0),
                        new Translation2d(0, 0),
                    },
                    "SHOOTING_REGION");

    public static PolygonRegion LEFT_PASS_REGION =
            new PolygonRegion(
                    new Translation2d[] {
                        new Translation2d(5.153, 4.021),
                        new Translation2d(5.153, 8.042),
                        new Translation2d(16.513, 8.042),
                        new Translation2d(16.513, 4.021),
                        new Translation2d(5.153, 4.021),
                    },
                    "LEFT_PASS_REGIONs");

    public static PolygonRegion RIGHT_PASS_REGION =
            new PolygonRegion(
                    new Translation2d[] {
                        new Translation2d(5.153, 0),
                        new Translation2d(5.153, 4.021),
                        new Translation2d(16.513, 4.021),
                        new Translation2d(16.513, 0),
                        new Translation2d(5.153, 0),
                    },
                    "RIGHT_PASS_REGION");
}
