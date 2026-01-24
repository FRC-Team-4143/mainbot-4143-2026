package frc.robot.lib2026;

import com.marswars.geometry.PolygonRegion;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldRegions {
    public static PolygonRegion SHOOTING_REGION =
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

    public static PolygonRegion HUB_REGION =
            new PolygonRegion(
                    new Translation2d[] {
                        new Translation2d(4.031, 3.43),
                        new Translation2d(4.031, 4.62),
                        new Translation2d(5.221, 4.62),
                        new Translation2d(5.221, 3.43),
                        new Translation2d(4.031, 3.43),
                    },
                    "HUB_REGION");
    public static PolygonRegion OPP_HUB_REGION =
            new PolygonRegion(
                    new Translation2d[] {
                        new Translation2d(12.301, 3.43),
                        new Translation2d(12.301, 4.62),
                        new Translation2d(13.491, 4.62),
                        new Translation2d(13.491, 3.43),
                        new Translation2d(12.301, 3.43),
                    },
                    "OPP_HUB_REGION");

    public static PolygonRegion TOWER_REGION =
            new PolygonRegion(
                    new Translation2d[] {
                        new Translation2d(0, 3.625),
                        new Translation2d(0, 4.815),
                        new Translation2d(1.02, 4.815),
                        new Translation2d(1.02, 3.625),
                        new Translation2d(0, 3.625),
                    },
                    "TOWER_REGION");

    public static PolygonRegion OPP_TOWER_REGION =
            new PolygonRegion(
                    new Translation2d[] {
                        new Translation2d(15.351, 3.625),
                        new Translation2d(15.351, 4.815),
                        new Translation2d(16.371, 4.815),
                        new Translation2d(16.371, 3.625),
                        new Translation2d(15.351, 3.625),
                    },
                    "OPP_TOWER_REGION");

    public static PolygonRegion DEPOT_REGION =
            new PolygonRegion(
                    new Translation2d[] {
                        new Translation2d(0, 5.42), new Translation2d(0, 6.52),
                    },
                    "DEPOT_REGION");
}
