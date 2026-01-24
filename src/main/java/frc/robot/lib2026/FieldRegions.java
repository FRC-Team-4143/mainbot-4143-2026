package frc.robot.lib2026;

import java.util.ArrayList;
import java.util.List;

import com.marswars.geometry.PolygonRegion;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldRegions {
    public static PolygonRegion ALLIANCE_ZONE = new PolygonRegion(
            new Translation2d[] {
                    new Translation2d(0, 0),
                    new Translation2d(0, 8.042),
                    new Translation2d(3.963, 8.042),
                    new Translation2d(3.963, 0),
                    new Translation2d(0, 0),
            },
            "ALLIANCE_ZONE");

    public static PolygonRegion LEFT_PASS_REGION = new PolygonRegion(
            new Translation2d[] {
                    new Translation2d(5.153, 4.021),
                    new Translation2d(5.153, 8.042),
                    new Translation2d(16.513, 8.042),
                    new Translation2d(16.513, 4.021),
                    new Translation2d(5.153, 4.021),
            },
            "LEFT_PASS_REGIONs");

    public static PolygonRegion RIGHT_PASS_REGION = new PolygonRegion(
            new Translation2d[] {
                    new Translation2d(5.153, 0),
                    new Translation2d(5.153, 4.021),
                    new Translation2d(16.513, 4.021),
                    new Translation2d(16.513, 0),
                    new Translation2d(5.153, 0),
            },
            "RIGHT_PASS_REGION");

    public static PolygonRegion HUB_REGION = new PolygonRegion(
            new Translation2d[] {
                    new Translation2d(4.031, 3.43),
                    new Translation2d(4.031, 4.62),
                    new Translation2d(5.221, 4.62),
                    new Translation2d(5.221, 3.43),
                    new Translation2d(4.031, 3.43),
            },
            "HUB_REGION");
    public static PolygonRegion OPP_HUB_REGION = new PolygonRegion(
            new Translation2d[] {
                    new Translation2d(12.301, 3.43),
                    new Translation2d(12.301, 4.62),
                    new Translation2d(13.491, 4.62),
                    new Translation2d(13.491, 3.43),
                    new Translation2d(12.301, 3.43),
            },
            "OPP_HUB_REGION");

    public static PolygonRegion TOWER_REGION = new PolygonRegion(
            new Translation2d[] {
                    new Translation2d(0, 3.625),
                    new Translation2d(0, 4.815),
                    new Translation2d(1.02, 4.815),
                    new Translation2d(1.02, 3.625),
                    new Translation2d(0, 3.625),
            },
            "TOWER_REGION");

    public static PolygonRegion OPP_TOWER_REGION = new PolygonRegion(
            new Translation2d[] {
                    new Translation2d(15.351, 3.625),
                    new Translation2d(15.351, 4.815),
                    new Translation2d(16.371, 4.815),
                    new Translation2d(16.371, 3.625),
                    new Translation2d(15.351, 3.625),
            },
            "OPP_TOWER_REGION");

    public static PolygonRegion DEPOT_REGION = new PolygonRegion(
            new Translation2d[] {
                    new Translation2d(0, 5.42),
                    new Translation2d(0, 6.52),
                    new Translation2d(.69, 6.52),
                    new Translation2d(.69, 5.42),
                    new Translation2d(0, 5.42),
            },
            "DEPOT_REGION");

    public static PolygonRegion OPP_DEPOT_REGION = new PolygonRegion(
            new Translation2d[] {
                    new Translation2d(15.851, 2.45),
                    new Translation2d(15.851, 3.55),
                    new Translation2d(16.541, 3.55),
                    new Translation2d(16.541, 2.45),
                    new Translation2d(16.851, 2.45),
            },
            "OPP_DEPOT_REGION");

    public static PolygonRegion HOLD_ZONE = new PolygonRegion(
            new Translation2d[] {
                    new Translation2d(4.301, 0),
                    new Translation2d(4.301, 8.07),
                    new Translation2d(5.221, 8.07),
                    new Translation2d(5.221, 0),
                    new Translation2d(4.301, 0),
            },
            "HOLD_ZONE");
    public static PolygonRegion NEUTRAL_ZONE = new PolygonRegion(
            new Translation2d[] {
                    new Translation2d(4.031, 0),
                    new Translation2d(4.031, 8.07),
                    new Translation2d(13.491, 8.07),
                    new Translation2d(13.491, 0),
                    new Translation2d(4.031, 0),
            },
            "NEUTRAL_ZONE");

    public static PolygonRegion OPP_HOLD_ZONE = new PolygonRegion(
            new Translation2d[] {
                    new Translation2d(12.301, 0),
                    new Translation2d(12.301, 8.07),
                    new Translation2d(13.491, 8.07),
                    new Translation2d(13.491, 0),
                    new Translation2d(12.301, 0),
            },
            "OPP_HOLD_ZONE");

    public static PolygonRegion OPP_ALLIANCE_ZONE = new PolygonRegion(
            new Translation2d[] {
                    new Translation2d(12.516, 0),
                    new Translation2d(12.516, 8.07),
                    new Translation2d(16.541, 8.07),
                    new Translation2d(16.541, 0),
                    new Translation2d(12.516, 0),
            },
            "OPP_ALLIANCE_ZONE");

    public static PolygonRegion OPP_ALLIANCE_HOLD_ZONE = new PolygonRegion(
            new Translation2d[] {
                    new Translation2d(12.301, 3.43),
                    new Translation2d(12.301, 4.62),
                    new Translation2d(16.541, 4.62),
                    new Translation2d(16.541, 3.43),
                    new Translation2d(12.301, 3.43),
            },
            "OPP_ALLIANCE_HOLD_ZONE");

    public static ArrayList<PolygonRegion> HOLD_REGIONS = new ArrayList<>(
            List.of(
                    HOLD_ZONE,
                    OPP_HOLD_ZONE,
                    OPP_ALLIANCE_HOLD_ZONE));

    public static void flipRegions() {
        DataLogManager.log("Flipping Regions to" + DriverStation.getAlliance().get().toString());

        populateTable();
    }

}
