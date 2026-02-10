package frc.robot.lib2026;

import com.marswars.geometry.AllianceFlipUtil;
import com.marswars.geometry.PolygonRegion;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

public class FieldRegions {
    public static PolygonRegion ALLIANCE_ZONE =
            new PolygonRegion(
                    new Translation2d[] {
                        new Translation2d(0, 0),
                        new Translation2d(0, 8.042),
                        new Translation2d(3.963, 8.042),
                        new Translation2d(3.963, 0),
                        new Translation2d(0, 0),
                    },
                    "ALLIANCE_ZONE");

    public static PolygonRegion LEFT_PASS_REGION =
            new PolygonRegion(
                    new Translation2d[] {
                        new Translation2d(5.153, 4.021),
                        new Translation2d(5.153, 8.042),
                        new Translation2d(16.513, 8.042),
                        new Translation2d(16.513, 4.021),
                        new Translation2d(5.153, 4.021),
                    },
                    "LEFT_PASS_REGION");

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
                        new Translation2d(11.355, 3.43),
                        new Translation2d(11.355, 4.62),
                        new Translation2d(12.605, 4.62),
                        new Translation2d(12.605, 3.43),
                        new Translation2d(11.355, 3.43),
                    },
                    "OPP_HUB_REGION");

    public static PolygonRegion TOWER_REGION =
            new PolygonRegion(
                    new Translation2d[] {
                        new Translation2d(0, 3.148584),
                        new Translation2d(0, 4.342384),
                        new Translation2d(1.155, 4.342384),
                        new Translation2d(1.155, 3.148584),
                        new Translation2d(0, 3.148584),
                    },
                    "TOWER_REGION");

    public static PolygonRegion OPP_TOWER_REGION =
            new PolygonRegion(
                    new Translation2d[] {
                        new Translation2d(15.466, 3.726688),
                        new Translation2d(15.466, 4.915),
                        new Translation2d(16.541, 4.915),
                        new Translation2d(16.541, 3.726688),
                        new Translation2d(15.466, 3.726688),
                    },
                    "OPP_TOWER_REGION");

    public static PolygonRegion DEPOT_REGION =
            new PolygonRegion(
                    new Translation2d[] {
                        new Translation2d(0, 5.42),
                        new Translation2d(0, 6.52),
                        new Translation2d(.69, 6.52),
                        new Translation2d(.69, 5.42),
                        new Translation2d(0, 5.42),
                    },
                    "DEPOT_REGION");

    public static PolygonRegion OPP_DEPOT_REGION =
            new PolygonRegion(
                    new Translation2d[] {
                        new Translation2d(15.851, 1.570736),
                        new Translation2d(15.851, 2.637536),
                        new Translation2d(16.541, 2.637536),
                        new Translation2d(16.541, 1.570736),
                        new Translation2d(15.851, 1.570736),
                    },
                    "OPP_DEPOT_REGION");

    public static PolygonRegion HOLD_ZONE =
            new PolygonRegion(
                    new Translation2d[] {
                        new Translation2d(4.015594, 0),
                        new Translation2d(4.015594, 8.07),
                        new Translation2d(5.221, 8.07),
                        new Translation2d(5.221, 0),
                        new Translation2d(4.015594, 0),
                    },
                    "HOLD_ZONE");
    public static PolygonRegion NEUTRAL_ZONE =
            new PolygonRegion(
                    new Translation2d[] {
                        new Translation2d(4.625594, 0),
                        new Translation2d(4.625594, 8.07),
                        new Translation2d(11.895394, 8.07),
                        new Translation2d(11.895394, 0),
                        new Translation2d(4.625594, 0),
                    },
                    "NEUTRAL_ZONE");

    public static PolygonRegion OPP_HOLD_ZONE =
            new PolygonRegion(
                    new Translation2d[] {
                        new Translation2d(11.355, 0),
                        new Translation2d(11.355, 8.07),
                        new Translation2d(12.605, 8.07),
                        new Translation2d(12.605, 0),
                        new Translation2d(11.355, 0),
                    },
                    "OPP_HOLD_ZONE");

    public static PolygonRegion OPP_ALLIANCE_ZONE =
            new PolygonRegion(
                    new Translation2d[] {
                        new Translation2d(12.516, 0),
                        new Translation2d(12.516, 8.07),
                        new Translation2d(16.541, 8.07),
                        new Translation2d(16.541, 0),
                        new Translation2d(12.516, 0),
                    },
                    "OPP_ALLIANCE_ZONE");

    public static PolygonRegion OPP_ALLIANCE_HOLD_ZONE =
            new PolygonRegion(
                    new Translation2d[] {
                        new Translation2d(12.301, 3.43),
                        new Translation2d(12.301, 4.62),
                        new Translation2d(16.541, 4.62),
                        new Translation2d(16.541, 3.43),
                        new Translation2d(12.301, 3.43),
                    },
                    "OPP_ALLIANCE_HOLD_ZONE");

    public static ArrayList<PolygonRegion> HOLD_REGIONS =
            new ArrayList<>(List.of(HOLD_ZONE, OPP_HOLD_ZONE, OPP_ALLIANCE_HOLD_ZONE));

    public static void flipRegions(boolean flip) {
        if (flip) {
            DEPOT_REGION = AllianceFlipUtil.apply(DEPOT_REGION);
            ALLIANCE_ZONE = AllianceFlipUtil.apply(ALLIANCE_ZONE);
            OPP_ALLIANCE_HOLD_ZONE = AllianceFlipUtil.apply(OPP_ALLIANCE_HOLD_ZONE);
            OPP_ALLIANCE_ZONE = AllianceFlipUtil.apply(OPP_ALLIANCE_ZONE);
            OPP_DEPOT_REGION = AllianceFlipUtil.apply(OPP_DEPOT_REGION);
            OPP_HOLD_ZONE = AllianceFlipUtil.apply(OPP_HOLD_ZONE);
            OPP_HUB_REGION = AllianceFlipUtil.apply(OPP_HUB_REGION);
            OPP_TOWER_REGION = AllianceFlipUtil.apply(OPP_TOWER_REGION);
            TOWER_REGION = AllianceFlipUtil.apply(TOWER_REGION);
            HUB_REGION = AllianceFlipUtil.apply(HUB_REGION);
            HOLD_ZONE = AllianceFlipUtil.apply(HOLD_ZONE);
            NEUTRAL_ZONE = AllianceFlipUtil.apply(NEUTRAL_ZONE);
            RIGHT_PASS_REGION = AllianceFlipUtil.apply(RIGHT_PASS_REGION);
            LEFT_PASS_REGION = AllianceFlipUtil.apply(LEFT_PASS_REGION);
        }
    }
}
