package frc.robot.lib2026;

import com.marswars.geometry.AllianceFlipUtil;
import com.marswars.geometry.PolygonRegion;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;

// examples of field regions

/*public class FieldRegions {
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
                    "LEFT_PASS_REGION");*/


    //public static ArrayList<PolygonRegion> HOLD_REGIONS =
            //new ArrayList<>(List.of(HOLD_ZONE, OPP_HOLD_ZONE, OPP_ALLIANCE_HOLD_ZONE));

    /**
     * Flips the field regions based of FIELD_SYMMETRY type.
     *
     * @apiNote This does not keep track of Red/Blue
     */
    /*public static void flipRegions() {
        DEPOT_REGION = AllianceFlipUtil.apply(DEPOT_REGION);
        OUTPOST_REGION = AllianceFlipUtil.apply(OUTPOST_REGION);
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
    }*/

