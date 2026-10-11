package frc.robot.autos.bt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.marswars.auto.BehaviorTreeAuto;
import com.marswars.bt.MwLibNodes;
import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.PortInfo;
import com.marswars.bt.core.TreeSpec;
import com.marswars.bt.xml.BtXmlParser;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Every XML auto (deploy/autos/trees/*.xml, plus trees/unlisted/*.xml which are kept out of the
 * chooser) must instantiate against the robot's node set and only reference trajectories that exist
 * in deploy/choreo. Also writes build/bt/nodes.xml, the node-spec palette the BT editor (or Groot2)
 * loads.
 */
class BtAutosParseTest {
    static final Path AUTOS = Path.of("src/main/deploy/autos/trees");
    static final Path CHOREO = Path.of("src/main/deploy/choreo");

    static BehaviorTreeFactory factory() {
        BehaviorTreeFactory factory = new BehaviorTreeFactory(() -> 0.0);
        AutoNodes.registerNodes(factory);
        return factory;
    }

    static List<Path> allAutos() {
        List<Path> files = new ArrayList<>(BehaviorTreeAuto.listAutos(AUTOS));
        files.addAll(BehaviorTreeAuto.listAutos(AUTOS.resolve("unlisted")));
        return files;
    }

    @Test
    void chooserHasTheConvertedAutos() {
        List<String> names =
                BehaviorTreeAuto.listAutos(AUTOS).stream()
                        .map(p -> p.getFileName().toString().replace(".xml", ""))
                        .toList();
        assertEquals(
                List.of(
                        "CitrusSynergy",
                        "CitrusSynergyFarBump",
                        "CitrusSynergyRight",
                        "Left_Trench_Bump_Swipe",
                        "Left_Trench_Trench_Swipe",
                        "Right_Trench_Bump_Swipe",
                        "Right_Trench_Trench_Swipe",
                        "Shoot",
                        "Slop_Auto"),
                names);
    }

    @Test
    void everyXmlAutoLoadsAndItsTrajectoriesExist() {
        BehaviorTreeFactory factory = factory();
        List<Path> files = allAutos();
        assertEquals(12, files.size());
        for (Path file : files) {
            TreeSpec spec = BtXmlParser.parse(file);
            String stem = file.getFileName().toString().replace(".xml", "");
            assertEquals(stem, spec.mainTreeId(), file + ": main tree ID should match file name");
            BehaviorTree tree = factory.createTreeFromFile(file);
            assertFalse(tree.getNodes().isEmpty());
            for (String traj : BehaviorTreeAuto.discoverTrajectoryNames(spec, factory)) {
                assertTrue(
                        Files.exists(CHOREO.resolve(traj + ".traj")),
                        file.getFileName() + " references missing trajectory " + traj);
            }
        }
    }

    @Test
    void trajectoriesLoadInRunOrder() {
        BehaviorTreeFactory factory = factory();
        assertEquals(
                List.of("SynergyP1", "SynergyP2"),
                BehaviorTreeAuto.discoverTrajectoryNames(
                        BtXmlParser.parse(AUTOS.resolve("CitrusSynergy.xml")), factory));
        assertEquals(
                List.of("LTrenchStartBumpReturn", "LTrenchSwipeBumpReturn"),
                BehaviorTreeAuto.discoverTrajectoryNames(
                        BtXmlParser.parse(AUTOS.resolve("Left_Trench_Bump_Swipe.xml")), factory));
    }

    @Test
    void waitTimesAreTreeParameters() {
        assertEquals(
                List.of("middle_wait_msec:3000"),
                params("CitrusSynergy.xml"),
                "CitrusSynergy's middle wait is a tunable tree parameter (3 s)");
        assertEquals(
                List.of("middle_wait_msec:4000", "before_alliance_zone_wait_msec:1000"),
                params("CitrusSynergyFarBump.xml"));
        BehaviorTree tree = factory().createTreeFromFile(AUTOS.resolve("CitrusSynergy.xml"));
        assertEquals(3000, tree.getBlackboard().get("middle_wait_msec"));
    }

    static List<String> params(String file) {
        return BtXmlParser.parse(AUTOS.resolve(file)).mainTreeParameters().stream()
                .map((PortInfo p) -> p.name() + ":" + p.defaultValue())
                .toList();
    }

    @Test
    void nodeSpecIsMwLibPlusRobotNodes() throws Exception {
        String generated = factory().writeTreeNodesModelXml(false);
        Path file = Path.of("src/main/deploy/autos/nodes.xml");
        if (Boolean.getBoolean("bt.updateNodeSpec")) {
            Files.writeString(file, generated);
        }
        assertTrue(Files.exists(file), "missing " + file + "; run ./gradlew test -PupdateNodeSpec");
        assertEquals(
                generated,
                Files.readString(file),
                file + " is out of date; run ./gradlew test -PupdateNodeSpec");

        // MW-Lib's bundled node file comes first, unchanged; the robot's nodes are appended.
        String lib = MwLibNodes.bundledNodeSpecXml();
        String libSection = lib.substring(0, lib.indexOf("  </TreeNodesModel>"));
        assertTrue(generated.startsWith(libSection), "robot spec must start with MW-Lib's");
        String robotSection = generated.substring(libSection.length());
        assertTrue(robotSection.contains("<!-- Robot nodes -->"));
        assertTrue(robotSection.contains("<Action ID=\"SetIntakeState\">"));
        assertTrue(robotSection.contains("<Condition ID=\"IsShooterReady\">"));
        assertFalse(robotSection.contains("FollowTrajectory"), "swerve nodes come from MW-Lib");
    }
}
