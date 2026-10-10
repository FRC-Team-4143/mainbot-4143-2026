package frc.robot.autos.bt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.marswars.auto.BehaviorTreeAuto;
import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.PortInfo;
import com.marswars.bt.core.TreeSpec;
import com.marswars.bt.debug.BtLiveServer;
import com.marswars.bt.xml.BtXmlParser;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Every XML auto (deploy/autos/*.xml, plus deploy/autos/unlisted/*.xml which are kept out of the
 * chooser) must instantiate against the robot's node set and only reference trajectories that exist
 * in deploy/choreo. Also writes build/bt/nodes.xml, the node-spec palette the BT editor (or Groot2)
 * loads.
 */
class BtAutosParseTest {
    static final Path AUTOS = Path.of("src/main/deploy/autos");
    static final Path CHOREO = Path.of("src/main/deploy/choreo");

    static BehaviorTreeFactory factory() {
        BehaviorTreeFactory factory = new BehaviorTreeFactory(() -> 0.0);
        AutoNodes.registerNodes(factory);
        return factory;
    }

    static List<Path> allAutos() {
        List<Path> files = new ArrayList<>(BehaviorTreeAuto.listXml(AUTOS));
        files.addAll(BehaviorTreeAuto.listXml(AUTOS.resolve("unlisted")));
        return files;
    }

    @Test
    void chooserHasTheConvertedAutos() {
        List<String> names =
                BehaviorTreeAuto.listXml(AUTOS).stream()
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
                "replaces DynamicWaitCommand CitrusSynergy/MiddleWaitTime (3 s)");
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
    void writeNodeSpecForEditors() throws Exception {
        Path out = Path.of("build/bt/nodes.xml");
        Files.createDirectories(out.getParent());
        BtLiveServer.writeNodeSpec(factory(), out);
        String xml = Files.readString(out);
        assertTrue(xml.contains("<Action ID=\"FollowTrajectory\">"));
        assertTrue(xml.contains("<Condition ID=\"IsShooterReady\""));
    }
}
