package frc.robot.autos.bt;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.marswars.auto.BehaviorTreeAuto;
import com.marswars.bt.core.BehaviorTree;
import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.TreeSpec;
import com.marswars.bt.debug.BtLiveServer;
import com.marswars.bt.xml.BtXmlParser;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;

/**
 * Every deploy/autos/*.xml must instantiate against the robot's node set and only reference
 * trajectories that exist in deploy/choreo. Also writes build/bt/nodes.xml, the node-spec palette
 * the BT editor (or Groot2) loads.
 */
class BtAutosParseTest {
    static final Path AUTOS = Path.of("src/main/deploy/autos");
    static final Path CHOREO = Path.of("src/main/deploy/choreo");

    static BehaviorTreeFactory factory() {
        BehaviorTreeFactory factory = new BehaviorTreeFactory(() -> 0.0);
        AutoNodes.registerNodes(factory);
        return factory;
    }

    @Test
    void everyXmlAutoLoadsAndItsTrajectoriesExist() {
        BehaviorTreeFactory factory = factory();
        List<Path> files = BehaviorTreeAuto.listXml(AUTOS);
        assertFalse(files.isEmpty(), "no XML autos found in " + AUTOS);
        for (Path file : files) {
            TreeSpec spec = BtXmlParser.parse(file);
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
    void citrusSynergyLoadsBothPathsInOrder() {
        BehaviorTreeFactory factory = factory();
        TreeSpec spec = BtXmlParser.parse(AUTOS.resolve("CitrusSynergyBt.xml"));
        assertEquals(
                List.of("SynergyP1", "SynergyP2"),
                BehaviorTreeAuto.discoverTrajectoryNames(spec, factory));
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
