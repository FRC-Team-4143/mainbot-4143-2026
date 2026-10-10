package frc.robot.autos.bt;

import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.core.PortInfo;
import com.marswars.bt.core.PortType;
import com.marswars.bt.core.StatefulActionNode;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.List;

/**
 * RUNNING until the swerve's Choreo event tracker reports that {@code event} was passed on the
 * trajectory currently being followed.
 *
 * <p>Only meaningful while a {@code FollowTrajectory} runs, i.e. as a sibling of it under a {@code
 * Parallel}: the tracker resets when a new trajectory starts and stops when following ends, so a
 * wait placed after its trajectory never finishes. Replaces the global {@code
 * getChoreoEventTimeTrigger(...).onTrue(...)} bindings of the Java autos.
 */
public class WaitForChoreoEventNode extends StatefulActionNode {
    public static final String ID = "WaitForChoreoEvent";

    public static final List<PortInfo> PORTS =
            List.of(PortInfo.input("event", PortType.STRING, "Choreo event marker name"));

    public WaitForChoreoEventNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected NodeStatus onStart() {
        return onRunning();
    }

    @Override
    protected NodeStatus onRunning() {
        return SwerveSubsystem.getInstance().hasChoreoEventBeenPassed(getString("event"))
                ? NodeStatus.SUCCESS
                : NodeStatus.RUNNING;
    }

    @Override
    protected void onHalted() {}
}
