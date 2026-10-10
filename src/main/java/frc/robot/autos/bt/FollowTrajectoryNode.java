package frc.robot.autos.bt;

import com.marswars.auto.BehaviorTreeAuto;
import com.marswars.auto.ChoreoTrajectory;
import com.marswars.bt.core.NodeConfig;
import com.marswars.bt.core.NodeStatus;
import com.marswars.bt.core.PortInfo;
import com.marswars.bt.core.PortType;
import com.marswars.bt.core.StatefulActionNode;
import com.marswars.swerve_lib.SwerveStates;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.List;

/**
 * Drives a Choreo trajectory: selects it on the swerve, requests {@code path_state}, and succeeds
 * once {@code isAtChoreoSetpoint()} (and at least {@code min_time} seconds of the path have run).
 * On success or halt, requests {@code end_state}.
 *
 * <p>Same steps as the Java autos' {@code setDesiredChoreoTrajectoryCommand(...)} + {@code
 * startEnd(CHOREO_PATH, end).until(isAtChoreoSetpoint)}. The trajectory comes from the running
 * {@link BehaviorTreeAuto}'s alliance-flipped cache, which pre-loads every {@code trajectory}
 * literal in the XML.
 */
public class FollowTrajectoryNode extends StatefulActionNode {
    public static final String ID = "FollowTrajectory";

    public static final List<PortInfo> PORTS =
            List.of(
                    PortInfo.trajectory(
                            "trajectory", "Choreo trajectory name (deploy/choreo/<name>.traj)"),
                    PortInfo.enumInput(
                            "path_state",
                            SwerveStates.class,
                            SwerveStates.CHOREO_PATH,
                            "Swerve state used while following (CHOREO_PATH or"
                                    + " CHOREO_PATH_ROTATION_LOCK)"),
                    PortInfo.enumInput(
                            "end_state",
                            SwerveStates.class,
                            SwerveStates.IDLE,
                            "Swerve state requested when the path finishes or is halted"),
                    PortInfo.input(
                            "min_time",
                            PortType.DOUBLE,
                            "0",
                            "Minimum seconds on the path before it may succeed"));

    private SwerveStates end_state_ = SwerveStates.IDLE;
    private double min_time_ = 0.0;

    public FollowTrajectoryNode(String name, NodeConfig config) {
        super(name, config);
    }

    @Override
    protected NodeStatus onStart() {
        ChoreoTrajectory trajectory =
                BehaviorTreeAuto.trajectories(this).apply(getString("trajectory"));
        end_state_ = getEnum("end_state", SwerveStates.class);
        min_time_ = getDouble("min_time");

        SwerveSubsystem swerve = SwerveSubsystem.getInstance();
        swerve.setDesiredChoreoTrajectory(trajectory);
        swerve.setWantedState(getEnum("path_state", SwerveStates.class));
        return NodeStatus.RUNNING;
    }

    @Override
    protected NodeStatus onRunning() {
        SwerveSubsystem swerve = SwerveSubsystem.getInstance();
        if (swerve.isAtChoreoSetpoint() && swerve.hasChoreoTimeElapsed(min_time_)) {
            swerve.setWantedState(end_state_);
            return NodeStatus.SUCCESS;
        }
        return NodeStatus.RUNNING;
    }

    @Override
    protected void onHalted() {
        SwerveSubsystem.getInstance().setWantedState(end_state_);
    }
}
