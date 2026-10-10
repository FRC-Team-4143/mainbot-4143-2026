package frc.robot.autos.bt;

import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.NodeKind;
import com.marswars.bt.core.PortInfo;
import com.marswars.bt.core.PortType;
import com.marswars.swerve_lib.SwerveStates;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.lib2026.FieldTargets;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * The robot's behavior-tree vocabulary: every action/condition an XML auto in {@code deploy/autos/}
 * can use, on top of MW-Lib's built-in BT.CPP nodes.
 *
 * <p>Rule for new nodes: never call a subsystem's {@code getInstance()} in a node constructor, only
 * in {@code tick}/{@code onStart}/{@code onRunning}/{@code onHalted}. Trees are instantiated to
 * validate XML (and in unit tests) without touching hardware.
 */
public final class AutoNodes {
    /**
     * Shooter aim points selectable by name from XML ({@code <SetShooterTarget target="HUB"/>}).
     */
    public static final Map<String, Translation3d> SHOOTER_TARGETS = new LinkedHashMap<>();

    static {
        SHOOTER_TARGETS.put("HUB", FieldTargets.Shooter.HUB);
        SHOOTER_TARGETS.put("LEFT_PASS", FieldTargets.Shooter.LEFT_PASS);
        SHOOTER_TARGETS.put("RIGHT_PASS", FieldTargets.Shooter.RIGHT_PASS);
    }

    private AutoNodes() {}

    /** Factory with MW-Lib built-ins plus every robot node. */
    public static BehaviorTreeFactory createFactory() {
        BehaviorTreeFactory factory = new BehaviorTreeFactory();
        registerNodes(factory);
        return factory;
    }

    public static void registerNodes(BehaviorTreeFactory factory) {
        // ---------------------------------------------------------------- state requests
        factory.registerSetState(
                "SetIntakeState",
                "Request an intake state",
                IntakeStates.class,
                s -> IntakeSubsystem.getInstance().setWantedState(s));
        factory.registerSetState(
                "SetShooterState",
                "Request a shooter state",
                ShooterStates.class,
                s -> ShooterSubsystem.getInstance().setWantedState(s));
        factory.registerSetState(
                "SetSwerveState",
                "Request a swerve state",
                SwerveStates.class,
                s -> SwerveSubsystem.getInstance().setWantedState(s));
        factory.registerInstantAction(
                "SetShooterTarget",
                "Point the shooter at a named field target",
                List.of(
                        PortInfo.choiceInput(
                                "target",
                                List.copyOf(SHOOTER_TARGETS.keySet()),
                                "HUB",
                                "Field target to aim at")),
                n ->
                        ShooterSubsystem.getInstance()
                                .setTarget(SHOOTER_TARGETS.get(n.getString("target"))));

        // ---------------------------------------------------------------- driving
        factory.registerNodeType(
                FollowTrajectoryNode.ID,
                NodeKind.ACTION,
                "Follow a Choreo trajectory until its end setpoint is reached",
                FollowTrajectoryNode.PORTS,
                FollowTrajectoryNode::new);
        factory.registerNodeType(
                WaitForChoreoEventNode.ID,
                NodeKind.ACTION,
                "Wait until the running trajectory passes a Choreo event marker",
                WaitForChoreoEventNode.PORTS,
                WaitForChoreoEventNode::new);

        // ---------------------------------------------------------------- conditions
        factory.registerSimpleCondition(
                "IsAtChoreoSetpoint",
                "The swerve reached the end of its Choreo trajectory",
                List.of(),
                n -> SwerveSubsystem.getInstance().isAtChoreoSetpoint());
        factory.registerSimpleCondition(
                "HasChoreoTimeElapsed",
                "At least `seconds` of the current Choreo trajectory have run",
                List.of(PortInfo.input("seconds", PortType.DOUBLE, "Seconds on the path")),
                n -> SwerveSubsystem.getInstance().hasChoreoTimeElapsed(n.getDouble("seconds")));
        factory.registerSimpleCondition(
                "HasChoreoEventPassed",
                "The running trajectory passed a Choreo event marker",
                List.of(PortInfo.input("event", PortType.STRING, "Choreo event marker name")),
                n -> SwerveSubsystem.getInstance().hasChoreoEventBeenPassed(n.getString("event")));
        factory.registerSimpleCondition(
                "IsChassisStationary",
                "The chassis is not moving",
                List.of(),
                n -> SwerveSubsystem.getInstance().isChassisStationary());
        factory.registerSimpleCondition(
                "IsShooterReady",
                "Flywheel, hood and aim are within tolerance",
                List.of(),
                n -> ShooterSubsystem.getInstance().isShooterReady());
    }
}
