package frc.robot.autos.bt;

import com.marswars.bt.RobotStateNodes;
import com.marswars.bt.core.BehaviorTreeFactory;
import com.marswars.bt.core.PortInfo;
import com.marswars.bt.swerve.SwerveNodes;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotContainer;
import frc.robot.RobotStates;
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
        // ---------------------------------------------------------------- robot states
        // SetWantedState / IsSystemState: the same robot states (and sequences) teleop uses, from
        // the RobotContainer super state machine. Prefer these over the subsystem nodes below.
        RobotStateNodes.register(factory, RobotContainer::getInstance, RobotStates.class);

        // ---------------------------------------------------------------- subsystem states
        // For small one-off moves (an intake state on a path event) that aren't a robot state.
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
        // FollowTrajectory, WaitForChoreoEvent, SetSwerveState and the Choreo/chassis conditions
        // are MW-Lib's shared swerve nodes.
        SwerveNodes.register(factory, SwerveSubsystem::getInstance);

        // ---------------------------------------------------------------- conditions
        factory.registerSimpleCondition(
                "IsShooterReady",
                "Flywheel, hood and aim are within tolerance",
                List.of(),
                n -> ShooterSubsystem.getInstance().isShooterReady());
    }
}
