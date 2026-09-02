package frc.robot.subsystems.observertemplate;

import com.marswars.dashboard.DashboardBridge;
import com.marswars.dashboard.DashboardChannel;
import com.marswars.util.NumUtil;

/**
 * TEMPLATE -- not wired into RobotContainer, not meant to run as-is. Copy this whole package,
 * rename it and this class to match your game (e.g. "ReefObserver", "HubObserver"), then work
 * through the TODOs below.
 *
 * <p>Built on MW-Lib's {@code com.marswars.dashboard.DashboardBridge}/{@code DashboardChannel} --
 * see those classes' javadoc for the underlying NT/WebServer plumbing. For a fully-worked
 * prior-year example of the same *pattern* (before this plumbing was pulled out into MW-Lib), see
 * mainbot-2025's {@code frc.robot.subsystems.ReefObserver}.
 *
 * <p><b>Not an MwSubsystem.</b> There's no meaningful enum state for a dashboard bridge to
 * transition between, so this follows {@code AutoManager}'s shape instead: a plain singleton with
 * a {@code periodic()} method you call yourself from {@code Robot.java} (see
 * {@code disabledPeriodic()}'s {@code AutoManager.getInstance().periodic();} for the pattern) --
 * not {@code registerSubsystem()}.
 *
 * <p><b>Still needed beyond this file</b>, none of which MW-Lib provides on purpose:
 * <ul>
 *   <li>The HTML/CSS/JS dashboard web app under {@code src/main/deploy/<your deploy subdir>/},
 *       with NT topic names matching the {@link DashboardChannel}s declared below.
 *   <li>Whatever game-specific state you're actually tracking (an occupancy grid, piece counts,
 *       whatever this year's field looks like) -- {@code DashboardBridge} only moves raw ints and
 *       booleans over NetworkTables; it has no idea what any of them mean.
 * </ul>
 */
public class ObserverTemplate {
    private static ObserverTemplate instance_ = null;

    public static ObserverTemplate getInstance() {
        if (instance_ == null) {
            instance_ = new ObserverTemplate();
        }
        return instance_;
    }


    private static final DashboardChannel LEFT_ROCKET_CARGO = DashboardChannel.bidirectionalInt("left_rocket_cargo");
    private static final DashboardChannel LEFT_ROCKET_HATCH = DashboardChannel.bidirectionalInt("left_rocket_hatch");
    private static final DashboardChannel RIGHT_ROCKET_CARGO = DashboardChannel.bidirectionalInt("right_rocket_cargo");
    private static final DashboardChannel RIGHT_ROCKET_HATCH = DashboardChannel.bidirectionalInt("right_rocket_hatch");
    private static final DashboardChannel CARGO_SHIP_CARGO = DashboardChannel.bidirectionalInt("cargo_ship_cargo");
    private static final DashboardChannel CARGO_SHIP_HATCH = DashboardChannel.bidirectionalInt("cargo_ship_hatch");

    private static final DashboardBridge.Config CONFIG =
            new DashboardBridge.Config(
                    "/DeepSpace/ToRobot", "/DeepSpace/ToDashboard", 5802, "TODO_deploy_subdir");

    private final DashboardBridge bridge_ =
            new DashboardBridge(CONFIG, LEFT_ROCKET_CARGO, LEFT_ROCKET_HATCH, RIGHT_ROCKET_CARGO, RIGHT_ROCKET_HATCH, CARGO_SHIP_CARGO, CARGO_SHIP_HATCH);

    private boolean[] left_rocket_hatch = new boolean[6];
    private boolean[] left_rocket_cargo = new boolean[6];
    private boolean[] right_rocket_hatch = new boolean[6];
    private boolean[] right_rocket_cargo = new boolean[6];
    private boolean[] cargo_ship_hatch = new boolean[8];
    private boolean[] cargo_ship_cargo = new boolean[8];

    private ObserverTemplate() {
        bridge_.startWebServer();
    }

    /**
     * Call once per loop from Robot.java -- see AutoManager.get    // TODO: name these for your game's actual concepts (see ReefObserver's SELECTED_LEVEL,
    // L2_STATE, COOP_STATE, IS_ELIMS for a worked example). One DashboardChannel per NT topic.Instance().periodic() for the
     * calling pattern.
     */
    public void periodic() {
        bridge_.readInputs();

        // TODO: for each bidirectional channel, react to a dashboard-originated change.
        bridge_
                .getIntIfChanged(LEFT_ROCKET_CARGO)
                .ifPresent(bits -> left_rocket_cargo = NumUtil.unpackBits(bits, left_rocket_cargo.length));
        bridge_
                .getIntIfChanged(RIGHT_ROCKET_CARGO)
                .ifPresent(bits -> right_rocket_cargo = NumUtil.unpackBits(bits, right_rocket_cargo.length));
        bridge_
                .getIntIfChanged(LEFT_ROCKET_HATCH)
                .ifPresent(bits -> left_rocket_hatch = NumUtil.unpackBits(bits, left_rocket_hatch.length));
        bridge_
                .getIntIfChanged(RIGHT_ROCKET_HATCH)
                .ifPresent(bits -> right_rocket_hatch = NumUtil.unpackBits(bits, right_rocket_hatch.length));
        bridge_
                .getIntIfChanged(CARGO_SHIP_CARGO)
                .ifPresent(bits -> cargo_ship_cargo = NumUtil.unpackBits(bits, cargo_ship_cargo.length));
        bridge_
                .getIntIfChanged(CARGO_SHIP_HATCH)
                .ifPresent(bits -> cargo_ship_hatch = NumUtil.unpackBits(bits, cargo_ship_hatch.length));
        //bridge_.getBoolIfChanged(EXAMPLE_FLAG).ifPresent(value -> example_flag_ = value);

        // TODO: your own game logic updates example_bits_/example_flag_ here too, independent of
        // the dashboard -- e.g. the robot auto-scoring something without a human tapping anything.

        // Mirror the current authoritative state back out every loop -- DashboardBridge no-ops if
        // nothing actually changed since the last publish, so this is safe to call unconditionally.
        bridge_.set(LEFT_ROCKET_CARGO, NumUtil.packBits(left_rocket_cargo));
        bridge_.set(RIGHT_ROCKET_CARGO, NumUtil.packBits(right_rocket_cargo));
        bridge_.set(LEFT_ROCKET_HATCH, NumUtil.packBits(left_rocket_hatch));
        bridge_.set(RIGHT_ROCKET_HATCH, NumUtil.packBits(right_rocket_hatch));
        bridge_.set(CARGO_SHIP_CARGO, NumUtil.packBits(cargo_ship_cargo));
        bridge_.set(CARGO_SHIP_HATCH, NumUtil.packBits(cargo_ship_hatch));
        //bridge_.set(EXAMPLE_FLAG, example_flag_);
    }
}
