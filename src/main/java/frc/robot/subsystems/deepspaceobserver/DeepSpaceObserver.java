package frc.robot.subsystems.deepspaceobserver;

import com.marswars.dashboard.DashboardBridge;
import com.marswars.dashboard.DashboardChannel;
import com.marswars.util.NumUtil;

/**
 * A browser-dashboard observer for 2019 <b>Deep Space</b>. It tracks scored game pieces on three
 * structures -- the left rocket, the right rocket, and the center cargo ship -- each as a
 * hatch-panel bitfield and a cargo bitfield, one bit per bay. The dashboard packs a {@code
 * boolean[]} into one int (see {@link NumUtil#packBits}), the robot unpacks it, applies its own
 * logic, and mirrors its authoritative copy back out on the {@code /DeepSpace/ToDashboard} table.
 *
 * <p>This is one of two side-by-side example observers in this project (the other is {@code
 * frc.robot.subsystems.chargedupobserver.ChargedUpObserver}); both run at once on different ports.
 * See {@code src/main/deploy/deepspace/README.md} for how to add another year's observer.
 *
 * <p>Built on MW-Lib's {@code com.marswars.dashboard.DashboardBridge}/{@code DashboardChannel} --
 * see those classes' javadoc for the NT4/WebServer plumbing. {@code DashboardBridge} only moves raw
 * ints and booleans over NetworkTables; it has no idea what any of them mean.
 *
 * <p><b>Not an MwSubsystem.</b> There is no enum state to transition between, so this follows
 * {@code AutoManager}'s shape: a plain singleton whose {@code periodic()} is called directly from
 * {@code Robot.robotPeriodic()} (so it runs in every mode -- a human uses this during auto and
 * teleop, not just while disabled).
 */
public class DeepSpaceObserver {
    private static DeepSpaceObserver instance_ = null;

    public static DeepSpaceObserver getInstance() {
        if (instance_ == null) {
            instance_ = new DeepSpaceObserver();
        }
        return instance_;
    }

    // One DashboardChannel per NT topic; the string is the wire name the web app publishes to /
    // subscribes from. Bit i of each packed int is bay i.
    private static final DashboardChannel LEFT_ROCKET_HATCH =
            DashboardChannel.bidirectionalInt("left_rocket_hatch");
    private static final DashboardChannel LEFT_ROCKET_CARGO =
            DashboardChannel.bidirectionalInt("left_rocket_cargo");
    private static final DashboardChannel RIGHT_ROCKET_HATCH =
            DashboardChannel.bidirectionalInt("right_rocket_hatch");
    private static final DashboardChannel RIGHT_ROCKET_CARGO =
            DashboardChannel.bidirectionalInt("right_rocket_cargo");
    private static final DashboardChannel CARGO_SHIP_HATCH =
            DashboardChannel.bidirectionalInt("cargo_ship_hatch");
    private static final DashboardChannel CARGO_SHIP_CARGO =
            DashboardChannel.bidirectionalInt("cargo_ship_cargo");

    // Table paths, web-server port, and deploy subdir. Every topic name above and both table paths
    // must be spelled identically in src/main/deploy/deepspace/index.js. Ports 5800-5810 are the
    // ones the FMS leaves open; 5800 is the default dashboard, 5803 is the Charged Up observer.
    private static final DashboardBridge.Config CONFIG =
            new DashboardBridge.Config(
                    "/DeepSpace/ToRobot", "/DeepSpace/ToDashboard", 5802, "deepspace");

    private final DashboardBridge bridge_ =
            new DashboardBridge(
                    CONFIG,
                    LEFT_ROCKET_HATCH,
                    LEFT_ROCKET_CARGO,
                    RIGHT_ROCKET_HATCH,
                    RIGHT_ROCKET_CARGO,
                    CARGO_SHIP_HATCH,
                    CARGO_SHIP_CARGO);

    // The robot's authoritative copy of each surface. A rocket has 6 bays (3 levels x 2 columns);
    // the cargo ship has 8 (2 on the front, 3 per side).
    private boolean[] left_rocket_hatch_ = new boolean[6];
    private boolean[] left_rocket_cargo_ = new boolean[6];
    private boolean[] right_rocket_hatch_ = new boolean[6];
    private boolean[] right_rocket_cargo_ = new boolean[6];
    private boolean[] cargo_ship_hatch_ = new boolean[8];
    private boolean[] cargo_ship_cargo_ = new boolean[8];

    private DeepSpaceObserver() {
        bridge_.startWebServer();
    }

    /** Call once per loop from {@code Robot.robotPeriodic()}. */
    public void periodic() {
        bridge_.readInputs();

        // --- Inbound: apply dashboard-originated changes. getIntIfChanged is empty on ticks where
        //     the human did not touch that surface. -------------------------------------------
        bridge_.getIntIfChanged(LEFT_ROCKET_HATCH)
                .ifPresent(bits -> left_rocket_hatch_ = NumUtil.unpackBits(bits, 6));
        bridge_.getIntIfChanged(LEFT_ROCKET_CARGO)
                .ifPresent(bits -> left_rocket_cargo_ = NumUtil.unpackBits(bits, 6));
        bridge_.getIntIfChanged(RIGHT_ROCKET_HATCH)
                .ifPresent(bits -> right_rocket_hatch_ = NumUtil.unpackBits(bits, 6));
        bridge_.getIntIfChanged(RIGHT_ROCKET_CARGO)
                .ifPresent(bits -> right_rocket_cargo_ = NumUtil.unpackBits(bits, 6));
        bridge_.getIntIfChanged(CARGO_SHIP_HATCH)
                .ifPresent(bits -> cargo_ship_hatch_ = NumUtil.unpackBits(bits, 8));
        bridge_.getIntIfChanged(CARGO_SHIP_CARGO)
                .ifPresent(bits -> cargo_ship_cargo_ = NumUtil.unpackBits(bits, 8));

        // --- Robot-side logic: these can also change with nobody touching the dashboard (auto
        //     scoring a piece off a sensor, etc.). Do that here, before the mirror-out below. ---

        // --- Outbound: mirror the authoritative state back out every loop. Safe to call every
        //     loop unconditionally -- DashboardBridge no-ops when a value has not changed since it
        //     last hit the wire. --------------------------------------------------------------
        bridge_.set(LEFT_ROCKET_HATCH, NumUtil.packBits(left_rocket_hatch_));
        bridge_.set(LEFT_ROCKET_CARGO, NumUtil.packBits(left_rocket_cargo_));
        bridge_.set(RIGHT_ROCKET_HATCH, NumUtil.packBits(right_rocket_hatch_));
        bridge_.set(RIGHT_ROCKET_CARGO, NumUtil.packBits(right_rocket_cargo_));
        bridge_.set(CARGO_SHIP_HATCH, NumUtil.packBits(cargo_ship_hatch_));
        bridge_.set(CARGO_SHIP_CARGO, NumUtil.packBits(cargo_ship_cargo_));
    }

    // --- What the rest of the robot reads. ---------------------------------------------------

    /** Total scored game pieces across both rockets and the cargo ship. */
    public int totalPoints() {
        return count(left_rocket_hatch_)
                + count(left_rocket_cargo_)
                + count(right_rocket_hatch_)
                + count(right_rocket_cargo_)
                + count(cargo_ship_hatch_)
                + count(cargo_ship_cargo_);
    }

    private static int count(boolean[] bits) {
        int n = 0;
        for (boolean b : bits) {
            if (b) {
                n++;
            }
        }
        return n;
    }
}
