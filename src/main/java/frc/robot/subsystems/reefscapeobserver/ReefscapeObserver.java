package frc.robot.subsystems.reefscapeobserver;

import com.marswars.dashboard.DashboardBridge;
import com.marswars.dashboard.DashboardChannel;
import com.marswars.util.NumUtil;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * A browser-dashboard observer for 2025 <b>Reefscape</b>. It tracks the reef: coral on levels L2,
 * L3, L4 (a 12-bit bitfield each, one bit per branch), the L1 trough as a plain count, and algae (a
 * 6-bit bitfield). A human also toggles {@code coop} and {@code rp_focus}. The robot reports two
 * things back read-only: {@code is_elims} (from the match type) and {@code rp_complete} (whether
 * the Coral ranking point is met), and it overrides {@code coop} / {@code rp_focus} the way the
 * real game does -- no coop in elims, and focus clears once the RP is in hand.
 *
 * <p>This is the richest of the example observers -- it uses every {@link DashboardChannel} shape.
 * The others are {@code frc.robot.subsystems.deepspaceobserver.DeepSpaceObserver} and {@code
 * frc.robot.subsystems.chargedupobserver.ChargedUpObserver}; all run at once on different ports.
 * See {@code src/main/deploy/deepspace/README.md} for how to add another year's observer.
 *
 * <p>Built on MW-Lib's {@code com.marswars.dashboard.DashboardBridge}/{@code DashboardChannel} --
 * see those classes' javadoc for the NT4/WebServer plumbing. {@code DashboardBridge} only moves raw
 * ints and booleans over NetworkTables; it has no idea what any of them mean.
 *
 * <p><b>Not an MwSubsystem.</b> There is no enum state to transition between, so this follows
 * {@code AutoManager}'s shape: a plain singleton whose {@code periodic()} is called directly from
 * {@code Robot.robotPeriodic()}.
 *
 * <p>This is a scouting aid, not a rules engine. The RP arithmetic mirrors mainbot-2025's {@code
 * ReefObserver} (>= 5 coral on enough levels, coop lowering the bar by one).
 */
public class ReefscapeObserver {
    private static final int CORAL_NEEDED_PER_LEVEL = 5;

    private static ReefscapeObserver instance_ = null;

    public static ReefscapeObserver getInstance() {
        if (instance_ == null) {
            instance_ = new ReefscapeObserver();
        }
        return instance_;
    }

    // One DashboardChannel per NT topic. l2/l3/l4 are 12-bit branch bitfields; l1 is a scalar
    // count; algae is a 6-bit bitfield. coop / rp_focus are human toggles. is_elims / rp_complete
    // are OUTPUT_ONLY -- the robot derives them and only reports them.
    private static final DashboardChannel L1 = DashboardChannel.bidirectionalInt("l1");
    private static final DashboardChannel L2 = DashboardChannel.bidirectionalInt("l2");
    private static final DashboardChannel L3 = DashboardChannel.bidirectionalInt("l3");
    private static final DashboardChannel L4 = DashboardChannel.bidirectionalInt("l4");
    private static final DashboardChannel ALGAE = DashboardChannel.bidirectionalInt("algae");
    private static final DashboardChannel COOP = DashboardChannel.bidirectionalBool("coop");
    private static final DashboardChannel RP_FOCUS = DashboardChannel.bidirectionalBool("rp_focus");
    private static final DashboardChannel IS_ELIMS = DashboardChannel.outputBool("is_elims");
    private static final DashboardChannel RP_COMPLETE = DashboardChannel.outputBool("rp_complete");

    // Table paths, web-server port, and deploy subdir. Every topic name above and both table paths
    // must be spelled identically in src/main/deploy/reefscape/index.js. 5802 = Deep Space,
    // 5803 = Charged Up; two WebServers in one JVM only coexist on different ports.
    private static final DashboardBridge.Config CONFIG =
            new DashboardBridge.Config(
                    "/Reefscape/ToRobot", "/Reefscape/ToDashboard", 5804, "reefscape");

    private final DashboardBridge bridge_ =
            new DashboardBridge(
                    CONFIG, L1, L2, L3, L4, ALGAE, COOP, RP_FOCUS, IS_ELIMS, RP_COMPLETE);

    // The robot's authoritative copy of the reef.
    private int l1_ = 0; // trough count
    private boolean[] l2_ = new boolean[12];
    private boolean[] l3_ = new boolean[12];
    private boolean[] l4_ = new boolean[12];
    private boolean[] algae_ = new boolean[6];
    private boolean coop_ = false;
    private boolean rp_focus_ = false;
    private boolean is_elims_ = false;
    private boolean rp_complete_ = false;

    private ReefscapeObserver() {
        bridge_.startWebServer();
    }

    /** Call once per loop from {@code Robot.robotPeriodic()}. */
    public void periodic() {
        bridge_.readInputs();

        // --- Inbound: apply dashboard-originated changes. --------------------------------------
        bridge_.getIntIfChanged(L1).ifPresent(v -> l1_ = Math.max(0, v));
        bridge_.getIntIfChanged(L2).ifPresent(bits -> l2_ = NumUtil.unpackBits(bits, 12));
        bridge_.getIntIfChanged(L3).ifPresent(bits -> l3_ = NumUtil.unpackBits(bits, 12));
        bridge_.getIntIfChanged(L4).ifPresent(bits -> l4_ = NumUtil.unpackBits(bits, 12));
        bridge_.getIntIfChanged(ALGAE).ifPresent(bits -> algae_ = NumUtil.unpackBits(bits, 6));
        bridge_.getBoolIfChanged(COOP).ifPresent(v -> coop_ = v);
        bridge_.getBoolIfChanged(RP_FOCUS).ifPresent(v -> rp_focus_ = v);

        // --- Robot-side logic: the robot is authoritative over these. -------------------------
        is_elims_ = DriverStation.getMatchType() == DriverStation.MatchType.Elimination;
        if (is_elims_) {
            coop_ = false; // no coopertition in elims
        }
        rp_complete_ = coralRpComplete();
        if (rp_complete_) {
            rp_focus_ = false; // nothing left to focus on once the RP is met
        }

        // --- Outbound: mirror everything (including the OUTPUT_ONLY pair and any override above).
        bridge_.set(L1, l1_);
        bridge_.set(L2, NumUtil.packBits(l2_));
        bridge_.set(L3, NumUtil.packBits(l3_));
        bridge_.set(L4, NumUtil.packBits(l4_));
        bridge_.set(ALGAE, NumUtil.packBits(algae_));
        bridge_.set(COOP, coop_);
        bridge_.set(RP_FOCUS, rp_focus_);
        bridge_.set(IS_ELIMS, is_elims_);
        bridge_.set(RP_COMPLETE, rp_complete_);
    }

    // --- What the rest of the robot reads. -----------------------------------------------------

    /** Total coral across L1 (trough) + L2/L3/L4 branches. */
    public int coralCount() {
        return l1_ + count(l2_) + count(l3_) + count(l4_);
    }

    public int algaeCount() {
        return count(algae_);
    }

    public boolean coralRpComplete() {
        int levelsMet = 0;
        if (l1_ >= CORAL_NEEDED_PER_LEVEL) {
            levelsMet++;
        }
        if (count(l2_) >= CORAL_NEEDED_PER_LEVEL) {
            levelsMet++;
        }
        if (count(l3_) >= CORAL_NEEDED_PER_LEVEL) {
            levelsMet++;
        }
        if (count(l4_) >= CORAL_NEEDED_PER_LEVEL) {
            levelsMet++;
        }
        return (coop_ && levelsMet >= 3) || levelsMet >= 4;
    }

    public boolean isElims() {
        return is_elims_;
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
