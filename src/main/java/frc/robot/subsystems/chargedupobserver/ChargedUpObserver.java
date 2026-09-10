package frc.robot.subsystems.chargedupobserver;

import com.marswars.dashboard.DashboardBridge;
import com.marswars.dashboard.DashboardChannel;
import com.marswars.util.NumUtil;

/**
 * A browser-dashboard observer for 2023 <b>Charged Up</b>. It tracks the alliance community grid --
 * 3 rows (bottom / middle / top) of 9 nodes each -- as three 9-bit bitfield ints, one bit per
 * column. In the top and middle rows, columns {@code {1, 4, 7}} are cube nodes and the rest are
 * cone nodes; the bottom row is all hybrid (any piece). A human also toggles a {@code coopertition}
 * flag, and the robot computes the number of Links (three filled nodes in a row) and reports it
 * back read-only.
 *
 * <p>This is one of two side-by-side example observers in this project (the other is {@code
 * frc.robot.subsystems.deepspaceobserver.DeepSpaceObserver}); both run at once on different ports.
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
 * <p>This is a scouting aid, not a rules engine: the {@code coopertition} flag is a manual marker,
 * and it does not model RP thresholds, the Coopertition bonus activation window, auto mobility /
 * charge station, or endgame. The per-row Link arithmetic itself is faithful to the 2023 rule.
 */
public class ChargedUpObserver {
    private static ChargedUpObserver instance_ = null;

    public static ChargedUpObserver getInstance() {
        if (instance_ == null) {
            instance_ = new ChargedUpObserver();
        }
        return instance_;
    }

    // One DashboardChannel per NT topic. Bit c of each grid int is column c, left to right.
    // grid_* are bidirectional (the human sets them); links is OUTPUT_ONLY (the robot computes it
    // and only reports it -- there is no /ChargedUp/ToRobot/links topic).
    private static final DashboardChannel GRID_BOTTOM =
            DashboardChannel.bidirectionalInt("grid_bottom");
    private static final DashboardChannel GRID_MIDDLE =
            DashboardChannel.bidirectionalInt("grid_middle");
    private static final DashboardChannel GRID_TOP = DashboardChannel.bidirectionalInt("grid_top");
    private static final DashboardChannel COOPERTITION =
            DashboardChannel.bidirectionalBool("coopertition");
    private static final DashboardChannel LINKS = DashboardChannel.outputInt("links");

    // Table paths, web-server port, and deploy subdir. Every topic name above and both table paths
    // must be spelled identically in src/main/deploy/chargedup/index.js. 5802 is the Deep Space
    // observer; two WebServers in one JVM only coexist on different ports.
    private static final DashboardBridge.Config CONFIG =
            new DashboardBridge.Config(
                    "/ChargedUp/ToRobot", "/ChargedUp/ToDashboard", 5803, "chargedup");

    private final DashboardBridge bridge_ =
            new DashboardBridge(CONFIG, GRID_BOTTOM, GRID_MIDDLE, GRID_TOP, COOPERTITION, LINKS);

    // The robot's authoritative copy of the grid. Element c is column c (0..8).
    private boolean[][] grid_ = new boolean[3][9];
    private boolean coopertition_ = false;
    private int links_ = 0;

    private ChargedUpObserver() {
        bridge_.startWebServer();
    }

    /** Call once per loop from {@code Robot.robotPeriodic()}. */
    public void periodic() {
        bridge_.readInputs();

        // --- Inbound: apply dashboard-originated changes. ----------------------------------
        bridge_.getIntIfChanged(GRID_BOTTOM)
                .ifPresent(bits -> grid_[0] = NumUtil.unpackBits(bits, 9));
        bridge_.getIntIfChanged(GRID_MIDDLE)
                .ifPresent(bits -> grid_[1] = NumUtil.unpackBits(bits, 9));
        bridge_.getIntIfChanged(GRID_TOP)
                .ifPresent(bits -> grid_[2] = NumUtil.unpackBits(bits, 9));
        bridge_.getBoolIfChanged(COOPERTITION).ifPresent(value -> coopertition_ = value);

        // --- Robot-side logic: recompute derived state before the mirror-out. --------------
        links_ = linksInRow(grid_[0]) + linksInRow(grid_[1]) + linksInRow(grid_[2]);
        // --- Outbound: mirror authoritative state (including the OUTPUT_ONLY links) back out.
        //     Safe to call every loop -- DashboardBridge no-ops when nothing changed. --------
        bridge_.set(GRID_BOTTOM, NumUtil.packBits(grid_[0]));
        bridge_.set(GRID_MIDDLE, NumUtil.packBits(grid_[1]));
        bridge_.set(GRID_TOP, NumUtil.packBits(grid_[2]));
        bridge_.set(COOPERTITION, coopertition_);
        bridge_.set(LINKS, links_);
    }

    // --- What the rest of the robot reads. -------------------------------------------------

    /** Total filled nodes across all three grid rows. */
    public int totalNodes() {
        return count(grid_[0]) + count(grid_[1]) + count(grid_[2]);
    }

    /** Number of Links (three filled nodes in a row), summed across the three rows. */
    public int links() {
        return links_;
    }

    /**
     * Teleop point value of the grid state (2 / 3 / 5 per node by row, +5 per Link). Approximate.
     */
    public int totalPoints() {
        return 2 * count(grid_[0])
                + 3 * count(grid_[1])
                + 5 * count(grid_[2])
                + 5 * links_;
    }
    public boolean canSuperCharge(){
        return totalNodes() == 27;
    }

    public int getRPRequirement() {
        int count = 0;
        for(int j=0; j==3; j++){
            for(int i=3; i==6; i++){
            if(grid_[j][i]){
                count++;
            }
        }
        }
        if(coopertition_ && count >=3){
            return 5;
        }
        return 6;
    }
    public boolean hasRP(){
        return links_ >= getRPRequirement();
        
    }


    /** Greedy, non-overlapping: a node belongs to at most one Link. Max 3 per 9-wide row. */
    private static int linksInRow(boolean[] row) {
        int n = 0;
        for (int c = 0; c + 2 < row.length; ) {
            if (row[c] && row[c + 1] && row[c + 2]) {
                n++;
                c += 3;
            } else {
                c++;
            }
        }
        return n;
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
