package frc.robot.subsystems.hopper;

import com.marswars.mechanisms.RollerMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.hopper.HopperConstants.HopperStates;
import java.util.Arrays;
import java.util.List;

public class HopperSubsystem extends MwSubsystem<HopperStates, HopperConstants> {
    private static HopperSubsystem instance_ = null;

    public static HopperSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new HopperSubsystem();
        }
        return instance_;
    }

    // private RollerMech feeder_;
    private RollerMech hopper_;
    private final Timer hopper_timer_ = new Timer();
    private Debouncer debouncer_ =
            new Debouncer(CONSTANTS.DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);

    // Manual control variables
    private double manual_hopper_percent_ = 0.0;
    private double manual_feeder_percent_ = 0.0;

    public HopperSubsystem() {
        super(HopperStates.IDLE, new HopperConstants());
        // feeder_ =
        //         new RollerMech(
        //                 getSubsystemKey(),
        //                 "Feeder",
        //                 List.of(CONSTANTS.FEED_MOTOR_CONFIG),
        //                 CONSTANTS.FEED_GEAR_RATIO);

        hopper_ =
                new RollerMech(
                        getSubsystemKey(),
                        "Hopper",
                        List.of(CONSTANTS.HOPPER_MOTOR_CONFIG),
                        CONSTANTS.HOPPER_GEAR_RATIO);

        DogLog.tunable(
                getSubsystemKey() + "Manual/Hopper Percent",
                manual_hopper_percent_,
                (val) -> manual_hopper_percent_ = val);
        DogLog.tunable(
                getSubsystemKey() + "Manual/Feeder Percent",
                manual_feeder_percent_,
                (val) -> manual_feeder_percent_ = val);
    }

    @Override
    public void handleStateTransition(HopperStates wanted) {
        boolean jammed = isJammed();
        DogLog.log(getSubsystemKey() + "Jammed", jammed);
        if (jammed && (system_state_ == HopperStates.SHOOTING)) {
            system_state_ = HopperStates.UNJAM_REVERSE;
            hopper_timer_.reset();
            hopper_timer_.start();
        }
        if (hopper_timer_.hasElapsed(CONSTANTS.UNJAMM_TIMER)
                && ((system_state_ == HopperStates.UNJAM_REVERSE)
                        || (system_state_ == HopperStates.UNJAM_FORWARD))) {
            system_state_ =
                    (system_state_ == HopperStates.UNJAM_REVERSE)
                            ? HopperStates.UNJAM_FORWARD
                            : HopperStates.UNJAM_REVERSE;
            hopper_timer_.reset();
        }
        if ((system_state_ == HopperStates.UNJAM_REVERSE) && (!jammed)) {
            system_state_ = HopperStates.SHOOTING;
            hopper_timer_.stop();
        }
        if ((system_state_ == HopperStates.UNJAM_FORWARD) && (!jammed)) {
            system_state_ = HopperStates.SHOOTING;
            hopper_timer_.stop();
        }
        if ((system_state_ == HopperStates.IDLE) && (wanted == HopperStates.SHOOTING)) {
            system_state_ = HopperStates.SHOOTING;
        }
        if ((system_state_ == HopperStates.SHOOTING) && (wanted == HopperStates.IDLE)) {
            system_state_ = HopperStates.IDLE;
        }
        if (system_state_ == HopperStates.IDLE && wanted == HopperStates.TUNING) {
            system_state_ = HopperStates.TUNING;
        }
    }

    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case SHOOTING:
                hopper_.setTargetDutyCycle(CONSTANTS.HOPPER_DUTY_CYCLE);
                // feeder_.setTargetDutyCycle(CONSTANTS.FEED_DUTY_CYCLE);
                break;
            case UNJAM_REVERSE:
                hopper_.setTargetDutyCycle(-CONSTANTS.HOPPER_DUTY_CYCLE);
                // feeder_.setTargetDutyCycle(-CONSTANTS.FEED_DUTY_CYCLE);
                break;
            case UNJAM_FORWARD:
                hopper_.setTargetDutyCycle(CONSTANTS.HOPPER_DUTY_CYCLE);
                // feeder_.setTargetDutyCycle(CONSTANTS.FEED_DUTY_CYCLE);
                break;
            case MANUAL:
                hopper_.setTargetDutyCycle(manual_hopper_percent_);
                // feeder_.setTargetDutyCycle(manual_feeder_percent_);
                break;
            case TUNING:
                break;
            default:
            case IDLE:
                hopper_.setTargetDutyCycle(0.0);
                // feeder_.setTargetDutyCycle(0.0);
                break;
        }
    }

    /**
     * returns true if jammed, false otherwise
     *
     * @return
     */
    public boolean isJammed() {
        boolean jamCondition =
                Math.abs(hopper_.getLeaderCurrent()) > CONSTANTS.HOPPER_DANGER_CURRENT;
        return debouncer_.calculate(jamCondition);
    }

    /** Applies load torque to simulate a jam */
    public void fakeJam() {
        hopper_.applyLoadTorque(50000);
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(hopper_);
    }

    @Override
    public void reset() {
        system_state_ = HopperStates.IDLE;
    }
}
