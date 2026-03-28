package frc.robot.subsystems.hopper;

import com.marswars.mechanisms.RollerMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.OI;
import frc.robot.subsystems.hopper.HopperConstants.HopperStates;
import java.util.Arrays;
import java.util.List;

public class HopperSubsystem extends MwSubsystem<HopperStates, HopperConstants> {
    private static HopperSubsystem instance_ = null;

    // private RollerMech feeder_;
    private RollerMech hopper_;
    private double manual_hopper_velocity_ = CONSTANTS.HOPPER_VELOCITY_TARGET;
    private final Timer hopper_timer_ = new Timer();
    private Debouncer debouncer_ =
            new Debouncer(CONSTANTS.DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);

    // getInstance
    public static HopperSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new HopperSubsystem();
        }
        return instance_;
    }

    // Constructor
    public HopperSubsystem() {
        super(HopperStates.IDLE, new HopperConstants());
        hopper_ =
                new RollerMech(
                        getSubsystemKey(),
                        "Hopper",
                        List.of(CONSTANTS.HOPPER_MOTOR_CONFIG),
                        CONSTANTS.HOPPER_GEAR_RATIO);

        DogLog.tunable(
                getSubsystemKey() + "/Hopper/TargetVelocity",
                manual_hopper_velocity_,
                (v) -> manual_hopper_velocity_ = v);
    }

    // reset
    @Override
    public void reset() {
        system_state_ = HopperStates.IDLE;
    }

    // getIos
    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(hopper_);
    }

    // handleStateTransition
    @Override
    public void handleStateTransition(HopperStates wanted) {
        boolean jammed = isJammed();
        DogLog.log(getSubsystemKey() + "Jammed", jammed);
        if (jammed && (system_state_ == HopperStates.SHOOTING)) {
            system_state_ = HopperStates.UNJAM_REVERSE;
            hopper_timer_.reset();
            hopper_timer_.start();
        } else if (hopper_timer_.hasElapsed(CONSTANTS.UNJAMM_TIMER)
                && ((system_state_ == HopperStates.UNJAM_REVERSE)
                        || (system_state_ == HopperStates.UNJAM_FORWARD))) {
            system_state_ =
                    (system_state_ == HopperStates.UNJAM_REVERSE)
                            ? HopperStates.UNJAM_FORWARD
                            : HopperStates.UNJAM_REVERSE;
            hopper_timer_.reset();
        } else if ((system_state_ == HopperStates.UNJAM_REVERSE) && (!jammed)) {
            system_state_ = HopperStates.SHOOTING;
            hopper_timer_.stop();
        } else if ((system_state_ == HopperStates.UNJAM_FORWARD) && (!jammed)) {
            system_state_ = HopperStates.SHOOTING;
            hopper_timer_.stop();
        } else {
            system_state_ = wanted;
        }
    }

    // updateLogic
    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case INTAKE:
            case SHOOTING:
                if (OI.getDriverControllerX()) hopper_.setTargetVelocity(-manual_hopper_velocity_);
                else hopper_.setTargetVelocity(manual_hopper_velocity_);
                break;
            case UNJAM_REVERSE:
                hopper_.setTargetVelocity(-manual_hopper_velocity_);
                break;
            case UNJAM_FORWARD:
                hopper_.setTargetVelocity(manual_hopper_velocity_);
                break;
            case TUNING:
                break;
            default:
            case IDLE:
                if (OI.getDriverControllerA()) hopper_.setTargetVelocity(-manual_hopper_velocity_);
                else hopper_.setTargetDutyCycle(0.0);
                break;
        }
    }

    // =============================================================================
    // PUBLIC HELPER METHODS
    // =============================================================================

    /**
     * @return true if jammed, false otherwise
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
}
