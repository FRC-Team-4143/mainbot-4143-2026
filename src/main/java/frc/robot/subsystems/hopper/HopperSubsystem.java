package frc.robot.subsystems.hopper;

import com.marswars.mechanisms.RollerMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;

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

    private RollerMech feeder_;
    private RollerMech hopper_;
    private final Timer hopper_timer_ = new Timer();
    private Debouncer debouncer_ = new Debouncer(CONSTANTS.DEBOUNCE_TIME, Debouncer.DebounceType.kBoth);

    public HopperSubsystem() {
        super(HopperStates.IDLE, new HopperContstants());
        feeder_ = new RollerMech(
                getSubsystemKey(),
                "Feeder",
                List.of(CONSTANTS.FEED_MOTOR_CONFIG),
                CONSTANTS.FEED_GEAR_RATIO);

        hopper_ = new RollerMech(
                getSubsystemKey(),
                "Hopper",
                List.of(CONSTANTS.HOPPER_MOTOR_CONFIG),
                CONSTANTS.HOPPER_GEAR_RATIO);
    }

    @Override
    public void handleStateTransition(HopperStates wanted) {
        boolean jammed = isJammed();
        DogLog.log(getSubsystemKey()+"jammed", jammed);
        if (jammed && (system_state_ == HopperStates.SHOOTING)) {
            system_state_ = HopperStates.UNJAMA;
            hopper_timer_.start();
        }
        if (hopper_timer_.hasElapsed(0.5) && (system_state_ == HopperStates.UNJAMA)
                || (system_state_ == HopperStates.UNJAMB)) {
            system_state_ = (system_state_ == HopperStates.UNJAMA)
                    ? HopperStates.UNJAMB
                    : HopperStates.UNJAMA;
            hopper_timer_.reset();
        }
        if ((system_state_ == HopperStates.IDLE) && (wanted == HopperStates.UNJAMB)) {
            system_state_ = HopperStates.UNJAMA;
        }
        if ((system_state_ == HopperStates.SHOOTING) && (wanted == HopperStates.UNJAMB)) {
            system_state_ = HopperStates.UNJAMA;
        }
        if ((system_state_ == HopperStates.UNJAMA) && (!jammed)) {
            system_state_ = HopperStates.SHOOTING;
        }
        if (((system_state_ == HopperStates.UNJAMB) && (wanted == HopperStates.IDLE))) {
            system_state_ = HopperStates.SHOOTING;
        }
        if ((system_state_ == HopperStates.IDLE) && (wanted == HopperStates.SHOOTING)) {
            system_state_ = HopperStates.SHOOTING;
        }
        if ((system_state_ == HopperStates.SHOOTING) && (wanted == HopperStates.IDLE)) {
            system_state_ = HopperStates.IDLE;
        }

    }

    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case IDLE:
                hopper_.setTargetDutyCycle(0.0);
                feeder_.setTargetDutyCycle(0.0);
                break;
            case SHOOTING:
                hopper_.setTargetDutyCycle(CONSTANTS.HOPPER_DUTY_CYCLE);
                feeder_.setTargetDutyCycle(CONSTANTS.FEED_DUTY_CYCLE);
                break;
            case UNJAMA:
                hopper_.setTargetDutyCycle(-CONSTANTS.FEED_DUTY_CYCLE);
                feeder_.setTargetDutyCycle(-CONSTANTS.FEED_DUTY_CYCLE);
                break;
            case UNJAMB:
                hopper_.setTargetDutyCycle(CONSTANTS.HOPPER_DUTY_CYCLE);
                feeder_.setTargetDutyCycle(CONSTANTS.FEED_DUTY_CYCLE);
                break;
            case PROFILE:
                break;
        }
        // Log Data
    }

    // return true if jammed, false otherwise
    public boolean isJammed() {
        if (system_state_ == HopperStates.UNJAMB) {
            return true;
        }
        boolean jamCondition_ = hopper_.getLeaderCurrent() > CONSTANTS.HOPPER_DANGER_CURRENT;
        return debouncer_.calculate(jamCondition_);
    }

    public void fakeJam() {
        hopper_.applyLoadTorque(50000);
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(feeder_, hopper_);
    }

    @Override
    public void reset() {
        system_state_ = HopperStates.IDLE;
    }
}
