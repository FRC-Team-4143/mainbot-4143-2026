package frc.robot.subsystems.roof;

import com.marswars.mechanisms.ElevatorMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.roof.RoofConstants.RoofStates;
import java.util.Arrays;
import java.util.List;

public class RoofSubsystem extends MwSubsystem<RoofStates, RoofConstants> {
    private static RoofSubsystem instance_ = null;

    // private ELEVATORMECH elevator
    private ElevatorMech elevator_;
    private double roofConfirmedDown = 0.0;
    private Debouncer safetyDebouncer =
            new Debouncer(CONSTANTS.SAFETY_DEBOUNCER_TIME_SECONDS, Debouncer.DebounceType.kRising);

    // getInstance
    public static RoofSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new RoofSubsystem();
        }
        return instance_;
    }

    // Constructor
    public RoofSubsystem() {
        super(RoofStates.DOWN, new RoofConstants());
        elevator_ =
                new ElevatorMech(
                        getSubsystemKey(),
                        "Elevator",
                        List.of(CONSTANTS.ELEVATOR_MOTOR_CONFIG),
                        CONSTANTS.ELEVATOR_GEAR_RATIO,
                        CONSTANTS.ELEVATOR_DRUM_RADIUS,
                        CONSTANTS.ELEVATOR_CARRIAGE_MASS_KG,
                        CONSTANTS.ELEVATOR_MAX_EXTENSION_METERS,
                        CONSTANTS.ELEVATOR_RIGGING_RATIO);
        elevator_.setCurrentPosition(0);
        SmartDashboard.putData(
                "Home Roof",
                Commands.runOnce(
                        () -> elevator_.setCurrentPosition(CONSTANTS.ELEVATOR_HOME_POSITION)));
        SmartDashboard.putData(
                "Auto Home Roof", Commands.runOnce(() -> setWantedState(RoofStates.ROOF_HOMING)));
    }

    // reset
    @Override
    public void reset() {
        system_state_ = RoofStates.DOWN;
    }

    // getIos
    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(elevator_);
    }

    // handleStateTransition
    @Override
    public void handleStateTransition(RoofStates wanted) {
        if (elevator_.getLeaderCurrent() > CONSTANTS.ELEVATOR_HOMING_CURRENT_THRESHOLD
                && system_state_ == RoofStates.ROOF_HOMING) {
            elevator_.setCurrentPosition(CONSTANTS.ELEVATOR_HOME_POSITION);
            setWantedState(RoofStates.DOWN);
            system_state_ = RoofStates.DOWN;
        } else if (wanted == RoofStates.DOWN
                && !(system_state_ == RoofStates.DOWN || system_state_ == RoofStates.SAFETY_DOWN)) {
            system_state_ = RoofStates.SAFETY_DOWN;
            return;

        } else if (safetyDebouncer.calculate(
                        elevator_.getLeaderCurrent() > CONSTANTS.ELEVATOR_HOMING_CURRENT_THRESHOLD)
                && system_state_ == RoofStates.SAFETY_DOWN) {
            roofConfirmedDown = elevator_.getCurrentPosition();
            system_state_ = RoofStates.DOWN;
        } else if (system_state_ == RoofStates.SAFETY_DOWN) {
            return;
        } else if (system_state_ != RoofStates.CLIMB && wanted == RoofStates.CLIMB) {
            system_state_ = wanted;
            elevator_.configSlot(0, CONSTANTS.ELEVATOR_CLIMB_POSITION_GAINS);
        } else if (wanted != RoofStates.CLIMB && system_state_ == RoofStates.CLIMB) {
            system_state_ = wanted;
            elevator_.configSlot(0, CONSTANTS.ELEVATOR_POSITION_GAINS);
        } else if (wanted == RoofStates.SQUEEZE
                && elevator_.getCurrentPosition() <= CONSTANTS.ELEVATOR_SQUEEZE_MIN_POSITION) {
            system_state_ = getIdlStates();
            setWantedState(getIdlStates());
        }
        system_state_ = wanted;
    }

    // updateLogic
    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case UP:
                elevator_.setTargetPosition(CONSTANTS.ELEVATOR_UP_POSITION_METERS);
                break;
            case DOWN:
                elevator_.setTargetPosition(roofConfirmedDown);
                break;
            case SAFETY_DOWN:
                elevator_.setTargetPosition(CONSTANTS.ELEVATOR_DOWN_POSITION_METERS - 0.05);
                break;
            case CLIMB:
                elevator_.setTargetPosition(CONSTANTS.ELEVATOR_DOWN_POSITION_METERS);
                break;
            case ROOF_HOMING:
                elevator_.setTargetDutyCycle(CONSTANTS.ELEVATOR_HOMING_DUTY_CYCLE);
                break;
            case SQUEEZE:
                elevator_.setTargetCurrent(CONSTANTS.ELEVATOR_SQUEEZE_CURRENT);
                break;
            case TUNING:
                break;
        }
    }

    // =============================================================================
    // PUBLIC HELPER METHODS
    // =============================================================================

    public boolean isDown() {
        return elevator_.getCurrentPosition() < (CONSTANTS.ELEVATOR_DOWN_POSITION_METERS + 0.02);
    }

    public RoofStates getIdlStates() {
        if (elevator_.getCurrentPosition() < CONSTANTS.ELEVATOR_SQUEEZE_MIN_POSITION) {
            return RoofStates.DOWN;
        } else {
            return RoofStates.UP;
        }
    }
}
