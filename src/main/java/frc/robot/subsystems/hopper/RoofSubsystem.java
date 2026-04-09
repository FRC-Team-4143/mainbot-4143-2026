package frc.robot.subsystems.hopper;

import com.marswars.mechanisms.ElevatorMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;

import frc.robot.subsystems.hopper.RoofConstants.RoofStates;

import java.util.Arrays;
import java.util.List;

public class RoofSubsystem extends MwSubsystem<RoofStates, RoofConstants> {
    private static RoofSubsystem instance_ = null;

    //private ELEVATORMECH elevator
    private ElevatorMech elevator_;

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
        //DOWN transitions
        if (system_state_ == RoofStates.DOWN && wanted == RoofStates.UP) {
            system_state_ = RoofStates.UP;
        } else {
            //left empty to not interfere with elevator state machine
        }
        //UP transitions
        if (system_state_ == RoofStates.UP && wanted == RoofStates.DOWN) {
            system_state_ = RoofStates.DOWN;
        } else {
            //left empty to not interfere with elevator state machine
        }
        // CLIMB transitions
        
        // blank for now, will be filled in when we implement the climb state
    }

    // updateLogic
    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case UP:
                elevator_.setTargetPosition(CONSTANTS.ELEVATOR_MAX_EXTENSION_METERS);
                break;
            case DOWN:
                elevator_.setTargetPosition(0.0);
                break;
            case CLIMB:
                break;
        }
    }

    // =============================================================================
    // PUBLIC HELPER METHODS
    // =============================================================================


}
