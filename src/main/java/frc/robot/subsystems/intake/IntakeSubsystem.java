package frc.robot.subsystems.intake;

import java.util.Arrays;
import java.util.List;

import com.marswars.mechanisms.RollerMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;

import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;

public class IntakeSubsystem extends MwSubsystem<IntakeStates, IntakeConstants> {
    private static IntakeSubsystem instance_ = null;

    public static IntakeSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new IntakeSubsystem();
        }
        return instance_;
    }

    private RollerMech intaker_;
    private RollerMech armer_;

    public IntakeSubsystem() {
        super(IntakeStates.PARK, new IntakeConstants());
        intaker_ = new RollerMech(
                getSubsystemKey(),
                "Intaker",
                List.of(CONSTANTS.INTAKE_MOTOR_CONFIG),
                CONSTANTS.INTAKE_GEAR_RATIO);

        armer_ = new RollerMech(
                getSubsystemKey(),
                "Armer",
                List.of(CONSTANTS.ARM_MOTOR_CONFIG),
                CONSTANTS.ARM_GEAR_RATIO);
    }

    @Override
    public void updateLogic(double timestamp) { // all placeholders right now
        switch (system_state_) {
            case PARK:
                intaker_.setTargetPosition(0.0);
                armer_.setTargetPosition(0.0);
                break;
            case UNDEPLOYED:
                intaker_.setTargetPosition(0.0);
                armer_.setTargetPosition(0.0);
                break;
            case DEPLOYED:
                intaker_.setTargetPosition(0.0);
                armer_.setTargetPosition(0.5);
                break;
            case ACTIVATED:
                intaker_.setTargetPosition(0.5);
                armer_.setTargetPosition(0.0);
                break;

        }
    }

    protected void handleStateTransistion(IntakeStates wantedState) {
        if ((system_state_ == IntakeStates.ACTIVATED)
                && ((wantedState == IntakeStates.UNDEPLOYED) || (wantedState == IntakeStates.PARK))) {
            system_state_ = IntakeStates.DEPLOYED;
        } else if ((system_state_ == IntakeStates.UNDEPLOYED) && (wantedState == IntakeStates.ACTIVATED)) {
            system_state_ = IntakeStates.DEPLOYED;
        } else if ((system_state_ == IntakeStates.PARK)
                && ((wantedState == IntakeStates.UNDEPLOYED) || (wantedState == IntakeStates.ACTIVATED))) {
            system_state_ = IntakeStates.DEPLOYED;
        } else if ((system_state_ == IntakeStates.DEPLOYED) && (wantedState == IntakeStates.PARK)) {
            system_state_ = IntakeStates.UNDEPLOYED;
        } else {
            system_state_ = wantedState;
        }
    }

    @Override
    public void reset() {
        system_state_ = IntakeStates.PARK;
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(intaker_, armer_);
    }
}
