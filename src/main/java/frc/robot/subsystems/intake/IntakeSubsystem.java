package frc.robot.subsystems.intake;

import com.marswars.mechanisms.RollerMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import java.util.Arrays;
import java.util.List;

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
        super(IntakeStates.CLOSED, new IntakeConstants());
        intaker_ =
                new RollerMech(
                        getSubsystemKey(),
                        "Intaker",
                        List.of(CONSTANTS.INTAKE_MOTOR_CONFIG),
                        CONSTANTS.INTAKE_GEAR_RATIO);

        armer_ =
                new RollerMech(
                        getSubsystemKey(),
                        "Armer",
                        List.of(CONSTANTS.ARM_MOTOR_CONFIG),
                        CONSTANTS.ARM_GEAR_RATIO);
    }

    @Override
    public void updateLogic(double timestamp) { // all placeholders right now
        switch (system_state_) {
            case CLOSED:
                intaker_.setTargetDutyCycle(0.0);
                armer_.setTargetPosition(0.0);
                break;
            case DEPLOYED:
                intaker_.setTargetDutyCycle(0.0);
                armer_.setTargetPosition(0.5);
                break;
            case ROLLING:
                intaker_.setTargetDutyCycle(0.5);
                armer_.setTargetPosition(0.5);
                break;
        }
    }

    protected void handleStateTransition(IntakeStates wantedState) {
        if ((system_state_ == IntakeStates.CLOSED) && (wantedState == IntakeStates.ROLLING)) {
            system_state_ = IntakeStates.DEPLOYED;
        } else if ((system_state_ == IntakeStates.ROLLING)
                && (wantedState == IntakeStates.CLOSED)) {
            system_state_ = IntakeStates.DEPLOYED;
        } else if ((system_state_ == IntakeStates.CLOSED)
                && (wantedState == IntakeStates.DEPLOYED)) {
            system_state_ = IntakeStates.DEPLOYED;
        } else if ((system_state_ == IntakeStates.DEPLOYED)
                && (wantedState == IntakeStates.CLOSED)) {
            system_state_ = IntakeStates.CLOSED;
        } else if ((system_state_ == IntakeStates.DEPLOYED)
                && (wantedState == IntakeStates.ROLLING)) {
            system_state_ = IntakeStates.ROLLING;
        } else if ((system_state_ == IntakeStates.ROLLING)
                && (wantedState == IntakeStates.DEPLOYED)) {
            system_state_ = IntakeStates.DEPLOYED;
        }
    }

    @Override
    public void reset() {
        system_state_ = IntakeStates.CLOSED;
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(intaker_, armer_);
    }
}
