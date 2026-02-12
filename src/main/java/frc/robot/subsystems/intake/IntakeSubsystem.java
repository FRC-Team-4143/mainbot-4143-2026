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

    private RollerMech roller_;
    private RollerMech pivot_;

    public IntakeSubsystem() {
        super(IntakeStates.STORE, new IntakeConstants());
        roller_ =
                new RollerMech(
                        getSubsystemKey(),
                        "Roller",
                        List.of(CONSTANTS.ROLLER_MOTOR_CONFIG),
                        CONSTANTS.ROLLER_GEAR_RATIO);

        pivot_ =
                new RollerMech(
                        getSubsystemKey(),
                        "Pivot",
                        List.of(CONSTANTS.PIVOT_MOTOR_CONFIG),
                        CONSTANTS.PIVOT_GEAR_RATIO);
    }

    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case STORE:
                roller_.setTargetDutyCycle(0.0);
                pivot_.setTargetPosition(0.0);
                break;
            case DEPLOY:
                roller_.setTargetDutyCycle(0.0);
                pivot_.setTargetPosition(0.5);
                break;
            case INTAKE:
                roller_.setTargetDutyCycle(0.5);
                pivot_.setTargetPosition(0.5);
                break;
            case OUTTAKE:
                roller_.setTargetDutyCycle(-0.5);
                pivot_.setTargetPosition(0.5);
                break;
        }
    }

    protected void handleStateTransition(IntakeStates wantedState) {
        if ((system_state_ == IntakeStates.STORE) && (wantedState == IntakeStates.INTAKE)) {
            system_state_ = IntakeStates.DEPLOY;
        } else if ((system_state_ == IntakeStates.INTAKE) && (wantedState == IntakeStates.STORE)) {
            system_state_ = IntakeStates.DEPLOY;
        } else if ((system_state_ == IntakeStates.STORE) && (wantedState == IntakeStates.DEPLOY)) {
            system_state_ = IntakeStates.DEPLOY;
        } else if ((system_state_ == IntakeStates.DEPLOY) && (wantedState == IntakeStates.STORE)) {
            system_state_ = IntakeStates.STORE;
        } else if ((system_state_ == IntakeStates.DEPLOY) && (wantedState == IntakeStates.INTAKE)) {
            system_state_ = IntakeStates.INTAKE;
        } else if ((system_state_ == IntakeStates.INTAKE) && (wantedState == IntakeStates.DEPLOY)) {
            system_state_ = IntakeStates.DEPLOY;
        }
    }

    @Override
    public void reset() {
        system_state_ = IntakeStates.STORE;
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(roller_, pivot_);
    }
}
