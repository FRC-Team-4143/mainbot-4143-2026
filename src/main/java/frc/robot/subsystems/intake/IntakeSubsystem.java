package frc.robot.subsystems.intake;

import com.marswars.mechanisms.ArmMech;
import com.marswars.mechanisms.RollerMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
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
    private ArmMech pivot_;

    private double manaul_pivot_position_ = 0;
    private double manual_roller_percent_ = 0;

    public IntakeSubsystem() {
        super(IntakeStates.STORE, new IntakeConstants());
        roller_ =
                new RollerMech(
                        getSubsystemKey(),
                        "Roller",
                        List.of(CONSTANTS.ROLLER_MOTOR_CONFIG),
                        CONSTANTS.ROLLER_GEAR_RATIO);

        pivot_ =
                new ArmMech(
                        getSubsystemKey(),
                        "Pivot",
                        List.of(CONSTANTS.PIVOT_MOTOR_CONFIG),
                        CONSTANTS.PIVOT_GEAR_RATIO,
                        CONSTANTS.PIVOT_LENGTH,
                        CONSTANTS.PIVOT_MASS,
                        CONSTANTS.PIVOT_MIN,
                        CONSTANTS.PIVOT_MAX);
        pivot_.setCurrentPosition(CONSTANTS.PIVOT_HOME_POSITION);

        DogLog.tunable(
                getSubsystemKey() + "Manual/Pivot Position",
                manaul_pivot_position_,
                (val) -> manaul_pivot_position_ = val);
        DogLog.tunable(
                getSubsystemKey() + "Manual/Roller Percent",
                manual_roller_percent_,
                (val) -> manual_roller_percent_ = val);

        SmartDashboard.putData(
                "Home Pivot",
                Commands.runOnce(() -> pivot_.setCurrentPosition(CONSTANTS.PIVOT_HOME_POSITION))
                        .ignoringDisable(true));
    }

    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case STORE:
                roller_.setTargetDutyCycle(0.0);
                pivot_.setTargetPosition(CONSTANTS.PIVOT_STORE_POSITION);
                break;
            case DEPLOYING:
                roller_.setTargetDutyCycle(0.0);
                pivot_.setTargetPosition(CONSTANTS.PIVOT_DEPLOY_POSITION);
                break;
            case DEPLOYED:
                roller_.setTargetDutyCycle(0.0);
                pivot_.setTargetDutyCycle(0.0);
                break;
            case INTAKE:
                roller_.setTargetDutyCycle(0.5);
                pivot_.setTargetDutyCycle(0.0);
                break;
            case OUTTAKE:
                roller_.setTargetDutyCycle(-0.5);
                pivot_.setTargetDutyCycle(0.0);
                break;
            case MANUAL:
                roller_.setTargetDutyCycle(manual_roller_percent_);
                pivot_.setTargetPosition(manaul_pivot_position_);
                break;
            case TUNING:
                // No default behavior for tuning mode
                break;
            default:
            case IDLE:
                roller_.setTargetDutyCycle(0.0);
                pivot_.setTargetDutyCycle(0.0);
                break;
        }
    }

    @Override
    protected void handleStateTransition(IntakeStates wantedState) {
        // Handle MANUAL state transitions
        /*
        if (wantedState == IntakeStates.MANUAL) {
            system_state_ = IntakeStates.MANUAL;
        } else if (system_state_ == IntakeStates.MANUAL && wantedState != IntakeStates.MANUAL) {
            system_state_ = wantedState;
        } else if ((system_state_ == IntakeStates.STORE) && (wantedState == IntakeStates.INTAKE)) {
            system_state_ = IntakeStates.DEPLOYING;
        } else if ((system_state_ == IntakeStates.INTAKE) && (wantedState == IntakeStates.STORE)) {
            system_state_ = IntakeStates.STORE;
        } else if ((system_state_ == IntakeStates.STORE) && (wantedState == IntakeStates.DEPLOYED)) {
            system_state_ = IntakeStates.DEPLOYING;
        } else if ((system_state_ == IntakeStates.DEPLOYED) && (wantedState == IntakeStates.STORE)) {
            system_state_ = IntakeStates.STORE;
        } else if ((system_state_ == IntakeStates.DEPLOYED) && (wantedState == IntakeStates.INTAKE)) {
            system_state_ = IntakeStates.INTAKE;
        } else if ((system_state_ == IntakeStates.INTAKE) && (wantedState == IntakeStates.DEPLOYED)) {
            system_state_ = IntakeStates.DEPLOYED;
        } else if ((system_state_ == IntakeStates.IDLE) && (wantedState == IntakeStates.TUNING)) {
            system_state_ = IntakeStates.TUNING;
        } else if ((system_state_ == IntakeStates.INTAKE) && (wantedState == IntakeStates.OUTTAKE)) {
            system_state_ = IntakeStates.OUTTAKE;
        } else if ((system_state_ == IntakeStates.OUTTAKE) && (wantedState == IntakeStates.INTAKE)) {
            system_state_ = IntakeStates.INTAKE;
        }
        */
        if (system_state_ == IntakeStates.STORE && wantedState == IntakeStates.INTAKE) {
            system_state_ = IntakeStates.DEPLOYING;
        }
        else if (system_state_ == IntakeStates.STORE && wantedState == IntakeStates.OUTTAKE) {
            system_state_ = IntakeStates.DEPLOYING;
        }
        else if (system_state_ == IntakeStates.DEPLOYING && MathUtil.isNear(CONSTANTS.PIVOT_DEPLOY_POSITION, pivot_.getCurrentPosition(), CONSTANTS.PIVOT_TOLERANCE)) {
            system_state_ = IntakeStates.DEPLOYED;
        }
        else{
            system_state_ = wantedState;
        }
    }

    @Override
    public void reset() {
        system_state_ = IntakeStates.IDLE;
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(roller_, pivot_);
    }
}
