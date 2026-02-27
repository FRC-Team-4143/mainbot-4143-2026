package frc.robot.subsystems.intake;

import com.marswars.mechanisms.ArmMech;
import com.marswars.mechanisms.RollerMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import java.util.Arrays;
import java.util.List;

public class IntakeSubsystem extends MwSubsystem<IntakeStates, IntakeConstants> {
    private static IntakeSubsystem instance_ = null;

    private RollerMech roller_;
    private ArmMech pivot_;
    private final Timer intake_timer_ = new Timer();

    // getInstance
    public static IntakeSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new IntakeSubsystem();
        }
        return instance_;
    }

    // Constructor
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

        SmartDashboard.putData(
                "Home Pivot",
                Commands.runOnce(() -> pivot_.setCurrentPosition(CONSTANTS.PIVOT_HOME_POSITION))
                        .ignoringDisable(true));
    }

    // reset
    @Override
    public void reset() {
        system_state_ = IntakeStates.IDLE;
    }

    // getIos
    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(roller_, pivot_);
    }

    // handleStateTransition
    @Override
    protected void handleStateTransition(IntakeStates wantedState) {
        if (system_state_ == IntakeStates.STORE && wantedState == IntakeStates.INTAKE) {
            system_state_ = IntakeStates.DEPLOYING;
        } else if (system_state_ == IntakeStates.STORE && wantedState == IntakeStates.OUTTAKE) {
            system_state_ = IntakeStates.DEPLOYING;
        } else if (system_state_ == IntakeStates.DEPLOYING
                && MathUtil.isNear(
                        CONSTANTS.PIVOT_DEPLOY_POSITION,
                        pivot_.getCurrentPosition(),
                        CONSTANTS.PIVOT_TOLERANCE)) {
            system_state_ = IntakeStates.DEPLOYED;
        } else if (wantedState == IntakeStates.SHOOTING
                && (system_state_ != IntakeStates.SHOOTING
                        && system_state_ != IntakeStates.RACKING)) {
            intake_timer_.reset();
            intake_timer_.start();
            system_state_ = IntakeStates.SHOOTING;
        } else if (wantedState == IntakeStates.SHOOTING
                && system_state_ == IntakeStates.SHOOTING
                && intake_timer_.hasElapsed(CONSTANTS.SHOOTING_CYCLE_TIME)) {
            intake_timer_.reset();
            system_state_ = IntakeStates.RACKING;
        } else if (wantedState == IntakeStates.SHOOTING
                && system_state_ == IntakeStates.RACKING
                && intake_timer_.hasElapsed(CONSTANTS.SHOOTING_CYCLE_TIME)) {
            intake_timer_.reset();
            system_state_ = IntakeStates.SHOOTING;
        } else if (wantedState == IntakeStates.SHOOTING
                && !intake_timer_.hasElapsed(CONSTANTS.SHOOTING_CYCLE_TIME)) {
            // do nothing while time is elapsing
        } else {
            if (intake_timer_.isRunning()) intake_timer_.stop();
            system_state_ = wantedState;
        }
    }

    // updateLogic
    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case SHOOTING:
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
                roller_.setTargetDutyCycle(CONSTANTS.INTAKE_DUTY_CYCLE);
                pivot_.setTargetDutyCycle(0.0);
                break;
            case OUTTAKE:
                roller_.setTargetDutyCycle(-CONSTANTS.INTAKE_DUTY_CYCLE);
                pivot_.setTargetDutyCycle(0.0);
                break;
            case RACKING:
                roller_.setTargetDutyCycle(0.0);
                pivot_.setTargetPosition(CONSTANTS.PIVOT_RACKING_POSITION);
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
}
