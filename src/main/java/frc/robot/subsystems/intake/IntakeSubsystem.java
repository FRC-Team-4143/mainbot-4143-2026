package frc.robot.subsystems.intake;

import com.marswars.mechanisms.ArmMech;
import com.marswars.mechanisms.RollerMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
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
    private Timer timer = new Timer();
    private Debouncer homeDebouncer = new Debouncer(CONSTANTS.PIVOT_HOMING_WAIT_TIME, DebounceType.kFalling);
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
                        List.of(
                                CONSTANTS.ROLLER_MOTOR_CONFIG,
                                CONSTANTS.ROLLER_FOLLOWER_MOTOR_CONFIG),
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

        // SmartDashboard button to run a pivot zeroing/homing sequence
        SmartDashboard.putData(
                "Auto Home Pivot",
                Commands.runOnce(() -> setWantedState(IntakeStates.PIVOT_HOMING)));
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
        // If pivot current spikes while in homing state, set current position as home
        if (homeDebouncer.calculate(pivot_.getLeaderCurrent() > CONSTANTS.PIVOT_HOMING_CURRENT_THRESHOLD)
                && system_state_ == IntakeStates.PIVOT_HOMING) {
            pivot_.setCurrentPosition(CONSTANTS.PIVOT_HOME_POSITION);
            setWantedState(IntakeStates.DEPLOYED);
            system_state_ = IntakeStates.DEPLOYED;
        } else if (!MathUtil.isNear(
                        CONSTANTS.PIVOT_DEPLOY_POSITION,
                        pivot_.getCurrentPosition(),
                        CONSTANTS.DEPLOY_PIVOT_TOLERANCE)
                && (wantedState == IntakeStates.INTAKE
                        || wantedState == IntakeStates.OUTTAKE
                        || wantedState == IntakeStates.DEPLOYED)) {
            system_state_ = IntakeStates.DEPLOYING;
        } else if (system_state_ == IntakeStates.DEPLOYING) {
            if (MathUtil.isNear(
                    CONSTANTS.PIVOT_DEPLOY_POSITION,
                    pivot_.getCurrentPosition(),
                    CONSTANTS.DEPLOY_PIVOT_TOLERANCE)) {
                system_state_ = IntakeStates.DEPLOYED;
            }
        }else if(wantedState == IntakeStates.SQUEEZE && (system_state_ == IntakeStates.DEPLOYED || system_state_ == IntakeStates.DEPLOYING)){
            system_state_ = IntakeStates.SQUEEZEWAIT;
            timer.reset();
            timer.start();
        }else if(system_state_ == IntakeStates.SQUEEZEWAIT){
            if(timer.get() > CONSTANTS.SQUEEZEWAITTIME){
                system_state_ = IntakeStates.SQUEEZE;
            }
        }else if (wantedState == IntakeStates.SQUEEZE
                && pivot_.getCurrentPosition() > CONSTANTS.PIVOT_SQUEEZE_MAX_POSITION) {
            system_state_ = IntakeStates.SQUEEZE_HOLD;
        } else {
            system_state_ = wantedState;
        }
    }

    // updateLogic
    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case STORE:
                roller_.setTargetDutyCycle(0.1);
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
            case PIVOT_HOMING:
                // Drive the pivot slowly outward until the motor current indicates a
                // stall/contact
                roller_.setTargetDutyCycle(0.0);
                pivot_.setTargetDutyCycle(CONSTANTS.PIVOT_HOMING_DUTY_CYCLE);
                break;
            case SQUEEZE:
                timer.stop();
                roller_.setTargetDutyCycle(0.0);
                pivot_.setTargetCurrent(CONSTANTS.PIVOT_SQUEEZE_CURRENT);
                break;
            case SQUEEZEWAIT:
                roller_.setTargetDutyCycle(0.0);
                break;
            case SQUEEZE_HOLD:
                pivot_.setTargetPosition(CONSTANTS.PIVOT_SQUEEZE_HOLD_POSITION);
                roller_.setTargetDutyCycle(0.15);
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

    // =============================================================================
    // PUBLIC HELPER METHODS
    // =============================================================================

    /**
     * Returns the current angle of the pivot joint in radians. (Used for testing and visualization
     * purposes)
     *
     * @return the current angle of the pivot joint in radians
     */
    public double getPivotAngle() {
        return pivot_.getCurrentPosition();
    }
}
