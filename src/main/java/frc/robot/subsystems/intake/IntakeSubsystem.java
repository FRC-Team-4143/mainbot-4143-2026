package frc.robot.subsystems.intake;

import com.marswars.mechanisms.ArmMech;
import com.marswars.mechanisms.RollerMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import dev.doglog.DogLog;
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
    private double manual_roller_duty_cycle_ = CONSTANTS.INTAKE_DUTY_CYCLE;
    public final Timer racking_timer_ = new Timer();

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
                Commands.runOnce(() -> setWantedState(IntakeStates.PIVOT_HOMING))
                        .ignoringDisable(true));

        DogLog.tunable(
                getSubsystemKey() + "/Roller/TargetDutyCycle",
                manual_roller_duty_cycle_,
                (v) -> manual_roller_duty_cycle_ = v);
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
        // If pivot current spikes while in homing state, set the current position as
        // home
        if (pivot_.getLeaderCurrent() > CONSTANTS.PIVOT_HOMING_CURRENT_THRESHOLD
                && system_state_ == IntakeStates.PIVOT_HOMING) {
            pivot_.setCurrentPosition(CONSTANTS.PIVOT_HOME_POSITION);
            setWantedState(IntakeStates.DEPLOYED);
            system_state_ = IntakeStates.DEPLOYED;
        } else if (wantedState == IntakeStates.RACKING
                && !(system_state_ == IntakeStates.RACKED_IN
                        || system_state_ == IntakeStates.RACKED_OUT)) {
            racking_timer_.reset();
            racking_timer_.start();
            system_state_ = IntakeStates.RACKED_IN;
        } else if (wantedState == IntakeStates.RACKING
                && (system_state_ == IntakeStates.RACKED_IN
                        || system_state_ == IntakeStates.RACKED_OUT)
                && racking_timer_.hasElapsed(CONSTANTS.RACKING_CYCLE_TIME)) {
            system_state_ =
                    (system_state_ == IntakeStates.RACKED_IN)
                            ? IntakeStates.RACKED_OUT
                            : IntakeStates.RACKED_IN;
            racking_timer_.reset();
            racking_timer_.start();
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
        } else {
            if (wantedState == IntakeStates.RACKING) {
                return; // Don't allow directly setting RACKING state, since it is a transient state
                // for cycling between RACKED_IN and RACKED_OUT
            } else system_state_ = wantedState;
        }
    }

    // updateLogic
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
                roller_.setTargetDutyCycle(manual_roller_duty_cycle_);
                pivot_.setTargetDutyCycle(0.0);
                break;
            case OUTTAKE:
                roller_.setTargetDutyCycle(-manual_roller_duty_cycle_);
                pivot_.setTargetDutyCycle(0.0);
                break;
            case PIVOT_HOMING:
                // Drive the pivot slowly outward until the motor current indicates a
                // stall/contact
                roller_.setTargetDutyCycle(0.0);
                pivot_.setTargetDutyCycle(CONSTANTS.PIVOT_HOMING_DUTY_CYCLE);
                break;
            case RACKING:
                break;
            case RACKED_IN:
                roller_.setTargetDutyCycle(0.0);
                pivot_.setTargetPosition(CONSTANTS.PIVOT_RACKING_POSITION);
                break;
            case RACKED_OUT:
                roller_.setTargetDutyCycle(0.0);
                pivot_.setTargetPosition(CONSTANTS.PIVOT_DEPLOY_POSITION);
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
