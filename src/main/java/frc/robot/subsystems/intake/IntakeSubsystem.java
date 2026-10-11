package frc.robot.subsystems.intake;

import com.marswars.logging.MwLog;
import com.marswars.mechanisms.ArmMech;
import com.marswars.mechanisms.RollerMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
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

    private RollerMech roller_0_;
    private RollerMech roller_1_;
    private ArmMech pivot_;
    private Timer timer = new Timer();
    private Debouncer homeDebouncer =
            new Debouncer(CONSTANTS.PIVOT_HOMING_WAIT_TIME, DebounceType.kFalling);
    private double pivot_current_kG_ = CONSTANTS.PIVOT_kG;
    private double pivot_current_test = 0;

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
        roller_0_ =
                new RollerMech(
                        getSubsystemKey(),
                        "Roller0",
                        List.of(CONSTANTS.ROLLER_MOTOR_CONFIG),
                        CONSTANTS.ROLLER_GEAR_RATIO);
        roller_1_ =
                new RollerMech(
                        getSubsystemKey(),
                        "Roller1",
                        List.of(CONSTANTS.ROLLER_FOLLOWER_MOTOR_CONFIG),
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
        MwLog.tunable(
                getSubsystemKey() + "Pivot/kG",
                CONSTANTS.PIVOT_kG,
                (newKg) -> pivot_current_kG_ = newKg);
        MwLog.tunable(
                getSubsystemKey() + "Pivot/test_squzze_current",
                pivot_current_test,
                (newTest) -> pivot_current_test = newTest);

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
        return Arrays.asList(roller_0_, roller_1_, pivot_);
    }

    // handleStateTransition
    //
    // Explicit (current state, request) -> next-state table, grouped by state. A request with
    // no matching arm is ignored. SQUEEZE_WAIT is a step state: nothing requests it.
    @Override
    protected void handleStateTransition(IntakeStates wanted) {
        // Run the debouncer every loop so its timing doesn't depend on the state
        boolean pivot_stalled =
                homeDebouncer.calculate(
                        pivot_.getLeaderSupplyCurrent() > CONSTANTS.PIVOT_HOMING_CURRENT_THRESHOLD);

        switch (system_state_) {
            case PIVOT_HOMING -> {
                // Drive the pivot until its current spikes, then take that as home
                if (pivot_stalled) {
                    pivot_.setCurrentPosition(CONSTANTS.PIVOT_HOME_POSITION);
                    setWantedState(IntakeStates.DEPLOYED);
                    system_state_ = IntakeStates.DEPLOYED;
                    return;
                }
                switch (wanted) {
                    case SQUEEZE -> squeeze();
                    case STORE, DEPLOYED, INTAKE, OUTTAKE, IDLE, SQUEEZE_HOLD, TUNING ->
                            system_state_ = wanted;
                    default -> {}
                }
            }
            case DEPLOYED -> {
                switch (wanted) {
                    // Settle before the squeeze engages
                    case SQUEEZE -> {
                        system_state_ = IntakeStates.SQUEEZE_WAIT;
                        timer.restart();
                    }
                    case STORE, INTAKE, OUTTAKE, IDLE, SQUEEZE_HOLD, PIVOT_HOMING, TUNING ->
                            system_state_ = wanted;
                    default -> {}
                }
            }
            case SQUEEZE_WAIT -> {
                // Ignores every request until the settle time is up
                if (timer.get() > CONSTANTS.SQUEEZE_WAIT_TIME) {
                    system_state_ = IntakeStates.SQUEEZE;
                }
            }
            case STORE, INTAKE, OUTTAKE, IDLE, SQUEEZE, SQUEEZE_HOLD, TUNING -> {
                switch (wanted) {
                    case SQUEEZE -> squeeze();
                    case STORE,
                                    DEPLOYED,
                                    INTAKE,
                                    OUTTAKE,
                                    IDLE,
                                    SQUEEZE_HOLD,
                                    PIVOT_HOMING,
                                    TUNING ->
                            system_state_ = wanted;
                    default -> {}
                }
            }
        }
    }

    /** Squeeze straight away (no settle), or hold once the pivot is past the squeeze limit. */
    private void squeeze() {
        if (pivot_.getCurrentPosition() > CONSTANTS.PIVOT_SQUEEZE_MAX_POSITION) {
            system_state_ = IntakeStates.SQUEEZE_HOLD;
            setWantedState(IntakeStates.SQUEEZE_HOLD);
        } else {
            system_state_ = IntakeStates.SQUEEZE;
        }
    }

    // updateLogic
    @Override
    public void updateLogic(double timestamp) {
        double feedforward = pivot_current_kG_ * Math.cos(pivot_.getCurrentPosition());

        switch (system_state_) {
            case STORE:
                roller_0_.setTargetDutyCycle(0.1);
                roller_1_.setTargetDutyCycle(0.1);
                pivot_.setTargetPosition(CONSTANTS.PIVOT_STORE_POSITION);
                break;
            case DEPLOYED:
                roller_0_.setTargetDutyCycle(0.0);
                roller_1_.setTargetDutyCycle(0.0);
                pivot_.setTargetPosition(CONSTANTS.PIVOT_DEPLOY_POSITION);
                break;
            case INTAKE:
                roller_0_.setTargetDutyCycle(CONSTANTS.INTAKE_DUTY_CYCLE);
                roller_1_.setTargetDutyCycle(CONSTANTS.INTAKE_DUTY_CYCLE);
                pivot_.setTargetPosition(CONSTANTS.PIVOT_DEPLOY_POSITION);
                break;
            case OUTTAKE:
                roller_0_.setTargetDutyCycle(-CONSTANTS.INTAKE_DUTY_CYCLE);
                roller_1_.setTargetDutyCycle(-CONSTANTS.INTAKE_DUTY_CYCLE);
                pivot_.setTargetDutyCycle(0.0);
                break;
            case PIVOT_HOMING:
                // Drive the pivot slowly outward until the motor current indicates a
                // stall/contact
                roller_0_.setTargetDutyCycle(0.0);
                roller_1_.setTargetDutyCycle(0.0);
                pivot_.setTargetDutyCycle(CONSTANTS.PIVOT_HOMING_DUTY_CYCLE);
                break;
            case SQUEEZE:
                timer.stop();
                roller_0_.setTargetDutyCycle(0.0);
                roller_1_.setTargetDutyCycle(0.0);
                pivot_.setTargetCurrentWithFF(CONSTANTS.PIVOT_SQUEEZE_CURRENT, feedforward);
                break;
            case SQUEEZE_WAIT:
                roller_0_.setTargetDutyCycle(0.0);
                roller_1_.setTargetDutyCycle(0.0);
                pivot_.setTargetDutyCycle(0);
                break;
            case SQUEEZE_HOLD:
                pivot_.setTargetPosition(CONSTANTS.PIVOT_SQUEEZE_HOLD_POSITION);
                roller_0_.setTargetDutyCycle(0.15);
                roller_1_.setTargetDutyCycle(0.15);
                break;
            case TUNING:
                break;
            default:
            case IDLE:
                roller_0_.setTargetDutyCycle(0.0);
                roller_1_.setTargetDutyCycle(0.0);
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
