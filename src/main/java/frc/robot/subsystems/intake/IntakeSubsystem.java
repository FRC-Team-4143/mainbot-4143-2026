package frc.robot.subsystems.intake;

import com.marswars.mechanisms.ArmMech;
import com.marswars.mechanisms.RollerMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import java.util.Arrays;
import java.util.List;

public class IntakeSubsystem extends MwSubsystem<IntakeStates, IntakeConstants> {
    private static IntakeSubsystem instance_ = null;

    private RollerMech roller_;
    private ArmMech pivot_;
    private int counter;

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
        if (!MathUtil.isNear(
                        CONSTANTS.PIVOT_DEPLOY_POSITION,
                        pivot_.getCurrentPosition(),
                        CONSTANTS.DEPLOY_PIVOT_TOLERANCE)
                && wantedState == IntakeStates.INTAKE) {
            system_state_ = IntakeStates.DEPLOYING;
        } else if (system_state_ == IntakeStates.STORE && wantedState == IntakeStates.OUTTAKE) {
            system_state_ = IntakeStates.DEPLOYING;
        } else if (system_state_ == IntakeStates.DEPLOYING) {
            if (MathUtil.isNear(
                    CONSTANTS.PIVOT_DEPLOY_POSITION,
                    pivot_.getCurrentPosition(),
                    CONSTANTS.DEPLOY_PIVOT_TOLERANCE)) {
                system_state_ = IntakeStates.DEPLOYED;
            }
        } else {
            system_state_ = wantedState;
            counter = 0;
        }
    }

    // updateLogic
    @Override
    public void updateLogic(double timestamp) {
        counter++;
        switch (system_state_) {
            case STORE:
                if(counter > 25)
                    roller_.setTargetDutyCycle(0.0);
                else
                    roller_.setTargetDutyCycle(CONSTANTS.INTAKE_DUTY_CYCLE);
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
