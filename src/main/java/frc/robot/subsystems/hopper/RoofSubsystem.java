package frc.robot.subsystems.hopper;

import com.marswars.mechanisms.ElevatorMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.hopper.RoofConstants.RoofStates;
import java.util.Arrays;
import java.util.List;

public class RoofSubsystem extends MwSubsystem<RoofStates, RoofConstants> {
    private static RoofSubsystem instance_ = null;

    // private ELEVATORMECH elevator
    private ElevatorMech elevator_;

    private Mechanism2d mech = new Mechanism2d(3,3);

    private MechanismRoot2d root = mech.getRoot("climber",2,0);

    private MechanismLigament2d m_elevator;

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
        m_elevator = root.append(new MechanismLigament2d("elevator", 0.548, 90));
        SmartDashboard.putData("Mech2d", mech);
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
    // @Override
    // public void handleStateTransition(RoofStates wanted) {
    // }

    // updateLogic
    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case UP:
                elevator_.setTargetPosition(CONSTANTS.ELEVATOR_UP_POSITION_METERS);
                break;
            case DOWN:
                elevator_.setTargetPosition(CONSTANTS.ELEVATOR_DOWN_POSITION_METERS);
                break;
            case CLIMB:
                break;
            case TUNING:
                break;
        }
            m_elevator.setLength(elevator_.getCurrentPosition()+0.548);
    }

    // =============================================================================
    // PUBLIC HELPER METHODS
    // =============================================================================


    public boolean isDown(){
        return elevator_.getCurrentPosition() < (CONSTANTS.ELEVATOR_DOWN_POSITION_METERS + 0.02);
    }
}
