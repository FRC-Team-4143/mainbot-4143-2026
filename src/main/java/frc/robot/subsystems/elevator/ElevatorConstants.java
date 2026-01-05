package frc.robot.subsystems.elevator;

import frc.mw_lib.mechanisms.FxMotorConfig;
import frc.mw_lib.subsystem.MwConstants;

public class ElevatorConstants extends MwConstants {

    public enum ElevatorStates {
        IDLE,
        MOVE_TO_POSITION,
        HOLD,
        TUNING
    }

    public FxMotorConfig leader_motor_config = new FxMotorConfig();

    public final double elevator_gear_ratio;
    public final double elevator_drum_radius;
    public final double elevator_carrige_mass;
    public final double elevator_rigging_ratio;
    public final double elevator_max_extension;

    public ElevatorConstants() {
        leader_motor_config.loadFromConfig(getSystemName(), "elevator_mech", "leader_motor");

        elevator_gear_ratio = getDoubleConstant("elevator_mech", "gear_ratio");
        elevator_drum_radius = getDoubleConstant("elevator_mech", "drum_radius");
        elevator_carrige_mass = getDoubleConstant("elevator_mech", "carrige_mass");
        elevator_rigging_ratio = getDoubleConstant("elevator_mech", "rigging_ratio");
        elevator_max_extension = getDoubleConstant("elevator_mech", "max_extension");
    }
    

}
