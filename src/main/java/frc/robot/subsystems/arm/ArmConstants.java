package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import frc.mw_lib.mechanisms.FxMotorConfig;
import frc.mw_lib.subsystem.MwConstants;

public class ArmConstants extends MwConstants {

    public enum ArmStates {
        IDLE,
        MOVE_TO_POSITION,
        HOLD,
        TUNING
    }

    public FxMotorConfig leader_motor_config = new FxMotorConfig();

    public final double arm_gear_ratio;
    public final double arm_length;
    public final double arm_mass;
    public final double arm_min_angle;
    public final double arm_max_angle;

    public ArmConstants() {
        leader_motor_config.loadFromConfig(getSystemName(), "arm_mech", "leader_motor");

        arm_gear_ratio = getDoubleConstant("arm_mech", "gear_ratio");
        arm_length = getDoubleConstant("arm_mech", "length");
        arm_mass = getDoubleConstant("arm_mech", "mass");
        arm_min_angle = Units.degreesToRadians(getDoubleConstant("arm_mech", "min_angle"));
        arm_max_angle = Units.degreesToRadians(getDoubleConstant("arm_mech", "max_angle"));
    }

}
