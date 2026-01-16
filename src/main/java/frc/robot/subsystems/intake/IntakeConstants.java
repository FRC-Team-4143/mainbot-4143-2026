package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import frc.mw_lib.subsystem.MwConstants;
import frc.mw_lib.util.FxMotorConfig;
import frc.mw_lib.util.FxMotorConfig.FxMotorType;

public class IntakeConstants extends MwConstants  {
    
    public enum IntakeStates {
        IDLE,
        DEPLOY,
        PICKUP,
        RETRACT;
    }
    //Intake Constants
    public final int INTAKE_ID = 1; //ID here;
    public final boolean INTAKE_INVERTED = false; //true or false;
    public final double INTAKE_GEAR_RATIO = 1; //gear ratio here ex 1.0;
    public final double INTAKE_WHEEL_RADIUS_METERS = 1; // must be in meters ex:Units.inchesToMeters(3);
    public final double INTAKE_WHEEL_MASS_KG = 1; // kg, approximate;
    public final double INTAKE_WHEEL_INERTIA = 0.5 * INTAKE_WHEEL_MASS_KG * Math.pow(INTAKE_WHEEL_RADIUS_METERS, 2.0); // kg m^2, approximate
    public final FxMotorConfig INTAKE_MOTOR_CONFIG = new FxMotorConfig();
    
    //Arm Constants
    public final int ARMID = 1; //ID here;
    public final double ARM_LENGTH = 1; // in meters
    public final double ARM_MASS = 1; // in kg
    public final double ARM_MIN_ANGLE = 0; //in radians
    public final double ARM_MAX_ANGLE = 0; // in radians
    public final boolean ARM_INVERTED = false; //true or false;
    public final double ARM_GEAR_RATIO = 1; //gear ratio here ex 1.0;



    // Control Setpoints
    public final double INTAKE_DUTY_CYCLE = 1; //power here; // 50% power for shooting
    public final double ARM_DUTY_CYCLE = 1;//power here;


    public IntakeConstants() {
        INTAKE_MOTOR_CONFIG.can_id= INTAKE_ID;
        INTAKE_MOTOR_CONFIG.motor_type = FxMotorType.X60;
        INTAKE_MOTOR_CONFIG.canbus_name = "rio";
        INTAKE_MOTOR_CONFIG.config = new TalonFXConfiguration();
    }
}

