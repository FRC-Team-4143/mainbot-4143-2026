package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Meter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.marswars.subsystem.MwConstants;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.FxMotorConfig.FxMotorType;

import edu.wpi.first.units.Units;

// IMPORTANT
// Change ALL placeholders BEFORE branch merge
// Delete all Place Holders

public class ClimberConstants extends MwConstants {
    public enum ClimberStates {
        STORED,
        EXTENDING,
        DEPLOYED,
        TELEOP_ENGAGE,
        TELEOP_CLIMB_UP,
        IDLE_FINALE,
        TELEOP_CLIMB_DOWN,
        AUTO_ENGAGE,
        AUTO_CLIMB_UP,
        IDLE_AUTO,
        AUTO_CLIMB_DOWN,
        DISENGAGE,
        STORING
    }

    //Extender configs
    public final int  EXTENDERMOTER_ID = 1; //Place holder
    public final boolean EXTENDERMOTER_INVERTED = false; //Place Holder
    public final double EXTENDER_GEAR_RATIO = 1.0; //place holder
    public final FxMotorConfig EXTENDER_MOTOR_CONFIG = new FxMotorConfig();

    //Arm configs
    public final int  ARM_MOTER_ID = 1; //Place holder
    public final boolean ARM_MOTER_INVERTED = false; //Place Holder
    public final double ARM_GEAR_RATIO = 1.0; //place holder
    public final FxMotorConfig ARM_MOTOR_CONFIG = new FxMotorConfig();
    public final double ARM_LENGTH = 1.0; // Meters, Place Holder
    public final double ARM_MASS = 1.0; // KG, Place Holder
    public final double ARM_MIN_ANGLE = 1.0; // Radians, Place Holder
    public final double ARM_MAX_ANGLE = 2.0; // Radians, Place Holder

    public ClimberConstants() {
        // Configure Indexer Motor
        EXTENDER_MOTOR_CONFIG.can_id = EXTENDERMOTER_ID;
        EXTENDER_MOTOR_CONFIG.motor_type = FxMotorType.FALCON500; //Place Holder
        EXTENDER_MOTOR_CONFIG.canbus_name = "CANivore"; // Place Holder
        EXTENDER_MOTOR_CONFIG.config = new TalonFXConfiguration(); // Place Holder

        ARM_MOTOR_CONFIG.can_id = ARM_MOTER_ID; // Place Holder
        ARM_MOTOR_CONFIG.motor_type = FxMotorType.X60; // Place Holder
        ARM_MOTOR_CONFIG.canbus_name = "rio"; // Place Holder
        ARM_MOTOR_CONFIG.config = new TalonFXConfiguration(); // Place Holder
    }
}
