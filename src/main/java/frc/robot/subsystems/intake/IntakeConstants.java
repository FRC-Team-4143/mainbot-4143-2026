package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.mw_lib.subsystem.MwConstants;

public class IntakeConstants extends MwConstants {

  public enum IntakeStates {
    DEPLOY,
    PURGE,
    CLIMB_STAGE,
    PICK_UP,
    IDLE
  }

  public IntakeConstants() {
  }

  // General Intake Constants
  public final double INTAKE_OFF_SET_Y = getDoubleConstant("offset_y");
  public final double STATOR_CURRENT_LIMIT = getDoubleConstant("stator_current_limit");

  // TOF Constants
  public final int TIME_OF_FLIGHT_ID = getIntConstant("tof", "id");
  public final double TOF_CORAL_DISTANCE = getDoubleConstant("tof", "coral_distance");

  // Intake Motor Constants
  public final int INTAKE_ID = getIntConstant("intake", "id");
  public final double INTAKE_IN_SPEED = getDoubleConstant("intake", "in_speed");
  public final double INTAKE_OUT_SPEED = getDoubleConstant("intake", "out_speed");

  // Pivot Motor Constants
  public final int PIVOT_ID = getIntConstant("pivot", "id");
  public final double PIVOT_MECH_RATIO = getDoubleConstant("pivot", "mech_ratio");
  public final Slot0Configs PIVOT_GAINS = new Slot0Configs().withKP(getDoubleConstant("pivot", "kp"));

  public final Rotation2d PIVOT_OFFSET = Rotation2d.fromDegrees(35);
  public final Rotation2d PIVOT_DEPLOYED_ANGLE = Rotation2d.fromDegrees(-35);
  public final Rotation2d PIVOT_STATION_ANGLE = Rotation2d.fromDegrees(59);
  public final Rotation2d PIVOT_CLIMB_ANGLE = Rotation2d.fromDegrees(0);

}
