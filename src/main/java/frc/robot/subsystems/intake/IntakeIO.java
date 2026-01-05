package frc.robot.subsystems.intake;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.mw_lib.subsystem.SubsystemIO;

public abstract class IntakeIO extends SubsystemIO<IntakeConstants> {
  public IntakeIO(IntakeConstants constants) {
    super(constants);
  }

  /** Current angle of the pivot */
  public Rotation2d pivot_current_angle = Rotation2d.kZero;

  /** Applied voltage to the pivot */
  public double pivot_applied_voltage = 0.0;

  /** Current in amperes for the pivot */
  public double pivot_current = 0.0;

  /** Temperature in degrees Fahrenheit for the pivot */
  public double pivot_temp = 0.0;

  /** Target angle of the pivot */
  public Rotation2d pivot_target_angle = pivot_current_angle;

  /** Applied voltage to the roller */
  public double roller_applied_voltage = 0.0;

  /** Current in amperes for the roller */
  public double roller_current = 0.0;

  /** Temperature in degrees Fahrenheit for the roller */
  public double roller_temp = 0.0;

  /** Target output for the roller */
  public double roller_target_output = 0.0;

  /** Time-of-flight sensor distance */
  public double tof_dist = 0.0;

  /** Logs data to DogLog.  */
  @Override
  public void logData() {
    DogLog.log(getSubsystemKey() + "Pivot/Position/Current", pivot_current_angle);
    DogLog.log(getSubsystemKey() + "Pivot/Position/Target", pivot_target_angle);
    DogLog.log(getSubsystemKey() + "Pivot/AppliedVoltage", pivot_applied_voltage);
    DogLog.log(getSubsystemKey() + "Pivot/Current", pivot_current);
    DogLog.log(getSubsystemKey() + "Pivot/Temp", pivot_temp);

    DogLog.log(getSubsystemKey() + "Roller/AppliedVoltage", roller_applied_voltage);
    DogLog.log(getSubsystemKey() + "Roller/Current", roller_current);
    DogLog.log(getSubsystemKey() + "Roller/Temp", roller_temp);
    DogLog.log(getSubsystemKey() + "Roller/TargetOutput", roller_target_output);

    DogLog.log(getSubsystemKey() + "TOFDistance", tof_dist);
  }
}