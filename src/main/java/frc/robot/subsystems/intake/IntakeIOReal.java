package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Fahrenheit;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.math.geometry.Rotation2d;

public class IntakeIOReal extends IntakeIO {
  private final TalonFX pivot_motor_;
  private final TalonFX roller_motor_;
  private final TimeOfFlight tof_;
  private final PositionVoltage pivotRequest;

  public IntakeIOReal(IntakeConstants constants) {
    super(constants);

    pivot_motor_ = new TalonFX(CONSTANTS.PIVOT_ID);
    roller_motor_ = new TalonFX(CONSTANTS.INTAKE_ID);
    tof_ = new TimeOfFlight(CONSTANTS.TIME_OF_FLIGHT_ID);
    pivotRequest = new PositionVoltage(0);

    pivot_motor_.getConfigurator().apply(CONSTANTS.PIVOT_GAINS);
    pivot_motor_
        .getConfigurator()
        .apply(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(CONSTANTS.STATOR_CURRENT_LIMIT)
                .withStatorCurrentLimitEnable(true));
    pivot_motor_
        .getConfigurator()
        .apply(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(CONSTANTS.PIVOT_MECH_RATIO));
  }

  @Override
  public void readInputs(double timestamp) {
    pivot_current_angle = Rotation2d.fromRotations(pivot_motor_.getPosition().getValueAsDouble());
    pivot_applied_voltage = pivot_motor_.getMotorVoltage().getValueAsDouble();
    pivot_current = pivot_motor_.getTorqueCurrent().getValueAsDouble();
    pivot_temp = pivot_motor_.getDeviceTemp().getValue().in(Fahrenheit);

    roller_applied_voltage = roller_motor_.getMotorVoltage().getValueAsDouble();
    roller_current = roller_motor_.getTorqueCurrent().getValueAsDouble();
    roller_temp = roller_motor_.getDeviceTemp().getValue().in(Fahrenheit);

    tof_dist = tof_.getRange();
  }

  @Override
  public void writeOutputs(double timestamp) {
    roller_motor_.set(roller_target_output);
    pivot_motor_.setControl(pivotRequest.withPosition(pivot_target_angle.getRotations()));
  }
}
