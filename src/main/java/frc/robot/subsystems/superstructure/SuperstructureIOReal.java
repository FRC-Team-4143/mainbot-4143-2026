package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Fahrenheit;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;

public class SuperstructureIOReal extends SuperstructureIO {

  private final TalonFX leader_motor_;
  private final TalonFX follower_motor_;
  private final TalonFX arm_motor_;
  private final CANcoder arm_encoder_;

  private final MotionMagicVoltage elevator_control_request_ = new MotionMagicVoltage(0.0);
  private final MotionMagicVoltage arm_control_request_ = new MotionMagicVoltage(0.0);

  public SuperstructureIOReal(SuperstructureConstants constants) {
    super(constants);

    // Initialize the TalonFX motors
    leader_motor_ = new TalonFX(CONSTANTS.ELEVATOR_LEADER_ID);
    follower_motor_ = new TalonFX(CONSTANTS.ELEVATOR_FOLLOWER_ID);
    arm_motor_ = new TalonFX(CONSTANTS.ARM_MOTOR_ID);
    arm_encoder_ = new CANcoder(CONSTANTS.ARM_ENCODER_ID);

    // Apply the configurations to the motors
    leader_motor_.getConfigurator().apply(CONSTANTS.ELEVATOR_LEADER_CONFIG);
    follower_motor_.getConfigurator().apply(CONSTANTS.ELEVATOR_FOLLOWER_CONFIG);
    arm_motor_.getConfigurator().apply(CONSTANTS.ARM_MOTOR_CONFIG);

    // Apply the configuration to the arm encoder
    CANcoderConfiguration config = new CANcoderConfiguration();
    arm_encoder_.getConfigurator().refresh(config);
    config.MagnetSensor.SensorDirection =
        CONSTANTS.ARM_ENCODER_CONFIG.MagnetSensor.SensorDirection;
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        CONSTANTS.ARM_ENCODER_CONFIG.MagnetSensor.AbsoluteSensorDiscontinuityPoint;
    arm_encoder_.getConfigurator().apply(config);

    arm_motor_.setPosition(arm_encoder_.getAbsolutePosition().getValueAsDouble());
  }

  /** Updates the set of loggable inputs. */
  public void readInputs(double timestamp) {
    // Update leader motor inputs
    current_leader_position = leader_motor_.getPosition().getValueAsDouble();
    current_leader_velocity = leader_motor_.getVelocity().getValueAsDouble();
    leader_applied_voltage = leader_motor_.getMotorVoltage().getValueAsDouble();
    leader_current = leader_motor_.getTorqueCurrent().getValueAsDouble();
    leader_temp = leader_motor_.getDeviceTemp().getValue().in(Fahrenheit);

    // Update follower motor inputs
    current_follower_position = follower_motor_.getPosition().getValueAsDouble();
    current_follower_velocity = follower_motor_.getVelocity().getValueAsDouble();
    follower_applied_voltage = follower_motor_.getMotorVoltage().getValueAsDouble();
    follower_current = follower_motor_.getTorqueCurrent().getValueAsDouble();
    follower_temp = follower_motor_.getDeviceTemp().getValue().in(Fahrenheit);

    // Update additional inputs
    current_elevator_position = current_leader_position * CONSTANTS.ROTATIONS_TO_TRANSLATION;
    current_elevator_velocity = current_leader_velocity * CONSTANTS.ROTATIONS_TO_TRANSLATION;

    // Update arm motor inputs
    current_arm_position = Units.rotationsToRadians(arm_motor_.getPosition().getValueAsDouble());
    current_arm_velocity = Units.rotationsToRadians(arm_motor_.getVelocity().getValueAsDouble());
    arm_applied_voltage = arm_motor_.getMotorVoltage().getValueAsDouble();
    arm_current = arm_motor_.getTorqueCurrent().getValueAsDouble();
    arm_temp = arm_motor_.getDeviceTemp().getValue().in(Fahrenheit);
    arm_encoder_position = arm_encoder_.getAbsolutePosition().getValueAsDouble();
  }

  /** Writes the desired outputs to the motors. */
  public void writeOutputs(double timestamp) {
    // Set the control request for the leader motor
    elevator_control_request_.withPosition(
        target_elevator_position / CONSTANTS.ROTATIONS_TO_TRANSLATION);
    leader_motor_.setControl(elevator_control_request_);

    // Set the follower motor to follow the leader motor
    follower_motor_.setControl(new StrictFollower(leader_motor_.getDeviceID()));

    // Set the arm motor control request
    arm_motor_.setControl(
        arm_control_request_.withPosition(Units.radiansToRotations(target_arm_position)));
  }

  /** Zeroes the elevator position. */
  public void tarePosition() {
    leader_motor_.setPosition(0.0);
    follower_motor_.setPosition(0.0);
    arm_motor_.setPosition(0.0);
    arm_encoder_.setPosition(0.0);
  }

  /**
   * Updates the gains for the elevator.
   *
   * @param gains The new gains to apply.
   */
  public void updateElevatorGains(Slot0Configs gains) {
    DataLogManager.log("Updating Elevator Gains: " + gains.toString());
    leader_motor_.getConfigurator().apply(gains);
    follower_motor_.getConfigurator().apply(gains);
  }

  /**
   * Updates the gains for the arm.
   *
   * @param gains The new gains to apply.
   */
  public void updateArmGains(Slot0Configs gains) {
    DataLogManager.log("Updating Arm Gains: " + gains.toString());
    arm_motor_.getConfigurator().apply(gains);
  }
}
