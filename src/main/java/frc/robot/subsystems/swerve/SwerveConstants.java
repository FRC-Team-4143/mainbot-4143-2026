package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.mw_lib.subsystem.MwConstants;
import frc.mw_lib.swerve_lib.module.ModuleType;

public class SwerveConstants extends MwConstants {
  // Current system states for the swerve drive
  enum SwerveStates {
    FIELD_CENTRIC,
    ROBOT_CENTRIC,
    CHOREO_PATH,
    ROTATION_LOCK,
    TRACTOR_BEAM,
    IDLE
  }

  // CAN bus name and CAN ID for the pigeon2
  public final int PIGEON2_ID = 0;
  public final String PIGEON2_CANBUS_NAME = "CANivore";

  // CAN bus names for each of the swerve modules
  public final String MODULE_CANBUS_NAME = "CANivore";

  public final double DRIVE_INERTIA = 0.025; // kg*m^2, inertia of the drive motor
  public final double STEER_INERTIA = 0.004; // kg*m^2, inertia of the steer motor
  public final double DRIVE_FRICTION_VOLTAGE = 0.2;
  public final double STEER_FRICTION_VOLTAGE = 0.2;

  public final double ROBOT_MASS_KG = Units.lbsToKilograms(140.0); // Mass of the robot in pounds
  public final double BUMPER_WIDTH_METERS = Units.inchesToMeters(34.75); // Width of the bumpers in meters (y
  // axis : left -> right)
  public final double BUMPER_LENGTH_METERS = Units.inchesToMeters(34.75); // Length of the bumpers in meters (x
  // axis : front -> back)
  public final double WHEEL_COF = 1.916; // Coefficient of friction for the wheels VEX Grip V2

  // Forward Reference rotation constants
  public enum OperatorPerspective {
    BLUE_ALLIANCE(Rotation2d.fromDegrees(0.0)),
    RED_ALLIANCE(Rotation2d.fromDegrees(180.0));

    private OperatorPerspective(Rotation2d heading) {
      this.heading = heading;
    }

    public final Rotation2d heading;
  }

  // The steer motor uses MotionMagicVoltage control
  private final Slot0Configs STEER_GAINS = new Slot0Configs().withKP(80).withKI(0).withKD(0).withKS(0).withKV(0)
      .withKA(0);
  // When using closed-loop control, the drive motor uses VelocityVoltage
  private final Slot0Configs DRIVE_GAINS = new Slot0Configs().withKP(0.5).withKI(0).withKD(0).withKS(0.18)
      .withKV(0.117).withKA(0);

  // Control Constants for the swerve modules
  public final double SLIP_CURRENT_AMPS = getDoubleConstant("com", "slip_current");
  public final double SPEED_AT_12V_MPS = getDoubleConstant("com", "speed_at_12v");
  public final double COUPLE_RATIO = getDoubleConstant("com", "couple_ratio");
  public final double WHEEL_RADIUS_METERS = Units.inchesToMeters(getDoubleConstant("com", "wheel_radius_inches"));
  public final double MAX_TRANSLATION_RATE = getDoubleConstant("com", "max_translation_rate");
  public final double MAX_ANGULAR_RATE = getDoubleConstant("com", "max_angular_rate");

  private final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> CONSTANT_CREATOR = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
      // .withDriveMotorGearRatio(kDriveGearRatio) Unique to Module Type
      // .withSteerMotorGearRatio(kSteerGearRatio) Unique to Module Type
      .withCouplingGearRatio(COUPLE_RATIO)
      .withWheelRadius(WHEEL_RADIUS_METERS)
      .withSteerMotorGains(STEER_GAINS)
      .withDriveMotorGains(DRIVE_GAINS)
      .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
      .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
      .withSlipCurrent(SLIP_CURRENT_AMPS)
      .withSpeedAt12Volts(SPEED_AT_12V_MPS)
      .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
      .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
      // .withFeedbackSource(kSteerFeedbackType) Not set for Analog Encoder
      .withDriveMotorInitialConfigs(new TalonFXConfiguration())
      .withSteerMotorInitialConfigs(new TalonFXConfiguration())
      .withEncoderInitialConfigs(new CANcoderConfiguration())
      .withSteerInertia(STEER_INERTIA)
      .withDriveInertia(DRIVE_INERTIA)
      .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
      .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE);

  // Front Left Module Constants
  private final ModuleType FL_MODULE_TYPE = ModuleType.getModuleType("MK4N-L2+");
  public final Translation2d FL_MODULE_TRANSLATION = new Translation2d(Units.inchesToMeters(11.4),
      Units.inchesToMeters(11.4));
  public final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FL_MODULE_CONSTANTS = CONSTANT_CREATOR
      .createModuleConstants(
          2,
          1,
          0,
          0,
          FL_MODULE_TRANSLATION.getX(),
          FL_MODULE_TRANSLATION.getY(),
          getBoolConstant("fl", "invert_drive"),
          FL_MODULE_TYPE.steerInverted,
          false)
      .withDriveMotorGearRatio(FL_MODULE_TYPE.driveRatio)
      .withSteerMotorGearRatio(FL_MODULE_TYPE.steerRatio);

  // Front Right Module Constants
  private final ModuleType FR_MODULE_TYPE = ModuleType.getModuleType("MK4I-L2+");
  public final Translation2d FR_MODULE_TRANSLATION = new Translation2d(Units.inchesToMeters(11.4),
      Units.inchesToMeters(-11.4));
  public final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FR_MODULE_CONSTANTS = CONSTANT_CREATOR
      .createModuleConstants(
          4,
          3,
          1,
          0,
          FR_MODULE_TRANSLATION.getX(),
          FR_MODULE_TRANSLATION.getY(),
          getBoolConstant("fr", "invert_drive"),
          FR_MODULE_TYPE.steerInverted,
          false)
      .withDriveMotorGearRatio(FR_MODULE_TYPE.driveRatio)
      .withSteerMotorGearRatio(FR_MODULE_TYPE.steerRatio);

  // Back Left Module Constants
  private final ModuleType BL_MODULE_TYPE = ModuleType.getModuleType("MK4N-L2+");
  public final Translation2d BL_MODULE_TRANSLATION = new Translation2d(Units.inchesToMeters(-11.4),
      Units.inchesToMeters(11.4));
  public final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BL_MODULE_CONSTANTS = CONSTANT_CREATOR
      .createModuleConstants(
          6,
          5,
          2,
          0,
          BL_MODULE_TRANSLATION.getX(),
          BL_MODULE_TRANSLATION.getY(),
          getBoolConstant("bl", "invert_drive"),
          BL_MODULE_TYPE.steerInverted,
          false)
      .withDriveMotorGearRatio(BL_MODULE_TYPE.driveRatio)
      .withSteerMotorGearRatio(BL_MODULE_TYPE.steerRatio);

  // Back Right Module Constants
  private final ModuleType BR_MODULE_TYPE = ModuleType.getModuleType("MK4I-L2+");
  public final Translation2d BR_MODULE_TRANSLATION = new Translation2d(Units.inchesToMeters(-11.4),
      Units.inchesToMeters(-11.4));
  public final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BR_MODULE_CONSTANTS = CONSTANT_CREATOR
      .createModuleConstants(
          8,
          7,
          3,
          0,
          BR_MODULE_TRANSLATION.getX(),
          BR_MODULE_TRANSLATION.getY(),
          getBoolConstant("br", "invert_drive"),
          BR_MODULE_TYPE.steerInverted,
          false)
      .withDriveMotorGearRatio(BR_MODULE_TYPE.driveRatio)
      .withSteerMotorGearRatio(BR_MODULE_TYPE.steerRatio);

  // Choreo Constants
  public final boolean FLIP_TRAJECTORY_ON_RED = true;
  public final double CHOREO_TRANSLATION_ERROR_MARGIN = Units.inchesToMeters(1.0);
  public final double CHOREO_X_CONTROLLER_KP = 0.0;
  public final double CHOREO_X_CONTROLLER_KI = 0.0;
  public final double CHOREO_X_CONTROLLER_KD = 0.0;
  public final double CHOREO_Y_CONTROLLER_KP = 0.0;
  public final double CHOREO_Y_CONTROLLER_KI = 0.0;
  public final double CHOREO_Y_CONTROLLER_KD = 0.0;
  public final double CHOREO_THETA_CONTROLLER_KP = 0.0;
  public final double CHOREO_THETA_CONTROLLER_KI = 0.0;
  public final double CHOREO_THETA_CONTROLLER_KD = 0.0;

  // Tractor Beam Constants
  public final double TRACTOR_BEAM_TRANSLATION_ERROR_MARGIN = Units.inchesToMeters(0.5);
  public final double TRACTOR_BEAM_STATIC_FRICTION_CONSTANT = 0.1;
  public final double TRACTOR_BEAM_CONTROLLER_KP = 0.0;
  public final double TRACTOR_BEAM_CONTROLLER_KI = 0.0;
  public final double TRACTOR_BEAM_CONTROLLER_KD = 0.0;

  // Simulation Constants
  private final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> SIM_MODULE_CONSTANTS_FACTORY = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
      .withDriveMotorGearRatio(FL_MODULE_CONSTANTS.DriveMotorGearRatio)
      .withSteerMotorGearRatio(16.0) // Maximum gear ratio sim can support
      .withCouplingGearRatio(COUPLE_RATIO)
      .withWheelRadius(WHEEL_RADIUS_METERS)
      .withSteerMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
      .withDriveMotorClosedLoopOutput(ClosedLoopOutputType.Voltage)
      .withSlipCurrent(SLIP_CURRENT_AMPS)
      .withSpeedAt12Volts(SPEED_AT_12V_MPS)
      .withDriveMotorType(DriveMotorArrangement.TalonFX_Integrated)
      .withSteerMotorType(SteerMotorArrangement.TalonFX_Integrated)
      .withDriveMotorInitialConfigs(new TalonFXConfiguration())
      .withSteerMotorInitialConfigs(new TalonFXConfiguration())
      .withEncoderInitialConfigs(new CANcoderConfiguration())
      .withSteerInertia(KilogramSquareMeters.of(0.05)) // Adjust steer inertia
      .withDriveInertia(DRIVE_INERTIA)
      .withDriveFrictionVoltage(Volts.of(0.1)) // Adjust friction voltages
      .withSteerFrictionVoltage(Volts.of(0.05)) // Adjust friction voltages
      .withDriveMotorGains(FL_MODULE_CONSTANTS.DriveMotorGains)
      .withSteerMotorGains( // Adjust steer motor PID gains for simulation
          new Slot0Configs()
              .withKP(70)
              .withKI(0.5)
              .withKD(4.5)
              .withKS(0)
              .withKV(1.91)
              .withKA(0)
              .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign));

  public final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> SIM_FL_MODULE_CONSTANTS = SIM_MODULE_CONSTANTS_FACTORY
      .createModuleConstants(
          2,
          1,
          0,
          0,
          FL_MODULE_CONSTANTS.LocationX,
          FL_MODULE_CONSTANTS.LocationY,
          false,
          false,
          false);
  public final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> SIM_FR_MODULE_CONSTANTS = SIM_MODULE_CONSTANTS_FACTORY
      .createModuleConstants(
          4,
          3,
          1,
          0,
          FR_MODULE_CONSTANTS.LocationX,
          FR_MODULE_CONSTANTS.LocationY,
          false,
          false,
          false);
  public final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> SIM_BL_MODULE_CONSTANTS = SIM_MODULE_CONSTANTS_FACTORY
      .createModuleConstants(
          6,
          5,
          2,
          0,
          BL_MODULE_CONSTANTS.LocationX,
          BL_MODULE_CONSTANTS.LocationY,
          false,
          false,
          false);
  public final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> SIM_BR_MODULE_CONSTANTS = SIM_MODULE_CONSTANTS_FACTORY
      .createModuleConstants(
          8,
          7,
          3,
          0,
          BR_MODULE_CONSTANTS.LocationX,
          BR_MODULE_CONSTANTS.LocationY,
          false,
          false,
          false);

  @SuppressWarnings("unchecked")
  public final DriveTrainSimulationConfig MAPLE_SIM_DRIVETRAIN_CONFIG = new DriveTrainSimulationConfig(
      Kilograms.of(ROBOT_MASS_KG),
      Meters.of(BUMPER_LENGTH_METERS),
      Meters.of(BUMPER_WIDTH_METERS),
      Meters.of(SIM_FL_MODULE_CONSTANTS.LocationX - SIM_BL_MODULE_CONSTANTS.LocationX),
      Meters.of(SIM_FL_MODULE_CONSTANTS.LocationY - SIM_FR_MODULE_CONSTANTS.LocationY),
      COTS.ofPigeon2(),
      new SwerveModuleSimulationConfig(
          DCMotor.getKrakenX60(1),
          DCMotor.getFalcon500(1),
          SIM_FL_MODULE_CONSTANTS.DriveMotorGearRatio,
          SIM_FL_MODULE_CONSTANTS.SteerMotorGearRatio,
          Volts.of(SIM_FL_MODULE_CONSTANTS.DriveFrictionVoltage),
          Volts.of(SIM_FL_MODULE_CONSTANTS.SteerFrictionVoltage),
          Meters.of(SIM_FL_MODULE_CONSTANTS.WheelRadius),
          KilogramSquareMeters.of(SIM_FL_MODULE_CONSTANTS.SteerInertia),
          WHEEL_COF));
}
