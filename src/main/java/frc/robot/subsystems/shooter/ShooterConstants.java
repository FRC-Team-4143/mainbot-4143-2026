package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.marswars.geometry.LaunchCalculator;
import com.marswars.mechanisms.MotorConfig;
import com.marswars.mechanisms.MotorConfig.TalonMotorType;
import com.marswars.subsystem.MwConstants;
import com.marswars.util.PhoenixUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class ShooterConstants extends MwConstants {

    // =============================================================================
    // ENUMS AND STATE DEFINITIONS
    // =============================================================================

    public enum ShooterStates {
        /** Aiming at target with shooter spinning and robot rotating, waiting to be ready */
        SHOOT_WAIT,
        /** Dumping game pieces out backwards */
        DUMP,
        /** Actively shooting game pieces into target */
        SHOOT,
        /** Idle state with shooter mechanisms at rest */
        IDLE,
        /** Tracking target with shooter spinning but not shooting */
        TRACKING,
        /** Manual tuning mode for testing and calibration */
        TUNING,
        /** Manual control mode for hub shooting */
        MANUAL_HUB,
        /** Manual control mode for passing */
        MANUAL_PASS,
        /** Smooth return to zero velocity for clean disable */
        SPIN_DOWN,
        /** Aiming at target with shooter spinning and robot rotating, with no intent to SHOOT */
        AIMING,
        /** Homing the hood to its home position */
        HOOD_HOMING
    }

    public enum TargetType {
        HUB,
        PASS
    }

    // =============================================================================
    // CAN IDS AND HARDWARE CONFIGURATION
    // =============================================================================

    // Motor CAN IDs
    public final int SHOOTER_LEADER_ID = 10;
    public final int SHOOTER_FOLLOWER_1_ID = 11;
    public final int SHOOTER_FOLLOWER_2_ID = 12;
    public final int SHOOTER_FOLLOWER_3_ID = 13;
    public final int INDEXER_ID = 14;
    public final int ACCELERATOR_ID = 15;
    public final int HOOD_ID = 16;
    public final int HOPPER_MOTOR_ID = 20;

    // =============================================================================
    // MECHANICAL CONSTANTS - FLYWHEEL
    // =============================================================================

    // Motor configuration
    public final boolean FLYWHEEL_LEADER_INVERTED = true;
    public final boolean FLYWHEEL_FOLLOWER_1_INVERTED = true;
    public final boolean FLYWHEEL_FOLLOWER_2_INVERTED = false;
    public final boolean FLYWHEEL_FOLLOWER_3_INVERTED = false;

    // Physical properties
    public final double FLYWHEEL_GEAR_RATIO = 32.0 / 24.0;
    public final double FLYWHEEL_WHEEL_RADIUS_METERS = Units.inchesToMeters(2);
    public final double FLYWHEEL_INERTIA =
            (21.394 * 0.00029264)
                    + (0.812 * 2); // kg m^2, approximate, 0.00029264 is the conversion factor from
    // lb in² to kg m²
    public final double FLYWHEEL_EFF_FACTOR = 2.2;

    // Control gains
    public final Slot1Configs FLYWHEEL_VELOCITY_GAINS =
            new Slot1Configs().withKP(0.5).withKV(0.159).withKI(0.5).withKD(0.005);

    // =============================================================================
    // MECHANICAL CONSTANTS - INDEXER
    // =============================================================================

    public final boolean INDEXER_INVERTED = true;
    public final double INDEXER_GEAR_RATIO = 28.0 / 11.0;
    public final Slot1Configs INDEXER_VELOCITY_GAINS =
            new Slot1Configs().withKD(0.01).withKI(1).withKP(0.5).withKV(0.129);

    // =============================================================================
    // MECHANICAL CONSTANTS - ACCELERATOR
    // =============================================================================

    public final boolean ACCELERATOR_INVERTED = true;
    public final double ACCELERATOR_GEAR_RATIO = (40.0 / 11.0) * (23.0 / 26.0);
    public final Slot1Configs ACCELERATOR_VELOCITY_GAINS =
            new Slot1Configs().withKD(0.0).withKI(0).withKP(0.0).withKV(0.0);

    // =============================================================================
    // MECHANICAL CONSTANTS - HOPPER
    // =============================================================================

    public final boolean HOPPER_MOTOR_INVERTED = false;
    public final double HOPPER_GEAR_RATIO = 1.0;
    public final Slot1Configs HOPPER_VELOCITY_GAINS = new Slot1Configs().withKV(.122).withKP(0.5);

    // =============================================================================
    // MECHANICAL CONSTANTS - HOOD
    // =============================================================================

    public final boolean HOOD_INVERTED = true;
    public final double HOOD_GEAR_RATIO =
            68; // motor rotations / output mechanism rotations / original 44/ 11 * 170/100

    // Physical limits - hood angles in radians
    public final double HOOD_MIN_ANGLE = Units.degreesToRadians(51.0);
    public final double HOOD_MAX_ANGLE = Units.degreesToRadians(90.0);
    public final double HOOD_HOME_POSITION = HOOD_MAX_ANGLE;

    // Control gains
    public final Slot0Configs HOOD_POSITION_GAINS =
            new Slot0Configs().withKP(90).withKD(0.15).withKS(0.25);
    public final double HOOD_KV = 0.0; // Hood feedforward gain (V/(rad/s))

    // =============================================================================
    // CONTROL AND OPERATIONAL CONSTANTS
    // =============================================================================

    // Shooter geometry and positioning
    public final Transform2d SHOOTER_CENTER =
            new Transform2d(
                    new Translation2d(
                            Units.inchesToMeters(getDoubleConstant("translation", "x")),
                            Units.inchesToMeters(getDoubleConstant("translation", "y"))),
                    Rotation2d.fromDegrees(getDoubleConstant("rotation", "z")));

    // Flywheel control
    public final double FLYWHEEL_FILTER_TIME_CONSTANT = 0.15; // seconds for velocity smoothing
    public final double SHOOTER_IDLE_SPEED = 180.0; // rad/s

    // Indexer control
    public final double INDEXER_DUTY_CYCLE = 1.0;
    public final double IDLE_INDEXER_DUTY_CYCLE = 0.0;
    public final double INDEXER_VELOCITY = 250; // rad/s

    // Accelerator control
    public final double ACCELERATOR_DUTY_CYCLE = 1.0;
    public final double IDLE_ACCELERATOR_DUTY_CYCLE = 0.0;
    public final double ACCELERATOR_VELOCITY = 300; // rad/s

    // Hopper control
    public final double HOPPER_VELOCITY_TARGET = 200; // rad/s

    // Hood control
    public final double HOOD_IDLE_POSITION = Units.degreesToRadians(88);
    public final double HOOD_HOMING_DUTY_CYCLE = 0.15;
    public final double HOOD_HOMING_CURRENT_THRESHOLD = 5.0; // Amps, stall detection threshold

    // Shooter ready detection
    public final double SHOOTER_READY_DEBOUNCE_TIME = 0.2; // seconds

    // Shot detection (for detecting when a game piece has left the shooter)
    public final double SHOOTING_DETECTION_VELOCITY_FACTOR = 0.99; // Velocity threshold factor
    public final double SHOOTING_DETECTION_TIME = 0.65; // seconds for debouncing

    // =============================================================================
    // MANUAL MODE SETPOINTS
    // =============================================================================

    // Hub shooting (close range)
    public final double FLYWHEEL_MANUAL_HUB_VELOCITY = 200.0; // rad/s
    public final double HOOD_MANUAL_HUB_ANGLE = Units.degreesToRadians(72.76 - 1.0); // radians

    // Passing (long range)
    public final double FLYWHEEL_MANUAL_PASS_VELOCITY = 200.0; // rad/s
    public final double HOOD_MANUAL_PASS_ANGLE = Units.degreesToRadians(51.57); // radians

    // =============================================================================
    // MOTOR CONFIGURATION OBJECTS
    // =============================================================================
    public final MotorConfig SHOOTER_LEADER_MOTOR_CONFIG = new MotorConfig();
    public final MotorConfig SHOOTER_FOLLOWER_MOTOR_1_CONFIG = new MotorConfig();
    public final MotorConfig SHOOTER_FOLLOWER_MOTOR_2_CONFIG = new MotorConfig();
    public final MotorConfig SHOOTER_FOLLOWER_MOTOR_3_CONFIG = new MotorConfig();
    public final MotorConfig INDEXER_MOTOR_CONFIG = new MotorConfig();
    public final MotorConfig ACCELERATOR_MOTOR_CONFIG = new MotorConfig();
    public final MotorConfig HOOD_MOTOR_CONFIGS = new MotorConfig();
    public final MotorConfig HOPPER_MOTOR_CONFIG = new MotorConfig();

    // =============================================================================
    // LAUNCH CALCULATOR - Map-based shooting with motion compensation
    // =============================================================================
    public final LaunchCalculator HUB_LAUNCH_CALCULATOR;
    public final LaunchCalculator PASS_LAUNCH_CALCULATOR;

    // =============================================================================
    // CONSTRUCTOR - MOTOR CONFIGURATION INITIALIZATION
    // =============================================================================

    public ShooterConstants() {
        // Initialize Launch Calculator with robot-to-shooter transform
        HUB_LAUNCH_CALCULATOR =
                new LaunchCalculator("Subsystem/Shooter/HubLaunchCalculator/", SHOOTER_CENTER);

        // Configure range limits
        HUB_LAUNCH_CALCULATOR.setMinDistance(0); // Minimum shooting distance in meters
        HUB_LAUNCH_CALCULATOR.setMaxDistance(7.0); // Maximum shooting distance in meters
        HUB_LAUNCH_CALCULATOR.setPhaseDelay(0.03); // Processing and actuator delay in seconds

        // Populate hood angle map (distance in meters -> angle in radians)
        // Empirically determined values from testing
        HUB_LAUNCH_CALCULATOR.addHoodAnglePoint(0.75, 1.5);
        HUB_LAUNCH_CALCULATOR.addHoodAnglePoint(1.34, 1.4);
        HUB_LAUNCH_CALCULATOR.addHoodAnglePoint(1.78, 1.35);
        HUB_LAUNCH_CALCULATOR.addHoodAnglePoint(2.17, 1.32);
        HUB_LAUNCH_CALCULATOR.addHoodAnglePoint(2.81, 1.32);
        HUB_LAUNCH_CALCULATOR.addHoodAnglePoint(3.82, 1.2);
        HUB_LAUNCH_CALCULATOR.addHoodAnglePoint(4.40, 1.2);
        HUB_LAUNCH_CALCULATOR.addHoodAnglePoint(4.77, 1.15);
        HUB_LAUNCH_CALCULATOR.addHoodAnglePoint(5.60, 1.12);
        HUB_LAUNCH_CALCULATOR.addHoodAnglePoint(6.50, 1.15);

        // Populate flywheel speed map (distance in meters -> speed in rad/s)
        // Empirically determined values from testing
        HUB_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(0.75, 140);
        HUB_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(1.34, 140);
        HUB_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(1.78, 160);
        HUB_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(2.17, 160);
        HUB_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(2.81, 175);
        HUB_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(3.82, 184);
        HUB_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(4.40, 196);
        HUB_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(4.77, 196);
        HUB_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(5.60, 225); 
        HUB_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(6.50, 245); 

        // Populate time of flight map (distance in meters -> time in seconds)
        // Empirically determined values from testing
        HUB_LAUNCH_CALCULATOR.addTimeOfFlightPoint(1.5, 1.129);
        HUB_LAUNCH_CALCULATOR.addTimeOfFlightPoint(3, 1.293);
        HUB_LAUNCH_CALCULATOR.addTimeOfFlightPoint(5.00, 1.413);

        PASS_LAUNCH_CALCULATOR =
                new LaunchCalculator("Subsystem/Shooter/PassLaunchCalculator/", SHOOTER_CENTER);

        // Configure range limits
        PASS_LAUNCH_CALCULATOR.setMinDistance(0.5); // Minimum passing distance in meters
        PASS_LAUNCH_CALCULATOR.setMaxDistance(18.0); // Maximum passing distance in meters
        PASS_LAUNCH_CALCULATOR.setPhaseDelay(0.03); // Processing and actuator delay in seconds

        // Populate hood angle map (distance in meters -> angle in radians)
        PASS_LAUNCH_CALCULATOR.addHoodAnglePoint(4.00, 1.1);
        PASS_LAUNCH_CALCULATOR.addHoodAnglePoint(6.00, 1.1);
        PASS_LAUNCH_CALCULATOR.addHoodAnglePoint(8.00, 1.1);
        PASS_LAUNCH_CALCULATOR.addHoodAnglePoint(12.00, 0.95);

        // Populate flywheel speed map (distance in meters -> speed in rad/s)
        // Empirically determined values from testing
        PASS_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(4.00, 135);
        PASS_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(6.00, 172);
        PASS_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(8.00, 207);
        PASS_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(12.00, 260);

        // Populate time of flight map (distance in meters -> time in seconds)
        // Empirically determined values from testing
        PASS_LAUNCH_CALCULATOR.addTimeOfFlightPoint(4, 1.129);
        PASS_LAUNCH_CALCULATOR.addTimeOfFlightPoint(11, 1.293);
        PASS_LAUNCH_CALCULATOR.addTimeOfFlightPoint(18, 1.413);

        // =============================================================================
        // MOTOR CONFIGURATION INITIALIZATION
        // =============================================================================
        // Configure Indexer Leader Motor
        INDEXER_MOTOR_CONFIG.can_id = INDEXER_ID;
        INDEXER_MOTOR_CONFIG.motor_type = TalonMotorType.X44;
        INDEXER_MOTOR_CONFIG.canbus_name = "rio";
        TalonFXConfiguration indexer_leader_config = new TalonFXConfiguration();
        indexer_leader_config.MotorOutput.Inverted = PhoenixUtil.toInvertedValue(INDEXER_INVERTED);
        indexer_leader_config.Voltage.PeakForwardVoltage = 8.0;
        indexer_leader_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexer_leader_config.Slot1 = INDEXER_VELOCITY_GAINS;
        INDEXER_MOTOR_CONFIG.apply(indexer_leader_config);

        ACCELERATOR_MOTOR_CONFIG.can_id = ACCELERATOR_ID;
        ACCELERATOR_MOTOR_CONFIG.motor_type = TalonMotorType.X44;
        ACCELERATOR_MOTOR_CONFIG.canbus_name = "rio";
        TalonFXConfiguration accelerator_config = new TalonFXConfiguration();
        accelerator_config.MotorOutput.Inverted = PhoenixUtil.toInvertedValue(ACCELERATOR_INVERTED);
        accelerator_config.Voltage.PeakForwardVoltage = 8.0;
        accelerator_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        accelerator_config.Slot1 = ACCELERATOR_VELOCITY_GAINS;
        ACCELERATOR_MOTOR_CONFIG.apply(accelerator_config);

        // Configure Shooter Leader Motor
        SHOOTER_LEADER_MOTOR_CONFIG.can_id = SHOOTER_LEADER_ID;
        SHOOTER_LEADER_MOTOR_CONFIG.motor_type = TalonMotorType.X60;
        SHOOTER_LEADER_MOTOR_CONFIG.canbus_name = "rio";
        TalonFXConfiguration shooter_leader_config = new TalonFXConfiguration();
        shooter_leader_config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(FLYWHEEL_LEADER_INVERTED);
        shooter_leader_config.Slot1 = FLYWHEEL_VELOCITY_GAINS;
        shooter_leader_config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooter_leader_config.Voltage.PeakReverseVoltage = 0.0;
        // MotionMagic tuned for ~3 second ramp to 300 rad/s (47.75 rot/s)
        // Acceleration: 16 rot/s² (47.75 / 3)
        // Jerk: 80 rot/s³ (5x acceleration for smooth motion)
        shooter_leader_config.MotionMagic.MotionMagicAcceleration = 16.0;
        shooter_leader_config.MotionMagic.MotionMagicJerk = 80.0;
        // shooter_leader_config.CurrentLimits.StatorCurrentLimit = 120.0;
        // shooter_leader_config.CurrentLimits.StatorCurrentLimitEnable = true;
        shooter_leader_config.CurrentLimits.SupplyCurrentLimit = 40.0;
        shooter_leader_config.CurrentLimits.SupplyCurrentLimitEnable = true;
        SHOOTER_LEADER_MOTOR_CONFIG.apply(shooter_leader_config);

        // Configure Shooter Follower 1 Motor
        SHOOTER_FOLLOWER_MOTOR_1_CONFIG.can_id = SHOOTER_FOLLOWER_1_ID;
        SHOOTER_FOLLOWER_MOTOR_1_CONFIG.motor_type = TalonMotorType.X60;
        SHOOTER_FOLLOWER_MOTOR_1_CONFIG.canbus_name = "rio";
        TalonFXConfiguration shooter_follower_1_config = new TalonFXConfiguration();
        shooter_follower_1_config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(FLYWHEEL_FOLLOWER_1_INVERTED);
        shooter_follower_1_config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooter_follower_1_config.Voltage.PeakReverseVoltage = 0.0;
        // shooter_follower_1_config.CurrentLimits.StatorCurrentLimit = 120.0;
        // shooter_follower_1_config.CurrentLimits.StatorCurrentLimitEnable = true;
        shooter_follower_1_config.CurrentLimits.SupplyCurrentLimit = 40.0;
        shooter_follower_1_config.CurrentLimits.SupplyCurrentLimitEnable = true;
        SHOOTER_FOLLOWER_MOTOR_1_CONFIG.apply(shooter_follower_1_config);

        // Configure Shooter Follower 2 Motor
        SHOOTER_FOLLOWER_MOTOR_2_CONFIG.can_id = SHOOTER_FOLLOWER_2_ID;
        SHOOTER_FOLLOWER_MOTOR_2_CONFIG.motor_type = TalonMotorType.X60;
        SHOOTER_FOLLOWER_MOTOR_2_CONFIG.canbus_name = "rio";
        TalonFXConfiguration shooter_follower_2_config = new TalonFXConfiguration();
        shooter_follower_2_config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(FLYWHEEL_FOLLOWER_2_INVERTED);
        shooter_follower_2_config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooter_follower_2_config.Voltage.PeakReverseVoltage = 0.0;
        // shooter_follower_2_config.CurrentLimits.StatorCurrentLimit = 120.0;
        // shooter_follower_2_config.CurrentLimits.StatorCurrentLimitEnable = true;
        shooter_follower_2_config.CurrentLimits.SupplyCurrentLimit = 40.0;
        shooter_follower_2_config.CurrentLimits.SupplyCurrentLimitEnable = true;
        SHOOTER_FOLLOWER_MOTOR_2_CONFIG.apply(shooter_follower_2_config);

        // Configure Shooter Follower 3 Motor
        SHOOTER_FOLLOWER_MOTOR_3_CONFIG.can_id = SHOOTER_FOLLOWER_3_ID;
        SHOOTER_FOLLOWER_MOTOR_3_CONFIG.motor_type = TalonMotorType.X60;
        SHOOTER_FOLLOWER_MOTOR_3_CONFIG.canbus_name = "rio";
        TalonFXConfiguration shooter_follower_3_config = new TalonFXConfiguration();
        shooter_follower_3_config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(FLYWHEEL_FOLLOWER_3_INVERTED);
        shooter_follower_3_config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooter_follower_3_config.Voltage.PeakReverseVoltage = 0.0;
        // shooter_follower_3_config.CurrentLimits.StatorCurrentLimit = 120.0;
        // shooter_follower_3_config.CurrentLimits.StatorCurrentLimitEnable = true;
        shooter_follower_3_config.CurrentLimits.SupplyCurrentLimit = 40.0;
        shooter_follower_3_config.CurrentLimits.SupplyCurrentLimitEnable = true;
        SHOOTER_FOLLOWER_MOTOR_3_CONFIG.apply(shooter_follower_3_config);

        // Configure Hood Motor
        HOOD_MOTOR_CONFIGS.can_id = HOOD_ID;
        HOOD_MOTOR_CONFIGS.motor_type = TalonMotorType.X44;
        HOOD_MOTOR_CONFIGS.canbus_name = "rio";
        TalonFXConfiguration hood_config = new TalonFXConfiguration();
        hood_config.MotorOutput.Inverted = PhoenixUtil.toInvertedValue(HOOD_INVERTED);
        hood_config.Slot0 = HOOD_POSITION_GAINS;
        hood_config.CurrentLimits.SupplyCurrentLimit = 10;
        hood_config.CurrentLimits.SupplyCurrentLimitEnable = true;
        hood_config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
        HOOD_MOTOR_CONFIGS.apply(hood_config);

        // Configure Hopper Motor
        HOPPER_MOTOR_CONFIG.can_id = HOPPER_MOTOR_ID;
        HOPPER_MOTOR_CONFIG.motor_type = TalonMotorType.X44;
        HOPPER_MOTOR_CONFIG.canbus_name = "rio";
        TalonFXConfiguration hopper_config = new TalonFXConfiguration();
        hopper_config.Slot1 = HOPPER_VELOCITY_GAINS;
        HOPPER_MOTOR_CONFIG.apply(hopper_config);
    }
}
