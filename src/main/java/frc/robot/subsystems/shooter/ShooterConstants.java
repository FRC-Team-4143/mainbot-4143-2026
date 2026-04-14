package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
    public final int INDEXER_LEADER_ID = 14;
    public final int INDEXER_FOLLOWER_ID = 15;
    public final int HOOD_ID = 16;

    // =============================================================================
    // MECHANICAL CONSTANTS - FLYWHEEL
    // =============================================================================

    public final boolean FLYWHEEL_LEADER_INVERTED = true;
    public final boolean FLYWHEEL_FOLLOWER_1_INVERTED = true;
    public final boolean FLYWHEEL_FOLLOWER_2_INVERTED = true;
    public final boolean FLYWHEEL_FOLLOWER_3_INVERTED = true;
    public final double FLYWHEEL_GEAR_RATIO = 1.5;
    public final double FLYWHEEL_WHEEL_RADIUS_METERS = Units.inchesToMeters(2);
    public final double FLYWHEEL_INERTIA = 21.394 * 0.00029264; // kg m^2, approximate, 0.00029264 is the conversion factor from lb in² to kg m²
    public final double FLYWHEEL_EFF_FACTOR = 2.2;
    public final Slot1Configs FLYWHEEL_VELOCITY_GAINS =
            new Slot1Configs().withKP(0.5).withKV(0.118).withKI(2);

    public final double FLYWHEEL_FILTER_TIME_CONSTANT =
            0.15; // seconds for flywheel velocity smoothing

    // Manual mode flywheel velocities (rad/s) - TUNE THESE!
    public final double FLYWHEEL_MANUAL_HUB_VELOCITY =
            (295.0 * 1.1) + 2.0; // Flywheel speed for hub shots
    public final double FLYWHEEL_MANUAL_PASS_VELOCITY = 375.0; // Flywheel speed for passing

    @Deprecated
    public final double FLYWHEEL_MANUAL_VELOCITY =
            260.0; // Use FLYWHEEL_MANUAL_HUB_VELOCITY instead

    // =============================================================================
    // MECHANICAL CONSTANTS - INDEXER
    // =============================================================================

    public final boolean INDEXER_LEADER_INVERTED = false;
    public final boolean INDEXER_FOLLOWER_INVERTED = false;
    public final double INDEXER_GEAR_RATIO = 24.0 / 18.0;
    public final Slot1Configs INDEXER_VELOCITY_GAINS =
            new Slot1Configs().withKD(0.01).withKI(1).withKP(0.5).withKV(0.129);

    // =============================================================================
    // MECHANICAL CONSTANTS - HOOD
    // =============================================================================

    public final boolean HOOD_INVERTED = false;
    public final double HOOD_GEAR_RATIO =
            9.0 * (372.0 / 40.0); // motor rotations / output mechanism rotations
    // Min/max physical hood angles (radians). Configure to match the mechanical limits
    public final double HOOD_MIN_ANGLE = Units.degreesToRadians(30);
    public final double HOOD_HOME_POSITION = Units.degreesToRadians(81.170);
    public final double HOOD_MAX_ANGLE = HOOD_HOME_POSITION;
    public final Slot0Configs HOOD_POSITION_GAINS = new Slot0Configs().withKP(100).withKD(0.15);

    // Manual mode hood angles (radians) - TUNE THESE!
    public final double HOOD_MANUAL_HUB_ANGLE =
            Units.degreesToRadians(72.76 - 1.0); // Hood angle for hub shots
    public final double HOOD_MANUAL_PASS_ANGLE =
            Units.degreesToRadians(51.57); // Hood angle for passing

    public final double HOOD_HOMING_DUTY_CYCLE = 0.15;
    public final double HOOD_HOMMING_CURRENT_THRESHOLD =
            5.0; // Amps, threshold for detecting stall during homing

    @Deprecated
    public final double HOOD_MANUAL_ANGLE =
            Units.degreesToRadians(0); // Use HOOD_MANUAL_HUB_ANGLE instead

    // Hood feedforward gain for velocity (V/(rad/s)) - converts hood angular velocity to voltage
    public final double HOOD_KV = 0.0;

    // =============================================================================
    // CONTROL AND OPERATIONAL CONSTANTS
    // =============================================================================
    public final double INDEXER_DUTY_CYCLE = 0.5; // 30% power for indexing
    public final double IDLE_INDEXER_DUTY_CYCLE = 0;
    public final double INDEXER_VELOCITY = 250;
    public final double SHOOTER_IDLE_SPEED = 300.0;
    public final double HOOD_IDLE_POSITION = Units.degreesToRadians(80);
    public final Transform2d SHOOTER_CENTER =
            new Transform2d(
                    new Translation2d(
                            Units.inchesToMeters(getDoubleConstant("translation", "x")),
                            Units.inchesToMeters(getDoubleConstant("translation", "y"))),
                    Rotation2d.fromDegrees(getDoubleConstant("rotation", "z")));
    public final double SHOOTER_READY_DEBOUNCE_TIME = 0.2; // seconds

    // =============================================================================
    // MOTOR CONFIGURATION OBJECTS
    // =============================================================================
    public final MotorConfig SHOOTER_LEADER_MOTOR_CONFIG = new MotorConfig();
    public final MotorConfig SHOOTER_FOLLOWER_MOTOR_1_CONFIG = new MotorConfig();
    public final MotorConfig SHOOTER_FOLLOWER_MOTOR_2_CONFIG = new MotorConfig();
    public final MotorConfig SHOOTER_FOLLOWER_MOTOR_3_CONFIG = new MotorConfig();
    public final MotorConfig INDEXER_LEADER_MOTOR_CONFIG = new MotorConfig();
    public final MotorConfig INDEXER_FOLLOWER_MOTOR_CONFIG = new MotorConfig();
    public final MotorConfig HOOD_MOTOR_CONFIGS = new MotorConfig();

    // =============================================================================
    // LAUNCH CALCULATOR - Map-based shooting with motion compensation
    // =============================================================================
    public final LaunchCalculator HUB_LAUNCH_CALCULATOR;
    public final LaunchCalculator PASS_LAUNCH_CALCULATOR;
    
    // =============================================================================
    // Hopper empty
    // =============================================================================
        public final double SHOOTING_DETECTION_VELOCITY_FACTOR = 0.99; // factor applied to current speed to get threshold for triggering detection, needs tuning
        public final double SHOOTING_DETECTION_TIME = 0.65; // seconds for shooter detection to be true, needs tuning
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
        HUB_LAUNCH_CALCULATOR.addHoodAnglePoint(0.75, 1.399);
        HUB_LAUNCH_CALCULATOR.addHoodAnglePoint(1.34, 1.330);
        HUB_LAUNCH_CALCULATOR.addHoodAnglePoint(1.78, 1.325);
        HUB_LAUNCH_CALCULATOR.addHoodAnglePoint(2.17, 1.275);
        HUB_LAUNCH_CALCULATOR.addHoodAnglePoint(2.81, 1.225);
        HUB_LAUNCH_CALCULATOR.addHoodAnglePoint(3.82, 1.175);
        HUB_LAUNCH_CALCULATOR.addHoodAnglePoint(4.40, 1.114);
        HUB_LAUNCH_CALCULATOR.addHoodAnglePoint(4.77, 1.114);
        HUB_LAUNCH_CALCULATOR.addHoodAnglePoint(5.60, 1.114);
        HUB_LAUNCH_CALCULATOR.addHoodAnglePoint(6.50, 1.114);

        // Populate flywheel speed map (distance in meters -> speed in rad/s)
        // Empirically determined values from testing
        HUB_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(0.75, 230);
        HUB_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(1.34, 240);
        HUB_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(1.78, 280);
        HUB_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(2.17, 285.66);
        HUB_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(2.81, 290);
        HUB_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(3.82, 315);
        HUB_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(4.40, 330);
        HUB_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(4.77, 340);
        HUB_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(5.60, 445); // needs updating
        HUB_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(6.50, 518); // needs updating

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
        PASS_LAUNCH_CALCULATOR.addHoodAnglePoint(4.00, Units.degreesToRadians(85.94));
        PASS_LAUNCH_CALCULATOR.addHoodAnglePoint(6.00, Units.degreesToRadians(51.57));
        PASS_LAUNCH_CALCULATOR.addHoodAnglePoint(8.00, Units.degreesToRadians(51.57));
        PASS_LAUNCH_CALCULATOR.addHoodAnglePoint(12.00, Units.degreesToRadians(44.69));

        // Populate flywheel speed map (distance in meters -> speed in rad/s)
        // Empirically determined values from testing
        PASS_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(4.00, 260.0);
        PASS_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(6.00, 300.0);
        PASS_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(8.00, 375.0);
        PASS_LAUNCH_CALCULATOR.addFlywheelSpeedPoint(12.00, 450.0);

        // Populate time of flight map (distance in meters -> time in seconds)
        // Empirically determined values from testing
        PASS_LAUNCH_CALCULATOR.addTimeOfFlightPoint(4, 1.129);
        PASS_LAUNCH_CALCULATOR.addTimeOfFlightPoint(11, 1.293);
        PASS_LAUNCH_CALCULATOR.addTimeOfFlightPoint(18, 1.413);

        // =============================================================================
        // MOTOR CONFIGURATION INITIALIZATION
        // =============================================================================
        // Configure Indexer Leader Motor
        INDEXER_LEADER_MOTOR_CONFIG.can_id = INDEXER_LEADER_ID;
        INDEXER_LEADER_MOTOR_CONFIG.motor_type = TalonMotorType.X44;
        INDEXER_LEADER_MOTOR_CONFIG.canbus_name = "rio";
        TalonFXConfiguration indexer_leader_config = new TalonFXConfiguration();
        indexer_leader_config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(INDEXER_LEADER_INVERTED);
        indexer_leader_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        indexer_leader_config.Slot1 = INDEXER_VELOCITY_GAINS;
        INDEXER_LEADER_MOTOR_CONFIG.apply(indexer_leader_config);

        // Configure Indexer FOLLOWER Motor
        INDEXER_FOLLOWER_MOTOR_CONFIG.can_id = INDEXER_FOLLOWER_ID;
        INDEXER_FOLLOWER_MOTOR_CONFIG.motor_type = TalonMotorType.X44;
        INDEXER_FOLLOWER_MOTOR_CONFIG.canbus_name = "rio";
        TalonFXConfiguration indexer_follower_config = new TalonFXConfiguration();
        indexer_follower_config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(INDEXER_FOLLOWER_INVERTED);
        indexer_follower_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        INDEXER_FOLLOWER_MOTOR_CONFIG.apply(indexer_follower_config);

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
        // shooter_leader_config.CurrentLimits.SupplyCurrentLimit = 70.0;
        // shooter_leader_config.CurrentLimits.SupplyCurrentLimitEnable = true;
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
        // shooter_follower_1_config.CurrentLimits.SupplyCurrentLimit = 70.0;
        // shooter_follower_1_config.CurrentLimits.SupplyCurrentLimitEnable = true;
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
        // shooter_follower_2_config.CurrentLimits.SupplyCurrentLimit = 70.0;
        // shooter_follower_2_config.CurrentLimits.SupplyCurrentLimitEnable = true;
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
        // shooter_follower_3_config.CurrentLimits.SupplyCurrentLimit = 70.0;
        // shooter_follower_3_config.CurrentLimits.SupplyCurrentLimitEnable = true;
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
        HOOD_MOTOR_CONFIGS.apply(hood_config);
    }
}
