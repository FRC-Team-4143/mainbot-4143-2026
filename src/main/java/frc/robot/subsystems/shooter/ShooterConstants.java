package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.marswars.geometry.LaunchCalculator;
import com.marswars.subsystem.MwConstants;
import com.marswars.util.FxMotorConfig;
import com.marswars.util.FxMotorConfig.FxMotorType;
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
        /** Manual control mode with fixed setpoints */
        MANUAL,
        /** Smooth return to 0 velocity for clean disable */
        SPIN_DOWN,
        /** Aiming at target with shooter spinning and robot rotating, with no intent to SHOOT */
        AIMING
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
    public final double FLYWHEEL_GEAR_RATIO = 1.0;
    public final double FLYWHEEL_WHEEL_RADIUS_METERS = Units.inchesToMeters(2);
    public final double FLYWHEEL_MASS_KG = Units.lbsToKilograms(3.5);
    public final double FLYWHEEL_INERTIA =
            0.5
                    * FLYWHEEL_MASS_KG
                    * Math.pow(FLYWHEEL_WHEEL_RADIUS_METERS, 2.0); // kg m^2, approximate
    public final double FLYWHEEL_EFF_FACTOR = 2.2;
    public final Slot1Configs FLYWHEEL_VELOCITY_GAINS =
            new Slot1Configs().withKP(0.5).withKV(0.118).withKI(2);
    public final double FLYWHEEL_MANUAL_VELOCITY = 330.0;

    // =============================================================================
    // MECHANICAL CONSTANTS - INDEXER
    // =============================================================================

    public final boolean INDEXER_LEADER_INVERTED = false;
    public final boolean INDEXER_FOLLOWER_INVERTED = false;
    public final double INDEXER_GEAR_RATIO = 24.0 / 18.0;

    // =============================================================================
    // MECHANICAL CONSTANTS - HOOD
    // =============================================================================

    public final boolean HOOD_INVERTED = false;
    public final double HOOD_GEAR_RATIO =
            9.0 * (372.0 / 40.0); // motor rotations / output mechanism rotations
    // Min/max physical hood angles (radians). Configure to match the mechanical limits
    public final double HOOD_MIN_ANGLE = Units.degreesToRadians(30);
    public final double HOOD_MAX_ANGLE = Units.degreesToRadians(79);
    public final double HOOD_HOME_POSITION = Units.degreesToRadians(81.170);
    public final Slot0Configs HOOD_POSITION_GAINS = new Slot0Configs().withKP(100).withKD(0.15);
    public final double HOOD_MANUAL_ANGLE = Units.degreesToRadians(0);
    // Hood feedforward gain for velocity (V/(rad/s)) - converts hood angular velocity to voltage
    public final double HOOD_KV = 0.0;

    // =============================================================================
    // CONTROL AND OPERATIONAL CONSTANTS
    // =============================================================================
    public final double INDEXER_DUTY_CYCLE = 0.3; // 30% power for indexing
    public final double SHOOTER_IDLE_SPEED = 300.0;
    public final double HOOD_IDLE_POSITION = Units.degreesToRadians(80);
    public final Transform2d SHOOTER_CENTER =
            new Transform2d(
                    new Translation2d(
                            Units.inchesToMeters(getDoubleConstant("translation", "x")),
                            Units.inchesToMeters(getDoubleConstant("translation", "y"))),
                    Rotation2d.fromDegrees(getDoubleConstant("rotation", "z")));

    // =============================================================================
    // MOTOR CONFIGURATION OBJECTS
    // =============================================================================
    public final FxMotorConfig SHOOTER_LEADER_MOTOR_CONFIG = new FxMotorConfig();
    public final FxMotorConfig SHOOTER_FOLLOWER_MOTOR_1_CONFIG = new FxMotorConfig();
    public final FxMotorConfig SHOOTER_FOLLOWER_MOTOR_2_CONFIG = new FxMotorConfig();
    public final FxMotorConfig SHOOTER_FOLLOWER_MOTOR_3_CONFIG = new FxMotorConfig();
    public final FxMotorConfig INDEXER_LEADER_MOTOR_CONFIG = new FxMotorConfig();
    public final FxMotorConfig INDEXER_FOLLOWER_MOTOR_CONFIG = new FxMotorConfig();
    public final FxMotorConfig HOOD_MOTOR_CONFIGS = new FxMotorConfig();

    // =============================================================================
    // LAUNCH CALCULATOR - Map-based shooting with motion compensation
    // =============================================================================
    public final LaunchCalculator LAUNCH_CALCULATOR;

    // =============================================================================
    // CONSTRUCTOR - MOTOR CONFIGURATION INITIALIZATION
    // =============================================================================

    public ShooterConstants() {
        // Initialize Launch Calculator with robot-to-shooter transform
        LAUNCH_CALCULATOR =
                new LaunchCalculator("Subsystem/Shooter/LaunchCalculator/", SHOOTER_CENTER);

        // Configure range limits
        LAUNCH_CALCULATOR.setMinDistance(1.34); // Minimum shooting distance in meters
        LAUNCH_CALCULATOR.setMaxDistance(6.5); // Maximum shooting distance in meters
        LAUNCH_CALCULATOR.setPhaseDelay(0.03); // Processing and actuator delay in seconds

        // Populate hood angle map (distance in meters -> angle in radians)
        // Empirically determined values from testing
        LAUNCH_CALCULATOR.addHoodAnglePoint(1.34, Units.degreesToRadians(74.48));
        LAUNCH_CALCULATOR.addHoodAnglePoint(1.78, Units.degreesToRadians(74.48));
        LAUNCH_CALCULATOR.addHoodAnglePoint(2.17, Units.degreesToRadians(70.0));
        LAUNCH_CALCULATOR.addHoodAnglePoint(2.81, Units.degreesToRadians(71.62));
        LAUNCH_CALCULATOR.addHoodAnglePoint(3.82, Units.degreesToRadians(64.46));
        LAUNCH_CALCULATOR.addHoodAnglePoint(4.40, Units.degreesToRadians(64.46));
        LAUNCH_CALCULATOR.addHoodAnglePoint(4.77, Units.degreesToRadians(63.03));
        LAUNCH_CALCULATOR.addHoodAnglePoint(5.60, Units.degreesToRadians(63.03));
        LAUNCH_CALCULATOR.addHoodAnglePoint(6.50, Units.degreesToRadians(63.03));

        // Populate flywheel speed map (distance in meters -> speed in rad/s)
        // Empirically determined values from testing
        LAUNCH_CALCULATOR.addFlywheelSpeedPoint(1.34, 210.0);
        LAUNCH_CALCULATOR.addFlywheelSpeedPoint(1.78, 260.0);
        LAUNCH_CALCULATOR.addFlywheelSpeedPoint(2.17, 270.0);
        LAUNCH_CALCULATOR.addFlywheelSpeedPoint(2.81, 285.0);
        LAUNCH_CALCULATOR.addFlywheelSpeedPoint(3.82, 320.0);
        LAUNCH_CALCULATOR.addFlywheelSpeedPoint(4.40, 330.0);
        LAUNCH_CALCULATOR.addFlywheelSpeedPoint(4.77, 360.0);
        LAUNCH_CALCULATOR.addFlywheelSpeedPoint(5.60, 367.0);
        LAUNCH_CALCULATOR.addFlywheelSpeedPoint(6.50, 420.0);

        // Populate time of flight map (distance in meters -> time in seconds)
        // Empirically determined values from testing
        LAUNCH_CALCULATOR.addTimeOfFlightPoint(1.38, 0.90);
        LAUNCH_CALCULATOR.addTimeOfFlightPoint(1.88, 1.09);
        LAUNCH_CALCULATOR.addTimeOfFlightPoint(3.15, 1.11);
        LAUNCH_CALCULATOR.addTimeOfFlightPoint(4.55, 1.12);
        LAUNCH_CALCULATOR.addTimeOfFlightPoint(5.68, 1.16);

        // =============================================================================
        // MOTOR CONFIGURATION INITIALIZATION
        // =============================================================================
        // Configure Indexer Leader Motor
        INDEXER_LEADER_MOTOR_CONFIG.can_id = INDEXER_LEADER_ID;
        INDEXER_LEADER_MOTOR_CONFIG.motor_type = FxMotorType.X44;
        INDEXER_LEADER_MOTOR_CONFIG.canbus_name = "rio";
        INDEXER_LEADER_MOTOR_CONFIG.config = new TalonFXConfiguration();
        INDEXER_LEADER_MOTOR_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(INDEXER_LEADER_INVERTED);

        // Configure Indexer FOLLOWER Motor
        INDEXER_FOLLOWER_MOTOR_CONFIG.can_id = INDEXER_FOLLOWER_ID;
        INDEXER_FOLLOWER_MOTOR_CONFIG.motor_type = FxMotorType.X44;
        INDEXER_FOLLOWER_MOTOR_CONFIG.canbus_name = "rio";
        INDEXER_FOLLOWER_MOTOR_CONFIG.config = new TalonFXConfiguration();
        INDEXER_FOLLOWER_MOTOR_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(INDEXER_FOLLOWER_INVERTED);

        // Configure Shooter Leader Motor
        SHOOTER_LEADER_MOTOR_CONFIG.can_id = SHOOTER_LEADER_ID;
        SHOOTER_LEADER_MOTOR_CONFIG.motor_type = FxMotorType.X60;
        SHOOTER_LEADER_MOTOR_CONFIG.canbus_name = "rio";
        SHOOTER_LEADER_MOTOR_CONFIG.config = new TalonFXConfiguration();
        SHOOTER_LEADER_MOTOR_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(FLYWHEEL_LEADER_INVERTED);
        SHOOTER_LEADER_MOTOR_CONFIG.config.Slot1 = FLYWHEEL_VELOCITY_GAINS;
        SHOOTER_LEADER_MOTOR_CONFIG.config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        SHOOTER_LEADER_MOTOR_CONFIG.config.Voltage.PeakReverseVoltage = 0.0;
        // MotionMagic tuned for ~3 second ramp to 300 rad/s (47.75 rot/s)
        // Acceleration: 16 rot/s² (47.75 / 3)
        // Jerk: 80 rot/s³ (5x acceleration for smooth motion)
        SHOOTER_LEADER_MOTOR_CONFIG.config.MotionMagic.MotionMagicAcceleration = 16.0;
        SHOOTER_LEADER_MOTOR_CONFIG.config.MotionMagic.MotionMagicJerk = 80.0;
        // SHOOTER_LEADER_MOTOR_CONFIG.config.CurrentLimits.StatorCurrentLimit = 120.0;
        // SHOOTER_LEADER_MOTOR_CONFIG.config.CurrentLimits.StatorCurrentLimitEnable = true;
        // SHOOTER_LEADER_MOTOR_CONFIG.config.CurrentLimits.SupplyCurrentLimit = 70.0;
        // SHOOTER_LEADER_MOTOR_CONFIG.config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Configure Shooter Follower 1 Motor
        SHOOTER_FOLLOWER_MOTOR_1_CONFIG.can_id = SHOOTER_FOLLOWER_1_ID;
        SHOOTER_FOLLOWER_MOTOR_1_CONFIG.motor_type = FxMotorType.X60;
        SHOOTER_FOLLOWER_MOTOR_1_CONFIG.canbus_name = "rio";
        SHOOTER_FOLLOWER_MOTOR_1_CONFIG.config = new TalonFXConfiguration();
        SHOOTER_FOLLOWER_MOTOR_1_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(FLYWHEEL_FOLLOWER_1_INVERTED);
        SHOOTER_FOLLOWER_MOTOR_1_CONFIG.config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        SHOOTER_FOLLOWER_MOTOR_1_CONFIG.config.Voltage.PeakReverseVoltage = 0.0;
        // SHOOTER_FOLLOWER_MOTOR_1_CONFIG.config.CurrentLimits.StatorCurrentLimit = 120.0;
        // SHOOTER_FOLLOWER_MOTOR_1_CONFIG.config.CurrentLimits.StatorCurrentLimitEnable = true;
        // SHOOTER_FOLLOWER_MOTOR_1_CONFIG.config.CurrentLimits.SupplyCurrentLimit = 70.0;
        // SHOOTER_FOLLOWER_MOTOR_1_CONFIG.config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Configure Shooter Follower 2 Motor
        SHOOTER_FOLLOWER_MOTOR_2_CONFIG.can_id = SHOOTER_FOLLOWER_2_ID;
        SHOOTER_FOLLOWER_MOTOR_2_CONFIG.motor_type = FxMotorType.X60;
        SHOOTER_FOLLOWER_MOTOR_2_CONFIG.canbus_name = "rio";
        SHOOTER_FOLLOWER_MOTOR_2_CONFIG.config = new TalonFXConfiguration();
        SHOOTER_FOLLOWER_MOTOR_2_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(FLYWHEEL_FOLLOWER_2_INVERTED);
        SHOOTER_FOLLOWER_MOTOR_2_CONFIG.config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        SHOOTER_FOLLOWER_MOTOR_2_CONFIG.config.Voltage.PeakReverseVoltage = 0.0;
        // SHOOTER_FOLLOWER_MOTOR_2_CONFIG.config.CurrentLimits.StatorCurrentLimit = 120.0;
        // SHOOTER_FOLLOWER_MOTOR_2_CONFIG.config.CurrentLimits.StatorCurrentLimitEnable = true;
        // SHOOTER_FOLLOWER_MOTOR_2_CONFIG.config.CurrentLimits.SupplyCurrentLimit = 70.0;
        // SHOOTER_FOLLOWER_MOTOR_2_CONFIG.config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Configure Shooter Follower 3 Motor
        SHOOTER_FOLLOWER_MOTOR_3_CONFIG.can_id = SHOOTER_FOLLOWER_3_ID;
        SHOOTER_FOLLOWER_MOTOR_3_CONFIG.motor_type = FxMotorType.X60;
        SHOOTER_FOLLOWER_MOTOR_3_CONFIG.canbus_name = "rio";
        SHOOTER_FOLLOWER_MOTOR_3_CONFIG.config = new TalonFXConfiguration();
        SHOOTER_FOLLOWER_MOTOR_3_CONFIG.config.MotorOutput.Inverted =
                PhoenixUtil.toInvertedValue(FLYWHEEL_FOLLOWER_3_INVERTED);
        SHOOTER_FOLLOWER_MOTOR_3_CONFIG.config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        SHOOTER_FOLLOWER_MOTOR_3_CONFIG.config.Voltage.PeakReverseVoltage = 0.0;
        // SHOOTER_FOLLOWER_MOTOR_3_CONFIG.config.CurrentLimits.StatorCurrentLimit = 120.0;
        // SHOOTER_FOLLOWER_MOTOR_3_CONFIG.config.CurrentLimits.StatorCurrentLimitEnable = true;
        // SHOOTER_FOLLOWER_MOTOR_3_CONFIG.config.CurrentLimits.SupplyCurrentLimit = 70.0;
        // SHOOTER_FOLLOWER_MOTOR_3_CONFIG.config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Configure Hood Motor
        HOOD_MOTOR_CONFIGS.can_id = HOOD_ID;
        HOOD_MOTOR_CONFIGS.motor_type = FxMotorType.X44;
        HOOD_MOTOR_CONFIGS.canbus_name = "rio";
        HOOD_MOTOR_CONFIGS.config = new TalonFXConfiguration();
        HOOD_MOTOR_CONFIGS.config.MotorOutput.Inverted = PhoenixUtil.toInvertedValue(HOOD_INVERTED);
        HOOD_MOTOR_CONFIGS.config.Slot0 = HOOD_POSITION_GAINS;
        HOOD_MOTOR_CONFIGS.config.CurrentLimits.SupplyCurrentLimit = 10;
        HOOD_MOTOR_CONFIGS.config.CurrentLimits.SupplyCurrentLimitEnable = true;
    }
}
