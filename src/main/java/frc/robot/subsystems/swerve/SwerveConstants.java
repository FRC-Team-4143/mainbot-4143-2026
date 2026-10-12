package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.marswars.mechanisms.MotorConfig.TalonMotorType;
import com.marswars.swerve_lib.MwSwerveConstants;
import com.marswars.swerve_lib.SwerveDriveConfig;
import com.marswars.swerve_lib.module.ModuleType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotConstants;

public class SwerveConstants extends MwSwerveConstants {

    /** Builds the constants variant for the robot the code is running on. */
    public static SwerveConstants create() {
        return switch (RobotConstants.ROBOT) {
            case BETA_BOT -> new BetaSwerveConstants();
            case SIM_BOT -> new SimSwerveConstants();
            default -> new SwerveConstants();
        };
    }

    // =============================================================================
    // PER-ROBOT CONSTANTS (defaults = AlphaBot)
    // =============================================================================
    // Non-final so robot variants (BetaSwerveConstants, SimSwerveConstants) can reassign them in
    // their constructors. The inherited MwSwerveConstants tunables (MAX_TRANSLATION_RATE,
    // CHOREO_*, TRACTOR_BEAM_*, ...) default to this robot's values and work the same way.

    public String CANBUS_NAME = "CANivore";
    public double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.8);
    public ModuleType MODULE_TYPE = ModuleType.getModuleType("TSN-P13-S18");
    public TalonMotorType STEER_MOTOR_TYPE = TalonMotorType.X44;
    public Slot0Configs DRIVE_GAINS_SLOT0 = new Slot0Configs();
    public Slot1Configs DRIVE_GAINS_SLOT1 =
            new Slot1Configs().withKS(0.2).withKV(0.1).withKA(0.02).withKG(0.3).withKP(8.0);
    public Slot0Configs STEER_GAINS_SLOT0 = new Slot0Configs().withKP(80.0);
    public Slot1Configs STEER_GAINS_SLOT1 = new Slot1Configs();
    public double DRIVE_SUPPLY_CURRENT_LIMIT = 40.0; // Amps
    public double DRIVE_STATOR_CURRENT_LIMIT = 40.0; // Amps
    public double STEER_SUPPLY_CURRENT_LIMIT = 40.0; // Amps
    public double STEER_STATOR_CURRENT_LIMIT = 40.0; // Amps

    // =============================================================================
    // HARDWARE CONSTANTS (identical on every robot)
    // =============================================================================

    public final int PIGEON2_ID = 0;
    public final double SPEED_AT_12V_MPS = 5.0;

    // =============================================================================
    // DRIVETRAIN HARDWARE
    // =============================================================================

    /** Called once by the subsystem, after any variant constructor has run. */
    @Override
    public SwerveDriveConfig getDriveConfig() {
        // Module CAN IDs (drive, steer, encoder) and positions are identical on every robot
        return SwerveDriveConfig.builder()
                .canbus(CANBUS_NAME)
                .pigeon2Id(PIGEON2_ID)
                .moduleType(MODULE_TYPE)
                .wheelRadius(WHEEL_RADIUS_METERS)
                .speedAt12V(SPEED_AT_12V_MPS)
                .driveGains(DRIVE_GAINS_SLOT0, DRIVE_GAINS_SLOT1)
                .driveCurrentLimits(DRIVE_SUPPLY_CURRENT_LIMIT, DRIVE_STATOR_CURRENT_LIMIT)
                .steerMotor(STEER_MOTOR_TYPE)
                .steerGains(STEER_GAINS_SLOT0, STEER_GAINS_SLOT1)
                .steerCurrentLimits(STEER_SUPPLY_CURRENT_LIMIT, STEER_STATOR_CURRENT_LIMIT)
                .frontLeft(1, 2, 0, inches(7.5, 7.5))
                .frontRight(3, 4, 1, inches(7.5, -7.5))
                .backLeft(5, 6, 2, inches(-7.5, 7.5))
                .backRight(7, 8, 3, inches(-7.5, -7.5))
                .build();
    }

    private static Translation2d inches(double x, double y) {
        return new Translation2d(Units.inchesToMeters(x), Units.inchesToMeters(y));
    }
}
