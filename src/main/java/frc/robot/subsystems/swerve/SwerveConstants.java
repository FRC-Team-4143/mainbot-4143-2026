package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.marswars.mechanisms.MotorConfig.TalonMotorType;
import com.marswars.swerve_lib.MwSwerveConstants;
import com.marswars.swerve_lib.SwerveDriveConfig;
import com.marswars.swerve_lib.module.ModuleType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotId;

/**
 * Swerve constants. The base class is AlphaBot; every other robot is a subclass whose constructor
 * starts from {@link #alphaHardware()} and changes only what differs. All fields are final.
 */
public class SwerveConstants extends MwSwerveConstants {

    /** Builds the constants variant for the robot the code is running on. */
    public static SwerveConstants create() {
        return switch (RobotId.current()) {
            case BETA_BOT -> new BetaSwerveConstants();
            case SIM_BOT -> new SimSwerveConstants();
            default -> new SwerveConstants();
        };
    }

    // =============================================================================
    // PHYSICAL ROBOT CONSTANTS
    // =============================================================================

    public final double ROBOT_MASS_KG = Units.lbsToKilograms(140.0); // Mass of the robot in pounds
    public final double BUMPER_WIDTH_METERS =
            Units.inchesToMeters(34.75); // Width of the bumpers in meters (y axis : left -> right)
    public final double BUMPER_LENGTH_METERS =
            Units.inchesToMeters(34.75); // Length of the bumpers in meters (x axis : front -> back)
    public final double BUMPER_THICKNESS_METERS =
            Units.inchesToMeters(3.0); // Thickness of the bumpers in meters

    // =============================================================================
    // MOTOR AND MECHANICAL CONSTANTS
    // =============================================================================

    public final double DRIVE_INERTIA = 0.025; // kg*m^2, inertia of the drive motor
    public final double STEER_INERTIA = 0.004; // kg*m^2, inertia of the steer motor
    public final double DRIVE_FRICTION_VOLTAGE = 0.2;
    public final double STEER_FRICTION_VOLTAGE = 0.2;

    public final double SLIP_CURRENT_AMPS = 50.0;
    public final double COUPLE_RATIO = 3.5;

    // =============================================================================
    // CONSTRUCTORS
    // =============================================================================
    // Control, Choreo and tractor beam tuning is inherited from MwSwerveConstants; the library
    // defaults are this robot's values. Pass a MwSwerveConstants.Tuning to override them.

    /** AlphaBot. */
    public SwerveConstants() {
        this(alphaHardware());
    }

    /** For robot variants: {@link #alphaHardware()} with that robot's differences applied. */
    protected SwerveConstants(SwerveDriveConfig.Builder hardware) {
        super(hardware.build());
    }

    /** For robot variants that also change the control tuning. */
    protected SwerveConstants(SwerveDriveConfig.Builder hardware, Tuning tuning) {
        super(hardware.build(), tuning);
    }

    /** AlphaBot's drivetrain hardware. Variants start from this and change only what differs. */
    protected static SwerveDriveConfig.Builder alphaHardware() {
        return SwerveDriveConfig.builder()
                .canbus("CANivore")
                .pigeon2Id(0)
                .moduleType(ModuleType.getModuleType("TSN-P13-S18"))
                .wheelRadius(Units.inchesToMeters(1.8))
                .speedAt12V(5.0)
                .driveGains(
                        new Slot0Configs(),
                        new Slot1Configs()
                                .withKS(0.2)
                                .withKV(0.1)
                                .withKA(0.02)
                                .withKG(0.3)
                                .withKP(8.0))
                .driveCurrentLimits(40.0, 40.0)
                .steerMotor(TalonMotorType.X44)
                .steerGains(new Slot0Configs().withKP(80.0), new Slot1Configs())
                .steerCurrentLimits(40.0, 40.0)
                // Module CAN IDs (drive, steer, encoder) and positions are identical on every robot
                .frontLeft(1, 2, 0, moduleLocation(7.5, 7.5))
                .frontRight(3, 4, 1, moduleLocation(7.5, -7.5))
                .backLeft(5, 6, 2, moduleLocation(-7.5, 7.5))
                .backRight(7, 8, 3, moduleLocation(-7.5, -7.5));
    }

    private static Translation2d moduleLocation(double x_inches, double y_inches) {
        return new Translation2d(Units.inchesToMeters(x_inches), Units.inchesToMeters(y_inches));
    }
}
