package frc.robot.subsystems.simulation;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.marswars.proxy_server.ProxyServerThread;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import com.marswars.swerve_lib.SwerveMeasurements.SwerveMeasurement;
import com.marswars.vision.MwVisionSim;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.lib2026.FieldRegions;
import frc.robot.lib2026.FuelSim;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.simulation.SimulationConstants.SimulationStates;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

public class SimulationSubsystem extends MwSubsystem<SimulationStates, SimulationConstants> {
    private static SimulationSubsystem instance_ = null;

    private MwVisionSim vision_sim_;
    private int hopper_fuel_count_ = 0;
    private boolean outpost_full = false;
    private final Random noise_generator_ = new Random();
    private double last_shot_timestamp_ = 0.0;

    // getInstance
    public static SimulationSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new SimulationSubsystem();
        }
        return instance_;
    }

    // Constructor
    public SimulationSubsystem() {
        super(SimulationStates.ACTIVE, new SimulationConstants());

        if (CONSTANTS.SIM_VISION_ENABLED) {
            vision_sim_ =
                    ProxyServerThread.getInstance()
                            .initializeVisionSimulation(
                                    LocalizationSubsystem.getInstance().getAprilTagLayout());
            vision_sim_.addCamera("Back-Camera", CONSTANTS.BACK_CAMERA_TRANSFORM);
            vision_sim_.addCamera("Left-Camera", CONSTANTS.LEFT_CAMERA_TRANSFORM);
            vision_sim_.addCamera("Right-Camera", CONSTANTS.RIGHT_CAMERA_TRANSFORM);
            LocalizationSubsystem.getInstance().enableSwerveMeasurementNoise();
        }

        // Setup Fuel Simulation
        FuelSim.getInstance()
                .registerRobot(
                        CONSTANTS.BASE_WIDTH,
                        CONSTANTS.BASE_LENGTH,
                        CONSTANTS.BUMPER_HEIGHT,
                        (CONSTANTS.SIM_VISION_ENABLED)
                                ? LocalizationSubsystem.getInstance()::getSmoothPose
                                : LocalizationSubsystem.getInstance()::getFieldPose,
                        LocalizationSubsystem.getInstance()::getChassisSpeedsFieldRelative);
        FuelSim.getInstance().enableAirResistance();

        // Setup Intake Simulation - Out the Front of the Robot
        FuelSim.getInstance()
                .registerIntake(
                        // Front Frame
                        CONSTANTS.BASE_LENGTH / 2.0,
                        // Intake Extension Range
                        CONSTANTS.BASE_LENGTH / 2.0 + CONSTANTS.INTAKE_MAX_EXTENSION,
                        // Left Frame
                        -CONSTANTS.BASE_WIDTH / 2.0,
                        // Right Frame
                        CONSTANTS.BASE_WIDTH / 2.0,
                        () ->
                                hopper_fuel_count_ < CONSTANTS.HOPPER_CAPACITY
                                        && IntakeSubsystem.getInstance().getSystemState()
                                                == IntakeStates.INTAKE,
                        () -> hopper_fuel_count_++);

        // Start Fuel Simulation
        if (CONSTANTS.SIM_FUEL_ENABLED) FuelSim.getInstance().start();

        // Setup autonomous reset
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::resetForAuto));
        resetForAuto();
    }

    // reset
    @Override
    public void reset() {
        system_state_ = SimulationStates.ACTIVE;
    }

    // getIos
    @Override
    public List<SubsystemIoBase> getIos() {
        // This subsystem has no IOs
        return Arrays.asList();
    }

    // updateLogic
    @Override
    public void updateLogic(double timestamp) {
        // Vision Simulation
        if (CONSTANTS.SIM_VISION_ENABLED) {
            Pose2d robot_pose = LocalizationSubsystem.getInstance().getSmoothPose();
            ProxyServerThread.getInstance().updateVisionSimulation(robot_pose);
        }

        // Get Fuel from the Outpost
        if (DriverStation.isAutonomousEnabled()
                && FieldRegions.OUTPOST_REGION.contains(
                        LocalizationSubsystem.getInstance().getFieldPose())
                && outpost_full) {
            hopper_fuel_count_ =
                    Math.min(
                            hopper_fuel_count_ + 24,
                            CONSTANTS.HOPPER_CAPACITY); // Simulate picking up a full load of fuel
            // from the outpost
            outpost_full = false; // Simulate the outpost being emptied after one pickup
        }

        // FuelSim
        launchFuel(timestamp);
        FuelSim.getInstance().updateSim();
        DogLog.log(getSubsystemKey() + "FuelSim/Fuel", FuelSim.getInstance().getLoggableFuel());
        DogLog.log(getSubsystemKey() + "FuelSim/HopperCount", hopper_fuel_count_);
    }

    // =============================================================================
    // PUBLIC HELPER METHODS
    // =============================================================================

    /** Launches a fuel from the shooter in the simulation. */
    @SuppressWarnings("unused")
    public void launchFuel(double timestamp) {
        if (ShooterSubsystem.getInstance().getSystemState() == ShooterStates.SHOOT
                && hopper_fuel_count_ > 0
                && CONSTANTS.SIM_FUEL_ENABLED) {
            if (timestamp - last_shot_timestamp_ >= CONSTANTS.SECONDS_PER_SHOT) {
                FuelSim.getInstance()
                        .launchFuel(
                                MetersPerSecond.of(
                                        ShooterSubsystem.getInstance().getLaunchVelocity()),
                                Radians.of(ShooterSubsystem.getInstance().getLaunchAngle()),
                                Radians.of(
                                        LocalizationSubsystem.getInstance()
                                                .getFieldPose()
                                                .getRotation()
                                                .rotateBy(CONSTANTS.SHOOTER_LAUNCH_ROTATION)
                                                .getRadians()),
                                CONSTANTS.SHOOTER_LAUNCH_OFFSET);
                ShooterSubsystem.getInstance()
                        .applyLoadFromBall(
                                CONSTANTS.FUEL_MASS_KG
                                        * ShooterSubsystem.getInstance().getLaunchVelocity()
                                        * CONSTANTS.FLYWHEEL_RADIUS_M
                                        / CONSTANTS.CONTACT_TIME_SEC);
                hopper_fuel_count_--;
                last_shot_timestamp_ = timestamp;
            }
        }
    }

    /** Resets the simulation for autonomous mode. */
    public void resetForAuto() {
        // Move robot to starting pose
        LocalizationSubsystem.getInstance().resetPoseEstimatorAuto();

        // Reset fuel simulation
        hopper_fuel_count_ = 0;
        FuelSim.getInstance().clearFuel();
        FuelSim.getInstance().spawnStartingFuel();
    }

    /**
     * Adds Gaussian noise to a complete swerve measurement. Applies noise to both gyro and module
     * position measurements.
     *
     * @param clean_measurement The clean swerve measurement
     * @return Swerve measurement with added noise
     */
    public SwerveMeasurement addNoise(SwerveMeasurement clean_measurement) {
        SwerveMeasurement noisy_measurement = clean_measurement;
        noisy_measurement.gyro_yaw = addGyroNoise(clean_measurement.gyro_yaw);
        noisy_measurement.module_positions =
                addModulePositionNoise(clean_measurement.module_positions);

        return noisy_measurement;
    }

    // =============================================================================
    // PRIVATE HELPER METHODS
    // =============================================================================

    /**
     * Adds Gaussian noise to gyro measurement for simulation.
     *
     * @param clean_gyro The clean gyro measurement
     * @return Gyro measurement with added noise
     */
    private Rotation2d addGyroNoise(Rotation2d clean_gyro) {
        double noise = noise_generator_.nextGaussian() * CONSTANTS.GYRO_NOISE_STD_DEV;
        return clean_gyro.plus(Rotation2d.fromRadians(noise));
    }

    /**
     * Adds Gaussian noise to module positions for simulation.
     *
     * @param clean_positions The clean module position measurements
     * @return Module positions with added noise
     */
    private SwerveModulePosition[] addModulePositionNoise(SwerveModulePosition[] clean_positions) {
        SwerveModulePosition[] noisy_positions = new SwerveModulePosition[clean_positions.length];
        for (int i = 0; i < clean_positions.length; i++) {
            double distance_noise =
                    noise_generator_.nextGaussian() * CONSTANTS.MODULE_POSITION_NOISE_STD_DEV;
            noisy_positions[i] =
                    new SwerveModulePosition(
                            clean_positions[i].distanceMeters + distance_noise,
                            clean_positions[i].angle);
        }
        return noisy_positions;
    }
}
