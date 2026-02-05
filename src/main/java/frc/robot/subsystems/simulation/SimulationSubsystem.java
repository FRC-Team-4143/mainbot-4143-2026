package frc.robot.subsystems.simulation;

import com.marswars.auto.AutoManager;
import com.marswars.geometry.AllianceFlipUtil;
import com.marswars.geometry.LaunchTrajectory.TrajectorySol;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.lib2026.FuelSim;
import frc.robot.subsystems.localization.LocalizationConstants.LocalizationStates;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterStates;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.simulation.SimulationConstants.SimulationStates;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Random;

public class SimulationSubsystem extends MwSubsystem<SimulationStates, SimulationConstants> {
    private static SimulationSubsystem instance_ = null;

    public static SimulationSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new SimulationSubsystem();
        }
        return instance_;
    }

    private MwVisionSim vision_sim_;
    private int hopper_fuel_count_ = 0;
    private final Random noise_generator_ = new Random();
    private double last_shot_timestamp_ = 0.0;

    public SimulationSubsystem() {
        super(SimulationStates.ACTIVE, new SimulationConstants());

        if (CONSTANTS.SIM_VISION_ENABLED) {
            vision_sim_ =
                    ProxyServerThread.getInstance()
                            .initializeVisionSimulation(
                                    LocalizationSubsystem.getInstance().getAprilTagLayout());
            vision_sim_.addDefaultCameras();
            LocalizationSubsystem.getInstance().setWantedState(LocalizationStates.VISION_SIM);
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

        // Setup Intake Similation
        FuelSim.getInstance()
                .registerIntake(
                        -CONSTANTS.BASE_LENGTH / 2.0 - CONSTANTS.INTAKE_MAX_EXTENSION,
                        -CONSTANTS.BASE_LENGTH / 2.0,
                        -CONSTANTS.BASE_WIDTH / 2.0,
                        CONSTANTS.BASE_WIDTH / 2.0,
                        () -> hopper_fuel_count_ < CONSTANTS.HOPPER_CAPACITY && IntakeSubsystem.getInstance().getSystemState() == IntakeStates.ROLLING,
                        () -> hopper_fuel_count_++);

        // Start Fuel Simulation
        if (CONSTANTS.SIM_FUEL_ENABLED) FuelSim.getInstance().start();

        // Setup autonomous reset
        RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::resetForAuto));
        resetForAuto();
    }

    // @Override
    // public void handleStateTransition(SimulationStates wanted) {
    // }

    @Override
    public void updateLogic(double timestamp) {
        // Vision Simulation
        if (CONSTANTS.SIM_VISION_ENABLED) {
            Pose2d robot_pose = LocalizationSubsystem.getInstance().getSmoothPose();
            ProxyServerThread.getInstance().updateVisionSimulation(robot_pose);
        }

        if(ShooterSubsystem.getInstance().getSystemState() == ShooterStates.SHOOT && hopper_fuel_count_ > 0 && CONSTANTS.SIM_FUEL_ENABLED){
            // Rate limit shooting to 15 balls per second
            if (timestamp - last_shot_timestamp_ >= CONSTANTS.SECONDS_PER_SHOT) {
                launchFuel();
                last_shot_timestamp_ = timestamp;
            }
        }

        // FuelSim
        FuelSim.getInstance().updateSim();
        DogLog.log(getSubsystemKey() + "FuelSim/Fuel", FuelSim.getInstance().getLoggableFuel());
        DogLog.log(getSubsystemKey() + "FuelSim/HopperCount", hopper_fuel_count_);
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        // This subsystem has no IOs
        return Arrays.asList();
    }

    @Override
    public void reset() {
        system_state_ = SimulationStates.ACTIVE;
    }

    /** Launches a fuel from the shooter in the simulation. */
    public void launchFuel(){
        TrajectorySol solution = ShooterSubsystem.getInstance().getCurrentSolution();

        // If the current solution is not valid, do not try to launch fuel!!!
        // This should never happen during normal operation, but could happen during testing
        if(!solution.valid){
            return;
        }

        FuelSim.getInstance().launchFuel(
            MetersPerSecond.of(solution.velocity), 
            Radians.of(solution.exit_angle),
            Radians.of(solution.heading_angle), 
            Meters.of(CONSTANTS.SHOOTER_LAUNCH_HEIGHT));
        hopper_fuel_count_--;
    }

    /** Resets the simulation for autonomous mode. */
    public void resetForAuto() {
        // Move robot to starting pose
        Pose2d start_pose = AutoManager.getInstance().getSelectedAuto().getStartPose();
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            start_pose = AllianceFlipUtil.apply(start_pose);
        }
        LocalizationSubsystem.getInstance().resetPoseEstimator(start_pose);

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
