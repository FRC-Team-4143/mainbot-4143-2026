package frc.robot.subsystems.swerve;

import dev.doglog.DogLog;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import frc.mw_lib.swerve_lib.gyro.Gyro;
import frc.mw_lib.swerve_lib.gyro.GyroIOSim;
import frc.mw_lib.swerve_lib.module.Module;
import frc.mw_lib.swerve_lib.module.ModuleIOTalonFXSim;
import frc.robot.Constants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class SwerveIOSim extends SwerveIO {

  private final Pose2d SIM_START_POSE = new Pose2d(3, 3, Rotation2d.kZero);

  private final Module[] modules_ = new Module[4]; // FL, FR, BL, BR
  private final Gyro gyro_;

  private final SwerveDrivePoseEstimator pose_estimator_;
  private final SwerveDriveKinematics kinematics_;

  SwerveIOSim(SwerveConstants constants) {
    super(constants);
    // Configure MapleSim
    Constants.SWERVE_SIMULATOR = new SwerveDriveSimulation(CONSTANTS.MAPLE_SIM_DRIVETRAIN_CONFIG, SIM_START_POSE);
    // Configure Gyro
    gyro_ = new Gyro(new GyroIOSim(Constants.SWERVE_SIMULATOR.getGyroSimulation()));
    kinematics_ = new SwerveDriveKinematics(
      new Translation2d[] {
      CONSTANTS.FL_MODULE_TRANSLATION,
      CONSTANTS.FR_MODULE_TRANSLATION,
      CONSTANTS.BL_MODULE_TRANSLATION,
      CONSTANTS.BR_MODULE_TRANSLATION,
      });
    // Configure Modules
    modules_[0] =
        new Module(
            new ModuleIOTalonFXSim(
                CONSTANTS.SIM_FL_MODULE_CONSTANTS, CONSTANTS.MODULE_CANBUS_NAME, Constants.SWERVE_SIMULATOR.getModules()[0]),
            0,
            CONSTANTS.SIM_FL_MODULE_CONSTANTS);
    modules_[1] =
        new Module(
            new ModuleIOTalonFXSim(
                CONSTANTS.SIM_FR_MODULE_CONSTANTS, CONSTANTS.MODULE_CANBUS_NAME, Constants.SWERVE_SIMULATOR.getModules()[1]),
            1,
            CONSTANTS.SIM_FR_MODULE_CONSTANTS);
    modules_[2] =
        new Module(
            new ModuleIOTalonFXSim(
                CONSTANTS.SIM_BL_MODULE_CONSTANTS, CONSTANTS.MODULE_CANBUS_NAME, Constants.SWERVE_SIMULATOR.getModules()[2]),
            2,
            CONSTANTS.SIM_BL_MODULE_CONSTANTS);
    modules_[3] =
        new Module(
            new ModuleIOTalonFXSim(
                CONSTANTS.SIM_BR_MODULE_CONSTANTS, CONSTANTS.MODULE_CANBUS_NAME, Constants.SWERVE_SIMULATOR.getModules()[3]),
            3,
            CONSTANTS.SIM_BR_MODULE_CONSTANTS);
    // Configure Pose Estimator
    pose_estimator_ =
        new SwerveDrivePoseEstimator(
            kinematics_,
            new Rotation2d(),
            new SwerveModulePosition[] {
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
            },
            SIM_START_POSE);
    // Setup MapleSim Drive Train Simulation
    SimulatedArena.getInstance().addDriveTrainSimulation(Constants.SWERVE_SIMULATOR);
  }

  @Override
  public void readInputs(double timestamp) {
    Swerve.odometry_lock_.lock(); // Prevents odometry updates while reading data
    for (var module : modules_) {
      module.periodic();
    }
    gyro_.periodic();
    Swerve.odometry_lock_.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules_) {
        module.stop();
      }
    }

    // Update odometry
    double[] sample_timestamps =
        modules_[0].getOdometryTimestamps(); // All signals are sampled together
    int sample_count = sample_timestamps.length;
    for (int i = 0; i < sample_count; i++) {
      // Read wheel positions and deltas from each module
      for (int module_index = 0; module_index < 4; module_index++) {
        module_positions[module_index] = modules_[module_index].getOdometryPositions()[i];
        module_deltas[module_index] =
            new SwerveModulePosition(
                module_positions[module_index].distanceMeters
                    - last_module_positions[module_index].distanceMeters,
                module_positions[module_index].angle);
        last_module_positions[module_index] = module_positions[module_index];
        module_states[module_index] = modules_[module_index].getState();
      }
      chassis_speeds = kinematics_.toChassisSpeeds(module_states);

      // Update gyro angle
      if (gyro_.isConnected()) {
        // Use the real gyro angle
        raw_gyro_rotation = gyro_.getOdometryYawPositions()[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics_.toTwist2d(module_deltas);
        raw_gyro_rotation = raw_gyro_rotation.plus(new Rotation2d(twist.dtheta));
      }
      pose_estimator_.updateWithTime(sample_timestamps[i], raw_gyro_rotation, module_positions);
    }
    pose = pose_estimator_.getEstimatedPosition();

    // Update Simulated Position
    DogLog.log(
        "FieldSimulation/RobotPosition", Constants.SWERVE_SIMULATOR.getSimulatedDriveTrainPose());
  }

  public void writeOutputs(double timestamp) {
    current_request_parameters.kinematics = kinematics_;
    current_request_parameters.moduleLocations = getModuleTranslations();
    current_request.apply(current_request_parameters, modules_);
  }

  public void resetPose(Pose2d pose) {
    Constants.SWERVE_SIMULATOR.setSimulationWorldPose(pose);
    pose_estimator_.resetPosition(pose.getRotation(), new SwerveModulePosition[4], pose);
  }

  public Module[] getModules() {
    return modules_;
  }

  private Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      modules_[0].getTranslation(), // FL
      modules_[1].getTranslation(), // FR
      modules_[2].getTranslation(), // BL
      modules_[3].getTranslation() // BR
    };
  }
}
