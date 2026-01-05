package frc.robot.subsystems.swerve;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.mw_lib.subsystem.SubsystemIO;
import frc.mw_lib.swerve_lib.ChassisRequest;
import frc.mw_lib.swerve_lib.ChassisRequest.ChassisRequestParameters;
import frc.mw_lib.swerve_lib.module.Module;

public abstract class SwerveIO extends SubsystemIO<SwerveConstants> {

  public SwerveIO(SwerveConstants constants) {
    super(constants);
  }

  public SwerveModuleState[] module_states =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
      };
  public SwerveModulePosition[] module_positions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  public SwerveModulePosition[] module_deltas =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  public SwerveModulePosition[] last_module_positions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  public ChassisSpeeds chassis_speeds = new ChassisSpeeds();
  public Rotation2d raw_gyro_rotation = Rotation2d.kZero;
  public Pose2d pose = new Pose2d();

  public ChassisRequest current_request = new ChassisRequest.Idle();
  public ChassisRequestParameters current_request_parameters = new ChassisRequestParameters();

  /** Logs data to DogLog.  */
  @Override
  public void logData(){
    DogLog.log(getSubsystemKey() + "ModuleStates", module_states);
    DogLog.log(getSubsystemKey() + "ModulePositions", module_positions);
    DogLog.log(getSubsystemKey() + "ModuleDeltas", module_deltas);
    DogLog.log(getSubsystemKey() + "LastModulePositions", last_module_positions);
    DogLog.log(getSubsystemKey() + "ChassisSpeeds", chassis_speeds);
    DogLog.log(getSubsystemKey() + "RawGyroRotation", raw_gyro_rotation);
    DogLog.log(getSubsystemKey() + "Pose", pose);
    DogLog.log(getSubsystemKey() + "CurrentRequestType", current_request.getClass().getSimpleName());
  }

  void resetPose(Pose2d pose) {}

  Module[] getModules() {
    return new Module[4];
  }
}
