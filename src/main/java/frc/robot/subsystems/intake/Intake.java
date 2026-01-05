package frc.robot.subsystems.intake;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.mw_lib.subsystem.MwSubsystem;
import frc.mw_lib.subsystem.SubsystemIoBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;

public class Intake extends MwSubsystem<IntakeStates, IntakeConstants> {

  private static Intake instance_ = null;

  public static Intake getInstance() {
    if (instance_ == null) {
      instance_ = new Intake();
    }
    return instance_;
  }

  private IntakeIO io;

  /** Constructor for the Intake subsystem. */
  public Intake() {
    super(IntakeStates.IDLE, new IntakeConstants());

    // Figure out what I/O Container to use
    if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
      this.io = new IntakeIOReal(CONSTANTS);
    } else {
      this.io = new IntakeIOSim(CONSTANTS);
    }

    this.system_state_ = IntakeStates.IDLE;
  }

  @Override
  public void updateLogic(double timestamp) {
    switch (system_state_) {
      case DEPLOY:
        io.pivot_target_angle = Rotation2d.fromDegrees(0.0);
        io.roller_target_output = 0.5;
        break;
      case PURGE:
        io.pivot_target_angle = Rotation2d.fromDegrees(0.0);
        io.roller_target_output = 0.5;
        break;
      case CLIMB_STAGE:
        io.pivot_target_angle = Rotation2d.fromDegrees(0.0);
        io.roller_target_output = 0.5;
        break;
      case PICK_UP:
        io.pivot_target_angle = Rotation2d.fromDegrees(0.0);
        io.roller_target_output = 0.5;
        break;
      case IDLE:
      default:
        io.pivot_target_angle = Rotation2d.fromDegrees(0.0);
        io.roller_target_output = 0.5;
        break;
    }
  }

  @Override
  public void reset() {}

  @Override
  public List<SubsystemIoBase> getIos() {
    return Arrays.asList(io);
  }
}
