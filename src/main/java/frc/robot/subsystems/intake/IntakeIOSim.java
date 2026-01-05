package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import frc.robot.Constants;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;

public class IntakeIOSim extends IntakeIO {

  private final IntakeSimulation INTAKE_SIMULATOR;

  private final double INTAKE_WIDTH = 0.438150; // meters
  private final double LENGTH_EXTENDED = 0.281353; // meters

  public IntakeIOSim(IntakeConstants constants) {
    super(constants);

    INTAKE_SIMULATOR =
        IntakeSimulation.OverTheBumperIntake(
            "Coral",
            Constants.SWERVE_SIMULATOR,
            Meters.of(INTAKE_WIDTH),
            Meters.of(LENGTH_EXTENDED),
            IntakeSide.BACK,
            1);
  }

  @Override
  public void readInputs(double timestamp) {
    tof_dist =
        (INTAKE_SIMULATOR.getGamePiecesAmount() > 0)
            ? CONSTANTS.TOF_CORAL_DISTANCE
            : 1000.0; // Simulated Time of Flight sensor distance
  }

  @Override
  public void writeOutputs(double timestamp) {
    if (roller_target_output > 0) {
      INTAKE_SIMULATOR.startIntake();
    } else if (roller_target_output < 0) {
      INTAKE_SIMULATOR.obtainGamePieceFromIntake();
    } else {
      INTAKE_SIMULATOR.stopIntake();
    }
  }
}
