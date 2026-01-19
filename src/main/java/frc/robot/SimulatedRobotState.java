package frc.robot;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;

public class SimulatedRobotState {
    private static SwerveDriveSimulation swerve_simulation_;

    public static void configure() {

        // Add the swerve drive simulation to the simulated arena
        swerve_simulation_ = SwerveSubsystem.getInstance().getSwerveSimulation();
        SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
        SimulatedArena.getInstance().addDriveTrainSimulation(swerve_simulation_);
        SimulatedArena.getInstance().resetFieldForAuto();
    }
}
