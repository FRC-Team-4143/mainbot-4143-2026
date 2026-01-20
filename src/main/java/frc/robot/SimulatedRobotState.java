package frc.robot;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import dev.doglog.DogLog;

public class SimulatedRobotState {
    private static SwerveDriveSimulation swerve_simulation_;

    /**
     * Configures the simulated robot state by adding the swerve drive simulation to the simulated
     */
    public static void configure() {
        // Add the swerve drive simulation to the simulated arena
        swerve_simulation_ = SwerveSubsystem.getInstance().getSwerveSimulation();
        SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
        SimulatedArena.getInstance().addDriveTrainSimulation(swerve_simulation_);
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    /**
     * Updates the simulated robot state by logging the robot pose and fuel positions to DogLog.
     */
    public static void update(){
        SimulatedArena.getInstance().simulationPeriodic();
        DogLog.log("FieldSimulation/RobotPose",
                swerve_simulation_.getSimulatedDriveTrainPose());
        DogLog.log("FieldSimulation/Fuel",
                SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel"));
    }
}
