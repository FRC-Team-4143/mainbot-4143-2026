package frc.mw_lib.simulation;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import java.util.List;
import org.ironmaple.simulation.SimulatedArena;

/**
 *
 *
 * <h1>The playing field for the 2025 FRC Game: Reefscape</h1>
 *
 * <p>This class represents the playing field for the 2025 FRC game, Reefscape.
 *
 * <p>It extends {@link SimulatedArena} and includes specific details of the Reefscape game environment.
 */
public class ArenaEvergreen extends SimulatedArena {
    public static final class EvergreenObstacleMap extends FieldMap {


        private static final double FIELD_LENGTH_METERS = Units.feetToMeters(54) + Units.inchesToMeters(1);
        private static final double FIELD_WIDTH_METERS = Units.feetToMeters(26) + Units.inchesToMeters(7);

        public EvergreenObstacleMap() {
            super();

            // blue wall
            super.addBorderLine(new Translation2d(0.0, 0.0), new Translation2d(0.0, FIELD_WIDTH_METERS));

            // red wall
            super.addBorderLine(new Translation2d(FIELD_LENGTH_METERS, 0.0), new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS));

            // left walls
            super.addBorderLine(new Translation2d(0.0, FIELD_WIDTH_METERS), new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS));

            // right walls
            super.addBorderLine(new Translation2d(0.0, 0.0), new Translation2d(FIELD_LENGTH_METERS, 0.0));

        }
    }

    public ArenaEvergreen() {
        super(new EvergreenObstacleMap());
    }

    @Override
    public void placeGamePiecesOnField() {
        // No pre-placed game pieces in Evergreen
    }

    @Override
    public synchronized List<Pose3d> getGamePiecesPosesByType(String type) {
        // No game pieces in on elements in Evergreen
        List<Pose3d> poses = super.getGamePiecesPosesByType(type);
        return poses;
    }

    @Override
    public synchronized void clearGamePieces() {
        super.clearGamePieces();
    }
}
