package frc.mw_lib.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** This class models a region of the field. Credit to frc-3061 for base code */
public abstract class Region {
  /**
   * Log the points of the region. These can be visualized using AdvantageScope to confirm that the
   * regions are properly defined.
   */
  public abstract void logPoints();

  /** Returns true if the region contains a given Pose2d. */
  public abstract boolean contains(Pose2d other);

  public abstract String getName();

  public abstract void constructRegion();

  public boolean willContain(Pose2d robotPose, ChassisSpeeds robotSpeed, double time){
    return contains(robotPose.exp(robotSpeed.toTwist2d(time)));
  }
}
