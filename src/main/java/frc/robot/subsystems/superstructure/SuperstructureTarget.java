package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;

public class SuperstructureTarget {

  public enum Targets {
    INTAKE_CLEAR("INTAKE_CLEAR", 0.90041, Rotation2d.fromDegrees(-45)),
    CLIMB("CLIMB", 0.90041, Rotation2d.fromDegrees(121.201)),
    CORAL_INTAKE("CORAL_INTAKE", 0.75, Rotation2d.fromDegrees(-117)),
    L4("L4", 2.109, Rotation2d.fromDegrees(-19)),
    L4_STAGING("L4_STAGING", 2.109, Rotation2d.fromDegrees(-65)),
    L3("L3", 1.323265, Rotation2d.fromDegrees(-5.05468)),
    L3_FAR("L3_FAR", 1.423500, Rotation2d.fromDegrees(-4.515)),
    L2("L2", 0.94895, Rotation2d.fromDegrees(-3.6035)),
    L2_FAR("L2_FAR", 1.0235, Rotation2d.fromDegrees(-4.515)),
    L1("L1", 0.824, Rotation2d.fromDegrees(-23.703)),
    L1_STAGING("L1_STAGING", 0.824, Rotation2d.fromDegrees(0)),
    L1_FLICK("L1_FLICK", 0.919, Rotation2d.fromDegrees(0)),
    ALGAE_STOW("ALGAE_STOW", 0.90769, Rotation2d.fromDegrees(-12.9199)),
    ALGAE_LOW("ALGAE_LOW", 0.90848, Rotation2d.fromDegrees(-12.9199)),
    ALGAE_HIGH("ALGAE_HIGH", 1.35857, Rotation2d.fromDegrees(-12.9199)),
    ALGAE_PROCESSOR("ALGAE_PROCESSOR", 0.7829, Rotation2d.fromDegrees(-40.1660)),
    BARGE("BARGE", 2.08, Rotation2d.fromDegrees(23.3789));

    public final SuperstructureTarget target;

    private Targets(String tgt_name, double elevator_height, Rotation2d arm_angle) {
      target = new SuperstructureTarget(tgt_name, elevator_height, arm_angle);
    }
  }

  private final double elevator_height_;
  private double elevator_offset_ = 0;
  private final Rotation2d arm_angle_;
  private Rotation2d arm_offset_ = Rotation2d.kZero;
  private final String name;

  SuperstructureTarget(String tgt_name, double elevator_height, Rotation2d arm_angle) {
    elevator_height_ = elevator_height;
    arm_angle_ = arm_angle;
    name = tgt_name;
  }

  // Height Methods
  /**
   * Returns the current height including the active offset
   *
   * @return
   */
  public double getHeight() {
    return elevator_height_ + elevator_offset_;
  }

  /**
   * Adjusts the height offset by the supplied increment
   *
   * @param offset
   */
  public void offsetHeight(double offset) {
    elevator_offset_ += offset;
  }

  /**
   * Returns the current stored angle offset
   *
   * @return
   */
  public double getHeightOffset() {
    return elevator_offset_;
  }

  /** Reset the height offset to 0 */
  public void resetHeightOffset() {
    elevator_offset_ = 0;
  }

  // Angle Methods
  /**
   * Returns the current angle including the active offset
   *
   * @return target angle
   */
  public Rotation2d getAngle() {
    return arm_angle_.rotateBy(arm_offset_);
  }

  /**
   * Adjusts the angle offset by the supplied increment
   *
   * @param offset
   */
  public void offsetAngle(Rotation2d offset) {
    arm_offset_ = arm_offset_.rotateBy(offset);
  }

  public String getName() {
    return name;
  }

  /**
   * Returns the current stored angle offset
   *
   * @return
   */
  public Rotation2d getAngleOffset() {
    return arm_offset_;
  }

  /** Resets the angle offset to 0 */
  public void resetAngleOffset() {
    arm_offset_ = new Rotation2d();
  }

  /** Resets both height and angle offsets */
  public void resetOffsets() {
    resetAngleOffset();
    resetHeightOffset();
  }

  @Override
  public boolean equals(Object other){
    if (other == null) return false;
    if (other == this) return true;
    if (!(other instanceof SuperstructureTarget)) return false;
    return name == ((SuperstructureTarget)other).getName();
  }
}
