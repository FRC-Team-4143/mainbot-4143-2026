package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class Mechanism3dViz {

    private static final Pose3d HOOD_POSE_OFFSET =
            new Pose3d(-0.177, 0.0, 0.579, new Rotation3d(0.0, 0.0, 0.0));
    private static final double HOOD_HOME_POSITION = Units.degreesToRadians(81.170);

    private static final Pose3d INTAKE_POSE_OFFSET =
            new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
    private static final double INTAKE_HOME_POSITION = Units.degreesToRadians(11.0);
    private static final double INTAKE_STORE_POSITION = Units.degreesToRadians(95.0);
    private static final double INTAKE_RANGE = INTAKE_STORE_POSITION - INTAKE_HOME_POSITION;

    

    static Pose3d robot_pose_ = new Pose3d();
    static Pose3d hood_pose_ = HOOD_POSE_OFFSET;
    static Pose3d intake_pose_ = new Pose3d();
    static Pose3d climber_pose_ = new Pose3d();

    /**
     * Publishes the current poses of the robot and mechanisms to DogLog for visualization in
     * AdvantageScope 3D Field
     */
    public static void publish() {
        updateRobotPose();
        updateHoodPose();
        updateIntakePose();
        

        DogLog.log("Mechanism3dViz/BasePose", robot_pose_);
        DogLog.log("Mechanism3dViz/HoodPose", hood_pose_);
        DogLog.log("Mechanism3dViz/IntakePose", intake_pose_);
        DogLog.log("Mechanism3dViz/ClimberPose", climber_pose_);
    }

    /**
     * Updates the pose of the robot based on its current field pose. The pitch of the robot will
     * change as the climber flip joint moves.
     */
    private static void updateRobotPose() {
        // Apply 5 degree deadband - robot doesn't start to flip until after 5 degrees
        robot_pose_ =
                new Pose3d(LocalizationSubsystem.getInstance().getFieldPose());
                        
                
    }

    /** Updates the pose of the hood mechanism based on its current angle. */
    private static void updateHoodPose() {
        double hood_angle = ShooterSubsystem.getInstance().getLaunchAngle() - HOOD_HOME_POSITION;
        hood_pose_ =
                new Pose3d(HOOD_POSE_OFFSET.getTranslation(), new Rotation3d(0.0, hood_angle, 0.0));
    }

    /** Updates the pose of the intake mechanism based on its current position. */
    private static void updateIntakePose() {
        double intake_angle = IntakeSubsystem.getInstance().getPivotAngle() - INTAKE_HOME_POSITION;
        double intake_ratio = intake_angle / INTAKE_RANGE;
        intake_pose_ =
                INTAKE_POSE_OFFSET.transformBy(
                        new Transform3d(-0.3 * intake_ratio, 0.0, 0.0, Rotation3d.kZero));
    }

}
