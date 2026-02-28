package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.localization.LocalizationSubsystem;

public class Mechanism3dViz {

    static private final Pose3d HOOD_POSE_OFFSET = new Pose3d(-0.177, 0.0, 0.579, new Rotation3d(0.0, 0.0, 0.0));
    static private final double HOOD_HOME_POSITION = Units.degreesToRadians(81.170);

    static private final Pose3d INTAKE_POSE_OFFSET = new Pose3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0));
    static private final double INTAKE_HOME_POSITION = Units.degreesToRadians(11.0);
    static private final double INTAKE_STORE_POSITION = Units.degreesToRadians(95.0);
    static private final double INTAKE_RANGE = INTAKE_STORE_POSITION - INTAKE_HOME_POSITION;

    static private final Pose3d CLIMBER_POSE_OFFSET = new Pose3d(-0.299, 0.279, 0.0, new Rotation3d(0.0, 0.0, Units.degreesToRadians(-10.5)));
    
    static Pose3d robot_pose_ = new Pose3d();
    static Pose3d hood_pose_ = HOOD_POSE_OFFSET;
    static Pose3d intake_pose_ = new Pose3d();
    static Pose3d climber_pose_ = new Pose3d();

    static public void publish(){
        updateRobotPose();
        updateHoodPose();
        updateIntakePose();
        updateClimberPose();

        DogLog.log("Mechanism3dViz/BasePose", robot_pose_);
        DogLog.log("Mechanism3dViz/HoodPose", hood_pose_);
        DogLog.log("Mechanism3dViz/IntakePose", intake_pose_);
        DogLog.log("Mechanism3dViz/ClimberPose", climber_pose_);
    }

    /**
     * Updates the pose of the robot based on its current field pose.
     * The pitch of the robot will change as the climber flip joint moves.
     */
    static private void updateRobotPose(){
        robot_pose_ = new Pose3d(LocalizationSubsystem.getInstance().getFieldPose());
    }

    /**
     * Updates the pose of the hood mechanism based on its current angle.
     */
    static private void updateHoodPose(){
        double hood_angle = ShooterSubsystem.getInstance().getLaunchAngle() - HOOD_HOME_POSITION;
        hood_pose_ = new Pose3d(HOOD_POSE_OFFSET.getTranslation(), new Rotation3d(0.0, hood_angle, 0.0));
    }
    
    /**
     * Updates the pose of the intake mechanism based on its current position.
     */
    static private void updateIntakePose(){
        double intake_angle = IntakeSubsystem.getInstance().getPivotAngle() - INTAKE_HOME_POSITION;
        double intake_ratio = intake_angle / INTAKE_RANGE;
        intake_pose_ = INTAKE_POSE_OFFSET.transformBy(new Transform3d(-0.3 * intake_ratio, 0.0, 0.0, Rotation3d.kZero));
    }

    /**
     * Updates the pose of the climber mechanism based on its current deploy angle.
     */
    static private void updateClimberPose(){
        climber_pose_ = CLIMBER_POSE_OFFSET.rotateBy(new Rotation3d(0.0, ClimberSubsystem.getInstance().getDeployAngle(), 0.0));
    }

}
