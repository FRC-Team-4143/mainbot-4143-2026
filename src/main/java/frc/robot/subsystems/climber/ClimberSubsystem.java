package frc.robot.subsystems.climber;

import com.marswars.mechanisms.ArmMech;
import com.marswars.mechanisms.RollerMech;
import com.marswars.sensors.tof.PwfTof;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.climber.ClimberConstants.ClimberStates;
import frc.robot.subsystems.localization.LocalizationSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import java.util.Arrays;
import java.util.List;

public class ClimberSubsystem extends MwSubsystem<ClimberStates, ClimberConstants> {
    private static ClimberSubsystem instance_ = null;

    private RollerMech deploy_joint_;
    private ArmMech flip_joint_;
    private PwfTof left_sensor_;
    private PwfTof right_sensor_;

    private double left_sensor_range_;
    private double right_sensor_range_;

    // getInstance
    public static ClimberSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new ClimberSubsystem();
        }
        return instance_;
    }

    // Constructor
    public ClimberSubsystem() {
        super(ClimberStates.STOWED, new ClimberConstants());
        deploy_joint_ =
                new RollerMech(
                        getSubsystemKey(),
                        "Extender",
                        List.of(CONSTANTS.EXTENDER_MOTOR_CONFIG),
                        CONSTANTS.EXTENDER_GEAR_RATIO);
        flip_joint_ =
                new ArmMech(
                        getSubsystemKey(),
                        "Arm",
                        List.of(CONSTANTS.ARM_MOTOR_CONFIG),
                        CONSTANTS.ARM_GEAR_RATIO,
                        CONSTANTS.ARM_LENGTH,
                        CONSTANTS.ARM_MASS,
                        CONSTANTS.ARM_MAX_ANGLE,
                        CONSTANTS.ARM_MIN_ANGLE);
        left_sensor_ =
                new PwfTof(
                    getSubsystemKey(),
                    CONSTANTS.LEFT_SENSOR_ID,
                    CONSTANTS.SENSOR_MODE,
                    0.002); // correct?
        right_sensor_ =
                new PwfTof(
                    getSubsystemKey(),
                    CONSTANTS.RIGHT_SENSOR_ID,
                    CONSTANTS.SENSOR_MODE,
                    0.002); // correct?
    }

    // =============================================================================
    // TOWER ALIGNMENT SENSOR METHODS
    // =============================================================================

    /**
     * Updates the sensor readings from both Time-of-Flight sensors.
     * Should be called periodically to keep readings fresh.
     */
    public void updateSensorReadings() {
        left_sensor_range_ = left_sensor_.getRange();
        right_sensor_range_ = right_sensor_.getRange();
    }

    /**
     * Checks if the left sensor can see the tower (discrete detection).
     * @return true if left sensor detects tower within threshold
     */
    public boolean leftSensorSeesTower() {
        return left_sensor_range_ > 0 && left_sensor_range_ < CONSTANTS.SENSOR_DETECTION_THRESHOLD;
    }

    /**
     * Checks if the right sensor can see the tower (discrete detection).
     * @return true if right sensor detects tower within threshold
     */
    public boolean rightSensorSeesTower() {
        return right_sensor_range_ > 0 && right_sensor_range_ < CONSTANTS.SENSOR_DETECTION_THRESHOLD;
    }

    /**
     * Checks if both sensors can see the tower (robot is aligned).
     * @return true if both sensors detect the tower
     */
    public boolean isAlignedWithTower() {
        return leftSensorSeesTower() && rightSensorSeesTower();
    }

    /**
     * Checks if at least one sensor can see the tower.
     * Both sensors maxed out means we can't see the tower at all.
     * @return true if at least one sensor detects the tower
     */
    public boolean canSeeTower() {
        return leftSensorSeesTower() || rightSensorSeesTower();
    }

    /**
     * Gets the current reading from the left Time-of-Flight sensor.
     * @return Distance in millimeters
     */
    public double getLeftSensorRange() {
        return left_sensor_range_;
    }

    /**
     * Gets the current reading from the right Time-of-Flight sensor.
     * @return Distance in millimeters
     */
    public double getRightSensorRange() {
        return right_sensor_range_;
    }

    /**
     * Determines the alignment status for line-following logic.
     * @return String describing what action to take: "MOVE_LEFT", "MOVE_RIGHT", "ALIGNED", or "NO_TOWER"
     */
    public String getAlignmentStatus() {
        boolean leftSees = leftSensorSeesTower();
        boolean rightSees = rightSensorSeesTower();
        
        // Both sensors maxed = can't see tower at all
        if (!leftSees && !rightSees) {
            return "NO_TOWER";
        }
        
        // Both see tower = aligned
        if (leftSees && rightSees) {
            return "ALIGNED";
        }
        
        // Left sees but right doesn't = too far right, move left
        if (leftSees && !rightSees) {
            return "MOVE_LEFT";
        }
        
        // Right sees but left doesn't = too far left, move right
        return "MOVE_RIGHT";
    }


    // reset
    @Override
    public void reset() {
        system_state_ = ClimberStates.STOWED;
    }

    // getIos
    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(deploy_joint_, flip_joint_, left_sensor_, right_sensor_);
    }

    // handleStateTransition
    @Override
    public void handleStateTransition(ClimberStates wanted_state) {
        if (system_state_ == ClimberStates.STOWED && wanted_state == ClimberStates.DEPLOY) {
            system_state_ = ClimberStates.DEPLOY;
        } else {
        } // no command
        if (system_state_ == ClimberStates.DEPLOY && isDeployed() && wanted_state == ClimberStates.APPROACHING) {
            system_state_ = ClimberStates.APPROACHING;
        } else if (system_state_ == ClimberStates.DEPLOY
                && isDeployed()
                && wanted_state == ClimberStates.L1_CLIMB) {
            system_state_ = ClimberStates.L1_CLIMB;
        } else if (system_state_ == ClimberStates.DEPLOY
                && isDeployed()
                && wanted_state == ClimberStates.L3_CLIMB) {
            system_state_ = ClimberStates.L3_CLIMB;
        } else if (system_state_ == ClimberStates.DEPLOY && wanted_state == ClimberStates.STOWED) {
            system_state_ = ClimberStates.STOWED;
        } else {
        } // no commands
        if (system_state_ == ClimberStates.APPROACHING && wanted_state == ClimberStates.L1_CLIMB) {
            system_state_ = ClimberStates.L1_CLIMB;
        } else if (system_state_ == ClimberStates.APPROACHING && wanted_state == ClimberStates.DEPLOY) {
            system_state_ = ClimberStates.DEPLOY;
        } else {
        } // no commands
        if (system_state_ == ClimberStates.L1_CLIMB && wanted_state == ClimberStates.L1_DOWN) {
            system_state_ = ClimberStates.L1_DOWN;
        } else {
        } // no commands
        if (system_state_ == ClimberStates.L1_DOWN && wanted_state == ClimberStates.DEPLOY) {
            system_state_ = ClimberStates.DEPLOY;
        } else {
        } // no commands
    }

    // updateLogic
    @Override
    public void updateLogic(double timestamp) {
        // Update sensor readings every cycle for tower alignment
        updateSensorReadings();
        
        switch (system_state_) {
            case STOWED:
                deploy_joint_.setTargetPosition(CONSTANTS.EXTENDER_STOWED_ANGLE);
                flip_joint_.setTargetPosition(CONSTANTS.ARM_L0_POSITION);
                break;
            case DEPLOY:
                deploy_joint_.setTargetPosition(CONSTANTS.EXTENDER_DEPLOYED_ANGLE);
                break;
            case APPROACHING:
                // Tower approach with sensor-based line following
                // Keep climber deployed during approach
                deploy_joint_.setTargetPosition(CONSTANTS.EXTENDER_DEPLOYED_ANGLE);
                flip_joint_.setTargetPosition(CONSTANTS.ARM_L0_POSITION);
                
                // Line-following logic - discrete sensor feedback
                boolean leftSees = leftSensorSeesTower();
                boolean rightSees = rightSensorSeesTower();
                
                // Determine chassis speeds based on discrete sensor readings
                double forwardSpeed = CONSTANTS.APPROACH_FORWARD_SPEED;
                double strafeSpeed = 0.0;
                
                if (!leftSees && !rightSees) {
                    // Both sensors maxed out - can't see tower at all
                    // Stop moving, we've lost the line
                    forwardSpeed = 0.0;
                    strafeSpeed = 0.0;
                } else if (leftSees && !rightSees) {
                    // Left sees tower, right doesn't = too far right
                    // Move left to get back on line
                    strafeSpeed = -CONSTANTS.APPROACH_STRAFE_SPEED; // Negative = left
                } else if (!leftSees && rightSees) {
                    // Right sees tower, left doesn't = too far left
                    // Move right to get back on line
                    strafeSpeed = CONSTANTS.APPROACH_STRAFE_SPEED; // Positive = right
                } else {
                    // Both sensors see tower = aligned
                    // Drive straight forward
                    strafeSpeed = 0.0;
                }
                
                // Get current rotation for field-relative control
                Rotation2d currentRotation = LocalizationSubsystem.getInstance()
                        .getFieldPose()
                        .getRotation();
                
                // Create field-relative chassis speeds
                ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    forwardSpeed,   // X: Forward
                    strafeSpeed,    // Y: Strafe (- = left, + = right)
                    0.0,            // Omega: No rotation
                    currentRotation
                );
                
                // Command swerve to execute the speeds with rotation lock
                SwerveSubsystem.getInstance().setChassisSpeedRotationLock(speeds, currentRotation);
                break;
            case L1_CLIMB:
                flip_joint_.setTargetPosition(CONSTANTS.ARM_L1_CLIMB);
                deploy_joint_.setTargetPosition(CONSTANTS.EXTENDER_DEPLOYED_ANGLE);
                break;
            case L1_DOWN:
                flip_joint_.setTargetPosition(CONSTANTS.ARM_L0_POSITION);
                deploy_joint_.setTargetPosition(CONSTANTS.EXTENDER_DEPLOYED_ANGLE);
                break;
            case L3_CLIMB:
                flip_joint_.setTargetPosition(CONSTANTS.ARM_L3_CLIMB);
                deploy_joint_.setTargetPosition(CONSTANTS.EXTENDER_DEPLOYED_ANGLE);
                break;
        }
    }

    // =============================================================================
    // PRIVATE HELPER METHODS
    // =============================================================================

    private boolean isDeployed() {
        return (MathUtil.isNear(
                CONSTANTS.EXTENDER_DEPLOYED_ANGLE,
                deploy_joint_.getCurrentPosition(),
                CONSTANTS.EXTENDER_TOLERANCE_ANGLE));
    }
}
