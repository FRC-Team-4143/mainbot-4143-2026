package frc.robot.subsystems.climber;

import com.marswars.mechanisms.ArmMech;
import com.marswars.mechanisms.RollerMech;
import com.marswars.sensors.tof.PwfTof;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.climber.ClimberConstants.ClimberStates;

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
    private double alignment_error_; // Positive = need to move right, Negative = need to move left

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
        calculateAlignmentError();
    }

    /**
     * Calculates the alignment error based on the difference between left and right sensors.
     * Positive error means the robot needs to move RIGHT (left sensor is farther).
     * Negative error means the robot needs to move LEFT (right sensor is farther).
     */
    private void calculateAlignmentError() {
        // If left sensor reads farther than right, we're too far left -> move right (+)
        // If right sensor reads farther than left, we're too far right -> move left (-)
        alignment_error_ = left_sensor_range_ - right_sensor_range_;
    }

    /**
     * Returns the lateral alignment error in millimeters.
     * @return Positive if robot should move right, negative if it should move left
     */
    public double getAlignmentError() {
        return alignment_error_;
    }

    /**
     * Checks if the robot is aligned with the vertical tower support within tolerance.
     * @return true if both sensors are reading similar distances (aligned)
     */
    public boolean isAlignedWithTower() {
        return Math.abs(alignment_error_) <= CONSTANTS.SENSOR_ALIGNMENT_DEADBAND;
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
     * Checks if both sensors are detecting the tower within valid range.
     * @return true if both sensors have valid readings
     */
    public boolean areSensorsDetectingTower() {
        return left_sensor_range_ > 0 
            && right_sensor_range_ > 0
            && left_sensor_range_ < CONSTANTS.SENSOR_MAX_VALID_RANGE
            && right_sensor_range_ < CONSTANTS.SENSOR_MAX_VALID_RANGE;
    }

    /**
     * Calculates a normalized correction value for lateral movement (-1.0 to 1.0).
     * Use this to provide a strafe correction to the drivetrain.
     * Positive values mean strafe right, negative means strafe left.
     * 
     * @param maxCorrection Maximum correction value (typically drivetrain max strafe speed)
     * @return Correction value scaled to maxCorrection
     */
    public double getLateralCorrectionValue(double maxCorrection) {
        if (!areSensorsDetectingTower()) {
            return 0.0; // No correction if sensors don't have valid readings
        }

        // Apply a proportional gain to the alignment error
        double correction = alignment_error_ * CONSTANTS.SENSOR_ALIGNMENT_KP;

        // Clamp the correction to the maximum allowed
        correction = MathUtil.clamp(correction, -maxCorrection, maxCorrection);

        return correction;
    }

    /* sets the sensors ranges (how close the nearest object is)
     * @deprecated Use updateSensorReadings() instead
     */
    @Deprecated
    public void setSensors() { // incomplete
        left_sensor_range_ = left_sensor_.getRange();
        right_sensor_range_ = right_sensor_.getRange();
    }

    /**
     * Determines the alignment status and which direction correction is needed.
     * @return String describing alignment status: "ALIGNED", "MOVE_RIGHT", "MOVE_LEFT", or "NO_TOWER_DETECTED"
     */
    public String getAlignmentStatus() {
        if (!areSensorsDetectingTower()) {
            return "NO_TOWER_DETECTED";
        }
        
        if (isAlignedWithTower()) {
            return "ALIGNED";
        }
        
        return alignment_error_ > 0 ? "MOVE_RIGHT" : "MOVE_LEFT";
    }

    /* return which climber sensor is out of place, 
     * returns null otherwise
     * @deprecated Use getAlignmentStatus() or getLateralCorrectionValue() instead
    */
    @Deprecated
    public PwfTof checkSensors() { // incomplete
        setSensors();
        if(CONSTANTS.SENSOR_RANGE_TOLERANCE < left_sensor_range_) {
            return left_sensor_; // when returned, have the robot then move right a bit
        }
        else if(CONSTANTS.SENSOR_RANGE_TOLERANCE < right_sensor_range_) {
            return right_sensor_; // when returned, have the robot then move left a bit
        }
        return null; // when returned, simply have the robot move forward
        // would a new climber state work? called APPROCHING?
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
        if (system_state_ == ClimberStates.DEPLOY
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
