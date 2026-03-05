package frc.robot.subsystems.climber;

import com.marswars.mechanisms.ArmMech;
import com.marswars.mechanisms.RollerMech;
import com.marswars.subsystem.MwSubsystem;
import com.marswars.subsystem.SubsystemIoBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.ClimberConstants.ClimberStates;
import java.util.Arrays;
import java.util.List;

public class ClimberSubsystem extends MwSubsystem<ClimberStates, ClimberConstants> {
    private static ClimberSubsystem instance_ = null;

    private RollerMech deploy_joint_;
    private ArmMech flip_joint_;

    // Adjustment factor for fine-tuning positions (in radians)
    private double flip_adjustment_ = 0.0;

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
                        List.of(CONSTANTS.DEPLOY_MOTOR_CONFIG),
                        CONSTANTS.DEPLOY_GEAR_RATIO,
                        CONSTANTS.DEPLOY_MOI);
        flip_joint_ =
                new ArmMech(
                        getSubsystemKey(),
                        "Arm",
                        List.of(CONSTANTS.FLIP_MOTOR_CONFIG),
                        CONSTANTS.FLIP_GEAR_RATIO,
                        CONSTANTS.FLIP_LENGTH,
                        CONSTANTS.FLIP_MASS,
                        CONSTANTS.FLIP_MAX_ANGLE,
                        CONSTANTS.FLIP_MIN_ANGLE);

        SmartDashboard.putData(
                "Zero Climber Flip",
                Commands.runOnce(() -> flip_joint_.setCurrentPosition(CONSTANTS.FLIP_GROUND_ANGLE))
                        .withName("Zero Climber"));
        SmartDashboard.putData(
                "Zero Climber Deploy",
                Commands.runOnce(
                                () ->
                                        deploy_joint_.setCurrentPosition(
                                                CONSTANTS.DEPLOY_STOWED_ANGLE))
                        .withName("Zero Climber"));
    }

    // reset
    @Override
    public void reset() {
        system_state_ = ClimberStates.STOWED;
    }

    // getIos
    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(deploy_joint_, flip_joint_);
    }

    // handleStateTransition
    @Override
    public void handleStateTransition(ClimberStates wanted_state) {
        if (wanted_state == ClimberStates.TUNNING) {
            system_state_ = ClimberStates.TUNNING;
            flip_adjustment_ = 0.0; // Reset adjustment on state change
            return;
        }

        if ((wanted_state == ClimberStates.L1
                        || wanted_state == ClimberStates.L2
                        || wanted_state == ClimberStates.L3)
                && !isDeployed()) {
            DataLogManager.log("Cannot Climb while Climber is not Deployed");
            return;
        }

        // Stop Climbing for L1 Target and Hold Position
        if (wanted_state == ClimberStates.L1
                && MathUtil.isNear(
                        CONSTANTS.FLIP_L1_CLIMB + flip_adjustment_,
                        getFlipAngle(),
                        CONSTANTS.FLIP_ANGLE_TOLERANCE)) {
            system_state_ = ClimberStates.CLIMB_HOLD;
            return;
        }
        // Stop Climbing for L2 Target and Hold Position
        if (wanted_state == ClimberStates.L2
                && MathUtil.isNear(
                        CONSTANTS.FLIP_L2_CLIMB + flip_adjustment_,
                        getFlipAngle(),
                        CONSTANTS.FLIP_ANGLE_TOLERANCE)) {
            system_state_ = ClimberStates.CLIMB_HOLD;
            return;
        }
        // Stop Climbing for L3 Target and Hold Position
        if (wanted_state == ClimberStates.L3
                && MathUtil.isNear(
                        CONSTANTS.FLIP_L3_CLIMB_ANGLE + flip_adjustment_,
                        getFlipAngle(),
                        CONSTANTS.FLIP_ANGLE_TOLERANCE)) {
            system_state_ = ClimberStates.CLIMB_HOLD;
            return;
        }
        // Stop Climbing for GROUND Target and Hold Position
        if (wanted_state == ClimberStates.GROUND
                && MathUtil.isNear(
                        CONSTANTS.FLIP_GROUND_ANGLE + flip_adjustment_,
                        getFlipAngle(),
                        CONSTANTS.FLIP_ANGLE_TOLERANCE)) {
            system_state_ = ClimberStates.DEPLOY;
            return;
        }

        system_state_ = wanted_state;
    }

    // updateLogic
    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {
            case STOWED:
                flip_joint_.setTargetDutyCycle(
                        calculateClimbDutyCycle(CONSTANTS.FLIP_GROUND_ANGLE));
                deploy_joint_.setTargetPosition(CONSTANTS.DEPLOY_STOWED_ANGLE);
                flip_adjustment_ = 0;
                break;
            case DEPLOY:
                flip_joint_.setTargetDutyCycle(
                        calculateClimbDutyCycle(CONSTANTS.FLIP_GROUND_ANGLE + flip_adjustment_));
                deploy_joint_.setTargetPosition(CONSTANTS.DEPLOY_DEPLOYED_ANGLE);
                break;
            case L1:
                flip_joint_.setTargetDutyCycle(
                        calculateClimbDutyCycle(CONSTANTS.FLIP_L1_CLIMB + flip_adjustment_));
                deploy_joint_.setTargetPosition(CONSTANTS.DEPLOY_DEPLOYED_ANGLE);
                break;
            case L2:
                flip_joint_.setTargetDutyCycle(
                        calculateClimbDutyCycle(CONSTANTS.FLIP_L2_CLIMB + flip_adjustment_));
                deploy_joint_.setTargetPosition(CONSTANTS.DEPLOY_DEPLOYED_ANGLE);
                break;
            case GROUND:
                flip_joint_.setTargetDutyCycle(
                        calculateClimbDutyCycle(CONSTANTS.FLIP_GROUND_ANGLE + flip_adjustment_));
                deploy_joint_.setTargetPosition(CONSTANTS.DEPLOY_DEPLOYED_ANGLE);
                break;
            case L3:
                flip_joint_.setTargetDutyCycle(
                        calculateClimbDutyCycle(CONSTANTS.FLIP_L3_CLIMB_ANGLE + flip_adjustment_));
                deploy_joint_.setTargetPosition(CONSTANTS.DEPLOY_DEPLOYED_ANGLE);
                break;
            case CLIMB_HOLD:
                flip_joint_.setTargetDutyCycle(0);
                deploy_joint_.setTargetPosition(CONSTANTS.DEPLOY_DEPLOYED_ANGLE);
                break;
            case TUNNING:
                // allow manual control of both joints through the dashboard for testing and
                // calibration
                break;
        }
    }

    // =============================================================================
    // PUBLIC HELPER METHODS
    // =============================================================================

    /**
     * Returns the current angle of the deploy joint in radians. (Used for testing and visualization
     * purposes)
     *
     * @return the current angle of the deploy joint in radians
     */
    public double getDeployAngle() {
        return deploy_joint_.getCurrentPosition();
    }

    /**
     * Returns the current angle of the flip joint in radians. (Used for testing and visualization
     * purposes)
     *
     * @return the current angle of the flip joint in radians
     */
    public double getFlipAngle() {
        return flip_joint_.getCurrentPosition();
    }

    /**
     * Toggles the deploy state between DEPLOY and STOWED. Does nothing if the current state is not
     * DEPLOY or STOWED.
     */
    public Command toggleDeployCommand() {
        String to_state_string =
                (system_state_ == ClimberStates.DEPLOY)
                        ? "STOWED"
                        : (system_state_ == ClimberStates.STOWED) ? "DEPLOY" : "OTHER";
        return Commands.runOnce(
                        () -> {
                            if (system_state_ != ClimberStates.DEPLOY
                                    && system_state_ != ClimberStates.STOWED
                                    && system_state_ != ClimberStates.TUNNING) {
                                DataLogManager.log(
                                        "Can not Toggle Deploy/Stowed state from: "
                                                + system_state_);
                                return; // if we're not in a state where deploy can be toggled, do
                                // nothing
                            }

                            if (isDeployed()) {
                                setWantedState(ClimberStates.STOWED);
                            } else {
                                setWantedState(ClimberStates.DEPLOY);
                            }
                        })
                .withName("Toggle Deploy: " + to_state_string);
    }

    /**
     * Command to bump the climber up slightly. Only works from CLIMB_HOLD state. Hold the
     * button/trigger to continue bumping up.
     *
     * @return Command that bumps up while held, returns to CLIMB_HOLD when released
     */
    public Command bumpUpCommand() {
        return Commands.run(
                        () -> {
                            flip_adjustment_ += CONSTANTS.FLIP_ADJUSTMENT_INCREMENT;
                        })
                .withName("Bump Up");
    }

    /**
     * Command to bump the climber down slightly. Only works from CLIMB_HOLD state. Hold the
     * button/trigger to continue bumping down.
     *
     * @return Command that bumps down while held, returns to CLIMB_HOLD when released
     */
    public Command bumpDownCommand() {
        return Commands.run(
                        () -> {
                            flip_adjustment_ -= CONSTANTS.FLIP_ADJUSTMENT_INCREMENT;
                        })
                .withName("Bump Down");
    }

    /**
     * Resets the flip adjustment to zero.
     *
     * @return Command that resets the adjustment
     */
    public Command resetAdjustmentCommand() {
        return Commands.runOnce(
                        () -> {
                            flip_adjustment_ = 0.0;
                            DataLogManager.log("Flip adjustment reset to 0");
                        })
                .withName("Reset Adjustment");
    }

    // =============================================================================
    // PRIVATE HELPER METHODS
    // =============================================================================

    /**
     * Calculates the needed duty cycle to move towards the target angle for climbing states.
     * Returns positive duty cycle if current angle is below target (climbing up), negative duty
     * cycle if current angle is above target (moving down), or 0.0 if at target within tolerance.
     *
     * @param target_angle The target angle in radians for the current state
     * @return The duty cycle to apply (-1.0 to 1.0)
     */
    private double calculateClimbDutyCycle(double target_angle) {
        double current_angle = getFlipAngle();

        // If we're within tolerance of the target, return 0
        if (MathUtil.isNear(target_angle, current_angle, CONSTANTS.FLIP_ANGLE_TOLERANCE)) {
            return 0.0;
        }

        // If current angle is less than target, we need to climb up (positive duty cycle)
        if (current_angle < target_angle) {
            return CONSTANTS.FLIP_CLIMB_UP_DUTY_CYCLE;
        }
        // If current angle is greater than target, we need to move down (negative duty cycle)
        else {
            return CONSTANTS.FLIP_DOWN_UP_DUTY_CYCLE;
        }
    }

    /**
     * Checks if the climber is currently deployed by comparing the deploy joint's current angle to
     * the deployed angle within a certain tolerance.
     *
     * @return true if the climber is deployed, false otherwise
     */
    private boolean isDeployed() {
        return (MathUtil.isNear(
                CONSTANTS.DEPLOY_DEPLOYED_ANGLE,
                deploy_joint_.getCurrentPosition(),
                CONSTANTS.DEPLOY_TOLERANCE_ANGLE));
    }
}
