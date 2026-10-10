package frc.robot.subsystems.swerve;

import com.marswars.swerve_lib.MwSwerveSubsystem;
import com.marswars.swerve_lib.SwerveDriverInputs;
import frc.robot.OI;
import frc.robot.subsystems.localization.LocalizationSubsystem;

/**
 * Robot swerve subsystem. All drive behaviour lives in {@link MwSwerveSubsystem}; this class wires
 * in the driver controller and localization pose.
 */
public class SwerveSubsystem extends MwSwerveSubsystem<SwerveConstants> {

    private static SwerveSubsystem instance_ = null;

    // getInstance
    public static SwerveSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new SwerveSubsystem();
        }
        return instance_;
    }

    // Constructor
    private SwerveSubsystem() {
        super(
                SwerveConstants.create(),
                () -> LocalizationSubsystem.getInstance().getFieldPose(),
                new SwerveDriverInputs(
                        OI::getDriverJoystickLeftX,
                        OI::getDriverJoystickLeftY,
                        OI::getDriverJoystickRightX,
                        OI::getDriverJoystickPOV));
    }
}
