package frc.robot.subsystems.swerve;

import com.marswars.swerve_lib.MwSwerveSubsystem;
import com.marswars.swerve_lib.SwerveDriverInputs;
import com.marswars.swerve_lib.SwerveStates;
import frc.robot.OI;
import frc.robot.subsystems.localization.LocalizationSubsystem;

/**
 * Robot swerve subsystem. All drive behaviour lives in {@link MwSwerveSubsystem}; this class wires
 * in the driver controller and localization pose and applies the team's driver preferences.
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

    /** Holding the POV always drives in the matching crawl state, whatever was requested. */
    @Override
    protected SwerveStates remapWantedState(SwerveStates wanted) {
        if (OI.getDriverJoystickPOV().isPresent()) {
            return crawlVariantOf(wanted);
        }
        return wanted;
    }
}
