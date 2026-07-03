package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.marswars.swerve_lib.module.ModuleType;
import edu.wpi.first.math.util.Units;

/** BetaBot's swerve constants — only the values that differ from AlphaBot. */
public class BetaSwerveConstants extends SwerveConstants {

    @Override
    protected void configure() {
        WHEEL_RADIUS_METERS = Units.inchesToMeters(1.978);
        MODULE_TYPE = ModuleType.getModuleType("TSN-P13-S16");
        DRIVE_GAINS_SLOT1 = new Slot1Configs().withKS(0.224).withKV(0.62).withKP(0.25);
    }
}
