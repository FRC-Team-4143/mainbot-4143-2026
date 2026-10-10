package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.marswars.swerve_lib.module.ModuleType;
import edu.wpi.first.math.util.Units;

/** BetaBot's swerve constants — only the values that differ from AlphaBot. */
public class BetaSwerveConstants extends SwerveConstants {

    public BetaSwerveConstants() {
        super(
                alphaHardware()
                        .moduleType(ModuleType.getModuleType("TSN-P13-S16"))
                        .wheelRadius(Units.inchesToMeters(1.978))
                        .driveSlot1(new Slot1Configs().withKS(0.224).withKV(0.62).withKP(0.25))
                        .driveCurrentLimits(35.0, 105.0)
                        .steerCurrentLimits(20.0, 60.0));
    }
}
