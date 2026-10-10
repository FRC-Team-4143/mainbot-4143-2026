package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.marswars.mechanisms.MotorConfig.TalonMotorType;
import com.marswars.swerve_lib.module.ModuleType;

/** SimBot's swerve constants — only the values that differ from AlphaBot. */
public class SimSwerveConstants extends SwerveConstants {

    public SimSwerveConstants() {
        super(
                alphaHardware()
                        .canbus("rio")
                        .moduleType(ModuleType.getModuleType("MK4I-L2+"))
                        .steerMotor(TalonMotorType.X60)
                        .driveSlot0(new Slot0Configs().withKG(0.21).withKP(35.0))
                        .steerGains(
                                new Slot0Configs().withKG(0.21).withKP(35.0),
                                new Slot1Configs()
                                        .withKS(0.2)
                                        .withKV(0.1)
                                        .withKA(0.02)
                                        .withKG(0.3)
                                        .withKP(8.0)));
    }
}
