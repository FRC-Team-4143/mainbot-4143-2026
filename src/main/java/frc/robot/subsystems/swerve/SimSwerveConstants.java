package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.marswars.mechanisms.MotorConfig.TalonMotorType;
import com.marswars.swerve_lib.module.ModuleType;

/** SimBot's swerve constants — only the values that differ from AlphaBot. */
public class SimSwerveConstants extends SwerveConstants {

    @Override
    protected void configure() {
        CANBUS_NAME = "rio";
        MODULE_TYPE = ModuleType.getModuleType("MK4I-L2+");
        STEER_MOTOR_TYPE = TalonMotorType.X60;
        DRIVE_GAINS_SLOT0 = new Slot0Configs().withKG(0.21).withKP(35.0);
        STEER_GAINS_SLOT0 = new Slot0Configs().withKG(0.21).withKP(35.0);
        STEER_GAINS_SLOT1 =
                new Slot1Configs().withKS(0.2).withKV(0.1).withKA(0.02).withKG(0.3).withKP(8.0);
    }
}
