package frc.mw_lib.mechanisms;

import java.util.List;

import frc.mw_lib.util.FxMotorConfig;

public class TurretMech extends ArmMech{
    public TurretMech(String logging_prefix, List<FxMotorConfig> motor_configs, double gear_ratio, double length, double mass_kg,
            double min_angle, double max_angle){
                super(logging_prefix, motor_configs, gear_ratio, length, mass_kg, min_angle, max_angle, false);
            }
}
