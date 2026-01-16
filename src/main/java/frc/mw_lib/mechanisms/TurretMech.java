package frc.mw_lib.mechanisms;

import java.util.List;

public class TurretMech extends ArmMech{
    public TurretMech(List<FxMotorConfig> motor_configs, double gear_ratio, double length, double mass_kg,
            double min_angle, double max_angle){
                super(motor_configs, gear_ratio, length, mass_kg, min_angle, max_angle, false);
            }
}
