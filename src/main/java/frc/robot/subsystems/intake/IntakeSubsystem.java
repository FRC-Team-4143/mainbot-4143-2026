package frc.robot.subsystems.intake;

import java.util.Arrays;
import java.util.List;

import dev.doglog.DogLog;
import frc.mw_lib.mechanisms.ArmMech;
import frc.mw_lib.mechanisms.RollerMech;
import frc.mw_lib.subsystem.MwSubsystem;
import frc.mw_lib.subsystem.SubsystemIoBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakeStates;

public class IntakeSubsystem extends MwSubsystem<IntakeStates, IntakeConstants> {
    private static IntakeSubsystem instance_ = null;

    public static IntakeSubsystem getInstance() {
        if (instance_ == null) {
            instance_ = new IntakeSubsystem();
        }
        return instance_;
    }
    private ArmMech Arm_;
    private RollerMech Roller_;

    public IntakeSubsystem() {
        super(IntakeStates.IDLE, new IntakeConstants());
        Roller_ = new RollerMech(getSubsystemKey(), CONSTANTS.INTAKE_MOTOR_CONFIG);
        Arm_ = new ArmMech(getSubsystemKey(), null, CONSTANTS.INTAKE_GEAR_RATIO, CONSTANTS.ARM_LENGTH, CONSTANTS.ARM_MASS, CONSTANTS.ARM_MIN_ANGLE, CONSTANTS.ARM_MAX_ANGLE);
    }

    // @Override
    // public void handleStateTransition(ShooterStates wanted) {
    // }

    @Override
    public void updateLogic(double timestamp) {
        switch (system_state_) {

        }
        // Log Data
    }

    @Override
    public List<SubsystemIoBase> getIos() {
        return Arrays.asList(Roller_);
    }

    @Override
    public void reset() {
        system_state_ = IntakeStates.IDLE;
    }
}

