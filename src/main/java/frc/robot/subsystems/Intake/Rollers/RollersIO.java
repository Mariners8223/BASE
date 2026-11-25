package frc.robot.subsystems.Intake.Rollers;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface RollersIO {
    @AutoLog
    class RollersInputs {

    }
    void setDutyCycle(double dutyCycle);
    void setFirstDutyCycle(double dutyCycle);
    void setSecondDutyCycle(double dutyCycle);
}