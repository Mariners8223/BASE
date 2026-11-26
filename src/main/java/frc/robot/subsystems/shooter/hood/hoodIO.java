package frc.robot.subsystems.Shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface hoodIO {
    @AutoLog
    public class hoodIOInputs {
        double hoodOpen;
        double hoodVelocity;
        double hoodStatorCurrent;
    }
    /** Turns the hood's soft limits (wrapper + Talon) on or off. Off is needed while homing below the old zero. */
    void setSoftLimitsEnabled(boolean enabled);
    void resetAngleEncoder();
    void openToPosition(double openPosition);
    void update(hoodIOInputs inputs);
    void setVoltage(double volts);
    void setDutyCycle(double dutyCycle);
}