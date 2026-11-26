package frc.robot.subsystems.Shooter.hood;

import com.ctre.phoenix6.StatusCode; //fix: CHANGE: new import, used by applyTalonSoftLimits()
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs; //fix: CHANGE: new import, used by applyTalonSoftLimits()

import edu.wpi.first.wpilibj.DriverStation; //fix: CHANGE: new import, used to report a failed soft limit config
import frc.util.MarinersController.MarinersTalonFX;
import frc.util.MarinersController.MarinersController.ControlMode;

public class hoodIOReal implements hoodIO {
    private final MarinersTalonFX hoodMotor;
    public hoodIOReal() {
        //fix: CHANGE: the gear ratio is now passed in the constructor (the 4th argument).
        // The old separate line "hoodMotor.getMeasurements().setGearRatio(...)" at the end was removed.
        // REASON: the gear ratio is set before anything that uses it, and applyTalonSoftLimits() below relies on it.
        hoodMotor = new MarinersTalonFX("Shooter/Hood Motor", hoodConstants.CONTROLLER_LOCATION, hoodConstants.MOTOR_ID, hoodConstants.GEAR_RATIO);
        hoodMotor.setCurrentLimits(hoodConstants.CURRENT_LIMIT, hoodConstants.CURRENT_THRESHOLD);
        hoodMotor.setMotorInverted(hoodConstants.IS_INVERTED);
        // NOTE: this is the wrapper's "soft limit". In Position mode it only clamps the TARGET to [0, MAX].
        // It does NOT stop the motor when the hood is already past MAX. That is why the hood went past the limit.
        hoodMotor.enableSoftLimits(0, hoodConstants.MAX_ANGLE_MOVEMENT_IN_MOTOR_ROTATIONS);
        hoodMotor.setMotorIdleMode(true);
        hoodMotor.setPIDF(hoodConstants.PID);
        hoodMotor.setMaxMinOutput(hoodConstants.HOOD_SPEED, -hoodConstants.HOOD_SPEED);
        //fix: CHANGE: new. Turns on the Talon's own soft limits (see the method below).
        // MUST stay the LAST line here: setMaxMinOutput() re-applies the whole Talon config, and if it ran
        // after this line it would turn the limits off again.
        applyTalonSoftLimits(true);
        // Faster stator current updates so homing sees the current jump quickly (default in the wrapper is 10 Hz).
        hoodMotor.getMotor().getStatorCurrent().setUpdateFrequency(50);
    }

    @Override
    public void setSoftLimitsEnabled(boolean enabled) {
        if (enabled) {
            hoodMotor.enableSoftLimits(0, hoodConstants.MAX_ANGLE_MOVEMENT_IN_MOTOR_ROTATIONS);
        } else {
            hoodMotor.disableSoftLimits();
        }
        applyTalonSoftLimits(enabled);
    }

    /**
     * fix: CHANGE: new method.
     * Turns on the TalonFX's built-in forward/reverse soft limits.
     * REASON: these are enforced by the motor controller itself, in every control mode (Position, Voltage,
     * SysID...). The Talon cuts output in that direction once the hood reaches the limit, whatever the code commands.
     * The thresholds are in MOTOR (rotor) rotations, so the hood rotations are multiplied by the gear ratio:
     * 0.081699 * 225 = about 18.4 motor rotations.
     * Done here with hoodMotor.getMotor() so MarinersTalonFX does not need to change.
     * NOTE: 0 is wherever the encoder was zeroed (robot boot), so boot with the hood fully closed.
     */
    private void applyTalonSoftLimits(boolean enabled) {
        SoftwareLimitSwitchConfigs limits = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(hoodConstants.MAX_ANGLE_MOVEMENT_IN_MOTOR_ROTATIONS * hoodConstants.GEAR_RATIO)
            .withForwardSoftLimitEnable(enabled)
            .withReverseSoftLimitThreshold(0)
            .withReverseSoftLimitEnable(enabled);

        StatusCode error = hoodMotor.getMotor().getConfigurator().apply(limits);
        if (!error.isOK()) {
            DriverStation.reportError("Hood: failed to set Talon soft limits: " + error, false);
        }
    }
    @Override
    public void resetAngleEncoder() {
        hoodMotor.resetMotorEncoder();
    }
    @Override
    public void openToPosition(double openPosition) {
        hoodMotor.setReference(openPosition, ControlMode.Position);
    }
    @Override
    public void update(hoodIOInputs inputs) {
        inputs.hoodOpen = hoodMotor.getPosition();
        inputs.hoodVelocity = hoodMotor.getVelocity();
        inputs.hoodStatorCurrent = hoodMotor.getMotor().getStatorCurrent().getValueAsDouble();
    }
    @Override
    public void setVoltage(double volts) {
        hoodMotor.setVoltage(volts);
    }
    @Override
    public void setDutyCycle(double dutyCycle) {
        hoodMotor.setDutyCycle(dutyCycle);
    }
}