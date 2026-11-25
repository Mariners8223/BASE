package frc.robot.subsystems.Shooter.hood;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.flywheel.flyWheelConstants;
import frc.util.TunableNumber;

public class hoodSubsystem extends SubsystemBase {
    private final hoodIO io;
    private final hoodIOInputsAutoLogged inputs = new hoodIOInputsAutoLogged();
    private double targetPosition = 0;
    private final TunableNumber tuningDutyCycle = new TunableNumber("Hood/DutyCycle", 0);
    private final TunableNumber tuningPosition = new TunableNumber("Hood/Position Rotations", 0);
    public hoodSubsystem() {
        io = new hoodIOReal();
        resetAngleEncoder();
    }
    public void resetAngleEncoder() {
        io.resetAngleEncoder();
    }
    public void open(double distanceToHub, boolean shouldClose) {
        targetPosition = hoodConstants.MAX_ANGLE_MOVEMENT_IN_MOTOR_ROTATIONS * flyWheelConstants.getHoodPercent(distanceToHub);
        if (shouldClose) {
            io.openToPosition(0);
        } else {
            io.openToPosition(targetPosition);
        }
    }
    public boolean atAngle() {
        return Math.abs(inputs.hoodOpen - targetPosition) < hoodConstants.POSITION_TOLERANCE_IN_MOTOR_ROTATIONS;
    }
    public void setVoltage(Voltage voltage) {
        io.setVoltage(voltage.in(Volts));
    }
    public boolean isNearTop() {
        return inputs.hoodOpen >= hoodConstants.MAX_ANGLE_MOVEMENT_IN_MOTOR_ROTATIONS - hoodConstants.SYSID_LIMIT_MARGIN_IN_MOTOR_ROTATIONS;
    }
    public boolean isNearBottom() {
        return inputs.hoodOpen <= hoodConstants.SYSID_LIMIT_MARGIN_IN_MOTOR_ROTATIONS;
    }
    /**
     * Test mode: drive the hood from AdvantageScope (/Tuning/Hood/...).
     * Editing DutyCycle runs open loop (capped at TUNING_MAX_DUTY_CYCLE).
     * Editing Position Rotations goes to that position, same units as MAX_ANGLE_MOVEMENT_IN_MOTOR_ROTATIONS (clamped 0 to MAX).
     */
    public Command tuningCommand() {
        return startRun(
            () -> {
                tuningDutyCycle.markSeen();
                tuningPosition.markSeen();
                io.setDutyCycle(0);
            },
            () -> {
                if (tuningDutyCycle.hasChanged()) {
                    io.setDutyCycle(MathUtil.clamp(tuningDutyCycle.get(), -hoodConstants.TUNING_MAX_DUTY_CYCLE, hoodConstants.TUNING_MAX_DUTY_CYCLE));
                }
                if (tuningPosition.hasChanged()) {
                    targetPosition = MathUtil.clamp(tuningPosition.get(), 0, hoodConstants.MAX_ANGLE_MOVEMENT_IN_MOTOR_ROTATIONS);
                    io.openToPosition(targetPosition);
                }
            })
            .finallyDo(() -> io.setDutyCycle(0))
            .withName("Hood Tuning");
    }
    @Override
    public void periodic() {
        io.update(inputs);
        Logger.processInputs("Shooter/Hood", inputs);
        Logger.recordOutput("Shooter/Hood/TargetPosition", targetPosition);
    }
}