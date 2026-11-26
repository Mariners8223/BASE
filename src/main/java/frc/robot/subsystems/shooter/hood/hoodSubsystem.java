package frc.robot.subsystems.Shooter.hood;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.flywheel.flyWheelConstants;
import frc.util.TunableNumber;

public class hoodSubsystem extends SubsystemBase {
    private final hoodIO io;
    private final hoodIOInputsAutoLogged inputs = new hoodIOInputsAutoLogged();
    private double targetPosition = 0;
    private boolean isHomed = false;
    private final TunableNumber tuningDutyCycle = new TunableNumber("Hood/DutyCycle", 0);
    private final TunableNumber tuningPosition = new TunableNumber("Hood/Position Rotations", 0);
    public hoodSubsystem() {
        io = new hoodIOReal();
        resetAngleEncoder();
    }
    public void resetAngleEncoder() {
        io.resetAngleEncoder();
    }
    public boolean isHomed() {
        return isHomed;
    }
    /**
     * Homing: drive the hood closed at -HOMING_DUTY_CYCLE (soft limits off, since we may be below the boot zero)
     * until the stator current jumps above HOMING_CURRENT_THRESHOLD_AMPS (hood hit the hard stop).
     * Then stop, zero the encoder and turn the soft limits back on.
     * The first HOMING_IGNORE_START_SECONDS are ignored so the start-up current spike is not mistaken for the stop.
     * If it times out or is interrupted, the encoder is NOT reset (it keeps the boot zero).
     */
    public Command homeCommand() {
        Debouncer stallDebouncer = new Debouncer(hoodConstants.HOMING_CURRENT_DEBOUNCE_SECONDS, DebounceType.kRising);
        Timer timer = new Timer();
        return startRun(
            () -> {
                isHomed = false;
                stallDebouncer.calculate(false);
                timer.restart();
                io.setSoftLimitsEnabled(false);
                io.setDutyCycle(-hoodConstants.HOMING_DUTY_CYCLE);
            },
            () -> io.setDutyCycle(-hoodConstants.HOMING_DUTY_CYCLE))
            .until(() -> {
                boolean currentHigh = timer.hasElapsed(hoodConstants.HOMING_IGNORE_START_SECONDS)
                    && Math.abs(inputs.hoodStatorCurrent) > hoodConstants.HOMING_CURRENT_THRESHOLD_AMPS;
                return stallDebouncer.calculate(currentHigh);
            })
            .finallyDo(interrupted -> {
                io.setDutyCycle(0);
                if (!interrupted) {
                    io.resetAngleEncoder();
                    isHomed = true;
                } else {
                    DriverStation.reportWarning("Hood: homing did not finish (timeout or interrupted), encoder not reset", false);
                }
                targetPosition = 0;
                io.setSoftLimitsEnabled(true);
            })
            // timeout goes AFTER finallyDo so a timeout counts as "interrupted" (no encoder reset)
            .withTimeout(hoodConstants.HOMING_TIMEOUT_SECONDS)
            .withName("Hood Homing");
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
        Logger.recordOutput("Shooter/Hood/IsHomed", isHomed);
    }
}