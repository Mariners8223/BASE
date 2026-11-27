package frc.robot.subsystems.Shooter.kicker;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.kicker.kickerIOInputsAutoLogged;
import frc.util.TunableNumber;

public class kickerSubsystem extends SubsystemBase {
    private final kickerIO io;
    private final kickerIOInputsAutoLogged inputs = new kickerIOInputsAutoLogged();
    private double angleToHub = 0;
    private final TunableNumber tuningDutyCycle = new TunableNumber("Kicker/DutyCycle", 0);
    public kickerSubsystem() {
        io = new kickerIOReal();
    }
    public double getAngleToHub() {
        return angleToHub;
    }
    public void setDutyCycle(double dutyCycle) {
        io.setDutyCycle(dutyCycle);
    }
    public void stop() {
        io.setDutyCycle(0);
    }
    /** Test mode: set the kicker duty cycle from AdvantageScope (/Tuning/Kicker/DutyCycle). Drives both kicker motors (lead + follower). */
    public Command tuningCommand() {
        return startRun(
            () -> {
                tuningDutyCycle.markSeen();
                stop();
            },
            () -> {
                if (tuningDutyCycle.hasChanged()) {
                    setDutyCycle(MathUtil.clamp(tuningDutyCycle.get(), -1, 1));
                }
            })
            .finallyDo(this::stop)
            .withName("Kicker Tuning");
    }
    @Override
    public void periodic() {
        io.update(inputs);
        Logger.processInputs("Shooter/Kicker", inputs);
        Logger.recordOutput("Shooter/Kicker/AngleToHub", angleToHub);
    }
}
