package frc.robot.subsystems.Intake.transfer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.TunableNumber;

public class transferSubsystem extends SubsystemBase {
    private final transferIO io;
    private final TunableNumber tuningTransferDutyCycle = new TunableNumber("Transfer/Transfer DutyCycle", 0);
    private final TunableNumber tuningFunnelDutyCycle = new TunableNumber("Transfer/Funnel DutyCycle", 0);
    public transferSubsystem() {
        io = new transferIOReal();
    }
    public void setDutyCycle(double dutyCycle) {
        io.setDutyCycle(dutyCycle);
    }
    public void stop() {
        io.setDutyCycle(0);
    }

    /** Test mode: set the transfer and funnel motors' duty cycles from AdvantageScope (/Tuning/Transfer/...). */
    public Command tuningCommand() {
        return startRun(
            () -> {
                tuningTransferDutyCycle.markSeen();
                tuningFunnelDutyCycle.markSeen();
                stop();
            },
            () -> {
                if (tuningTransferDutyCycle.hasChanged()) {
                    io.setTransferDutyCycle(MathUtil.clamp(tuningTransferDutyCycle.get(), -1, 1));
                }
                if (tuningFunnelDutyCycle.hasChanged()) {
                    io.setFunnelDutyCycle(MathUtil.clamp(tuningFunnelDutyCycle.get(), -1, 1));
                }
            })
            .finallyDo(this::stop)
            .withName("Transfer Tuning");
    }
}