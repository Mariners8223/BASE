package frc.robot.subsystems.Intake.rackAndPinion;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.TunableNumber;
import frc.robot.subsystems.Intake.rackAndPinion.rackAndPinionConstants.rackAndPinionState;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class rackAndPinionSubsystem extends SubsystemBase {
    rackAndPinionIO io;
    rackAndPinionInputsAutoLogged rackinputs = new rackAndPinionInputsAutoLogged();
    private final TunableNumber tuningDutyCycle = new TunableNumber("RackAndPinion/DutyCycle", 0);
    private final TunableNumber tuningPosition = new TunableNumber("RackAndPinion/Position Rotations", 0);
    
    public rackAndPinionSubsystem() {
        io = new rackAndPinionIOReal();
    }
    
    public void resetAngleEncoder(){
        io.resetAngleEncoder();
    }
    public void extend() {
        io.extendTo(rackAndPinionConstants.pinionRotation);
        rackAndPinionConstants.currentPinionState = rackAndPinionConstants.currentPinionState.OPEN;
    }
    public void extendTo(double position) {
        io.extendTo(position);
        rackAndPinionConstants.currentPinionState = rackAndPinionConstants.currentPinionState.HALF;
    }
    public void retract() {
        io.extendTo(0);
        rackAndPinionConstants.currentPinionState = rackAndPinionConstants.currentPinionState.CLOSE;

    }
    /**
     * Test mode: drive the rack and pinion from AdvantageScope (/Tuning/RackAndPinion/...).
     * Editing DutyCycle runs open loop (capped at MAX_PINION_POWER).
     * Editing Position Rotations goes to that motor position (clamped 0 = retracted to pinionRotation = extended).
     * The stall check in periodic() still stops the motor.
     */
    public Command tuningCommand() {
        return startRun(
            () -> {
                tuningDutyCycle.markSeen();
                tuningPosition.markSeen();
                io.stop();
            },
            () -> {
                if (tuningDutyCycle.hasChanged()) {
                    io.setDutyCycle(MathUtil.clamp(tuningDutyCycle.get(), -rackAndPinionConstants.MAX_PINION_POWER, rackAndPinionConstants.MAX_PINION_POWER));
                }
                if (tuningPosition.hasChanged()) {
                    io.extendTo(MathUtil.clamp(tuningPosition.get(), 0, rackAndPinionConstants.pinionRotation));
                }
            })
            .finallyDo(io::stop)
            .withName("RackAndPinion Tuning");
    }
    @Override
    public void periodic() {
        io.update(rackinputs);
        //fix: CHANGE: added Logger.processInputs. REASON: the inputs were read but never logged, so the rack position,
        // current and stall flags were missing from the logs and AdvantageKit replay could not reproduce this subsystem.
        Logger.processInputs("Intake/RackAndPinion", rackinputs);
        if (rackinputs.currentStalled) {
            io.stop();
        }
    }

    public boolean isCurrentStalled() {
        return rackinputs.currentStalled;
    }
    public boolean isJammed() {
        return rackinputs.currentStalled && !rackinputs.enchoderStalled;
    }
    public rackAndPinionConstants.rackAndPinionState rackAndPinionState() {
        return rackAndPinionConstants.currentPinionState;
    }


}