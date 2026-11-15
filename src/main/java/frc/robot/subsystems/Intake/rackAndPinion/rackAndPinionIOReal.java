package frc.robot.subsystems.Intake.rackAndPinion;

import frc.util.MarinersController.MarinersTalonFX;
import frc.util.MarinersController.MarinersController.ControlMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class rackAndPinionIOReal implements rackAndPinionIO {
    private final MarinersTalonFX pinionMotor;

    public rackAndPinionIOReal(){
        // FIX: added PID gains. extendTo() uses Position mode, which throws a NullPointerException
        // ("PID control on mode requires pid gains") when the motor has no gains.
        pinionMotor = new MarinersTalonFX("pinionMotor", rackAndPinionConstants.CONTROLLER_LOCATION,
                rackAndPinionConstants.motorID, rackAndPinionConstants.PID);
        pinionMotor.setCurrentLimits(rackAndPinionConstants.currentLimit, rackAndPinionConstants.currentThreshhold);
        pinionMotor.setProfile(new Constraints(0.405, 0.2));
    }
    public void resetAngleEncoder() {
        pinionMotor.resetMotorEncoder();
    }
    public void extendTo(double PinionRotation) {
        pinionMotor.setReference(PinionRotation, ControlMode.Position);
    }
    public void update(rackAndPinionInputs rackinputs) {
        rackinputs.PinionRotation = rackAndPinionConstants.getEnchoder(pinionMotor);
        rackinputs.current = rackAndPinionConstants.getCurrent(pinionMotor);
        rackinputs.currentStalled = rackAndPinionConstants.isCurrentStalled(rackinputs.current);
        rackinputs.enchoderStalled = rackAndPinionConstants.isEnchoderStalled(rackinputs.PinionRotation);
    }
    public void stop() {
        pinionMotor.stopMotor();
    }
}