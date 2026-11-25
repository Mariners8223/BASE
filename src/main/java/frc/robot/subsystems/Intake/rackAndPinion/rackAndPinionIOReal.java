package frc.robot.subsystems.Intake.rackAndPinion;

import frc.util.MarinersController.MarinersTalonFX;
import frc.util.MarinersController.MarinersController.ControlMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class rackAndPinionIOReal implements rackAndPinionIO {
    private final MarinersTalonFX pinionMotor;

    public rackAndPinionIOReal(){
        pinionMotor = new MarinersTalonFX("pinionMotor", rackAndPinionConstants.CONTROLLER_LOCATION,
                rackAndPinionConstants.motorID, rackAndPinionConstants.PID);
        pinionMotor.setCurrentLimits(rackAndPinionConstants.currentLimit, rackAndPinionConstants.currentThreshhold);
        pinionMotor.setProfile(new Constraints(0.405, 0.2));
    }
    public void resetAngleEncoder() {
        pinionMotor.resetMotorEncoder();
    }
    public void extendTo(double PinionRotation) {
        //fix: NOTE (not changed): in Position mode the setProfile(...) in the constructor is ignored - MarinersController only
        // uses the profile in ControlMode.ProfiledPosition. Do not just switch: with Constraints(0.405, 0.2) a 6-rotation
        // extend would take ~15 s. Either delete setProfile or raise the constraints, then use ProfiledPosition.
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
    public void setDutyCycle(double dutyCycle) {
        pinionMotor.setDutyCycle(dutyCycle);
    }
}