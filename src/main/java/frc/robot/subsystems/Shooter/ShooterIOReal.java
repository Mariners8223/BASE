package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

import frc.util.MarinersController.MarinersTalonFX;
import frc.util.MarinersController.MarinersController.ControlMode;
import frc.util.MarinersController.MarinersController.ControllerLocation;

public class ShooterIOReal implements ShooterIO {
  MarinersTalonFX leadMotor;
  MarinersTalonFX followMotor;
  MarinersTalonFX hoodMotor;
  public ShooterIOReal(){
    leadMotor = new MarinersTalonFX("shooter_Lead_Motor", ShooterConstants.CONTROLLER_LOCATION, ShooterConstants.LEAD_MOTOR_ID);
    leadMotor.setMotorInverted(ShooterConstants.IS_LEAD_INVERT);
    followMotor = new MarinersTalonFX("shooter_Follow_Motor",    ShooterConstants.CONTROLLER_LOCATION, ShooterConstants.FOLLOW_MOTOR_ID);
    followMotor.setMotorAsFollower(leadMotor,ShooterConstants.IS_FOLLOW_INVERT);
    
  }
  @Override
  public void setDutyCycle(double desired_dutyCycle){
    leadMotor.setDutyCycle(ShooterConstants.desired_dutyCycle);
    //leadMotor.setReference(desired_dutyCycle , ControlMode.DutyCycle);

  }

  @Override
  public double getAcceleration(){
    return leadMotor.getAcceleration();
  }

  @Override
  public double getVelocity(){
    return leadMotor.getVelocity();
  }

  @Override
  public void setVelocity(double desired_Velocity){
    leadMotor.setReference(desired_Velocity, ControlMode.Velocity);

  }

  @Override
  public void setVoltage(double desired_Voltage){
    leadMotor.setVoltage(desired_Voltage);
  }

  // @Override
  // public void setAngle(double desired_Angle){
  //   hoodMotor.setReference(desired_Angle / 360 , ControlMode.Position);
    

  // }
  // @Override 
  // public double getAngle(){

  

  // }
  @Override
  public void update(ShooterInputs shooterInputs){}



}
