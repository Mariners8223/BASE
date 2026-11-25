// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.Rollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.TunableNumber;

public class Rollers extends SubsystemBase {
  RollersIO io;
  private final TunableNumber tuningFirstDutyCycle = new TunableNumber("Rollers/First DutyCycle", 0);
  private final TunableNumber tuningSecondDutyCycle = new TunableNumber("Rollers/Second DutyCycle", 0);
  /** Creates a new Rollers. */
  public Rollers() {
    io = new RollersIOReal();
  }

  public void setDutyCycle(double dutyCycle)
  {
    io.setDutyCycle(dutyCycle);
  }
  public void stopMotor(){
    io.setDutyCycle(0);
  }

  /** Test mode: set each roller motor's duty cycle from AdvantageScope (/Tuning/Rollers/...). */
  public Command tuningCommand() {
    return startRun(
        () -> {
          tuningFirstDutyCycle.markSeen();
          tuningSecondDutyCycle.markSeen();
          stopMotor();
        },
        () -> {
          if (tuningFirstDutyCycle.hasChanged()) {
            io.setFirstDutyCycle(MathUtil.clamp(tuningFirstDutyCycle.get(), -1, 1));
          }
          if (tuningSecondDutyCycle.hasChanged()) {
            io.setSecondDutyCycle(MathUtil.clamp(tuningSecondDutyCycle.get(), -1, 1));
          }
        })
        .finallyDo(this::stopMotor)
        .withName("Rollers Tuning");
  }
}