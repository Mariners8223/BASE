// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.hood;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drives the hood closed at -HOMING_DUTY_CYCLE until the stator current jumps (hood hit the hard stop),
 * then stops and resets the encoder.
 */
public class homeHood extends Command {
  private final hoodSubsystem hood;
  private final Timer homingTimer = new Timer();
  private final Timer statorCurrentTimer = new Timer();

  /** Creates a new homeHood. */
  public homeHood(hoodSubsystem hood) {
    this.hood = hood;
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hood.setHomed(false);
    hood.setSoftLimitsEnabled(false);
    statorCurrentTimer.stop();
    statorCurrentTimer.reset();
    homingTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hood.setDutyCycle(-hoodConstants.HOMING_DUTY_CYCLE);

    // ignore the current spike when the motor starts
    boolean currentHigh = homingTimer.hasElapsed(hoodConstants.HOMING_IGNORE_START_SECONDS)
        && Math.abs(hood.getStatorCurrent()) > hoodConstants.HOMING_CURRENT_THRESHOLD_AMPS;

    if (currentHigh) {
      statorCurrentTimer.start();
    } else {
      statorCurrentTimer.stop();
      statorCurrentTimer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.setDutyCycle(0);
    if (!interrupted && hitHardStop()) {
      hood.resetAngleEncoder();
      hood.setHomed(true);
    } else {
      DriverStation.reportWarning("Hood: homing did not finish (timeout or interrupted), encoder not reset", false);
    }
    hood.setSoftLimitsEnabled(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hitHardStop() || homingTimer.hasElapsed(hoodConstants.HOMING_TIMEOUT_SECONDS);
  }

  private boolean hitHardStop() {
    return statorCurrentTimer.hasElapsed(hoodConstants.HOMING_CURRENT_DEBOUNCE_SECONDS);
  }
}
