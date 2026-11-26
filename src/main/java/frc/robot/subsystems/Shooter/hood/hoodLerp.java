// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter.hood;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class hoodLerp extends Command {
  private final hoodSubsystem hood;
  private final double lerp_expected_position;
  /** Creates a new hoodLerp. */
  public hoodLerp(hoodSubsystem hood, double lerp_expected_position) {
    this.hood = hood;
    this.lerp_expected_position = lerp_expected_position;
    addRequirements(hood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hood.openToPosition(MathUtil.interpolate(hoodConstants.LERP_MIN_VAL, hoodConstants.LERP_MAX_VALUE, lerp_expected_position));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
