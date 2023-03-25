// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class ZeroHeadingToAng extends CommandBase {
  private final Drive m_drive;
  private double m_angle;
  /** Creates a new ZeroHeadingToAng. */
  public ZeroHeadingToAng(Drive drive, double angle) {
    m_drive = drive;
    m_angle = angle;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    m_drive.zeroHeadingToAng(m_angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
